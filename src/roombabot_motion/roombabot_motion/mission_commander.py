#!/usr/bin/env python3

import math
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


# Loads the station poses recorded by station_recorder for the current map, shows them
# in Foxglove, and waits idle until commanded (via /mission/goto_station) before
# sending the robot anywhere -- reusing the existing /goal_pose -> waypoint_commander
# -> planner_server -> pure_pursuit pipeline.
#
# Stations are now taught by driving the robot to them (see station_recorder), so the
# recorded pose *is* the final pose. There is no marker-based final approach to hand
# off to, and arriving at the goal is the end of the mission.
#
# The handoff point is nonetheless still here, gated behind use_docking (default
# false). When the goal is reached it publishes the station id on
# /docking/target_station_id, which docking_controller currently answers with an
# immediate "docked". That keeps the plumbing -- topics, sequencing, the
# /docking/docked completion signal -- intact so a future final-approach sensor can be
# dropped in without touching this node again.
class MissionCommander(Node):
    def __init__(self):
        super().__init__("mission_commander_node")

        self.declare_parameter("map_name", "bedroom")
        self.declare_parameter("use_docking", False)

        self.map_name = self.get_parameter("map_name").value
        self.use_docking = self.get_parameter("use_docking").value

        self.stations = self.load_stations()
        # Station currently being navigated to, or None when idle.
        self.pending_station_id = None
        # Last station handed off to docking. /docking/docked carries only a bool, so
        # this is what names the station in the completion message.
        self.last_handoff_id = None

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.goal_publisher = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.marker_publisher = self.create_publisher(MarkerArray, "/stations/markers", latched_qos)
        self.dock_target_publisher = self.create_publisher(Int32, "/docking/target_station_id", 10)
        self.mission_complete_publisher = self.create_publisher(Int32, "/mission/arrived", 10)

        self.goto_subscriber = self.create_subscription(
            Int32, "/mission/goto_station", self.goto_station_callback, 10)
        self.goal_reached_subscriber = self.create_subscription(
            Bool, "/purepursuit/goal_reached", self.goal_reached_callback, 10)
        self.docked_subscriber = self.create_subscription(
            Bool, "/docking/docked", self.docked_callback, 10)

        self.publish_markers()

        self.get_logger().info(
            f"Mission commander ready for map '{self.map_name}' with "
            f"{len(self.stations)} station(s) -- waiting for a /mission/goto_station command."
            + ("" if self.use_docking else " Docking handoff disabled."))

    def load_stations(self):
        stations_path = os.path.join(
            get_package_share_directory("roombabot_mapping"), "maps", self.map_name, "stations.yaml")

        if not os.path.exists(stations_path):
            self.get_logger().warn(
                f"No stations.yaml found at {stations_path} -- run station_recorder and its "
                "record_station/save_stations services while mapping first.")
            return {}

        with open(stations_path, "r") as f:
            data = yaml.safe_load(f) or {}

        return {int(k): v for k, v in (data.get("stations") or {}).items()}

    def goto_station_callback(self, msg: Int32):
        self.go_to_station(msg.data)

    def go_to_station(self, station_id):
        station = self.stations.get(station_id)
        if station is None:
            self.get_logger().warn(
                f"Unknown station id {station_id} -- known stations: {sorted(self.stations)}")
            return

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = station["x"]
        goal.pose.position.y = station["y"]

        # Planar goal, so the quaternion is just a rotation about Z. Computing it here
        # drops the tf_transformations dependency, which is a Python package that has
        # to be pip-installed separately on the Pi.
        yaw = station["yaw"]
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self.pending_station_id = station_id
        self.goal_publisher.publish(goal)
        self.get_logger().info(
            f"Sending goal to station {station_id} ('{station.get('name', station_id)}')")

    def goal_reached_callback(self, msg: Bool):
        if not msg.data or self.pending_station_id is None:
            return

        station_id = self.pending_station_id
        self.pending_station_id = None

        if not self.use_docking:
            self.announce_arrival(station_id)
            return

        self.last_handoff_id = station_id
        dock_msg = Int32()
        dock_msg.data = station_id
        self.dock_target_publisher.publish(dock_msg)
        self.get_logger().info(
            f"Goal reached -- handing off to docking_controller for station {station_id}")

    def docked_callback(self, msg: Bool):
        if msg.data and self.use_docking:
            # The station id is not echoed back on /docking/docked, so report the last
            # one handed off. Only one mission is ever in flight at a time.
            self.announce_arrival(self.last_handoff_id)

    def announce_arrival(self, station_id):
        if station_id is None:
            return
        arrived = Int32()
        arrived.data = station_id
        self.mission_complete_publisher.publish(arrived)
        self.get_logger().info(f"Arrived at station {station_id} -- mission complete")

    def publish_markers(self):
        marker_array = MarkerArray()
        for station_id, station in self.stations.items():
            sphere = Marker()
            sphere.header.frame_id = "map"
            sphere.ns = "stations"
            sphere.id = station_id * 3
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = station["x"]
            sphere.pose.position.y = station["y"]
            sphere.pose.position.z = 0.1
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.15
            sphere.color.r = 0.1
            sphere.color.g = 0.4
            sphere.color.b = 0.9
            sphere.color.a = 1.0
            marker_array.markers.append(sphere)

            arrow = Marker()
            arrow.header.frame_id = "map"
            arrow.ns = "stations"
            arrow.id = station_id * 3 + 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = station["x"]
            arrow.pose.position.y = station["y"]
            arrow.pose.position.z = 0.1
            arrow.pose.orientation.z = math.sin(station["yaw"] / 2.0)
            arrow.pose.orientation.w = math.cos(station["yaw"] / 2.0)
            arrow.scale.x = 0.3
            arrow.scale.y = 0.04
            arrow.scale.z = 0.04
            arrow.color.r = 0.1
            arrow.color.g = 0.4
            arrow.color.b = 0.9
            arrow.color.a = 1.0
            marker_array.markers.append(arrow)

            text = Marker()
            text.header.frame_id = "map"
            text.ns = "stations"
            text.id = station_id * 3 + 2
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = station["x"]
            text.pose.position.y = station["y"]
            text.pose.position.z = 0.4
            text.scale.z = 0.2
            text.color.r = text.color.g = text.color.b = 1.0
            text.color.a = 1.0
            text.text = f"{station_id}: {station.get('name', station_id)}"
            marker_array.markers.append(text)

        self.marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = MissionCommander()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
