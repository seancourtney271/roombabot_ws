#!/usr/bin/env python3

import os
import math
import time
import yaml

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from roombabot_msgs.srv import RecordStation


# Teach-by-driving station recorder.
#
# The bumperbot version of this node watched for ArUco markers and computed a
# standoff pose in front of each one. With no camera on this robot there is nothing
# to detect, so stations are taught directly instead: drive the robot to exactly
# where it should stop, facing the way it should face, and call record_station. The
# pose that gets saved is the robot's own map-frame pose at that moment, which means
# what you drove to is precisely what the robot will drive back to -- no marker
# geometry, no standoff offset, and nothing to mis-estimate.
#
# A useful side effect: the recorded pose is reachable by construction. The old
# standoff calculation could and did place a goal inside the costmap's inflated
# region when a marker sat flush against a wall.
class StationRecorder(Node):
    def __init__(self):
        super().__init__("station_recorder_node")

        self.declare_parameter("map_name", "demo")
        # Empty by default -- if set, this exact path is used for save_stations instead
        # of deriving one from map_name.
        self.declare_parameter("stations_path", "")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("lookup_timeout", 2.0)

        self.map_name = self.get_parameter("map_name").value
        self.stations_path_override = self.get_parameter("stations_path").value
        self.global_frame = self.get_parameter("global_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.lookup_timeout = self.get_parameter("lookup_timeout").value

        # station_id -> {"x", "y", "yaw", "name"} in the map frame.
        self.stations = {}

        self.marker_publisher = self.create_publisher(MarkerArray, "/stations/found", 10)

        self.record_service = self.create_service(
            RecordStation, "record_station", self.record_station_callback)
        self.save_service = self.create_service(Trigger, "save_stations", self.save_stations_callback)

        self.get_logger().info(
            f"Station recorder ready for map '{self.map_name}'. Drive to each station, "
            "call 'record_station' with its id, then 'save_stations' to write stations.yaml.")

    def record_station_callback(self, request, response):
        try:
            transform = self._lookup_robot_pose()
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            response.success = False
            response.message = (
                f"Could not look up {self.global_frame} -> {self.robot_frame}: {ex}. "
                "Is SLAM running and publishing a map?")
            self.get_logger().warn(response.message)
            return response

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # Planar robot, so only the yaw component of the quaternion matters. Doing the
        # extraction inline avoids pulling in tf_transformations just for one formula.
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z))

        station_id = int(request.id)
        was_known = station_id in self.stations

        self.stations[station_id] = {
            "x": float(translation.x),
            "y": float(translation.y),
            "yaw": float(yaw),
            "name": str(request.name) if request.name else f"station_{station_id}",
        }

        response.success = True
        response.x = float(translation.x)
        response.y = float(translation.y)
        response.yaw = float(yaw)
        response.message = (
            f"{'Updated' if was_known else 'Recorded'} station {station_id} "
            f"('{self.stations[station_id]['name']}') at "
            f"x={translation.x:.3f} y={translation.y:.3f} yaw={math.degrees(yaw):.1f} deg")
        self.get_logger().info(response.message)

        self.publish_markers()
        return response

    def _lookup_robot_pose(self):
        # The TF listener exists only for the duration of this call. A permanent one
        # made this otherwise-idle node decode every /tf message in Python -- ~28% of a
        # Pi core (measured 2026-10-11) -- just to answer a service called a handful of
        # times per map. The listener's subscriptions sit in their own reentrant
        # callback group, so main()'s MultiThreadedExecutor keeps feeding the buffer
        # while this callback waits.
        buffer = Buffer()
        listener = TransformListener(buffer, self)
        try:
            deadline = time.monotonic() + self.lookup_timeout
            while True:
                try:
                    # "Latest available" is right here: there is no capture latency to
                    # compensate for, and the robot is parked when this is called.
                    return buffer.lookup_transform(
                        self.global_frame, self.robot_frame, rclpy.time.Time())
                except (LookupException, ConnectivityException, ExtrapolationException):
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(0.05)
        finally:
            listener.unregister()

    def publish_markers(self):
        marker_array = MarkerArray()
        for station_id, station in self.stations.items():
            sphere = Marker()
            sphere.header.frame_id = self.global_frame
            sphere.ns = "stations"
            sphere.id = station_id * 3
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = station["x"]
            sphere.pose.position.y = station["y"]
            sphere.pose.position.z = 0.1
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.15
            sphere.color.r = 0.1
            sphere.color.g = 0.9
            sphere.color.b = 0.1
            sphere.color.a = 1.0
            marker_array.markers.append(sphere)

            # Recorded heading matters now that it is the robot's own parked
            # orientation rather than a marker normal -- show it, so a station taught
            # facing the wrong way is obvious before the map is saved.
            arrow = Marker()
            arrow.header.frame_id = self.global_frame
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
            arrow.color.g = 0.9
            arrow.color.b = 0.1
            arrow.color.a = 1.0
            marker_array.markers.append(arrow)

            text = Marker()
            text.header.frame_id = self.global_frame
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
            text.text = f"{station_id}: {station['name']}"
            marker_array.markers.append(text)

        self.marker_publisher.publish(marker_array)

    def save_stations_callback(self, request, response):
        if not self.stations:
            response.success = False
            response.message = "No stations recorded yet -- call record_station at each station first."
            return response

        if self.stations_path_override:
            stations_path = self.stations_path_override
            os.makedirs(os.path.dirname(stations_path), exist_ok=True)
        else:
            maps_dir = os.path.join(self._source_maps_dir(), self.map_name)
            os.makedirs(maps_dir, exist_ok=True)
            stations_path = os.path.join(maps_dir, "stations.yaml")

        with open(stations_path, "w") as f:
            yaml.safe_dump({"stations": self.stations}, f, sort_keys=True)

        response.success = True
        response.message = (
            f"Saved {len(self.stations)} station(s) to {stations_path}. "
            "Rebuild roombabot_mapping before localizing so this reaches the install space too."
        )
        self.get_logger().info(response.message)
        return response

    def _source_maps_dir(self):
        # get_package_share_directory always points at the install space (by design --
        # install space is meant to work standalone, without src/ present). Maps are kept
        # in git under src/ though, so walk back from ".../install/roombabot_mapping/..."
        # to the workspace root and target src/roombabot_mapping/maps instead. This only
        # works for a standard (non-merged) colcon workspace layout, which is what this
        # robot uses.
        share_dir = get_package_share_directory("roombabot_mapping")
        workspace_root, sep, _ = share_dir.partition(os.sep + "install" + os.sep)
        if not sep:
            return os.path.join(share_dir, "maps")
        return os.path.join(workspace_root, "src", "roombabot_mapping", "maps")


def main(args=None):
    rclpy.init(args=args)
    node = StationRecorder()
    # Multi-threaded so the transient TF listener's callbacks can run while
    # record_station_callback is blocked waiting on the buffer.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
