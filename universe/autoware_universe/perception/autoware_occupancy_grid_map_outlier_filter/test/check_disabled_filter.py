"""Run with the built workspace sourced and an unused ROS_DOMAIN_ID (not the vehicle's)."""

import os
import struct
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster


def main():
    assert os.environ.get("ROS_DOMAIN_ID"), "Choose an unused, isolated ROS_DOMAIN_ID first"
    package = "autoware_occupancy_grid_map_outlier_filter"
    command = [
        f"{get_package_prefix(package)}/lib/{package}/{package}_node",
        "--ros-args", "--params-file",
        f"{get_package_share_directory(package)}/config/occupancy_grid_map_outlier_filter.param.yaml",
        "-p", "use_radius_search_2d_filter:=false", "-r", "__node:=origin_test_filter",
    ]
    process = subprocess.Popen(command)
    rclpy.init()
    node = rclpy.create_node("origin_test_input")
    try:
        broadcaster = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = 2500.0
        transform.transform.translation.y = 24500.0
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
        cloud_pub = node.create_publisher(PointCloud2, "/origin_test_filter/input/pointcloud", 10)
        map_pub = node.create_publisher(OccupancyGrid, "/origin_test_filter/input/occupancy_grid_map", 10)
        outputs = []
        subscription = node.create_subscription(
            PointCloud2, "/origin_test_filter/output/pointcloud", outputs.append,
            qos_profile_sensor_data,
        )
        cloud = PointCloud2(height=1, width=2, point_step=12, row_step=24, is_dense=True)
        cloud.header.frame_id = "base_link"
        cloud.fields = [PointField(name=n, offset=i * 4, datatype=7, count=1)
                        for i, n in enumerate(("x", "y", "z"))]
        # One low-confidence front point and one unfiltered rear point.
        cloud.data = struct.pack("ffffff", 10.0, 0.0, 1.0, -10.0, 0.0, 1.0)
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.resolution = 1.0
        grid.info.width = grid.info.height = 100
        grid.info.origin.position.x = 2450.0
        grid.info.origin.position.y = 24450.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * 10000
        deadline = time.monotonic() + 10
        while len(outputs) < 3 and time.monotonic() < deadline:
            assert process.poll() is None, "Filter process exited"
            cloud.header.stamp = grid.header.stamp = node.get_clock().now().to_msg()
            cloud_pub.publish(cloud)
            map_pub.publish(grid)
            rclpy.spin_once(node, timeout_sec=0.1)
        assert len(outputs) >= 3, "No sufficient output received"
        for output in outputs:
            assert output.width == 1, f"Unexpected extra points: {output.width}"
            xyz = struct.unpack_from("fff", output.data)
            assert all(abs(a - b) < 0.01 for a, b in zip(xyz, (-10, 0, 1))), xyz
        print("PASS: disabled filter rejects low-confidence points without origin placeholders")
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
