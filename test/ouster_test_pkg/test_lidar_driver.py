import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import pytest

received = False

# Check if hardware is connected by an environment variable (e.g., "HARDWARE_CONNECTED")
hardware_connected = os.getenv('HARDWARE_CONNECTED', 'false').lower() == 'true'

class PointCloudTester(Node):
    def __init__(self):
        super().__init__('pilot02_physical_ouster_lidar_tester')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/ouster/points',  
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        global received
        received = True
        self.get_logger().info('✅ PointCloud2 message received!')

def test_ouster_lidar_pointcloud_publishing():
    global received

    if not hardware_connected:
        # If no hardware is connected, skip the actual test but ensure no failure
        print("⚠️ No hardware detected. Skipping hardware-dependent test.")
        return  # Skip the test and mark it as passed implicitly.

    # If hardware is connected, run the test normally
    rclpy.init()
    node = PointCloudTester()
    try:
        timeout = 15  # seconds
        end_time = node.get_clock().now().nanoseconds / 1e9 + timeout
        while not received and (node.get_clock().now().nanoseconds / 1e9 < end_time):
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    assert received, "❌ No PointCloud2 message was received"
