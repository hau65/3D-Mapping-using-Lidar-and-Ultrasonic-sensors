#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from rclpy.time import Time

WHEEL_RADIUS = 0.08255

# laser_joint: xyz="0.257 0 0.075" trên chassis, rpy="0 0 pi"
LIDAR_X_OFFSET = 0.257
LIDAR_Z_OFFSET = 0.075 + WHEEL_RADIUS   # 0.15755 m so với base_footprint

# Điểm cực đại cho beam không có phản hồi (inf sau LaserScanRangeFilter@15 m).
# Đo từ laser → D từ base_footprint luôn > sensor_model.max_range = 15.0
# (hướng xấu nhất: 15.5 - 0.257 = 15.24 > 15.0) → octomap coi là free ray.
CLEARANCE_RANGE = 15.5


class LidarCloudNode(Node):
    def __init__(self):
        super().__init__('lidar_cloud_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PointCloud2, '/lidar_pointcloud', 10)
        self.create_subscription(LaserScan, '/scan_filtered', self._scan_cb, 10)

        self.get_logger().info("=== Lidar 3D Cloud Node ===")

    def _scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return

        ranges = np.asarray(msg.ranges, dtype=np.float64)
        angles = msg.angle_min + np.arange(n) * msg.angle_increment

        hit = np.isfinite(ranges) & (ranges >= msg.range_min + 1) & (ranges <= msg.range_max) # + 1 debug
        no_echo = np.isposinf(ranges)          # filter: upper_replacement_value = .inf
        mask = hit | no_echo
        if not mask.any():
            return

        d = np.where(hit, ranges, 0.0)
        d = np.where(no_echo, CLEARANCE_RANGE, d)

        # Điểm trong laser_frame (z = 0 — mặt cắt ngang)
        xs = d[mask] * np.cos(angles[mask])
        ys = d[mask] * np.sin(angles[mask])
        zs = np.zeros(xs.shape[0])
        pts = np.stack([xs, ys, zs], axis=1)

        # TF base_footprint <- laser_frame (xử lý yaw=pi của laser_joint)
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_footprint', 'laser_frame',
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.05))
        except Exception:
            self.get_logger().warn(
                'TF base_footprint<-laser_frame not available yet',
                throttle_duration_sec=2.0)
            return

        q = tf.transform.rotation
        r = np.array([
            [1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w)],
            [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z - q.x * q.w)],
            [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x * q.x + q.y * q.y)],
        ])
        t = np.array([tf.transform.translation.x,
                      tf.transform.translation.y,
                      tf.transform.translation.z])
        pts = pts @ r.T + t

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud = PointCloud2()
        cloud.header.stamp = msg.header.stamp
        cloud.header.frame_id = 'base_footprint'
        cloud.height = 1
        cloud.width = pts.shape[0]
        cloud.fields = fields
        cloud.point_step = 12
        cloud.row_step = 12 * pts.shape[0]
        cloud.is_bigendian = False
        cloud.is_dense = True
        cloud.data = pts.astype(np.float32).tobytes()

        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = LidarCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
