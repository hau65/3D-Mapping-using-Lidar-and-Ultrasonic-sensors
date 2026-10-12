#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range, PointCloud2, PointField
from std_msgs.msg import Header

WHEEL_RADIUS = 0.08255

SENSOR_HEIGHT = {
    'us_top_left':    0.48 + WHEEL_RADIUS,
    'us_top_right':   0.48 + WHEEL_RADIUS,
    'us_mid_1_left':  0.36 + WHEEL_RADIUS,
    'us_mid_1_right': 0.36 + WHEEL_RADIUS,
    'us_mid_2_left':  0.24 + WHEEL_RADIUS,
    'us_mid_2_right': 0.24 + WHEEL_RADIUS,
    'us_bot_left':    0.12 + WHEEL_RADIUS,
    'us_bot_right':   0.12 + WHEEL_RADIUS,
}

class Ultrasonic3DMapNode(Node):

    SENSOR_X_OFFSET      = 0.35   # m — offset sensor từ base_footprint
    LIDAR_Z_OFFSET       = 0.075  # m — offset lidar từ base_footprint 

    def __init__(self):
        super().__init__('ultrasonic_3d_map_node')
        
        self.sensor_names = [
            'us_top_left',    'us_top_right',
            'us_mid_1_left',  'us_mid_1_right',
            'us_mid_2_left',  'us_mid_2_right',
            'us_bot_left',    'us_bot_right',
        ]

        self.sensor_angles = {n: math.radians(0) for n in self.sensor_names}
        self.points = []

        self.get_logger().info("=== Ultrasonic 3D Map Node ===")

        # ── Subscriptions & Publishers & Timers ─────────────────────────────────────────────
        for name in self.sensor_names:
            self.create_subscription(
                Range, f'/ultrasonic/{name}',
                lambda msg, n=name: self._range_cb(msg, n), 10)
            
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.pointCloud2_pub = self.create_publisher(PointCloud2, '/ultrasonic_pointcloud', 10)

        self.create_timer(0.1, self._publish_pointcloud)
    
    #  ── Lidar callback ─────────────────────────────────────────────
    def lidar_callback(self, msg):
        for i, dist in enumerate(msg.ranges):
            if math.isinf(dist) or math.isnan(dist):
                continue

            # calculate the position of the point in 3D space based on the lidar scan data
            angle = msg.angle_min + i * msg.angle_increment
            x = dist * math.cos(angle)
            y = dist * math.sin(angle)
            z = self.LIDAR_Z_OFFSET
            self.points.append((x, y, z))

    # ── Ultrasonic sensors callback ───────────────────────────────────────────────────────
    def _range_cb(self, msg: Range, name: str):
        spread = 8
        angle = self.sensor_angles[name]
        dist = msg.range

        # calculate the position of the point in 3D space based on the ultrasonic sensor data
        for offset in range(-spread, spread+1):
            a = angle + math.radians(offset * (15.0 / spread));
            x = dist * math.cos(a) + self.SENSOR_X_OFFSET
            y = dist * math.sin(a)
            z = SENSOR_HEIGHT[name]
            self.points.append((x, y, z))

    # ── Publish PointCloud2 ───────────────────────────────────────────────────────
    def _publish_pointcloud(self):
        if not self.points:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_footprint'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]   

        cloud               = PointCloud2()
        cloud.header        = header
        cloud.height        = 1
        cloud.width         = len(self.points)
        cloud.fields        = fields
        cloud.point_step    = 12 
        cloud.row_step      = 12 * len(self.points)
        cloud.is_bigendian  = False
        cloud.is_dense      = True
        cloud.data          = np.array(self.points, dtype=np.float32).tobytes()

        self.pointCloud2_pub.publish(cloud)
        self.points = []

def main(args=None):
    rclpy.init(args=args)
    node = Ultrasonic3DMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()