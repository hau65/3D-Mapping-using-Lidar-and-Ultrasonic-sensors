#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, PointCloud2, PointField
from collections import deque
from scipy.spatial import cKDTree

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

SENSOR_Y_OFFSET = {
    'us_top_left':       0.17,
    'us_top_right':     -0.17,
    'us_mid_1_left':     0.17,
    'us_mid_1_right':   -0.17,
    'us_mid_2_left':     0.17,
    'us_mid_2_right':   -0.17,
    'us_bot_left':       0.17,
    'us_bot_right':     -0.17,
}

SENSOR_X_OFFSET      = 0.35   # m — offset sensor từ base_footprint
FILTER_WINDOW        = 5
SENSOR_MAX_RANGE     = 1.0

class Ultrasonic3DMapNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_3d_map_node')
        
        self.sensor_names = [
            'us_top_left',    'us_top_right',
            'us_mid_1_left',  'us_mid_1_right',
            'us_mid_2_left',  'us_mid_2_right',
            'us_bot_left',    'us_bot_right',
        ]

        self.buffers       = {n: deque(maxlen=FILTER_WINDOW) for n in self.sensor_names}
        self.sensor_angles = {n: math.radians(0) for n in self.sensor_names}
        self.points = []

        self.get_logger().info("=== Ultrasonic 3D Map Node ===")

        # ── Subscriptions & Publishers & Timers ─────────────────────────────────────────────
        for name in self.sensor_names:
            self.create_subscription(
                Range, f'/ultrasonic/{name}',
                lambda msg, n=name: self._range_cb(msg, n), 10)
            
        self.pointCloud2_pub = self.create_publisher(PointCloud2, '/ultrasonic_pointcloud', 10)

        self.create_timer(0.1, self._publish_pointcloud)

    @staticmethod
    def _cluster(points, eps=0.15, min_samples=5):
        pts = np.asarray(points, dtype=np.float32)
        if len(pts) == 0:
            return pts, np.empty(0, dtype=np.int32)

        tree = cKDTree(pts)
        neighbors = tree.query_ball_point(pts, r=eps)   # points within eps of each point
        labels = np.full(len(pts), -1, dtype=np.int32)  # -1 = noise
        cluster_id = 0

        for i, nb in enumerate(neighbors):
            if labels[i] != -1 or len(nb) < min_samples:
                continue                                # not a core point yet
            # flood fill: grow cluster from core point i
            labels[i] = cluster_id
            stack = list(nb)
            while stack:
                j = stack.pop()
                if labels[j] == -1:
                    labels[j] = cluster_id
                    if len(neighbors[j]) >= min_samples:  # border point extends cluster
                        stack.extend(k for k in neighbors[j] if labels[k] == -1)
            cluster_id += 1

        return pts, labels
    
    # ── Ultrasonic sensors callback ───────────────────────────────────────────────────────
    def _range_cb(self, msg: Range, name: str):
        if msg.min_range <= msg.range <= msg.max_range:
                    self.buffers[name].append(msg.range)
        elif msg.range >= msg.max_range:
            self.buffers[name].append(float('inf'))
        # < min_range → nhiễu cực gần, bỏ qua
        
        if (not self.buffers[name]):
            return

        finite_vals = [v for v in self.buffers[name] if not math.isinf(v)]
        filtered_dist = float(np.median(finite_vals)) if finite_vals else SENSOR_MAX_RANGE
            
        spread = 8
        angle = self.sensor_angles[name]
        
        for offset in range(-spread, spread + 1):
            a = angle + math.radians(offset * (15.0 / spread))

            x = filtered_dist * math.cos(a) + SENSOR_X_OFFSET
            y = filtered_dist * math.sin(a) + SENSOR_Y_OFFSET[name]
            z = SENSOR_HEIGHT[name]
            self.points.append((x, y, z))

    # ── Publish PointCloud2 ───────────────────────────────────────────────────────
    def _publish_pointcloud(self):
        if not self.points:
            return

        #clustering
        pts, labels = self._cluster(self.points, eps=0.15, min_samples=5)
        keep = labels != -1
        pts, labels = pts[keep], labels[keep]
        if len(pts) == 0:
            self.points = []
            return
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='cluster', offset=12, datatype=PointField.INT32, count=1),
        ]   

        arr = np.zeros(len(pts), dtype=[('x', '<f4'), ('y', '<f4'),
                                ('z', '<f4'), ('cluster', '<i4')])
        arr['x'], arr['y'], arr['z'], arr['cluster'] = pts[:, 0], pts[:, 1], pts[:, 2], labels

        cloud               = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = 'base_footprint'
        cloud.height        = 1
        cloud.width         = len(pts)
        cloud.fields        = fields
        cloud.point_step    = 16
        cloud.row_step      = 16 * len(pts)
        cloud.is_bigendian  = False
        cloud.is_dense      = True
        cloud.data = arr.tobytes() 

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