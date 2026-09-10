#!/usr/bin/env python3
"""
센서 데이터 처리 노드
- /lidar/points              : LiDAR PointCloud2 (3D, 16채널)
- /depth_camera/points       : PointCloud2
- /depth_camera/image_raw    : RGB Image
- /depth_camera/depth/image_raw : Depth Image
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np


class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # Gazebo 센서용 QoS (BestEffort)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # LiDAR subscriber (3D PointCloud2)
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.lidar_callback,
            sensor_qos
        )

        # PointCloud subscriber (Depth Camera)
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/depth_camera/points',
            self.pointcloud_callback,
            sensor_qos
        )

        # RGB Image subscriber
        self.rgb_sub = self.create_subscription(
            Image,
            '/depth_camera/image_raw',
            self.rgb_callback,
            sensor_qos
        )

        # Depth Image subscriber
        self.depth_sub = self.create_subscription(
            Image,
            '/depth_camera/depth/image_raw',
            self.depth_callback,
            sensor_qos
        )

        self.get_logger().info('SensorProcessor 시작! 토픽 수신 대기 중...')

    # ── LiDAR (3D PointCloud) ─────────────────────────────────────────────
    def lidar_callback(self, msg: PointCloud2):
        points = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if points.shape[0] == 0:
            return

        dists = np.linalg.norm(points, axis=1)
        min_dist = float(dists.min())
        max_dist = float(dists.max())
        mean_dist = float(dists.mean())

        # 전방 30도 (-15 ~ +15, x축이 전방이라고 가정) 평균 거리 계산
        angles = np.arctan2(points[:, 1], points[:, 0])
        front_mask = np.abs(angles) < np.deg2rad(15)
        front_vals = dists[front_mask]
        front_mean = float(front_vals.mean()) if front_vals.size > 0 else float('inf')

        self.get_logger().info(
            f'[LiDAR] min={min_dist:.2f}m  max={max_dist:.2f}m  '
            f'mean={mean_dist:.2f}m  front={front_mean:.2f}m  '
            f'valid_pts={points.shape[0]}'
        )

        # 전방 장애물 경고
        if front_mean < 0.5:
            self.get_logger().warn(f'[LiDAR] 전방 {front_mean:.2f}m 에 장애물!')

    # ── PointCloud ──────────────────────────────────────────────────────────
    def pointcloud_callback(self, msg: PointCloud2):
        total_points = msg.width * msg.height
        self.get_logger().info(
            f'[PointCloud] {msg.width}x{msg.height} = {total_points} pts  '
            f'encoding={msg.fields[0].name if msg.fields else "?"}...'
        )

    # ── RGB Image ───────────────────────────────────────────────────────────
    def rgb_callback(self, msg: Image):
        self.get_logger().info(
            f'[RGB] {msg.width}x{msg.height}  encoding={msg.encoding}  '
            f'step={msg.step}'
        )

    # ── Depth Image ─────────────────────────────────────────────────────────
    def depth_callback(self, msg: Image):
        # depth image는 32FC1 (float32) 포맷
        if msg.encoding == '32FC1':
            data = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            valid = data[np.isfinite(data) & (data > 0)]
            if len(valid) > 0:
                self.get_logger().info(
                    f'[Depth] {msg.width}x{msg.height}  '
                    f'min={valid.min():.2f}m  max={valid.max():.2f}m  '
                    f'mean={valid.mean():.2f}m'
                )
        else:
            self.get_logger().info(
                f'[Depth] {msg.width}x{msg.height}  encoding={msg.encoding}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = SensorProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
