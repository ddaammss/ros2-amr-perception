#!/usr/bin/env python3
"""LiDAR PointCloud2 노이즈 필터링 노드
- 구독: /lidar/points
- 발행: /lidar/points_filtered
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from amr_perception.pointcloud_filters import statistical_outlier_removal, filter_finite


class LidarPointCloudFilter(Node):
    def __init__(self):
        super().__init__('lidar_pointcloud_filter')

        self.declare_parameter('k_neighbors', 8)
        self.declare_parameter('std_ratio', 1.0)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.cloud_callback,
            sensor_qos
        )
        self.pub = self.create_publisher(PointCloud2, '/lidar/points_filtered', sensor_qos)

        self.get_logger().info('LidarPointCloudFilter 시작! /lidar/points 구독 중...')

    def cloud_callback(self, msg: PointCloud2):
        points = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        points = filter_finite(points)
        if points.shape[0] == 0:
            return

        k = self.get_parameter('k_neighbors').value
        std_ratio = self.get_parameter('std_ratio').value

        inlier_mask = statistical_outlier_removal(points, k=k, std_ratio=std_ratio)
        filtered = points[inlier_mask]

        out_msg = pc2.create_cloud_xyz32(msg.header, filtered.tolist())
        self.pub.publish(out_msg)

        removed = points.shape[0] - filtered.shape[0]
        self.get_logger().info(
            f'[LiDAR Filter] {points.shape[0]} -> {filtered.shape[0]} pts '
            f'({removed}개 제거, {removed / points.shape[0] * 100:.1f}%)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarPointCloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
