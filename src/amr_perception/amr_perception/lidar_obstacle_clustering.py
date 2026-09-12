#!/usr/bin/env python3
"""LiDAR 장애물 클러스터링 노드 (DBSCAN)
- 구독: /lidar/points_filtered
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from amr_perception.pointcloud_clustering import dbscan


class LidarObstacleClustering(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_clustering')

        self.declare_parameter('eps', 0.3)
        self.declare_parameter('min_samples', 5)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            PointCloud2,
            '/lidar/points_filtered',
            self.cloud_callback,
            sensor_qos
        )

        self.get_logger().info('LidarObstacleClustering 시작! /lidar/points_filtered 구독 중...')

    def cloud_callback(self, msg: PointCloud2):
        points = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if points.shape[0] == 0:
            return

        eps = self.get_parameter('eps').value
        min_samples = self.get_parameter('min_samples').value

        labels = dbscan(points, eps=eps, min_samples=min_samples)

        cluster_ids = sorted(set(labels.tolist()) - {-1})
        noise_count = int((labels == -1).sum())

        summary = []
        for cid in cluster_ids:
            cluster_points = points[labels == cid]
            centroid = cluster_points.mean(axis=0)
            summary.append(
                f'#{cid}: {cluster_points.shape[0]}pts centroid=({centroid[0]:.2f},{centroid[1]:.2f},{centroid[2]:.2f})'
            )

        self.get_logger().info(
            f'[Clustering] 장애물 {len(cluster_ids)}개, 노이즈 {noise_count}개 | ' + ' | '.join(summary)
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleClustering()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
