#!/usr/bin/env python3
"""포인트클라우드 장애물 클러스터링 알고리즘 (DBSCAN)"""

import numpy as np
from scipy.spatial import cKDTree


def dbscan(points: np.ndarray, eps: float = 0.3, min_samples: int = 5) -> np.ndarray:
    """밀도 기반 클러스터링. 반경 eps 안에 min_samples개 이상의 이웃(자기 자신 포함)이
    있는 점을 core point로 보고, core point끼리/core point의 이웃끼리 연쇄적으로
    묶어서 같은 클러스터로 확장한다.

    Returns
    -------
    labels : (N,) int array. 같은 클러스터는 같은 번호(0,1,2,...), 노이즈는 -1.
    """
    n = points.shape[0]
    labels = np.full(n, -1, dtype=int)
    visited = np.zeros(n, dtype=bool)

    tree = cKDTree(points)

    cluster_id = 0
    for i in range(n):
        if visited[i]:
            continue
        visited[i] = True

        neighbors = tree.query_ball_point(points[i], r=eps)
        if len(neighbors) < min_samples:
            continue  # 일단 노이즈. 나중에 다른 core point의 이웃이면 border로 승격될 수 있음

        # i는 core point -> 새 클러스터 시작, BFS 방식으로 계속 확장
        labels[i] = cluster_id
        seed_queue = list(neighbors)

        idx = 0
        while idx < len(seed_queue):
            j = seed_queue[idx]
            idx += 1

            if not visited[j]:
                visited[j] = True
                j_neighbors = tree.query_ball_point(points[j], r=eps)
                if len(j_neighbors) >= min_samples:
                    # j도 core point -> j의 이웃들도 확장 대상 큐에 추가
                    seed_queue.extend(j_neighbors)

            if labels[j] == -1:
                labels[j] = cluster_id

        cluster_id += 1

    return labels
