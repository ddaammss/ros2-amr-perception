#!/usr/bin/env python3
"""PointCloud2 노이즈 필터링 알고리즘 (Statistical Outlier Removal)"""

import numpy as np
from scipy.spatial import cKDTree


def filter_finite(points: np.ndarray) -> np.ndarray:
    """depth 카메라 등은 사거리를 벗어난 픽셀을 inf로 채우므로,
    KDTree에 넣기 전에 non-finite(NaN/Inf) 포인트를 제거한다."""
    return points[np.isfinite(points).all(axis=1)]


def statistical_outlier_removal(points: np.ndarray, k: int = 8, std_ratio: float = 1.0) -> np.ndarray:
    """포인트별로 k개 최근접 이웃까지의 평균 거리를 구하고,
    전체 평균(mu) + std_ratio * 표준편차(sigma) 를 넘는 포인트를 이상치로 판단해 제거한다.
    (PCL의 StatisticalOutlierRemoval과 동일한 통계 기준)

    LiDAR/카메라처럼 각도(또는 픽셀) 단위로 균일 샘플링하는 센서는 포인트 간 간격이
    원점(센서)에서 멀어질수록 부채꼴로 비례해서 벌어진다. 이웃 거리를 그대로 통계 내면
    "먼 거리의 정상적인 벽면"이 전부 이상치로 오판된다. 그래서 이웃 평균거리를 포인트의
    원점 거리(range)로 나눠 정규화한 뒤 통계를 낸다 — range에 비례하는 자연스러운 간격
    증가분을 상쇄하고, range와 무관하게 실제로 국소적으로 고립된 점만 남긴다.

    Returns
    -------
    inlier_mask : (N,) bool array. True인 인덱스만 정상 포인트.
    """
    n = points.shape[0]
    if n <= k:
        return np.ones(n, dtype=bool)

    # k+1: 자기 자신(거리 0)이 최근접 이웃에 포함되므로 하나 더 조회 후 제외
    tree = cKDTree(points)  # (N,3) 포인트들로 KD-Tree(공간 탐색 자료구조)를 구성
    dists, _ = tree.query(points, k=k + 1)  # 포인트마다 가장 가까운 k+1개 이웃까지의 거리를 구함. dists shape=(N, k+1)
    mean_dists = dists[:, 1:].mean(axis=1)  # 0번째 열(자기 자신, 거리 0)은 빼고 나머지 k개 이웃 거리의 평균을 포인트별로 계산 → shape=(N,)

    ranges = np.maximum(np.linalg.norm(points, axis=1), 1e-3)  # 포인트마다 원점(센서)까지의 거리(range) 계산. 0으로 나누기 방지용 최소값 1e-3 적용
    normalized = mean_dists / ranges  # 이웃평균거리를 range로 나눠 정규화 → 거리가 멀어질수록 자연스레 벌어지는 간격을 상쇄

    mu = normalized.mean()  # 정규화된 값 전체의 평균
    sigma = normalized.std()  # 정규화된 값 전체의 표준편차
    threshold = mu + std_ratio * sigma  # 이상치 판정 임계값. std_ratio가 클수록 관대(적게 제거), 작을수록 엄격(많이 제거)

    return normalized <= threshold
