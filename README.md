# ros2-amr-perception

ROS2 + Gazebo 기반 AMR(자율이동로봇) 센서 데이터 처리 파이프라인 프로젝트입니다.
LiDAR / Depth Camera 센서 데이터를 수신하여 노이즈 필터링, 장애물 클러스터링, Obstacle Detection을 직접 구현하고,
결과를 Nav2 costmap과 연결하는 자율주행 인지 스택을 구성합니다.

---

## 개발 환경

- OS: Ubuntu 22.04
- ROS2: Humble
- Simulator: Gazebo 11 (Classic)
- Language: Python 3

---

## 프로젝트 로드맵

### 1. 센서 데이터 처리 파이프라인 (진행 중)

- [x] Gazebo 로봇 모델 제작 (4륜 AMR)
- [x] 3D LiDAR 플러그인 장착 (Velodyne VLP-16 모사, 16채널 PointCloud2)
- [x] Depth Camera 플러그인 장착 (RGB + Depth + PointCloud2)
- [x] PointCloud2 노이즈 필터링 직접 구현 (Statistical Outlier Removal)
- [ ] 장애물 클러스터링 직접 구현 (Distance-threshold / DBSCAN)
- [ ] Obstacle Detection 로직 구현 및 MarkerArray 시각화
- [ ] 결과를 Nav2 costmap과 연결

### 2. Navigation / 경로계획 심화

- [ ] Nav2 Behavior Tree 커스터마이징
- [ ] Costmap layer(inflation, static, obstacle) 튜닝
- [ ] 실패 케이스(좁은 통로, 동적 장애물) 재현 및 파라미터별 차이 문서화

### 3. ros2_control 기반 로봇 제어

- [ ] Differential Drive 커스텀 Hardware Interface 작성
- [ ] DDS / QoS 멀티캐스트 이슈 트러블슈팅 경험 적용

### 4. SLAM 실습

- [ ] slam_toolbox / cartographer 시뮬레이션 환경 구성
- [ ] 파라미터(맵 해상도, loop closure 조건)에 따른 매핑 품질 비교

---

## 패키지 구조

```
gazebo_sensor_ws/
└── src/
    └── amr_perception/
        ├── urdf/
        │   └── my_robot.urdf.xacro     # 로봇 모델 (LiDAR + Depth Camera)
        ├── worlds/
        │   └── my_world.world          # 장애물이 있는 테스트 환경
        ├── launch/
        │   └── gazebo.launch.py        # Gazebo + 로봇 스폰 런치파일
        ├── amr_perception/
        │   ├── sensor_processor.py         # 센서 데이터 처리 노드
        │   ├── pointcloud_filters.py       # 노이즈 필터링 알고리즘 (SOR)
        │   └── lidar_pointcloud_filter.py  # LiDAR 필터 노드
        ├── rviz/
        │   └── pointcloud_filter_compare.rviz  # 필터 전/후 비교용 rviz 설정
        ├── package.xml
        └── CMakeLists.txt
```

---

## 센서 토픽

| 센서 | 토픽 | 메시지 타입 |
|------|------|------------|
| 3D LiDAR (16ch) | `/lidar/points` | `sensor_msgs/PointCloud2` |
| Depth PointCloud | `/depth_camera/points` | `sensor_msgs/PointCloud2` |
| RGB Image | `/depth_camera/image_raw` | `sensor_msgs/Image` |
| Depth Image | `/depth_camera/depth/image_raw` | `sensor_msgs/Image` |

---

## 설계 노트

### PointCloud2 노이즈 필터링 (Statistical Outlier Removal)

PCL의 SOR 알고리즘을 라이브러리 호출 없이 numpy/scipy로 직접 구현했다 (`pointcloud_filters.py`).
포인트마다 k개 최근접 이웃까지의 평균거리를 구하고, 전체 평균(μ) + std_ratio × 표준편차(σ)를
넘으면 이상치로 판단해 제거하는 방식이다.

**문제 발견**: 처음 구현한 버전(절대 거리 기준)을 실제 시뮬레이션 LiDAR(360°, 16채널)에 돌려보니,
6~8m 거리 구간의 포인트가 **100% 제거**되는 현상이 나타났다. 각도 기준으로 균일 샘플링하는 센서는
포인트 간 간격이 센서에서 멀어질수록 부채꼴로 비례해서 벌어지는데(0-2m 평균 간격 0.029m vs 6-8m
평균 간격 0.236m, 약 8배 차이), 원본 SOR은 이 절대 간격만 보고 먼 거리의 정상적인 벽면 데이터를
전부 "고립된 이상치"로 오판했다.

**해결**: 이웃 평균거리를 포인트의 원점(센서) 거리로 나눠 정규화한 뒤 통계를 내도록 수정했다.
그 결과 6-8m 구간 제거율이 100% → 0%로 개선됐고, 원거리 벽면 데이터가 정상적으로 보존됐다
(rviz로 필터 전/후 포인트를 겹쳐서 시각 검증 완료).

**알려진 한계**: 로봇 자체 차체/바퀴에서 반사되는 근거리 self-return(0-2m 구간)은 여전히 많이
제거된다(약 48%). 이는 SOR의 오작동이 아니라 근접 반사파가 벽면과 통계적으로 다른 분포를 갖기
때문으로 보이며, 근본적인 해결은 SOR 이전에 별도의 최소거리 셀프필터를 두는 것이다 (다음 단계 예정).

---

## 실행 방법

```bash
# 1. 빌드
cd ~/gazebo_sensor_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 2. Gazebo 실행
ros2 launch amr_perception gazebo.launch.py

# 3. 센서 처리 노드 실행
ros2 run amr_perception sensor_processor.py

# 4. 포인트클라우드 노이즈 필터링 노드 실행
ros2 run amr_perception lidar_pointcloud_filter.py

# 5. 필터 전/후 비교 (rviz)
rviz2 -d install/amr_perception/share/amr_perception/rviz/pointcloud_filter_compare.rviz

# 6. 토픽 확인
ros2 topic list
ros2 topic hz /lidar/points
```
