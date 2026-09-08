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
- [ ] PointCloud2 노이즈 필터링 직접 구현 (Statistical Outlier Removal)
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

### 5. 오픈소스 기여

- [ ] ROS2 / Nav2 / gz-sim 관련 이슈 리포트 또는 문서 개선 PR

---

## 패키지 구조

```
gazebo_sensor_ws/
└── src/
    └── my_robot/
        ├── urdf/
        │   └── my_robot.urdf.xacro     # 로봇 모델 (LiDAR + Depth Camera)
        ├── worlds/
        │   └── my_world.world          # 장애물이 있는 테스트 환경
        ├── launch/
        │   └── gazebo.launch.py        # Gazebo + 로봇 스폰 런치파일
        ├── my_robot/
        │   └── sensor_processor.py    # 센서 데이터 처리 노드
        ├── package.xml
        └── CMakeLists.txt
```

---

## 센서 토픽

| 센서 | 토픽 | 메시지 타입 |
|------|------|------------|
| 3D LiDAR (16ch) | `/lidar/points` | `sensor_msgs/PointCloud2` |
| Depth PointCloud | `/camera/depth/points` | `sensor_msgs/PointCloud2` |
| RGB Image | `/camera/color/image_raw` | `sensor_msgs/Image` |
| Depth Image | `/camera/depth/image_raw` | `sensor_msgs/Image` |

---

## 실행 방법

```bash
# 1. 빌드
cd ~/gazebo_sensor_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 2. Gazebo 실행
ros2 launch my_robot gazebo.launch.py

# 3. 센서 처리 노드 실행
ros2 run my_robot sensor_processor.py

# 4. 토픽 확인
ros2 topic list
ros2 topic hz /lidar/points
```
