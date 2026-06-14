# F1TENTH ROS2 Autonomous Driving

ROS2 기반 F1TENTH 시뮬레이션 환경에서 LiDAR와 Odometry 데이터를 사용해 자율주행 차량의 **인지 → 판단 → 경로 계획 → 경로 추종 → 차량 제어** 과정을 직접 구현한 프로젝트입니다.

본 프로젝트는 Nav2 전체 내비게이션 스택이나 강화학습 모델을 사용하지 않고, 자율주행 핵심 알고리즘을 Python 기반 ROS2 노드로 직접 구현한 것이 핵심입니다.

---

## 1. 프로젝트 개요

| 항목 | 내용 |
|---|---|
| 프로젝트명 | F1TENTH ROS2 Autonomous Driving |
| 개발 기간 | 2025.03 ~ 2025.06 |
| 개발 인원 | 1명 |
| 주요 분야 | Mobile Robot, Autonomous Driving, Path Planning / Control |
| 개발 환경 | Ubuntu 22.04, ROS2 Humble, Python, RViz, F1TENTH Gym |
| 주요 센서 | 2D LiDAR, Odometry |
| 제어 방식 | AckermannDriveStamped 기반 조향/속도 제어 |

---

## 2. 프로젝트 목표

F1TENTH 차량이 실내 트랙 환경에서 스스로 주행하기 위해 필요한 핵심 기능을 단계적으로 구현했습니다.

- LiDAR 기반 주변 장애물 인지
- TTC(Time-To-Collision) 기반 자동 긴급 정지
- PID 기반 Wall Following
- Follow-the-Gap 기반 장애물 회피
- Safety Bubble 기반 위험 영역 제거
- Waypoint 기반 경로 추종
- Occupancy Grid 기반 충돌 검사
- RRT / RRT* 기반 경로 계획
- Pure Pursuit 기반 조향각 계산
- Ackermann 차량 제어 명령 발행
- RViz 기반 경로 및 트리 시각화

---

## 3. 기술 스택

| 구분 | 사용 기술 |
|---|---|
| Middleware | ROS2 Humble |
| Language | Python |
| Simulator | F1TENTH Gym, F1TENTH Gym ROS |
| Visualization | RViz |
| Sensor Topic | `/scan`, `/ego_racecar/odom` |
| Control Topic | `/drive` |
| Control Message | `AckermannDriveStamped` |
| Planning | RRT, RRT*, Occupancy Grid |
| Tracking | Pure Pursuit |
| Safety | TTC, Safety Bubble |
| Avoidance | Follow-the-Gap |
| Wall Tracking | PID Control |

---

## 4. 전체 시스템 구조

```text
F1TENTH Gym / Simulator
        ↓
/scan LiDAR 데이터 수신
        ↓
장애물 거리 계산 및 위험도 판단
        ↓
Safety Node / Follow Gap / Wall Follow
        ↓
/ego_racecar/odom 기반 현재 위치 및 yaw 계산
        ↓
Waypoint 또는 목표 지점 선택
        ↓
Occupancy Grid 기반 충돌 검사
        ↓
RRT* 기반 경로 계획
        ↓
Pure Pursuit 기반 조향각 계산
        ↓
/drive AckermannDriveStamped 발행
        ↓
차량 조향 및 속도 제어
```

---

## 5. 패키지 구성

```text
src/
├── f1tenth_gym_ros/       # F1TENTH Gym ROS 시뮬레이션 및 map server 실행
├── safety_node/           # TTC 기반 자동 긴급 정지
├── wall_follow/           # PID 기반 벽 추종 주행
├── follow_gap/            # Follow-the-Gap 장애물 회피
├── pure_pursuit/          # Waypoint 기반 경로 추종
├── rrt_node/              # RRT 경로 계획 기본 구현
└── project_f1_tenth/      # 최종 통합: RRT* + Occupancy Grid + Pure Pursuit
```

---

## 6. 주요 구현 내용

### 6.1 Safety Node: 자동 긴급 정지

`/scan` LiDAR 데이터와 `/ego_racecar/odom`의 차량 속도를 이용해 TTC(Time-To-Collision)를 계산했습니다.

```text
TTC = obstacle_distance / relative_velocity
```

충돌 예상 시간이 임계값 이하로 내려가면 `/drive` 토픽으로 속도 0의 AckermannDriveStamped 메시지를 발행하여 차량을 정지시킵니다.

구현 특징:

- LiDAR beam 중 가장 가까운 장애물 탐색
- 현재 차량 속도 기반 충돌 예상 시간 계산
- 충돌 위험 시 즉시 제동 명령 발행
- 전방/측면 위험도 차이를 고려한 임계값 튜닝

---

### 6.2 Wall Following: PID 기반 벽 추종

LiDAR의 특정 각도 거리값을 사용해 벽과 차량 사이의 거리 오차를 계산하고, PID 제어를 통해 조향각을 결정했습니다.

구현 흐름:

```text
LiDAR 35도, 90도 거리 측정
        ↓
벽과 차량 사이 각도 alpha 계산
        ↓
미래 위치 기준 벽과의 예상 거리 계산
        ↓
목표 거리와의 error 계산
        ↓
PID 제어로 steering angle 계산
        ↓
조향각 크기에 따라 속도 조절
```

---

### 6.3 Follow-the-Gap: 장애물 회피

LiDAR 전방 영역을 전처리한 뒤, 장애물과 충돌 가능성이 높은 구간을 제거하고 가장 넓은 빈 공간으로 주행하도록 구현했습니다.

구현 흐름:

```text
LiDAR ranges 수신
        ↓
전방 관심 영역만 crop
        ↓
NaN / inf / 과도한 거리값 보정
        ↓
가까운 장애물 탐색
        ↓
Safety Bubble 적용
        ↓
가장 넓은 gap 탐색
        ↓
gap 중앙 방향으로 조향
```

초기에는 단순히 가장 먼 거리 방향으로 이동하는 방식이었지만, 차량 폭과 충돌 가능성을 반영하기 위해 Safety Bubble을 적용했습니다.

---

### 6.4 Pure Pursuit: Waypoint 기반 경로 추종

Waypoint CSV 파일을 불러와 차량의 현재 위치 기준으로 전방 목표점을 선택하고, 해당 목표점을 따라가도록 조향각을 계산했습니다.

구현 흐름:

```text
Odometry 수신
        ↓
현재 x, y, yaw 계산
        ↓
Waypoint 중 차량 전방에 있는 lookahead point 선택
        ↓
목표점을 차량 좌표계로 변환
        ↓
곡률 계산
        ↓
Ackermann steering angle 계산
        ↓
/drive 발행
```

조향각은 차량의 물리적 제한을 고려해 최대 조향각 범위로 제한했습니다.

---

### 6.5 RRT / RRT*: 경로 계획

최종 통합 패키지 `project_f1_tenth`에서는 RRT를 기반으로 경로를 생성하고, RRT*의 핵심 요소인 cost 기반 parent selection과 rewiring을 적용했습니다.

기본 흐름:

```text
1. 현재 차량 위치를 start node로 설정
2. 현재 목표 waypoint를 goal node로 설정
3. 주행 가능 영역에서 random sample 생성
4. tree에서 sample과 가장 가까운 nearest node 탐색
5. steer 함수로 새 node 생성
6. Occupancy Grid 기반 collision check 수행
7. 주변 near node 탐색
8. 가장 cost가 낮은 parent 선택
9. 새 node를 tree에 추가
10. rewiring으로 주변 node의 parent 재설정
11. goal 근처 도달 시 path 생성
12. 생성된 path를 Pure Pursuit 방식으로 추종
```

RRT* 적용 요소:

- `sample()`: 랜덤 샘플링
- `nearest()`: 가장 가까운 노드 탐색
- `steer()`: 일정 거리만큼 새 노드 생성
- `check_collision()`: Occupancy Grid 기반 충돌 검사
- `near()`: 주변 노드 탐색
- `cost()`: 시작 노드부터 현재 노드까지 누적 비용 계산
- `line_cost()`: 두 노드 사이 거리 비용 계산
- Best Parent Selection: 비용이 가장 낮은 부모 노드 선택
- Rewiring: 주변 노드의 경로 비용이 줄어들 경우 부모 노드 재연결

---

### 6.6 Occupancy Grid 기반 충돌 검사

맵 이미지를 Occupancy Grid 형태로 변환하여 주행 가능 영역과 장애물 영역을 구분했습니다.

초기에는 장애물 cell만 충돌 영역으로 판단했기 때문에 차량 중심점은 지나갈 수 있어도 실제 차량 body가 장애물에 닿는 문제가 있었습니다.

이를 해결하기 위해:

- 장애물 주변 영역 확장
- 차량 크기를 고려한 안전 margin 적용
- node와 edge 모두 충돌 검사
- 두 노드 사이 중간 지점 sampling 검사
- 실시간 LiDAR 장애물에 Safety Bubble 적용

을 적용했습니다.

---

## 7. 주요 토픽

| 토픽 | 메시지 타입 | 역할 |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR 거리 데이터 수신 |
| `/ego_racecar/odom` | `nav_msgs/Odometry` | 차량 위치, 자세, 속도 수신 |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | 차량 조향각 및 속도 명령 발행 |
| `/map` | `nav_msgs/OccupancyGrid` | 맵 정보 |
| `/rrt/map` | `nav_msgs/OccupancyGrid` | RRT용 Occupancy Grid |
| `/rrt/tree_markers` | `visualization_msgs/MarkerArray` | RRT tree 시각화 |
| `/waypoints_marker` | `visualization_msgs/Marker` | Waypoint 시각화 |

---

## 8. 실행 방법

### 8.1 의존성 설치

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ackermann-msgs \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-teleop-twist-keyboard

python3 -m pip install numpy pillow pyyaml scipy transforms3d
```

### 8.2 빌드

```bash
cd ~/sim_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 8.3 시뮬레이터 실행

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### 8.4 개별 알고리즘 실행

```bash
ros2 run safety_node safety_node
ros2 run wall_follow wall_follow
ros2 run follow_gap follow_gap
ros2 run pure_pursuit pure_pursuit
ros2 run rrt_node rrt_node
```

### 8.5 최종 통합 노드 실행

```bash
ros2 run project_f1_tenth project_f1_tenth
```

### 8.6 Waypoint 생성 및 시각화

```bash
ros2 run project_f1_tenth make_waypoint
ros2 run project_f1_tenth visualizer
```

---

## 9. 실행 전 경로 확인

현재 코드에는 일부 개인 PC 기준 절대경로가 포함되어 있습니다.

예시:

```text
/home/yoon/sim_ws/log/waypoint.csv
/home/yoon/sim_ws/log/wp-2025-05-14-09-31-38.csv
~/sim_ws/src/f1tenth_gym_ros/maps/levine.yaml
```

다른 PC에서 실행할 경우 위 경로를 본인 workspace 경로에 맞게 수정해야 합니다.

권장 개선 방향:

- waypoint 파일을 `data/waypoint.csv`처럼 별도 폴더로 이동
- 코드에서 절대경로 대신 ROS2 parameter 사용
- `get_package_share_directory()` 기반으로 map/waypoint 경로 관리

---

## 10. Nav2 및 강화학습 사용 여부

본 프로젝트는 Nav2 전체 주행 스택을 사용한 프로젝트가 아닙니다.

`f1tenth_gym_ros` 실행부에서 `nav2_map_server`는 map을 publish하기 위한 용도로 사용될 수 있지만, 실제 주행 판단, 경로 계획, 경로 추종, 제어는 직접 구현한 Python ROS2 노드에서 수행했습니다.

또한 강화학습 기반 프로젝트가 아닙니다.

```text
Nav2 기반 자율주행 X
강화학습 기반 policy X
직접 구현한 고전 자율주행 알고리즘 O
RRT* 기반 경로 계획 O
Pure Pursuit 기반 경로 추종 O
```

---

## 11. 성과

- LiDAR 기반 자동 긴급 정지 기능 구현
- PID 기반 벽 추종 주행 구현
- Follow-the-Gap 기반 장애물 회피 구현
- Safety Bubble 기반 위험 영역 제거
- Waypoint 기반 경로 추종 구현
- Pure Pursuit 기반 조향각 계산
- Occupancy Grid 기반 충돌 검사 구현
- RRT* 기반 경로 계획 구현
- RViz 기반 tree/path/waypoint 시각화 구현
- ROS2 topic 기반 인지-계획-제어 파이프라인 직접 구성

---

## 12. 프로젝트 요약

ROS2 기반 F1TENTH Gym 환경에서 LiDAR/Odometry 데이터를 직접 처리하여 자율주행 차량의 인지-판단-계획-제어 파이프라인을 구현했습니다. TTC 기반 긴급 정지, PID 기반 Wall Following, Follow-the-Gap 장애물 회피, Safety Bubble 기반 위험 영역 제거, Pure Pursuit 경로 추종, Occupancy Grid 충돌 검사, RRT* 경로 계획을 직접 구현했으며, AckermannDriveStamped 메시지를 통해 차량의 조향각과 속도를 제어했습니다.
