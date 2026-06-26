# 2026 AMR Project

Storagy AMR 시뮬레이션 환경에서 ROS 2 Humble, Gazebo, Cartographer, Nav2를 사용해
자율주행을 실습하고, Nav2 경로의 안전성·회전량·에너지 추정치를 점수로 비교하는 프로젝트입니다.

이 저장소는 기본 Storagy Docker 실습 환경 위에 다음 기능을 추가합니다.

- 장애물과 더 멀리 주행하도록 조정한 Nav2 DWB 안전 주행 파라미터
- `/plan`, `/cmd_vel`, `/local_costmap/costmap` 기반 실시간 주행 점수 모니터
- baseline/tuned 주행 CSV 결과 비교 도구
- 막힌 경로에서 재계획과 복구 순서를 개선하기 위한 Behavior Tree 예시

## 주요 구성

| 경로 | 설명 |
| --- | --- |
| `storagy-practice-ws-docker/` | ROS 2 Humble, Gazebo, Cartographer, Nav2가 포함된 Storagy Docker 실습 환경 |
| `config/nav2_params_dwb_safe.yaml` | 안전 거리, DWB critic, 속도/가속도 제한을 조정한 Nav2 파라미터 |
| `scripts/nav2_score_monitor.py` | Nav2 주행 중 점수와 세부 지표를 출력하고 CSV로 저장하는 ROS 2 노드 |
| `scripts/compare_nav2_score_runs.py` | baseline/tuned CSV를 비교해 점수 변화량을 출력하는 도구 |
| `scripts/install_score_tools_to_storagy.sh` | 점수 측정 도구와 튜닝 파라미터를 Storagy 패키지에 복사하는 설치 스크립트 |
| `behavior_trees/score_replanning_recovery.xml` | 재계획·clear costmap·backup·spin·wait 복구 흐름을 담은 Nav2 BT 예시 |
| `docs/` | 적용 순서, 튜닝 기준, 테스트 체크리스트 문서 |

## 요구 사항

- Docker Desktop 또는 Docker Engine + Docker Compose
- 브라우저: noVNC 접속용
- Windows 사용자는 WSL 또는 Git Bash 권장
- Docker 메모리 6GB 이상 권장

## 빠른 시작

저장소를 받은 뒤 루트에서 점수 기반 Nav2 파일을 Storagy 패키지 안으로 복사합니다.

```bash
git clone https://github.com/itsjinnykim/2026-amr-project.git
cd 2026-amr-project
bash scripts/install_score_tools_to_storagy.sh storagy-practice-ws-docker
```

Docker 실습 환경을 실행합니다.

```bash
cd storagy-practice-ws-docker
docker compose up -d
```

브라우저에서 다음 주소로 접속합니다.

```text
http://localhost:6080
```

noVNC 터미널에서 시뮬레이터를 실행합니다.

```bash
ros2 launch storagy sim.launch.py use_rviz:=false
```

다른 noVNC 터미널에서 튜닝된 Nav2 파라미터로 navigation을 실행합니다.

```bash
ros2 launch storagy navigation.launch.py \
  params_file:=/opt/storagy-practice-ws-docker/src/storagy/param/nav2_params_dwb_safe.yaml
```

RViz가 열리면 `2D Pose Estimate`로 초기 위치를 지정하고, `Nav2 Goal`로 목표 지점을 찍어
주행을 테스트합니다.

## 처음 실행할 때 지도 만들기

`navigation.launch.py`는 기본적으로 `map/warehouse.yaml`을 사용합니다. 지도 파일이 없다면 먼저
Cartographer로 지도를 만들어야 합니다.

시뮬레이터를 켠 상태에서 noVNC 터미널 1:

```bash
ros2 launch storagy mapping.launch.py
```

noVNC 터미널 2:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

로봇을 천천히 움직여 창고 공간을 스캔한 뒤 지도를 저장합니다.

```bash
ros2 run nav2_map_server map_saver_cli -f src/storagy/map/warehouse
```

## 점수 기반 비교 실험

baseline과 tuned 주행을 같은 시작 위치와 같은 목표 지점으로 각각 실행한 뒤 비교합니다.

baseline Nav2:

```bash
ros2 launch storagy navigation.launch.py
```

baseline 점수 모니터:

```bash
python3 /opt/storagy-practice-ws-docker/src/storagy/scripts/nav2_score_monitor.py \
  --ros-args -p csv_path:=/tmp/nav2_baseline.csv
```

tuned Nav2:

```bash
ros2 launch storagy navigation.launch.py \
  params_file:=/opt/storagy-practice-ws-docker/src/storagy/param/nav2_params_dwb_safe.yaml
```

tuned 점수 모니터:

```bash
python3 /opt/storagy-practice-ws-docker/src/storagy/scripts/nav2_score_monitor.py \
  --ros-args -p csv_path:=/tmp/nav2_tuned.csv
```

결과 비교:

```bash
python3 /opt/storagy-practice-ws-docker/src/storagy/scripts/compare_nav2_score_runs.py \
  --baseline /tmp/nav2_baseline.csv \
  --tuned /tmp/nav2_tuned.csv
```

점수는 낮을수록 좋습니다.

```text
score =
  distance_weight * path_length_m
  + time_weight * elapsed_time_s
  + rotation_weight * accumulated_abs_rotation_rad
  + safety_weight * safety_penalty
  + energy_weight * estimated_energy_cost
  + blocked_weight if blocked
```

비교 결과에는 최종 점수, 평균 점수, 경로 길이, 최소 장애물 거리, 회전량, 에너지 추정치,
속도/가속도 변화량, 정지/재시작 횟수, blocked 샘플 수, costmap 통계가 포함됩니다.

## 튜닝 방향

- 장애물에 너무 가까우면 `inflation_radius`와 `BaseObstacle.scale`을 올립니다.
- 좁지만 통과 가능한 통로를 못 지나가면 footprint를 먼저 확인하고 `inflation_radius`를 낮춥니다.
- 회전이나 oscillation이 많으면 `RotateToGoal.scale`, `Twirling.scale`, `max_vel_theta`를 조정합니다.
- 에너지 추정치가 높으면 `max_vel_x`, `acc_lim_x`, `decel_lim_x`를 낮추고 stop/restart 횟수를 확인합니다.

자세한 튜닝 기준은 `docs/dwb_tuning_note.md`와 `docs/storagy_score_nav2_runbook.md`를 참고하세요.

## 참고 문서

- `docs/storagy_score_nav2_runbook.md`: Docker/실제 로봇 적용 순서
- `docs/nav2_score_based_project_scope.md`: 구현된 기능과 향후 작업 범위
- `docs/dwb_tuning_note.md`: DWB critic과 costmap 기반 점수 튜닝 메모
- `docs/dwb_test_request.md`: baseline/tuned 테스트 체크리스트

## 라이선스

이 저장소는 MIT License를 따릅니다. 자세한 내용은 `LICENSE`를 확인하세요.
