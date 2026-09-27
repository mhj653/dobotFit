# Robot Automation Studio

DOBOT MG400 중심의 Windows용 로봇 자동화 스튜디오 V1입니다. 첨부 ZIP 문서는 참고 요구사항으로 사용했으며, 실제 장비 명령/포트/피드백 구조는 공식 문서 없이 추측 구현하지 않았습니다.

## 실행

```powershell
cd D:\Samsung\dobot\RobotAutomationStudio
conda activate dobot-sim
python main.py
```

PyBullet backend까지 사용하려면 `environment.yml`로 만든 `dobot-sim` conda 환경에서 실행하세요. PyBullet 없이 UI/기본 로직만 확인할 새 pip 환경에서는 다음을 먼저 설치할 수 있습니다.

```powershell
python -m pip install -r requirements.txt
```

## 1~4단계 구현 상태

- 1단계 Simulation V1 완성도 향상
  - Dashboard: 시스템 상태, 장치 카드, 최근 로그
  - Robot: MG400 시뮬레이션 뷰, workspace view toolbar, PyBullet 표시 배율 선택, Move/Jog/Robot control 탭, 현재 Pose/Joint, target 입력, 현재 위치 읽기, MoveJ/MoveL, step Jog, position library 추가/편집/갱신/삭제
  - Position edit dialog: 이름, X/Y/Z/R, motion, speed, acceleration 편집
  - Sequence + Analysis: step 추가/복제/삭제, template 추가, enable/disable, condition, step별 speed/accel, 실행 전 validation, Device별 Command/Setting 선택 패널, Reset, STEP/AUTO, Previous/Current/Next 표시, PyBullet preview, step 실행 시간 분석
  - Project: `projects/TestProject` 아래 JSON 저장/로드, Project 화면 내부에서 Robot / Sequence + Analysis 하위 탭 운영
  - Settings: SIMULATION/REAL 전환, MG400 TCP/IP IP/port/timeout 설정, Light/Dark UI theme 전환
- 2단계 시뮬레이션 엔진 개선
  - `simulation.robot_model.KinematicMG400Model`로 시뮬레이션 엔진 분리
  - `simulation.pybullet_engine.PyBulletEngine`으로 PyBullet backend 구현
  - PyBullet이 import 가능하면 `DIRECT` physics world, floor plane, 간이 MG400 link bodies, gripper bodies를 생성하고 pose/joint/gripper 상태를 동기화
  - PyBullet camera render를 Robot canvas에 표시하고, 왼쪽 드래그로 X/Y 이동, 마우스 휠로 Z 이동 제어
  - MoveJ easing path, MoveL linear path 샘플 생성
  - workspace/reach/floor/software limit 검증
  - fallback Qt robot canvas에서 작업영역, 최근 path, gripper open/close 상태 시각화
  - `dobot-sim` conda 환경에서는 PyBullet backend로 동작하며, PyBullet을 import할 수 없는 환경에서는 kinematic engine으로 fallback
- 3단계 테스트/구조 안정화
  - ProjectManager, SequenceEngine, KinematicMG400Model, RealMG400 fake TCP server 테스트
  - REAL 모드 미연결 시 Simulation fallback 금지 테스트
  - PyBullet optional 동작 테스트
- 4단계 Real MG400 연동 준비
  - DOBOT MG400/M1Pro 4-axis TCP/IP SDK 기준 RealMG400 driver 구현
  - Dashboard `29999`, Move `30003`, Feedback `30004` 설정
  - `EnableRobot`, `DisableRobot`, `ClearError`, `GetPose`, `GetAngle`, `MovJ`, `MovL`, `MoveJog`, `Sync`, `DI`, `DO` 명령 경로 구현

## 기능별 상태

- I/O: DI/DO 8채널 시뮬레이션, 수동 토글, soft gripper mapping 표시
- Gripper: DOBOT Soft Gripper simulation Open/Close, DO01/DO02 및 DI01 반영
- Vision: Intel RealSense D405 RGB-D 연결/캡처/Depth preview/ROI 기반 3D locate, Detection Setup, camera-to-robot calibration matrix, detection 변수 생성
- Robot: 왼쪽 큰 Robot View, 오른쪽 Move/Jog/Robot 탭형 제어 패널, 하단 Position Library 구조로 재배치했습니다. 제어 패널은 스크롤 가능하므로 작은 창이나 고배율 DPI에서도 Jog/콤보박스가 잘리지 않습니다. Robot View toolbar의 `View`에서 PyBullet 표시 배율을 75/100/125/150% 또는 Fill로 선택할 수 있습니다.
- Sequence + Analysis: MoveJ/MoveL/Jog, Gripper Open/Close, SetDO, Wait Time, WaitDI, Vision Detect, PLC SetOutput STEP/AUTO 실행, Device에 따라 Command/Setting 후보 자동 변경, step별 enable/condition/speed/accel, sequence validation, Pick/Place/Vision template, PyBullet preview, step별 실행 시간 기록
- Project Workspace: Project 저장/로드와 Robot / Sequence + Analysis 작업을 하나의 Project 탭에서 운영
- Settings: robot/end effector/camera/PLC/system 설정 UI
- Appearance: Settings > System에서 Light/Dark theme을 전환할 수 있으며 선택값은 PC 앱 설정에 저장됩니다.

## Mock / Real 구분

- SimulationMG400: kinematic simulation engine을 통해 path, pose, joint, limit, gripper 상태를 갱신합니다.
- SimulationIO, SimulationSoftGripper, MockPLC: 실제 장비 없이 전체 흐름 테스트용입니다.
- RealSenseD405Camera: `pyrealsense2` 기반 RGB-D capture를 지원합니다. Vision 화면에서 `Connect D405`, `Capture`, `Test Locate`를 실행하면 color/depth frame과 `vision_x/y/z/r`, `vision_pixel_u/v`, `vision_depth_mm`, `vision_score` 변수를 생성합니다. Detection Method에서 Blob threshold, polarity, area, blur, morphology, circularity를 설정하고, Calibration에서 camera-to-robot 4x4 matrix를 적용할 수 있습니다.
- RealMG400: DOBOT MG400/M1Pro 4-axis TCP/IP SDK 기준으로 Dashboard `29999`, Move `30003`, Feedback `30004` 설정을 지원합니다. `EnableRobot`, `DisableRobot`, `ClearError`, `GetPose`, `GetAngle`, `MovJ`, `MovL`, step Jog, `Sync`, `DI`, `DO` 명령 경로를 구현했습니다. Robot 탭의 `Read Current`는 `GetPose`/`GetAngle`로 현재 위치를 읽어 target 입력에 자동 반영합니다. Real Robot은 실제 장비 제어 경로이며 PyBullet simulation backend와 분리되어 있습니다.
- DobotSoftGripper real path: 실제 gripper I/O mapping과 현장 배선 확인 전까지 `DRIVER NOT AVAILABLE`을 반환합니다.
- REAL 모드에서 연결 실패 또는 미연결 상태이면 Simulation 명령으로 자동 fallback하지 않습니다.

## 프로젝트 파일

저장 시 아래 파일이 생성됩니다.

- `project.json`
- `robot.json`에는 MG400 model/mode/IP/port/timeout 설정이 저장됩니다.
- `positions.json`
- `sequence.json`
- `io_mapping.json`
- `tools.json`
- `vision.json`
- `calibration.json`

## 테스트

pytest 설치 시:

```powershell
python -m pytest tests
```

pytest가 없는 환경에서도 기본 unittest로 확인할 수 있습니다.

```powershell
python -m unittest discover -s tests
```

현재 테스트는 PyBullet이 설치된 환경에서는 PyBullet scene 생성과 object count를 검증하고, 미설치 환경에서는 fallback을 검증합니다.

## PyBullet 설치 상태

이 PC에는 Miniconda 기반 `dobot-sim` 환경을 만들고 conda-forge의 PyBullet을 설치했습니다.

```powershell
conda env create -f environment.yml
conda activate dobot-sim
python main.py --self-check
```

정상 상태에서는 `RobotAutomationStudio self-check backend=PyBullet physics_objects=7`이 출력됩니다. 기본 Python 3.11 환경에서는 PyBullet Windows wheel을 받지 못할 수 있으므로, 실제 PyBullet backend와 배포 빌드는 `dobot-sim` conda 환경을 기준으로 합니다.

## RealSense D405

이 PC의 `dobot-sim` 환경에는 `pyrealsense2`를 설치했고, 연결된 Intel RealSense D405 장치를 확인했습니다.

```powershell
conda activate dobot-sim
python -m pip install pyrealsense2
python main.py
```

앱에서 `Vision > Connect D405`를 누른 뒤 `Capture RGB-D`와 `Detect + Locate`로 RGB/depth frame과 ROI 기반 3D locate를 확인할 수 있습니다. Sequence의 `Vision Detect`는 RealSense가 연결된 경우 RealSense detection 변수를 사용합니다.

## 배포

개발 PC에서 `dobot-sim` conda 환경이 준비되어 있으면 다음으로 PyBullet 포함 onedir 패키지를 생성합니다.

```powershell
build.bat
```

출력 목표는 `dist/RobotAutomationStudio/RobotAutomationStudio.exe`입니다. 다른 PC로 옮길 때는 exe 파일 하나만이 아니라 `dist/RobotAutomationStudio` 폴더 전체를 복사해야 합니다.

## 알려진 제한

- 실제 MG400 URDF/mesh 모델 파일은 아직 포함하지 않았습니다. PyBullet backend는 간이 link bodies로 동작하도록 구현되어 있습니다.
- Real MG400은 TCP/IP 명령 경로까지 구현되어 있으나 실제 장비 검증은 필요합니다. DobotStudio Pro에서 TCP/IP secondary development mode를 켠 뒤 LAN1 기본 `192.168.1.6` 또는 현장 IP로 연결해야 합니다.
- Basler, Mitsubishi PLC 드라이버는 공식 SDK/API 문서와 장비 연결 정보가 필요합니다.
- RealSense D405 detect는 OpenCV Blob center와 depth locate를 사용합니다. 새 검출 알고리즘은 Detection Method 레지스트리와 camera detector map에 추가하는 구조로 확장할 수 있습니다.
- RealSense D405 detect 결과는 calibration matrix를 적용하기 전에는 테스트 좌표입니다. 실제 Robot Move에 사용하려면 camera-to-robot calibration matrix와 안전 검증을 먼저 확정해야 합니다.
- 장시간 sequence 실행은 V1에서 UI thread 기반 timer로 실행됩니다. 실제 장비 driver 연결 시 QThread/worker로 분리해야 합니다.
