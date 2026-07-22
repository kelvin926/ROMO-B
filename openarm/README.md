# OpenArm-v1 양팔 CAN 제어 소스

이 폴더는 `OpenArm_Portable_Full_20260721.zip`에서 OpenArm-v1 양팔을
SocketCAN으로 제어하는 데 필요한 부분만 선별해 가져온 독립 영역이다.
ROMO-B 구동부 코드와 섞지 않았으며, 파일을 가져오는 과정에서 CAN 프레임이나
모터 명령은 전송하지 않았다.

## 포함 범위

- `src/openarm_can`: Enactic OpenArm CAN 라이브러리 v1.2.9 원본
  - C++ SocketCAN 및 Damiao motor API
  - Python binding 소스와 예제
  - CAN 검색, 상태 확인, ID/파라미터 설정용 CLI 소스
  - 원본 Apache-2.0 라이선스와 빌드 파일
- `config/openarm_v1_bimanual.yaml`: 이 장비의 좌·우 CAN 인터페이스,
  모터 종류 및 송수신 ID를 한곳에 정리한 참조 설정
- `config/joint_limits.yaml`: OpenArm-v1 관절 제한 원본
- `config/control_gains.yaml`: OpenArm-v1 제어 gain 원본

## 제외 범위

다음 항목은 이번 범위와 무관해 가져오지 않았다.

- ExoArm-7, QnBot SDK/HMI 및 ExoArm-to-OpenArm bridge
- RealSense, Livox 및 기타 비전 코드와 캘리브레이션
- ACT, DP3, LeRobot, 모방학습 코드
- 학습 데이터, 영상, 로그 및 모델 체크포인트
- 과거 ZIP 백업과 GitHub 스냅샷
- URDF/mesh 전체 묶음

## 하드웨어 스냅샷

- 통신: Classic CAN, 1 Mbit/s, CAN-FD 비활성
- 왼팔: 기록 당시 `can1`
- 오른팔: 기록 당시 `can0`
- 각 팔: 7관절과 gripper, 송신 ID 1~8, 수신 ID 17~24
- J1/J2: DM8009, J3/J4: DM4340, J5/J6/J7/gripper: DM4310

USB-CAN 연결 순서에 따라 `can0`과 `can1`이 바뀔 수 있으므로 매 세션 실제
배정을 확인해야 한다. 위 내용은 `config/openarm_v1_bimanual.yaml`에 기록돼
있다. ROMO-B 웹 콘솔의 내장 SocketCAN 백엔드는 이 설정과 관절 제한을
읽으며, 원본 `openarm_can` 라이브러리는 YAML을 자동으로 읽지 않는다.

## 웹 양팔 제어

통합 콘솔 `http://127.0.0.1:8765/`의 **OpenArm 양팔** 탭에서 별도 명령어
없이 다음 작업을 수행할 수 있다.

- 왼팔/오른팔 CAN 인터페이스 지정, 연결, 상태 갱신 및 연결 해제
- 각 팔 8개 모터의 online, 현재각, 목표각, 속도, 토크, 온도, fault 및 RX/TX 확인
- 피드백이 모두 정상일 때 현재 자세 유지로 모터 enable, 개별/양팔 disable
- J1~J7과 gripper의 관절 제한 안에서 숫자·슬라이더·1도 단위 목표 조작
- 팔별 8축 또는 양팔 16축 동시 보간, 속도와 MIT gain 배율 설정
- 현재 피드백 불러오기, 좌우 자세 복사, 브라우저 로컬 자세 저장/불러오기
- 모터 disable 및 `OPENARM ZERO` 확인 문구를 요구하는 영점 저장

웹 서버 시작이나 CAN 연결만으로 enable 또는 자세 명령을 보내지 않는다.
CAN 연결 시에는 ID 1~8 상태 갱신 요청만 보내며, **모터 활성화**는 사용자가
별도로 눌러야 한다. 연결 해제와 서버 종료는 가능한 모든 팔에 disable 명령을
먼저 보낸다.

## 소프트웨어 빌드

Ubuntu 22.04에서 CLI까지 빌드하려면 CMake 3.22 이상, C++17 및 CLI11이
필요하다. 이 프로젝트의 다른 ROS 패키지와 분리해 다음처럼 빌드할 수 있다.

```bash
sudo apt install libcli11-dev
cd /home/hyunseo/ROMO-B/openarm
colcon build --base-paths src --symlink-install
```

빌드 결과인 `build`, `install`, `log`는 저장소 공통 `.gitignore`에 의해
제외된다.

## 주의

`openarm_can`의 demo, Python example, calibration 및 일부 CLI 명령은 실제로
모터를 enable하거나 움직이고 영점/ID를 변경할 수 있다. 소스가 존재한다는 이유로
자동 실행되는 것은 없지만, 하드웨어 연결 상태에서 예제나 calibration 도구를
임의로 실행하면 안 된다. 이후 ROMO-B 웹 콘솔에 연결할 때도 상태 읽기와 명시적
enable/command 경로를 분리해야 한다.

## 출처

- 원본 아카이브: `OpenArm_Portable_Full_20260721.zip`
- 아카이브 생성일: 2026-07-21
- 원본 호스트: Ubuntu 22.04.5 x86-64
- `openarm_can` package version: 1.2.9
- 선별 추출 시 ZIP 내부 136개 대상 항목 CRC 검사 통과
