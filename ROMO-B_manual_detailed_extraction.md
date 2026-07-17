# ROMO-B 플랫폼 매뉴얼 상세 판독 정리본

원본: `Scan_20260717_154609.pdf`, 38쪽 스캔 문서

이 문서는 원본 스캔 페이지를 직접 판독하여, 자율주행 시스템 구축에 필요한 하드웨어 구조, 제어 구조, 상태 모드, 기구학, RS232 통신 프로토콜을 중심으로 재구성한 것입니다. 원문에서 수치 또는 설명이 서로 맞지 않는 부분은 별도로 표시했습니다.

---

## 1. 플랫폼 개요

ROMO-B는 실내와 실외 환경에서 사용할 수 있는 다목적 모빌리티 로봇 플랫폼입니다. 사용자가 목적에 맞게 LiDAR, GPS, 카메라, 초음파 센서, 상부 모듈 등을 장착할 수 있도록 설계되어 있습니다.

구동 방식은 `4WID-4WIS`입니다.

- 4WID: Four-wheel Independent Drive, 4륜 독립 구동
- 4WIS: Four-wheel Independent Steering, 4륜 독립 조향
- 각 바퀴에 인휠 BLDC 모터가 장착되어 4개 바퀴를 독립 구동
- 각 바퀴에 조향용 서보 모터가 장착되어 4개 바퀴를 독립 조향
- Ackermann 조향 적용 가능
- 전륜과 후륜을 각각 제어하여 최소 회전 반경 주행 가능
- 제자리 회전, Pivot 가능
- 매뉴얼 개요에는 Crab 주행 가능성이 언급되어 있으나, 공개된 상위제어 통신 프로토콜의 조향 모드는 2WIS, 4WIS, Pivot의 3개입니다. 별도 Crab 모드 번호는 없습니다.

매뉴얼에 제시된 활용 예시는 다음과 같습니다.

- Base Platform
- Digital Signage Robot Platform
- Delivery and Logistics Robot Platform
- Robot Armed Robot Platform

---

## 2. 핵심 사양

### 2.1 플랫폼 본체

| 항목 | 매뉴얼 기재값 |
|---|---:|
| 구동 방식 | 4륜 독립 구동, 4륜 독립 조향, 4WID-4WIS |
| 외형 크기 | 길이 756 mm, 너비 597 mm, 높이 342 mm |
| 지상고 | 62 mm |
| 축거 | 323 mm |
| 매뉴얼 표의 윤거 | 529 mm |
| 제어 기구학에 사용하는 조향 너클 중심 간 거리 | 390 mm |
| 플랫폼 무게 | 55 kg |
| 허용 Payload | 100 kg |
| 최대 주행 속도 | 5.4 km/h, 1.5 m/s |
| 연속 구동 시간 | 최대 약 3시간, 환경에 따라 변동 |
| 현가장치 | Dual A-arm, Shock absorber 없음 |
| 메인 프레임 | 알루미늄 |
| 플랫폼 상판 | 알루미늄 |
| 외장 커버 | PLA |

### 2.2 치수 해석상 주의점

매뉴얼은 `윤거`라는 표현을 두 가지 값에 사용합니다.

- 사양표와 외형도: 529 mm
- 제어 알고리즘: 390 mm

26쪽 설명에서는 529 mm를 실제 외형상 윤거로 언급하면서도, Ackermann 계산에는 조향장치 회전 중심 사이 거리인 390 mm를 사용한다고 명시합니다.

따라서 제어 모델에는 다음 값을 사용해야 합니다.

```text
L = 0.323 m   # 전후 조향축 사이 거리, 축거
W = 0.390 m   # 좌우 조향 너클 회전 중심 사이 거리
```

529 mm는 바퀴 또는 차체 외곽 기준 치수에 가깝고, Ackermann 모델의 트랙 폭으로 사용하면 안 됩니다.

---

## 3. 구동 모터와 조향 모터

### 3.1 인휠 BLDC 구동 모터, 바퀴 1개 기준

| 항목 | 매뉴얼 기재값 |
|---|---:|
| 형식 | In-wheel type Brushless DC Motor |
| 정격 출력 | 100~350 W |
| 극수 | 30 poles |
| 정격 전압 | 24~48 V |
| 정격 전류 | 8 A |
| 정격 토크 | 8 N·m, 78 kg·cm |
| 최대 토크 | 16 N·m, 157 kg·cm |
| 휠 크기 | 직경 약 206 mm |
| 엔코더 해상도 | 4096 PPR |
| 모터 무게 | 4 kg |
| 정격/최대 회전속도 | 200 rpm / 300 rpm |
| 200 rpm 기준 선속도 | 7.2 km/h, 2 m/s |

플랫폼 차원의 최대 명령 속도와 피드백 범위는 ±1.5 m/s입니다.

### 3.2 조향용 서보 모터, 바퀴 1개 기준

| 항목 | 매뉴얼 기재값 |
|---|---:|
| 입력 전압 | 24 V |
| 무부하 동작 속도 | 0.37 s / 60° at 24 VDC |
| 정격 토크 | 140 kg·cm |
| 최대 토크 | 400 kg·cm |
| 무부하 전류 | 0.1 A |
| Stall 전류 | 14 A |
| 입력 신호 | PWM, 800~2200 μs, 50~400 Hz |
| 모터 무게 | 765 g |
| 개별 조향 모터 범위 | -30°~+30° |

조향 부호 규칙은 다음과 같습니다.

- 플랫폼 사시 기준 안쪽 방향: In
- 플랫폼 사시 기준 바깥쪽 방향: Out
- 좌회전: 음수
- 우회전: 양수

---

## 4. 배터리와 충전기

### 4.1 배터리

| 항목 | 매뉴얼 기재값 |
|---|---:|
| 종류 | LiFePO4, 리튬 인산철 |
| 용량 | 614 Wh, 24 V / 24 Ah 표기 |
| 출력 전압 | 25.6 V nominal, 최대 27.2 V, 최소 23 V |
| 무게 | 5.3 kg |
| 작동 온도 | -20~45°C |

매뉴얼은 배터리 상태를 다음과 같이 안내합니다.

- 완충 전압: 약 27.2 V
- 충전 필요 기준: 23 V
- 23 V 이하가 표시되면 플랫폼 전원을 끄고 충전

### 4.2 충전기

| 항목 | 매뉴얼 기재값 |
|---|---:|
| 출력 전압 | 29.2 V |
| 최대 출력 전류 | 14 A |
| 완전 방전 후 완충 시간 | 약 90분 |
| 플랫폼 충전 커넥터 | XT60 |

충전기 LED:

| LED | 충전 중 | 완충 |
|---|---|---|
| LED1, 충전기 전원 | 빨간색 | 빨간색 |
| LED2, 배터리 충전 상태 | 빨간색 | 초록색 |

충전 절차:

1. 플랫폼 전원을 OFF
2. 충전 포트 커버 개방
3. XT60 방향과 극성을 확인하여 연결
4. 충전기 AC 전원 연결
5. 초기 전류는 약 14 A에서 시작하며 충전 완료 시 0 A로 감소
6. 완료 후 플랫폼 측 충전 커넥터를 먼저 분리
7. 이후 충전기 AC 전원 코드를 분리

충전 포트에는 전압이 존재하므로 물이나 습기가 닿지 않도록 관리해야 합니다.

---

## 5. 플랫폼 컨트롤러와 원격 조종기

### 5.1 Platform Control Unit, PCU

매뉴얼 기재 구성:

- 목적: 상위제어기 통신, 플랫폼 제어
- 메인 컨트롤러: National Instruments myRIO-1900
- 인터페이스 확장 보드
- 인터페이스: PWM, RS232, DIO

### 5.2 원격 조종기

송신기:

- Spectrum DXs, SPMR1010
- 7채널
- 2.4 GHz DSMX
- 운용 거리 600 m, 가시거리 내 사용 권장
- AA 알카라인 배터리 4개

수신기:

- Spectrum AR620

제품 수급 상황에 따라 원격 조종기 모델은 변경될 수 있다고 기재되어 있습니다.

---

## 6. 기계 및 외부 하드웨어 배치

플랫폼 섀시는 모듈 형식으로 전방, 중앙, 후방 영역으로 나뉘어 주요 부품이 배치됩니다.

### 6.1 상부

- 알루미늄 플랫폼 상판
- 다수의 체결 홀을 이용해 센서와 상부 모듈 장착 가능
- 배터리 전원 출력 포트
- 플랫폼 통신 포트

### 6.2 하부

- 4개 조향 모듈과 인휠 모터
- 조향 링크용 Tie rod
- 중앙 하판

### 6.3 좌측면

- 플랫폼 점검 패널

### 6.4 우측면

- 플랫폼 컨트롤 패널
- Power ON/OFF
- E-Stop
- LCD
- Voltmeter
- Charging port

### 6.5 전면

전면 커버를 제거하여 다음 옵션을 장착할 수 있습니다.

- 전방 측정용 LiDAR
- 초음파 센서 마운트

매뉴얼 전면 LiDAR 장착부는 전방 측정형 센서를 전제로 보입니다. Livox Mid-360처럼 360° 시야가 필요한 센서는 상판 중앙 또는 상판의 시야 차폐가 적은 위치에 장착하는 것이 더 적절합니다. 이 문장은 매뉴얼 기재가 아니라 시스템 구성상 권장 해석입니다.

---

## 7. 전체 제어 및 통신 구조도

매뉴얼 8쪽의 구조를 텍스트로 재구성하면 다음과 같습니다.

```text
 LiDAR(option) ─┐
 GPS(option) ───┤
 Camera(option) ├──> High-Level Controller, HLV ── RS232 ch4 ──> Platform Control Unit, PCU
 Ultrasonic ────┘                                           │
                                                            ├── RS232 ch1 ──> BLDC Motor Driver #1
                                                            │                  ├── In-wheel Motor #1, FR
                                                            │                  └── In-wheel Motor #4, RR
                                                            │
                                                            ├── RS232 ch2 ──> BLDC Motor Driver #2
                                                            │                  ├── In-wheel Motor #2, FL
                                                            │                  └── In-wheel Motor #3, RL
                                                            │
                                                            ├── RS232 ch3 ──> LCD Panel
                                                            ├── DIO <──────── E-Stop Switch
                                                            │
                                                            ├── PWM #1 ─────> Steer Servo #1, FL
                                                            ├── PWM #2 ─────> Steer Servo #2, FR
                                                            ├── PWM #3 ─────> Steer Servo #3, RL
                                                            ├── PWM #4 ─────> Steer Servo #4, RR
                                                            │
                                                            └── PWM #5~10 <── RC Receiver
                                                                               ))) RF ((( Remote Transmitter
```

용어:

- HLV, High-Level Controller: 사용자가 구현한 자율주행 알고리즘을 탑재한 상위 컴퓨터
- PCU, Platform Control Unit: 플랫폼 구동과 조향을 수행하는 임베디드 제어기

따라서 Ubuntu 22.04 ROS 2 PC는 HLV 역할을 하며, USB to RS232로 PCU와 연결하면 됩니다.

---

## 8. 컨트롤 패널

우측 컨트롤 패널에는 다음 요소가 있습니다.

1. Power ON/OFF Switch
2. E-Stop Switch
3. LCD Panel
4. Voltmeter
5. Charging Port

### 8.1 Power ON/OFF

- 플랫폼 전체 전원 계통에 전압을 인가
- 반드시 원격 조종기 전원을 먼저 켠 뒤 플랫폼 메인 전원을 켜도록 안내

### 8.2 E-Stop

- 플랫폼 긴급 정지 스위치
- E-Stop 시 모터 제어 출력을 정지하고 모터를 전기적 제동 상태로 운용
- 플랫폼 전원이 켜져 있을 때 Full Braking이 동작
- 플랫폼 전원이 꺼져 있으면 E-Stop을 눌러도 모터는 Free 상태로 움직일 수 있음

이 항목은 운반과 경사로 안전에 매우 중요합니다. 전원을 끄는 것은 기계식 주차 브레이크가 아닙니다.

### 8.3 LCD

LCD는 16x2이며 다음 네 영역을 표시합니다.

```text
[상위제어기 통신 상태] [E-Stop 상태]
[Manual/Auto 상태]     [조향 모드]
```

통신 상태:

- `HLV CON`: 상위제어기 통신 정상
- `DIS-CON`: 상위제어기 통신 단락 또는 미연결

주행 상태:

- `MANUAL`: 원격 조종기 수동 모드
- `AUTO`: 상위제어기 자동 모드
- `AUTO FAIL`: Auto 진입 조건 불만족 또는 통신 실패

E-Stop 표시:

- `Re_E-ST`: 원격 조종기 E-Stop On
- `Bu_E-ST`: 본체 스위치 패널 E-Stop On
- `Hi_E-ST`: 상위제어기 E-Stop 명령 On
- `E-STOP`: 세 종류 E-Stop 중 둘 이상이 동시에 On

조향 모드:

- `2WIS`: 전륜 조향
- `4WIS`: 4륜 조향
- `PIV`: 제자리 회전

### 8.4 Voltmeter

- 배터리 출력 전압 표시
- 완충 약 27.2 V
- 23 V 이하에서 충전 필요

---

## 9. 전원 켜기와 기본 구동 순서

1. 원격 조종기 전원을 먼저 ON
2. 플랫폼 컨트롤 패널의 메인 전원을 ON
3. 전압이 23 V 이상인지 확인
4. 약 20초간 부팅 대기
5. `GO! ROMO-B ^^` 메시지가 약 2초 표시
6. LCD에서 Manual/Auto와 조향 모드 확인
7. 처음 움직일 때 스로틀 명령을 25% 이내로 제한
8. 주행 종료 후 안전한 장소에서 플랫폼 전원을 OFF
9. 원격 조종기 전원을 OFF

전원을 켤 때 LCD에는 약 20초 동안 커서만 점멸할 수 있습니다.

---

## 10. 원격 조종기 조작

### 10.1 전원

- Power 버튼을 약 5초 누르면 ON/OFF
- 송신기 중앙 LED로 상태 확인

### 10.2 스위치 위치

Auto/Manual 3단 스위치:

- 0 또는 1: Manual
- 2: Auto

Steer mode 3단 스위치:

- 0: 2WIS
- 1: 4WIS
- 2: Pivot

E-Stop 2단 스위치:

- 0: Off
- 1: On

### 10.3 스틱 구성

조종기 Mode 2 기준:

- 좌측 스틱: Throttle
- 우측 스틱: Aileron, 조향

일반 주행:

- Throttle 위: 전진 가속 증가
- Throttle 중립: 가감속 0, 중립 복귀
- Throttle 아래: 후진 가속 증가
- Aileron 왼쪽: 좌조향 증가
- Aileron 오른쪽: 우조향 증가

Pivot:

- Throttle 위: CW 회전 증가
- Throttle 아래: CCW 회전 증가
- Aileron: 사용하지 않음

중립 주변에는 Dead band가 있습니다.

### 10.4 Trim

- Trim_1: Throttle 기본값 조정
- Trim_2: Aileron 기본값 조정

명령을 주지 않았는데 바퀴가 움직이거나 조향이 틀어지면 Trim을 확인해야 합니다.

---

## 11. 플랫폼 동작 모드

플랫폼은 다음 5개 상태를 가집니다.

1. Booting
2. Manual
3. Auto
4. Auto Fail
5. E-Stop

### 11.1 Booting Mode

| 항목 | 상태 |
|---|---|
| Speed | 0 m/s |
| Steer | 0° |
| Steer mode | 2WIS |
| E-Stop | Off |

부팅 중에는 원격 조종기 명령을 수행하지 않습니다.

### 11.2 Manual Mode

- 원격 조종기로 제어
- Steer mode 스위치로 2WIS, 4WIS, Pivot 선택
- 속도와 조향은 스틱 움직임에 비례
- 정지 상태에서 처음 출발할 때 스로틀 25% 이내 권장

### 11.3 Auto Mode

Auto Mode는 HLV와 PCU가 RS232 통신으로 플랫폼을 제어하는 자율주행 모드입니다.

진입 조건:

1. 원격 조종기의 Auto/Manual 스위치가 Auto 위치
2. HLV와 PCU 통신 정상
3. HLV→PCU 패킷의 `AorM` 값이 1

연결:

```text
Ubuntu HLV USB 포트
    └── USB to RS232 변환기
          └── 플랫폼 통신 포트
```

Auto Mode에서는 통신 패킷의 STEER_MODE로 2WIS, 4WIS, Pivot을 선택할 수 있습니다. 원격 조종기에 표시된 조향 모드와 Auto Mode의 실제 조향 모드가 다를 수 있습니다.

원격 조종기 E-Stop과 본체 패널 E-Stop은 Auto Mode에서도 동작합니다.

### 11.4 Auto Fail Mode

의미:

- HLV와 PCU 사이 통신 또는 Auto 진입 조건이 실패하여 자동 제어가 불가능한 상태

매뉴얼의 Auto Fail 조건:

1. 원격 조종기 Auto/Manual 스위치가 Auto가 아님
2. HLV→PCU 패킷이 프로토콜 형식에 맞지 않음
3. 상위제어기 통신 단절
4. HLV Alive Count가 0.3초 동안 갱신되지 않음
5. 패킷 `AorM` 값이 1이 아님

Auto Fail 표의 상태:

| 항목 | 상태 |
|---|---|
| Speed | 0 m/s |
| Steer | 0° |
| Steer mode | 원격 조종기 명령값 |
| E-Stop | Off |

중요한 원문 불일치:

- 20쪽 Auto Fail 표는 E-Stop을 Off로 표시합니다.
- 21쪽 E-Stop 실행 조건에는 `Auto Fail Mode가 실행될 때`가 포함되어 있습니다.

따라서 실제 펌웨어가 Auto Fail에서 단순 속도 0 명령만 적용하는지, E-Stop Full Braking까지 적용하는지는 벤치 시험으로 확인해야 합니다.

### 11.5 E-Stop Mode

- 기존 모든 명령을 무시
- Full Braking 수행

상태:

| 항목 | 상태 |
|---|---|
| Speed | 0 m/s |
| Steer | 0° |
| Steer mode | 원격 조종기 명령값 |
| E-Stop | On |

진입 조건:

1. 원격 조종기 E-Stop On
2. 본체 패널 E-Stop 버튼 누름
3. Auto Fail 발생
4. HLV→PCU 패킷의 ESTOP 값이 0에서 1로 변경

1, 2는 Manual과 Auto에서 모두 동작하며, 3, 4는 Auto Mode에서 동작한다고 설명합니다.

---

## 12. Ackermann 기구학 전제

매뉴얼의 제어 모델 파라미터:

```text
L = 0.323 m   # 축거
W = 0.390 m   # 좌우 조향 너클 회전 중심 간 거리
R            # 차량 중심 또는 무게중심에서 선회 중심까지 거리
δ            # 등가 차량 조향각
```

기본 관계:

```text
tan(δ) = L / R
R = L / tan(δ)
```

매뉴얼은 차량 중심 등가 조향각에서 각 휠의 조향각과 선속도를 계산합니다.

---

## 13. 2WIS, 전륜 조향 알고리즘

### 13.1 조향각

```text
R = L / tan(δ)

δ_FR = atan( L / (R - W/2) )
     = atan( L / (L/tan(δ) - W/2) )

δ_FL = atan( L / (R + W/2) )
     = atan( L / (L/tan(δ) + W/2) )

δ_RR = 0
δ_RL = 0
```

그림은 우회전 기준으로 FR이 내측, FL이 외측인 경우를 표현합니다. 반대 방향에서는 부호와 내외측 관계가 대칭으로 바뀝니다.

### 13.2 각 바퀴의 선속도

차량 중심 입력 속도를 `v`, 선회 각속도를 `ω`, `v = Rω`로 놓습니다.

```text
R_FR = sqrt( L² + (R - W/2)² )
R_FL = sqrt( L² + (R + W/2)² )

v_FR = v * R_FR / R
v_FL = v * R_FL / R
v_RR = v * (R - W/2) / R
v_RL = v * (R + W/2) / R
```

즉 내측 바퀴는 느리고 외측 바퀴는 빠르게 명령됩니다.

### 13.3 입력 범위와 예시

- 등가 조향 명령 범위: -22°~+22°
- 로봇 등가 조향각 18° 예시:
  - FL 약 15.2°
  - FR 약 22.0°
  - RL 0°
  - RR 0°

---

## 14. 4WIS, 4륜 조향 알고리즘

4WIS에서는 전륜과 후륜이 반대 방향으로 조향하여 선회 반경을 줄입니다.

### 14.1 조향각

차량 중심에서 전륜축과 후륜축까지 거리를 각각 `L/2`로 사용합니다.

```text
δ_F = atan( (L/2) / R )
δ_R = atan( (L/2) / R )
```

각 바퀴 조향각의 크기:

```text
δ_FR = δ_RR = atan( (L/2) / (R - W/2) )

δ_FL = δ_RL = atan( (L/2) / (R + W/2) )
```

매뉴얼 수식은 크기 중심으로 표현되어 있습니다. 실제 후륜 조향 방향은 전륜과 반대입니다.

### 14.2 각 바퀴의 선속도

```text
R_FR = R_RR = sqrt( (L/2)² + (R - W/2)² )
R_FL = R_RL = sqrt( (L/2)² + (R + W/2)² )

v_FR = v * R_FR / R
v_RR = v * R_RR / R
v_FL = v * R_FL / R
v_RL = v * R_RL / R
```

### 14.3 입력 범위와 예시

- 등가 조향 명령 범위: -18°~+18°
- 로봇 등가 조향각 18° 예시에서 매뉴얼이 제시한 바퀴별 크기:
  - FL 약 13.1°
  - FR 약 28.1°
  - RL 약 13.1°
  - RR 약 28.1°

후륜은 전륜과 반대 방향으로 조향합니다.

---

## 15. Pivot, 제자리 회전

이론상 차량 중심을 선회 중심으로 하면 바퀴 조향각은 ±45°가 적절하지만, 조향 모터 제한 때문에 ±30°로 설정되어 있습니다.

### 15.1 바퀴 조향각

| 바퀴 | 조향각 |
|---|---:|
| FL | +30° |
| FR | -30° |
| RL | -30° |
| RR | +30° |

### 15.2 바퀴 속도 방향

양수 속도 입력, CW:

| 바퀴 | 속도 범위 |
|---|---:|
| FL | 0~+1.5 m/s |
| FR | -1.5~0 m/s |
| RL | 0~+1.5 m/s |
| RR | -1.5~0 m/s |

음수 속도 입력, CCW:

| 바퀴 | 속도 범위 |
|---|---:|
| FL | -1.5~0 m/s |
| FR | 0~+1.5 m/s |
| RL | -1.5~0 m/s |
| RR | 0~+1.5 m/s |

Pivot에서는 별도의 조향 명령값보다 STEER_MODE와 SPEED 부호가 회전 방향을 결정합니다.

---

## 16. 사용상 주요 주의사항

1. 사고와 문제에 대한 책임은 사용자에게 있다고 기재
2. 경사와 마찰에 따라 속도가 일정하지 않을 수 있음
3. 경사로 밀림 방지 장치가 없음
4. 경사면에서 정지 제어 중이라도 전원 또는 제어가 해제되면 아래로 밀릴 수 있음
5. 전원 OFF 상태에서는 정지력이 유지되지 않으므로 운송 시 고임목 또는 강력 케이블로 고정
6. 방수 설계가 아니므로 비와 눈 환경에서 사용 금지
7. 젖은 노면과 급경사 도로에서 사용 금지
8. 사람을 안전장치 없이 탑승시키지 않음
9. 차량 통행이 많은 도로에서 사용하지 않음
10. 충분히 넓지 않은 공간에서는 원격 조종기 레버를 최대치까지 조작하지 않음
11. 배터리를 완전 방전시키지 않음
12. 원격 조종 중에는 항상 E-Stop을 누를 수 있도록 조종기를 소지
13. 내리막길에서 무리한 가속 금지
14. 원격 조종기를 먼저 켜고 플랫폼 전원을 켬
15. 종료 시 플랫폼 전원을 먼저 끄고 원격 조종기 전원을 끔
16. Auto Mode를 해제한 후 메인 전원을 끄도록 안내
17. 상위제어기 통신 시험은 바퀴를 지면에서 띄운 상태로 먼저 수행

---

## 17. 제조사 테스트 프로그램

매뉴얼은 Windows용 LabVIEW 기반 `ROMO-B HLV Test Program`을 설명합니다.

### 17.1 설치

- 설치 폴더 예시: `ROMO-B_4WIS_HLV_Install`
- 설치 실행: `install.exe`
- 설치 경로 예시: `C:\Program Files (x86)\ROMO-B_V1_4WIS`
- 실행 파일: `ROMO-B_HLV_TEST.exe`

### 17.2 프로그램 탭

1. Main
2. Platform Control Algorithm

Main 탭:

- USB COM 포트 선택
- HLV_AorM
- HLV_ESTOP
- HLV_STEERMODE
- HLV_SPEED
- HLV_STEER
- HLV_ALIVE
- HLV Serial Packet
- PCU_AorM
- PCU_ESTOP
- PCU_STEERMODE
- PCU_ALIVE
- FL/FR/RL/RR Speed Feedback
- FL/FR/RL/RR Steer Feedback
- PCU Serial Packet

Platform Control Algorithm 탭:

- 조향 모드 입력
- 차량 속도 입력
- 등가 조향각 입력
- L = 0.323 m
- W = 0.39 m
- 각 바퀴의 계산 조향각과 속도 확인

정상 연결 판단:

- Feedback 영역의 PCU_ALIVE가 계속 증가
- PCU_ALIVE가 증가하지 않으면 확인할 항목:
  1. COM 포트 번호
  2. 플랫폼 전원
  3. USB-RS232 케이블 연결

시험은 55 kg 플랫폼의 바퀴를 지면에서 띄운 상태로 수행하도록 안내합니다.

---

## 18. RS232 통신 사양

### 18.1 물리 및 프레임 사양

| 항목 | 값 |
|---|---|
| 인터페이스 | RS232 |
| Baud rate | 115200 bps |
| Parity | None |
| Stop bit | 1 |
| Data bits | 매뉴얼에 명시되지 않음 |
| 통신 주기 | 50 ms, 20 Hz |
| HLV→PCU Byte order | Big Endian |
| PCU→HLV Byte order | Little Endian |
| HLV→PCU 프레임 길이 | 13 bytes |
| PCU→HLV 프레임 길이 | 25 bytes |

일반적으로 8N1일 가능성이 높지만, Data bits는 원문에 없으므로 실제 장치 또는 제조사 프로그램으로 확인해야 합니다.

### 18.2 프레임 헤더와 종료 문자

매뉴얼은 필드명을 `S`, `T`, `X`, `ETX`로 표기하지만 일반적인 STX/ETX 제어문자와 다릅니다.

```text
Header: 0x53 0x54 0x58   # ASCII 'S' 'T' 'X'
Tail:   0x0D 0x0A        # CR LF
```

즉 첫 바이트가 `0x02`인 일반 STX 프레임으로 구현하면 안 됩니다.

---

## 19. HLV→PCU 명령 패킷, 13 bytes

### 19.1 바이트 배열

| Offset | 길이 | 필드 | 값/설명 |
|---:|---:|---|---|
| 0 | 1 | S | 0x53 |
| 1 | 1 | T | 0x54 |
| 2 | 1 | X | 0x58 |
| 3 | 1 | AorM | 0x00 Manual, 0x01 Auto |
| 4 | 1 | ESTOP | 0x00 Off, 0x01 On |
| 5 | 1 | STEER_MODE | 0x00 2WIS, 0x01 4WIS, 0x02 Pivot |
| 6 | 2 | SPEED | signed 16-bit, Big Endian, -150~150 |
| 8 | 2 | STEER | signed 16-bit, Big Endian, 표상 -300~300 |
| 10 | 1 | ALIVE | 0~255 |
| 11 | 1 | ETX[0] | 0x0D |
| 12 | 1 | ETX[1] | 0x0A |

### 19.2 SPEED 스케일

```text
raw_speed = command_speed_mps × 100
```

예:

```text
1.00 m/s  -> 100
0.50 m/s  -> 50
-0.20 m/s -> -20
```

음수는 후진입니다.

### 19.3 STEER 스케일

매뉴얼의 HLV→PCU 설명:

```text
Command Value 10 deg -> result value 10 deg
```

즉 문서상 명령 정수값 자체가 degree입니다.

```text
raw_steer = command_steer_deg
```

단, 패킷 표의 범위 `-300~300`과 실제 모드별 등가 조향 범위가 다릅니다.

- 2WIS 실제 명령 범위: -22°~+22°
- 4WIS 실제 명령 범위: -18°~+18°
- 개별 조향 피드백 범위: -30°~+30°

따라서 `-300~300`은 16비트 필드의 문서상 허용 raw 범위일 뿐, 실제 주행 명령에는 모드별 물리 제한을 적용해야 합니다.

### 19.4 ALIVE

- 0~255
- 매 50 ms 프레임마다 1씩 증가
- 255 다음 0으로 순환하는 것으로 해석
- 매뉴얼 Auto Fail 조건은 0.3초 동안 HLV Alive가 갱신되지 않는 경우
- 20 Hz 기준 약 6개 프레임 누락이 timeout에 해당

### 19.5 명령 프레임 예시

Auto, E-Stop Off, 2WIS, 0.50 m/s, +10°, Alive 1:

```text
53 54 58 01 00 00 00 32 00 0A 01 0D 0A
```

일반적인 signed int16 two's complement를 적용한 예:

Auto, E-Stop Off, 2WIS, -0.20 m/s, -5°, Alive 2:

```text
53 54 58 01 00 00 FF EC FF FB 02 0D 0A
```

두 번째 예의 signed 정수 표현 방식은 음수 범위가 제시된 점에 근거한 일반적인 구현 해석이며, 제조사 예제 바이트열은 매뉴얼에 없습니다.

---

## 20. PCU→HLV 피드백 패킷, 25 bytes

### 20.1 바이트 배열

| Offset | 길이 | 필드 | 값/설명 |
|---:|---:|---|---|
| 0 | 1 | S | 0x53 |
| 1 | 1 | T | 0x54 |
| 2 | 1 | X | 0x58 |
| 3 | 1 | AorM | 0x00 Manual, 0x01 Auto |
| 4 | 1 | ESTOP | 0x00 Off, 0x01 On |
| 5 | 1 | STEER_MODE | 0x00 2WIS, 0x01 4WIS, 0x02 Pivot |
| 6 | 2 | FL_SPEED | signed 16-bit Little Endian |
| 8 | 2 | FR_SPEED | signed 16-bit Little Endian |
| 10 | 2 | RL_SPEED | signed 16-bit Little Endian |
| 12 | 2 | RR_SPEED | signed 16-bit Little Endian |
| 14 | 2 | FL_STEER | signed 16-bit Little Endian |
| 16 | 2 | FR_STEER | signed 16-bit Little Endian |
| 18 | 2 | RL_STEER | signed 16-bit Little Endian |
| 20 | 2 | RR_STEER | signed 16-bit Little Endian |
| 22 | 1 | ALIVE | 0~255 |
| 23 | 1 | ETX[0] | 0x0D |
| 24 | 1 | ETX[1] | 0x0A |

### 20.2 속도 피드백

```text
speed_mps = raw_speed × 0.01
```

- 범위: -1.50~+1.50 m/s
- 음수: 후진
- 문서상 속도 오차: ±5%

### 20.3 조향 피드백

```text
steer_deg = raw_steer × 0.1
```

- 범위: -30.0~+30.0°
- 음수: 좌조향
- 문서상 조향 오차: ±1°

### 20.4 STEER_MODE 비정상값

문서에는 3~255와 같은 정의되지 않은 값을 입력하면 neutral로 설정한다고 기재되어 있습니다. 정상 구현에서는 0, 1, 2만 사용해야 합니다.

---

## 21. ROS 2와 Autoware 연결 시 직접 필요한 정보

매뉴얼로 확정할 수 있는 차량 인터페이스 입력과 출력은 다음과 같습니다.

### 21.1 Autoware/ROS 2에서 PCU로 보낼 값

```text
AorM
ESTOP
STEER_MODE
차량 중심 속도 v
등가 차량 조향각 δ
Alive counter
```

PCU 내부에서 매뉴얼의 Ackermann 알고리즘을 적용해 4개 바퀴의 개별 조향각과 속도를 생성하는 구조로 보입니다. 따라서 상위제어기가 일반 주행에서 각 바퀴 값을 직접 보내는 프로토콜은 아닙니다.

### 21.2 PCU에서 ROS 2로 받을 값

```text
현재 Manual/Auto 상태
현재 E-Stop 상태
현재 조향 모드
FL/FR/RL/RR 개별 바퀴 속도
FL/FR/RL/RR 개별 조향각
PCU Alive counter
```

이를 이용해 다음 ROS 2 정보를 생성할 수 있습니다.

- 차량 속도 피드백
- 실제 조향각 피드백
- 휠 오도메트리
- 제어 모드 상태
- E-Stop 진단
- 시리얼 통신 상태 진단

### 21.3 초기 Autoware 운용 모드 권장

매뉴얼의 2WIS 모드는 일반 Ackermann 차량 모델과 가장 가깝습니다.

```text
STEER_MODE = 0, 2WIS
최초 속도 제한 = 0.2~0.3 m/s
최초 조향 제한 = 약 ±5°
명령 송신 = 20 Hz
통신 timeout = 0.2~0.3 s보다 짧게 자체 감시
```

4WIS와 Pivot은 Autoware의 일반적인 bicycle model과 직접 일치하지 않으므로 2WIS 검증 후 별도 모드 관리 계층으로 확장하는 것이 안전합니다.

---

## 22. 매뉴얼에서 반드시 실물 검증해야 하는 부분

1. RS232 Data bits가 문서에 없음, 8N1 추정이지만 확인 필요
2. HLV 조향 명령은 문서상 1 raw = 1°이나, 실제 PCU 구현 확인 필요
3. Auto Fail에서 E-Stop Off인지 Full Braking인지 원문이 상충
4. AorM/ESTOP/STEER_MODE 바이트가 잘못된 값일 때 정확한 fail-safe 동작
5. Alive counter timeout 시 실제 정지 방식과 제동 토크
6. USB-RS232 변환기의 전압 레벨, DB9 핀 배열, null modem 여부
7. PCU 피드백이 전원을 켜면 계속 송신되는지, Auto 상태에서만 송신되는지
8. 프레임 중간 바이트 유실 시 재동기화 방식
9. 별도 checksum/CRC가 없음, 실제 프로토콜도 동일한지 확인
10. 실제 제어 지연과 20 Hz 명령 주기의 jitter 허용 범위
11. 개별 바퀴 피드백이 명령값인지 센서 실측값인지 확인
12. 4WIS 후륜 조향 부호 convention 확인
13. 529 mm와 390 mm 중 URDF와 기구학에 각각 어떤 기준을 사용할지 실측 확인
