# 🚗 RC Car 자율주행 공보 프로젝트

## ✨ 1. 프로젝트 개요

### • 목적
- 자재와 시간 비용을 절감하고, 소형 환경에서 자율주행 알고리즘과 통신 구조를 학습하기 위한 RC Car 기반 자율주행 시스템 구현

### • 사용 송수신기
- **RadioLink AT-9 Tx + R9DS Rx**
- 채널 구성: CH2(전후진), CH4(좌우 조향), CH5(모드 스위치)
- **Mode: Acrobatic** 모드에서 사용 (수동 PWM 제어에 적합)

### • 핵심 기술 개요
- 송수신기에서 수신한 PWM을 Arduino가 해석하여 수동 모드 동작 제어
- 자율주행 모드에서는 Raspberry Pi가 라인을 추적하여 PWM 값을 산출, 시리얼을 통해 Arduino로 전송
- Web UI(Flask + WebSocket)로 실시간 모니터링 및 수동 제어 가능

### • RC카의 주요 기능
- 라인 검출 및 중심 추적 기반 자율 주행
- 수동/자율 모드 전환 (모드 스위치 채널 기반)
- 실시간 웹 기반 UI 제공 (속도, 조향, 상태 시각화)

---

## ⚙️ 2. 하드웨어 구성

### • 사용 부품
- Raspberry Pi 4 + PiCamera2
- Arduino Uno
- RadioLink R9DS 수신기
- L298N Motor Driver
- DC Motor 2개, Servo Motor 1개
- 라인트랙 테이프
- LED Indicator (좌/우) 2개

### • 연결 구성도
- CH2 (전후진) → Arduino Pin 2  
- CH4 (좌우 조향) → Arduino Pin 3  
- CH5 (모드 전환) → Arduino Pin 4  
- PWM 출력: Pin 6 (속도), Pin 7 (조향)  
- LED: Pin 8 (좌), Pin 9 (우)  
- UART 시리얼 통신: Raspberry Pi ↔ Arduino (9600 baud)

---

## 🔄 3. 시스템 흐름도 및 작동 구조

### • 전체 흐름

```text
[RadioLink AT9 Tx]
        ↓ (PWM)
   [R9DS Rx Receiver]
        ↓ (PWM)
      [Arduino Uno]
        ↑ (Serial)
   [Raspberry Pi 4]
        ↑
     [PiCamera2]

        ↓
[DC Motor / Servo Motor]
        ↓
      [RC Car 주행]
```



### • 프로토콜 및 통신 구조
- **시리얼 통신**: Raspberry Pi → Arduino (`"speed,steering"` 문자열 전송)
- **WebSocket**: Flask Sock으로 UI 영상 및 상태 송수신
- **PWM 주기**: 50Hz 타이머 인터럽트 기반 제어 (Arduino)

---

## 4. 제어 방식 및 라인 검출 로직

### • 관심영역(ROI) 설정 및 라인 추적

PiCamera2에서 캡처한 프레임 하단 500px을 관심영역으로 설정:

```python
roi = gray[frame.shape[0] - self.roi_height : frame.shape[0], :]
