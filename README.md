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

### 🔍 이미지 처리 단계

```text
[처리 과정]

1. Gaussian Blur     → 노이즈 제거
2. Threshold         → 이진화
3. Contour 추출     → 외곽선 검출
4. 가장 큰 외곽선의 중심점 추적
```

### ⚙️ PWM 사용 방식

| 항목         | 값                 |
|--------------|--------------------|
| PWM 범위     | 1000 ~ 2000        |
| 중립값       | 1500               |
| 전진         | 1570 이상          |
| 후진         | 1435 이하          |
| 조향 보정    | 중심 오차 × 민감도 |


## 5. 주요 함수 및 코드 예시

### 1️⃣ `updateMode()` — 모드 전환 처리 (Arduino, `main.cpp`)

```cpp
// CH5의 PWM 값을 기반으로 수동/자동 모드 전환
void updateMode() {
  if (mode_pwm > 1600)
    current_mode = MODE_MANUAL;  // 수동 모드
  else
    current_mode = MODE_AUTO;    // 자율 모드
}
```

> RC 수신기의 CH5 PWM 값을 기준으로 `current_mode` 전역변수를 갱신합니다.  
> 모드 스위치를 조작하면 자율 주행 ↔ 수동 주행이 전환됩니다.

---

### 2️⃣ `processSerialData()` — 시리얼 입력 처리 (Arduino, `main.cpp`)

```cpp
// 라즈베리파이로부터 시리얼 수신 문자열 처리
void processSerialData() {
  static String serialData = "";
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      parseSerialCommand(serialData);  // 전체 명령 수신 완료
      serialData = "";
    } else {
      serialData += inChar;  // 누적 입력
    }
  }
}
```

> 시리얼 입력을 한 줄(`\n`) 단위로 읽어서 `parseSerialCommand()`로 넘깁니다.  
> 자율 모드에서 라즈베리파이와의 통신을 처리하는 핵심 함수입니다.

---

### 3️⃣ `parseSerialCommand()` — PWM 값 파싱 및 적용

```cpp
// 받은 문자열을 speed,steering 으로 분리하여 모터 제어
void parseSerialCommand(String data) {
  int commaIndex = data.indexOf(',');
  if (commaIndex > 0) {
    int speed = data.substring(0, commaIndex).toInt();
    int steering = data.substring(commaIndex + 1).toInt();
    analogWrite(MOTOR_PIN, speed);     // 속도 PWM 출력
    analogWrite(SERVO_PIN, steering);  // 조향 PWM 출력
  }
}
```

> "1570,1450" 같은 문자열을 분해하여 모터/서보에 PWM 값을 전송합니다.  
> 잘못된 데이터는 무시되도록 간단한 예외처리도 포함되어 있습니다.

---

### 4️⃣ `detect_line()` — 라인 검출 (Python, `app.py`)

```python
# ROI 설정 후 이진화 및 외곽선 추출 → 라인 중심 계산
def detect_line(self, frame):
    roi = frame[frame.shape[0] - self.roi_height : , :]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(blurred, self.thresh, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])  # 중심 좌표 X
            return cx
    return None
```

> 영상 하단 관심영역에서 이진화 + 외곽선 검출로 라인을 추적합니다.  
> 가장 큰 윤곽선의 중심을 계산해 자율 조향에 활용됩니다.

---

### 5️⃣ `calculate_steering()` — 중심 오차 기반 조향 보정

```python
# 라인 중심과 화면 중심 간 오차 → 조향 보정 PWM 산출
def calculate_steering(self, line_center, frame_width):
    error = (frame_width // 2) - line_center
    steering = self.neutral_steering + error * self.steering_sensitivity
    return int(steering)
```

> 화면 중앙에서 라인 중심이 얼마나 벗어났는지를 기반으로  
> 조향 PWM을 계산합니다. 민감도 계수를 곱해 보정 폭을 조절합니다.

---

### 6️⃣ `send_control_signal()` — PWM 명령 전송 (Python → Arduino)

```python
# 시리얼 포트로 speed,steering 문자열 전송
def send_control_signal(self, speed_pwm, steering_pwm):
    command = f"{speed_pwm},{steering_pwm}\\n"
    if self.serial and self.serial.is_open:
        self.serial.write(command.encode())
```

> 조향 및 속도 PWM 값을 아두이노로 전송하는 함수입니다.  
> 실패 방지를 위해 연결 상태를 매번 점검합니다.

---

### 7️⃣ `autonomous_drive()` — 자율주행 주 루프

```python
# 자율주행 모드에서 라인 감지 후 PWM 제어 수행
def autonomous_drive(self, frame):
    line_center = self.detect_line(frame)

    if line_center is not None:
        speed_pwm = self.base_speed
        steering_pwm = self.calculate_steering(line_center, frame.shape[1])
    else:
        speed_pwm = self.backword_speed  # 라인 미검출 시 후진
        steering_pwm = self.neutral_steering

    self.send_control_signal(speed_pwm, steering_pwm)
```

> 프레임을 분석하여 라인을 감지하고, 감지 실패 시 자동 후진하도록 구성되어 있습니다.  
> 자율주행의 핵심 루프 로직으로서 매 프레임마다 호출됩니다.

---

