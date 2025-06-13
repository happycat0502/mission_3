# 🚗 RC Car 자율주행 공보 프로젝트

## ✨ 1. 프로젝트 개요

### • 목적
- 자재와 시간 비용을 절감하고, 소형 환경에서 자율주행 알고리즘과 통신 구조를 학습하기 위한 RC Car 기반 자율주행 시스템 구현

### • 사용 송수신기
- **RadioLink AT-9 Tx + R9DS Rx**
- 채널 구성: CH2(전후진), CH4(좌우 조향), CH5(모드 스위치)
- **Mode: Acrobatic** 모드에서 사용 (수동 PWM 제어에 적합)
![image](https://github.com/user-attachments/assets/597c4d9a-94ef-4215-b657-ec14da977c54)



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
- orange board
- RadioLink R9DS 수신기
- DC Motor 2개, Servo Motor 1개
- 라인트랙 테이프
- LED Indicator (좌/우) 2개
  ![Uploading image.png…]()


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

## 6. 작동 예시

## 🚗 수동 주행 모드 (Manual Mode)

### 🔧 제어 방식
- RadioLink AT9 조종기에서 직접 RC카 제어
- CH2 → 전후진 PWM (1000~2000)
- CH4 → 좌우 조향 PWM
- CH5 → 모드 전환용 (1600 이상이면 수동 모드 진입)

---

### ⚙️ 동작 원리
1. Arduino가 CH2/CH4/CH5 PWM 신호를 입력받음
2. CH5 > 1600 → MODE_MANUAL 설정
3. 사용자 PWM 값을 그대로 모터와 서보모터에 출력



### 📌 핵심 코드 (Arduino `main.cpp`)

```cpp
void loop() {
  readPWM();           // CH2, CH4, CH5 PWM 신호 읽기
  updateMode();        // CH5로 모드 전환 여부 확인

  if (current_mode == MODE_MANUAL) {
    analogWrite(MOTOR_PIN, throttle_pwm);     // DC 모터 제어
    analogWrite(SERVO_PIN, steering_pwm);     // 서보모터 조향 제어
  }
}
```

> **설명:**  
> - 수신기에서 들어오는 PWM 값을 그대로 모터 제어에 사용합니다.  
> - 조종기에서의 입력이 RC카에 실시간 반영됩니다.  
> - 테스트 시 높은 응답성과 제어 편의성 확보에 용이합니다.



## 🤖 자율 주행 모드 (Autonomous Mode)

### 🔧 제어 방식
- PiCamera 영상 기반 라인 추적
- 중심 오차 → Steering 계산
- Raspberry Pi → Arduino에 시리얼로 PWM 값 전송

---

### ⚙️ 동작 원리
1. PiCamera로 실시간 영상 수신
2. 관심영역(ROI) 설정 후 이진화 및 Contour 추출
3. 라인의 중심 좌표를 추출
4. 화면 중심과의 오차를 기반으로 steering 보정값 계산
5. speed, steering PWM 값을 시리얼로 Arduino에 전송
6. Arduino는 해당 PWM을 모터에 적용



### 📌 핵심 코드 (Python `app.py`)

```python
def detect_line(self, frame):
    # 영상 하단 ROI 설정
    roi = frame[frame.shape[0] - self.roi_height:, :]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(blurred, self.thresh, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])  # 중심 x좌표
            return cx
    return None
```

```python
def calculate_steering(self, line_center, frame_width):
    error = (frame_width // 2) - line_center
    steering = self.neutral_steering + error * self.steering_sensitivity
    return int(steering)
```

```python
def autonomous_drive(self, frame):
    # 자율주행 메인 루프
    line_center = self.detect_line(frame)

    if line_center is not None:
        speed_pwm = self.base_speed
        steering_pwm = self.calculate_steering(line_center, frame.shape[1])
    else:
        # 라인을 잃으면 후진 PWM을 사용
        speed_pwm = self.backword_speed
        steering_pwm = self.neutral_steering

    self.send_control_signal(speed_pwm, steering_pwm)
```

```python
def send_control_signal(self, speed_pwm, steering_pwm):
    command = f"{speed_pwm},{steering_pwm}\\n"
    if self.serial and self.serial.is_open:
        self.serial.write(command.encode())
```

> **설명:**  
> - 영상 하단만 분석하여 연산량을 줄이고 반응속도 향상  
> - 라인 중심과 영상 중심 간의 오차를 기반으로 Steering 조향값 계산  
> - 조향값과 속도값은 문자열 형태로 Arduino에 전달



### 📌 핵심 코드 (Arduino `main.cpp`)

```cpp
void processSerialData() {
  static String serialData = "";
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      parseSerialCommand(serialData);
      serialData = "";
    } else {
      serialData += inChar;
    }
  }
}

void parseSerialCommand(String data) {
  int commaIndex = data.indexOf(',');
  if (commaIndex > 0) {
    int speed = data.substring(0, commaIndex).toInt();
    int steering = data.substring(commaIndex + 1).toInt();
    analogWrite(MOTOR_PIN, speed);
    analogWrite(SERVO_PIN, steering);
  }
}
```

> **설명:**  
> - Raspberry Pi에서 전송된 문자열 `"1570,1450"`을 speed/steering으로 분리  
> - 모터와 서보에 해당 PWM 출력 적용  
> - 통신 지연/에러 발생 시 fail-safe 루틴이 동작해 정지 처리


## 📊 수동 vs 자율 비교 요약

| 항목            | 수동 주행 모드                        | 자율 주행 모드                           |
|-----------------|----------------------------------------|------------------------------------------|
| 입력            | 조종기 PWM (CH2, CH4)                 | 카메라 영상 (PiCamera2)                 |
| 처리 장치       | Arduino 단독                           | Raspberry Pi에서 분석 후 Arduino 전송  |
| 조향 방식       | CH4 PWM → 서보                         | 영상 중심 좌표 기반 Steering 계산       |
| 속도 제어       | CH2 PWM → 모터                         | 고정 전진 속도 + 미검출 시 후진 적용    |
| 전환 방법       | CH5 PWM > 1600                         | CH5 PWM < 1600                           |
| 장점            | 직관적이고 빠른 반응 속도              | 사람이 개입하지 않아도 주행 가능       |



---

## 7. 문제 해결 & 트러블슈팅

## 🔧 문제 해결 및 트러블슈팅

### ✅ 1. Raspberry Pi ↔ Arduino 연결 시 PWM 신호 인식 실패

- **문제:** Raspberry Pi와 Arduino 연결 시 PWM 제어가 먹지 않거나, 아두이노 보드 자체가 오작동하거나 인식이 안 됨
- **원인:** `Arduino Uno의 0번(RX), 1번(TX) 핀`을 사용 중이었음 → 해당 핀은 USB 통신에도 쓰이기 때문에 충돌 발생
- **해결:** PWM 핀을 `2~9번` 등 **일반 디지털 핀**으로 재배정하고, `Serial` 통신은 `Serial` 포트 단독으로 사용하여 해결

> 💡 **주의:** 0, 1번 핀은 USB 업로드 및 Serial 통신과 겹치므로 PWM 제어에는 적합하지 않음



### ✅ 2. 급커브에서 라인 미검출 → 차량 멈춤

- **문제:** 선회 구간에서 라인이 ROI에 잡히지 않아 차량이 멈추거나 혼란스러운 동작 발생
- **초기 시도:** 카메라 위치 조정, ROI 범위 상단 확장
- **최종 해결:** 일정 시간 라인을 인식하지 못하면 **자동 후진 로직** 삽입 → 다시 라인을 검출할 기회 확보

#### 📌 코드 예시 (Python `autonomous_drive`)

```python
if line_center is not None:
    speed_pwm = self.base_speed            # 정상 추적 시 전진
    steering_pwm = self.calculate_steering(line_center, frame.shape[1])
else:
    speed_pwm = self.backword_speed        # 라인 미검출 시 후진
    steering_pwm = self.neutral_steering
```

> 💡 **핵심:** 시야에 라인이 사라졌을 때 멈추는 대신, 후진하여 다시 라인을 ROI에 유도



### ✅ 3. ROI 영역 설정 오류 → 라인 검출 실패

- **문제:** 카메라 화면의 상단 영역을 ROI로 설정하여 도로 라인이 보이지 않았음
- **해결:** ROI를 **화면 하단 영역**으로 조정함으로써 주행 중 전방 도로를 정확히 포착 가능

#### 📌 ROI 코드 예시

```python
roi = frame[frame.shape[0] - self.roi_height:, :]
```

- `frame.shape[0]`은 전체 높이 → ROI는 하단 N픽셀 영역
- `self.roi_height`는 ROI 세로 길이 (예: 80~120)

> 🎯 **설명:** ROI는 차량 전방 도로와 가까운 위치여야 실시간 반응이 빠름  
> 예를 들어, ROI 높이를 100으로 하면 전체 해상도 기준 마지막 100픽셀만 분석


### ✅ 4. 수동 주행 모드에서 조종기 모드 오류

- **문제:** 초기에는 조종기에서 `헬리콥터 모드`로 설정되어 PWM 값이 비정상 출력됨
- **해결:** `Acrobatic Mode`로 변경 후, PWM 출력값이 안정화되어 정상 제어 가능

> 💡 **설명:**  
> 헬리콥터 모드는 일부 채널을 자동으로 믹싱하거나 제한함 → 수동 PWM 제어에는 부적합  
> Acrobatic 모드는 각 채널을 **그대로 출력**하기 때문에 수동 주행에 필수



### ✅ 5. 배터리 전압 감소로 PWM 값 변화

- **문제:** 배터리 성능 저하로 인해 PWM 1500 기준값에서 전후진이 잘 작동하지 않음
- **해결:** 전진 기준값을 `1570`, 후진은 `1435` 정도로 **적절히 확장하여 보정**

| 항목       | 값         |
|------------|------------|
| 중립값     | 1500       |
| 전진 시작  | 1570 이상  |
| 후진 시작  | 1435 이하  |

> ⚙️ **팁:**  
> 전압 강하를 대비해 전진/후진 기준값에 여유를 두는 것이 실제 구동 안정성 향상에 효과적



## 📸 관심영역 (ROI, Region of Interest) 추가 설명

- 자율주행에서 가장 핵심적인 시각 인식 범위
- 하단 영역을 선택함으로써 차량의 **즉시 주행 경로**만을 집중 분석
- ROI를 너무 위에 설정하면 라인 검출이 늦거나 불가능해짐

### 📌 ROI 설정 방식 예시

```python
roi = frame[frame.shape[0] - 100:, :]  # 하단 100픽셀을 ROI로 사용
```

> 📷 **ROI 구성 흐름:**  
> 전체 프레임 → Grayscale → Gaussian Blur → Threshold → Contour 추출  
> → 가장 큰 외곽선 중심점(cx) → 화면 중심과 비교 → 조향 각 결정


---

## 8. 설치 및 실행 방법

### 🔌 Raspberry Pi (Python 환경)

```bash
# 필요한 패키지 설치
pip install opencv-python flask flask-sock numpy picamera2

# 실행
python3 app.py
```

- 웹 브라우저 접속 주소: `http://<라즈베리파이_IP>:5000`



### ⚙️ Arduino

- Arduino IDE에서 `main.cpp` 업로드
- 시리얼 속도 9600bps 확인
- PWM 핀, 수신 핀, 모드 핀 연결 확인

---

## 9. 팀원 & 기여자

| 이름 | 역할 |
|------|------|
| **변하연** | 수신기에서 PWM 신호 읽기, DC 모터/ 서보 모터에 PWM 출력, Pi Camera 영상 실시간 수신 및 처리 |
| **고광채** | 시리얼 통신 수신 및 피싱 처리, ROI 설정 및 라인 검출, 중심 오차 계산 및 조향 보정, Gaussian Blur → Threshold → Contour 중심 추적, 자율&수동 주행 로직 |

---



