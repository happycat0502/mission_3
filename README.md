# 라인 팔로잉 RC 카 프로젝트

이 프로젝트는 라즈베리 파이 5와 아두이노를 활용하여 카메라 기반 차선 인식 자율주행 RC 카를 구현합니다. 카메라로 차선을 감지하고, 마이크로초 단위의 정밀한 PWM 신호를 생성하여 RC 카를 제어합니다.

## 프로젝트 개요

- **비전 시스템**: 라즈베리 파이 5와 AI 카메라로 차선 인식
- **제어 시스템**: 아두이노를 통한 정밀 PWM 신호 생성
- **통신**: USB 시리얼 통신을 통한 라즈베리 파이와 아두이노 간 명령 전달

## 하드웨어 요구사항

- 라즈베리 파이 5
- 라즈베리 파이 AI 카메라 (IMX500 또는 호환 모델)
- 아두이노 보드 (Uno, Nano 등)
- RC 카 섀시 (서보 모터 및 ESC 포함)
- USB 케이블 (라즈베리 파이와 아두이노 연결용)
- 배터리 및 전원 공급 장치

## 소프트웨어 의존성

### 라즈베리 파이
- Python 3.7+
- OpenCV (`pip install opencv-python`)
- picamera2 (`apt install python3-picamera2`)
- pyserial (`pip install pyserial`)
- NumPy (`pip install numpy`)

### 아두이노
- TaskScheduler 라이브러리 (Arduino Library Manager에서 설치)

## 설치 방법

### 라즈베리 파이 설정

1. 라즈베리 파이 OS 설치 및 업데이트
   ```bash
   sudo apt update
   sudo apt upgrade
   ```

2. 필요한 패키지 설치
   ```bash
   sudo apt install python3-picamera2 python3-opencv python3-pip
   pip install pyserial numpy
   ```

3. 카메라 활성화
   ```bash
   sudo raspi-config
   # 인터페이스 옵션 > 카메라 > 활성화
   ```

### 아두이노 설정

1. Arduino IDE 설치
2. TaskScheduler 라이브러리 설치 (라이브러리 매니저 사용)
3. 아두이노 코드 업로드

## 코드 설명

### 아두이노 코드 ()
하드웨어 타이머를 사용하여 정밀한 PWM 신호를 생성합니다:
- 타이머 인터럽트로 마이크로초 단위 펄스 폭 제어
- 서보 모터와 ESC를 위한 50Hz 신호 생성
- 시리얼 통신으로 명령 수신 (S{조향값}T{속도값} 형식)

### 라즈베리 파이 코드 (main.py)
카메라로 차선을 감지하고 아두이노로 제어 명령을 전송합니다:
- 카메라 프레임 캡처 및 처리
- Canny 엣지 검출 및 허프 변환으로 라인 감지
- 왼쪽/오른쪽 차선 구분하여 조향 방향 결정
- 시리얼 통신으로 아두이노에 명령 전송

## 향후 개선 사항

1. **알고리즘 개선**: 
   - PID 제어 구현으로 부드러운 조향
   - 딥러닝 기반 차선 인식 (TensorFlow Lite)

2. **하드웨어 업그레이드**:
   - 더 빠른 카메라 프레임 레이트
   - 더 정확한 위치 추적을 위한 IMU 추가

3. **기능 추가**:
   - 장애물 감지 및 회피
   - 교차로 인식 및 의사 결정
   - 웹 인터페이스로 원격 모니터링