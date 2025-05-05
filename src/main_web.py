from flask import Flask, Response, render_template_string
import cv2
import numpy as np
import threading
import time
import serial
from picamera2 import Picamera2

# 환경 설정
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
WEB_PORT = 8080
ARDUINO_PORT = '/dev/ttyUSB0'
ARDUINO_BAUD = 9600

# 전역 변수
latest_frame = None
processed_frame = None
steering_value = 1500  # 중립 위치
throttle_value = 1500  # 중립 상태
frame_lock = threading.Lock()
running = True

# Flask 앱 초기화
app = Flask(__name__)

def setup_camera():
    """카메라 초기화 및 설정"""
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT)})
    picam2.configure(config)
    picam2.start()
    print("카메라 초기화 완료")
    return picam2

def setup_arduino():
    """아두이노 시리얼 연결 설정"""
    try:
        ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
        time.sleep(2)  # 아두이노 초기화 대기
        print("아두이노 연결 완료")
        return ser
    except serial.SerialException as e:
        print(f"아두이노 연결 실패: {e}")
        return None

def detect_lanes(frame):
    """차선 감지 및 조향각 계산"""
    global steering_value
    
    # 이미지를 그레이스케일로 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 가우시안 블러 적용
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Canny 엣지 검출
    edges = cv2.Canny(blur, 50, 150)
    
    # 관심 영역 설정 (하단 부분만 처리)
    height, width = edges.shape
    mask = np.zeros_like(edges)
    roi_vertices = np.array([[(0, height), 
                             (width//2 - 50, height//2 + 50), 
                             (width//2 + 50, height//2 + 50), 
                             (width, height)]], dtype=np.int32)
    cv2.fillPoly(mask, roi_vertices, 255)
    roi_edges = cv2.bitwise_and(edges, mask)
    
    # 허프 변환으로 직선 검출
    lines = cv2.HoughLinesP(roi_edges, 1, np.pi/180, 50, 
                           minLineLength=40, maxLineGap=100)
    
    # 결과 분석 및 방향 결정
    left_lines = []
    right_lines = []
    center_position = width // 2
    
    # 디버깅 이미지
    result_image = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    cv2.polylines(result_image, [roi_vertices], True, (0, 255, 0), 2)
    
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # 기울기 계산
            if x2 - x1 == 0:  # 수직선 예외처리
                continue
            slope = (y2 - y1) / (x2 - x1)
            
            # 왼쪽/오른쪽 선 구분
            if slope < 0 and x1 < center_position and x2 < center_position:
                left_lines.append(line[0])
                cv2.line(result_image, (x1, y1), (x2, y2), (255, 0, 0), 2)  # 파란색
            elif slope > 0 and x1 > center_position and x2 > center_position:
                right_lines.append(line[0])
                cv2.line(result_image, (x1, y1), (x2, y2), (0, 0, 255), 2)  # 빨간색
    
    # 조향 방향 계산
    new_steering = 1500  # 기본값: 중앙(1500μs)
    if len(left_lines) > 0 and len(right_lines) > 0:
        # 양쪽 차선이 모두 감지됨 - 중앙 유지
        new_steering = 1500
    elif len(left_lines) > len(right_lines):
        # 왼쪽 차선만 감지 - 오른쪽으로 조향
        new_steering = 1550
    elif len(right_lines) > len(left_lines):
        # 오른쪽 차선만 감지 - 왼쪽으로 조향
        new_steering = 1450
    
    # 갑작스러운 변화 방지 (평활화)
    steering_value = int(0.7 * steering_value + 0.3 * new_steering)
    
    # 상태 정보 이미지에 표시
    status_text = f"Left: {len(left_lines)}, Right: {len(right_lines)}, Steering: {steering_value}"
    cv2.putText(result_image, status_text, (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    print(f"좌측 선: {len(left_lines)}, 우측 선: {len(right_lines)}, 조향: {steering_value}")
    
    return result_image, steering_value

def camera_processing_loop(camera, arduino):
    """카메라 프레임 처리 및 아두이노 제어 루프"""
    global latest_frame, processed_frame, steering_value, throttle_value, running
    
    while running:
        try:
            # 프레임 가져오기
            frame = camera.capture_array()
            
            # 차선 감지 및 조향각 계산
            processed, steering = detect_lanes(frame)
            
            # 전역 변수 업데이트
            with frame_lock:
                latest_frame = frame.copy()
                processed_frame = processed.copy()
            
            # 아두이노로 명령 전송
            if arduino is not None:
                command = f"S{steering_value}T{throttle_value}\n"
                arduino.write(command.encode())
                print(f"명령 전송: {command.strip()}")
            
            # 프레임 레이트 제한
            time.sleep(0.05)
            
        except Exception as e:
            print(f"카메라 처리 오류: {e}")
            time.sleep(1)

def generate_frames(source_type):
    """웹 스트림용 프레임 생성기"""
    while running:
        with frame_lock:
            if source_type == 'raw' and latest_frame is not None:
                frame_to_show = latest_frame.copy()
            elif source_type == 'processed' and processed_frame is not None:
                frame_to_show = processed_frame.copy()
            else:
                time.sleep(0.1)
                continue
        
        # JPEG로 인코딩
        ret, buffer = cv2.imencode('.jpg', frame_to_show)
        if ret:
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                  b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
        time.sleep(0.1)

@app.route('/')
def index():
    """메인 웹 페이지"""
    html_template = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>라즈베리 파이 차선 인식</title>
        <style>
            body { font-family: Arial, sans-serif; margin: 20px; background-color: #f0f0f0; }
            h1 { color: #333; }
            .container { display: flex; flex-wrap: wrap; justify-content: center; }
            .video-container { margin: 10px; text-align: center; }
            .controls { margin: 20px 0; padding: 10px; background-color: #fff; border-radius: 5px; }
            button { padding: 8px 15px; margin: 5px; cursor: pointer; }
            .speed-slider { width: 300px; }
        </style>
    </head>
    <body>
        <h1>라즈베리 파이 차선 인식 시스템</h1>
        
        <div class="container">
            <div class="video-container">
                <h2>원본 영상</h2>
                <img src="/video_feed/raw" width="640" height="480" />
            </div>
            <div class="video-container">
                <h2>처리된 영상</h2>
                <img src="/video_feed/processed" width="640" height="480" />
            </div>
        </div>
        
        <div class="controls">
            <h2>RC 카 제어</h2>
            <div>
                <label for="speed">속도 조절:</label>
                <input type="range" id="speed" class="speed-slider" min="1400" max="1600" value="1500" />
                <span id="speed-value">1500</span>
            </div>
            <div>
                <button onclick="setSpeed(1500)">정지</button>
                <button onclick="setSpeed(1520)">저속 전진</button>
                <button onclick="setSpeed(1550)">중속 전진</button>
                <button onclick="setSpeed(1580)">고속 전진</button>
                <button onclick="setSpeed(1480)">후진</button>
            </div>
        </div>
        
        <script>
            const speedSlider = document.getElementById('speed');
            const speedValue = document.getElementById('speed-value');
            
            speedSlider.addEventListener('input', function() {
                const value = this.value;
                speedValue.textContent = value;
                updateSpeed(value);
            });
            
            function setSpeed(value) {
                speedSlider.value = value;
                speedValue.textContent = value;
                updateSpeed(value);
            }
            
            function updateSpeed(value) {
                fetch('/control/throttle/' + value)
                    .then(response => response.text())
                    .then(data => console.log(data))
                    .catch(error => console.error('Error:', error));
            }
        </script>
    </body>
    </html>
    """
    return render_template_string(html_template)

@app.route('/video_feed/<source_type>')
def video_feed(source_type):
    """비디오 스트림 엔드포인트"""
    return Response(generate_frames(source_type),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/control/throttle/<int:value>')
def control_throttle(value):
    """스로틀(속도) 제어 엔드포인트"""
    global throttle_value
    if 1400 <= value <= 1600:
        throttle_value = value
        return f"스로틀 값 설정: {value}"
    return "유효하지 않은 스로틀 값", 400

def cleanup(camera, arduino):
    """자원 정리 및 해제"""
    global running
    running = False
    
    if arduino is not None:
        arduino.write(b"S1500T1500\n")  # 모터 정지
        time.sleep(0.1)
        arduino.close()
    
    print("프로그램이 정상적으로 종료되었습니다.")

def main():
    """메인 함수"""
    # 카메라 및 아두이노 설정
    camera = setup_camera()
    arduino = setup_arduino()
    
    # 카메라 처리 스레드 시작
    processing_thread = threading.Thread(
        target=camera_processing_loop, 
        args=(camera, arduino)
    )
    processing_thread.daemon = True
    processing_thread.start()
    
    try:
        # Flask 웹 서버 시작
        app.run(host='0.0.0.0', port=WEB_PORT, debug=False, threaded=True)
    
    except KeyboardInterrupt:
        print("프로그램 종료 중...")
    
    finally:
        cleanup(camera, arduino)

if __name__ == '__main__':
    main()