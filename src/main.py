from picamera2 import Picamera2
import cv2
import numpy as np
import serial
import time

# 시리얼 연결 설정
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)  # 아두이노 연결 안정화 대기

def detect_lanes(frame):
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
            elif slope > 0 and x1 > center_position and x2 > center_position:
                right_lines.append(line[0])
    
    # 조향 방향 계산
    steering = 1500  # 기본값: 중앙(1500μs)
    if len(left_lines) > 0 and len(right_lines) > 0:
        # 양쪽 차선이 모두 감지됨 - 중앙 유지
        steering = 1500
    elif len(left_lines) > len(right_lines):
        # 왼쪽 차선만 감지 - 오른쪽으로 조향
        steering = 1550
    elif len(right_lines) > len(left_lines):
        # 오른쪽 차선만 감지 - 왼쪽으로 조향
        steering = 1450
    
    # 디버깅을 위해 이미지 파일로 저장 (필요시)
    # timestamp = int(time.time())
    # cv2.imwrite(f"/home/kwangchae/debug_frame_{timestamp}.jpg", frame)
    
    print(f"좌측 선: {len(left_lines)}, 우측 선: {len(right_lines)}, 조향: {steering}")
    
    return steering

def main():
    # 카메라 초기화
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480)})
    picam2.configure(config)
    picam2.start()
    print("카메라 초기화 완료")
    
    try:
        while True:
            # 프레임 가져오기
            frame = picam2.capture_array()
            
            # 차선 감지 및 조향각 계산 (imshow 없이)
            steering_value = detect_lanes(frame)
            
            # 조향각과 속도 명령 전송
            throttle = 1520  # 기본 전진 속도
            command = f"S{steering_value}T{throttle}\n"
            arduino.write(command.encode())
            print(f"명령 전송: {command.strip()}")
            
            # 필요시 프레임 속도 제한
            time.sleep(0.1)
                
    except KeyboardInterrupt:
        print("프로그램 종료 중...")
    finally:
        arduino.write(b"S1500T1500\n")  # 중지 명령
        arduino.close()
        print("프로그램 종료됨")

if __name__ == "__main__":
    main()