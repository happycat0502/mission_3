#!/usr/bin/env python3
"""
개선된 시리얼 통신 처리를 포함한 라인 추적 시스템
- 성공적인 연결 후 불필요한 재연결 시도 방지
"""

import cv2
import numpy as np
import serial
import time
import threading
from picamera2 import Picamera2
from libcamera import controls
from flask import Flask, render_template
from flask_sock import Sock
import base64
import json
import glob

class LineFollower:
    def __init__(self, serial_port=None, baud_rate=9600):
        # 기본 속성 초기화
        self.arduino = None
        self.serial_connected = False
        self.connection_stable = False  # 안정적 연결 상태 추가
        self.autonomous_mode = False  # 아두이노 자율주행 모드 상태
        self.manual_override = False  # 수동 오버라이드 상태
        self.arduino_status = ""  # 아두이노 상태 메시지
        
        # 시리얼 포트 자동 검색 및 설정
        self.setup_serial_connection(serial_port, baud_rate)
        
        # 카메라 설정
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (1920, 1080)}
        )
        self.picam2.configure(config)
        self.picam2.set_controls({"AwbMode": controls.AwbModeEnum.Auto})
        self.picam2.start()
        
        # 제어 파라미터
        self.image_center = 1920 // 2
        self.steering_sensitivity = 1.05
        self.base_speed = 1570
        self.max_steering = 2000
        self.backword_speed = 1435
        
        # 라인 검출 파라미터
        self.roi_height = 500
        self.roi_offset = 0
        
        # 제어 상태
        self.running = False
        self.current_steering = 1500
        self.current_speed = 1500
        
        # 시리얼 연결 모니터링 파라미터
        self.last_serial_check = time.time()
        self.serial_check_interval = 5.0  # 더 자주 체크
        self.connection_error_count = 0  # 연결 오류 카운터
        self.max_error_count = 2  # 오류 허용 횟수 감소
        self.last_successful_send = time.time()  # 마지막 성공적 전송 시간
        
        # 웹 스트리밍 설정
        self.app = Flask(__name__)
        self.sock = Sock(self.app)
        self.setup_web_routes()
    
    def find_arduino_ports(self):
        """아두이노가 연결된 시리얼 포트 찾기"""
        possible_ports = []
        
        # Linux에서 일반적인 USB 시리얼 포트들
        port_patterns = [
            '/dev/ttyUSB*',
            '/dev/ttyACM*', 
            '/dev/ttyS*'
        ]
        
        for pattern in port_patterns:
            possible_ports.extend(glob.glob(pattern))
        
        # 포트 정렬
        possible_ports.sort()
        
        print(f"발견된 시리얼 포트: {possible_ports}")
        return possible_ports
    
    def test_serial_port(self, port, baud_rate):
        """시리얼 포트 연결 테스트"""
        try:
            test_serial = serial.Serial(port, baud_rate, timeout=2)
            time.sleep(2)  # 아두이노 부팅 대기
            
            # 테스트 명령 전송
            test_serial.write(b"1500,1500\n")
            test_serial.flush()
            
            # 응답 확인 (선택사항)
            test_serial.close()
            return True
        except Exception as e:
            print(f"포트 {port} 테스트 실패: {e}")
            return False
    
    def setup_serial_connection(self, serial_port, baud_rate):
        """시리얼 연결 설정 (자동 검색 포함)"""
        ports_to_try = []
        
        if serial_port:
            # 지정된 포트가 있으면 우선 시도
            ports_to_try.append(serial_port)
        
        # 자동 검색된 포트들 추가
        ports_to_try.extend(self.find_arduino_ports())
        
        for port in ports_to_try:
            try:
                print(f"시리얼 포트 {port} 연결 시도...")
                if self.test_serial_port(port, baud_rate):
                    self.arduino = serial.Serial(port, baud_rate, timeout=1)
                    time.sleep(2)
                    self.serial_connected = True
                    self.connection_stable = True  # 안정적 연결 설정
                    self.connection_error_count = 0  # 오류 카운터 초기화
                    self.last_successful_send = time.time()
                    print(f"Arduino 연결 성공: {port}")
                    return
            except Exception as e:
                print(f"포트 {port} 연결 실패: {e}")
                continue
        
        print("사용 가능한 Arduino를 찾을 수 없습니다.")
        self.serial_connected = False
        self.connection_stable = False
    
    def reconnect_serial(self):
        """시리얼 연결 재시도"""
        if self.arduino:
            try:
                self.arduino.close()
            except:
                pass
            self.arduino = None
        
        self.serial_connected = False
        self.connection_stable = False
        self.connection_error_count = 0
        
        print("시리얼 연결 재시도...")
        self.setup_serial_connection(None, 9600)
    
    def check_serial_connection(self):
        """개선된 시리얼 연결 상태 확인"""
        current_time = time.time()
        
        # 연결이 안정적이고 최근에 성공적으로 전송했다면 자주 확인하지 않음
        if (self.connection_stable and 
            self.serial_connected and 
            current_time - self.last_successful_send < 30.0):  # 30초 이내 성공적 전송
            return
        
        # 정기적인 연결 확인
        if current_time - self.last_serial_check > self.serial_check_interval:
            self.last_serial_check = current_time
            
            # 연결이 불안정하거나 오류가 많이 발생한 경우에만 재연결 시도
            if (not self.serial_connected or 
                not self.arduino or 
                not self.arduino.is_open or
                self.connection_error_count >= self.max_error_count):
                
                print(f"연결 상태 불안정 (오류 횟수: {self.connection_error_count})")
                self.reconnect_serial()
    
    def send_control_signal(self, speed_pwm, steering_pwm):
        """개선된 아두이노 제어 신호 전송"""
        # 연결이 안정적이지 않은 경우에만 상태 확인
        if not self.connection_stable:
            self.check_serial_connection()
        
        if self.arduino and self.arduino.is_open and self.serial_connected:
            try:
                command = f"{speed_pwm},{steering_pwm}\n"
                self.arduino.write(command.encode())
                self.arduino.flush()
                
                # 성공적 전송 기록
                self.last_successful_send = time.time()
                self.connection_error_count = 0  # 성공시 오류 카운터 초기화
                
                # 연결이 안정적임을 확인
                if not self.connection_stable:
                    self.connection_stable = True
                    print("시리얼 연결이 안정화되었습니다.")
                    
            except serial.SerialException as e:
                print(f"시리얼 통신 오류: {e}")
                self.connection_error_count += 1
                self.connection_stable = False
                
                # 오류가 임계값을 넘으면 즉시 재연결
                if self.connection_error_count >= self.max_error_count:
                    print("연결 오류 임계값 초과, 재연결 시도")
                    self.serial_connected = False
                    self.reconnect_serial()
                    
            except Exception as e:
                print(f"기타 통신 오류: {e}")
                self.connection_error_count += 1
                self.connection_stable = False
        else:
            if self.serial_connected:  # 연결되어 있다고 생각했는데 실제로는 안된 경우
                print("시리얼 연결 상태 불일치 감지")
                self.serial_connected = False
                self.connection_stable = False
    
    def read_arduino_status(self):
        """아두이노 상태 메시지 읽기 (별도 함수로 분리)"""
        if not self.arduino or not self.arduino.is_open or not self.serial_connected:
            return
            
        try:
            # 아두이노 응답 처리 (상태 메시지 읽기)
            if self.arduino.in_waiting > 0:
                # 모든 대기 중인 데이터 읽기
                messages_processed = 0
                while self.arduino.in_waiting > 0 and messages_processed < 5:  # 최대 5개 메시지만 처리
                    response = self.arduino.readline().decode('utf-8').strip()
                    messages_processed += 1
                    
                    if response:
                        self.arduino_status = response
                        
                        # 아두이노 모드 상태 파싱
                        if "모드:" in response or "활성화" in response:
                            if "자율주행" in response:
                                if not self.autonomous_mode:
                                    self.autonomous_mode = True
                                    print("아두이노: 자율주행 모드 활성화")
                                    # 라인 추적 자동 시작
                                    if not self.running and not self.manual_override:
                                        print("자동으로 라인 추적 시작")
                                        self.start()
                            elif "수동" in response:
                                if self.autonomous_mode:
                                    self.autonomous_mode = False
                                    print("아두이노: 수동 모드 활성화")
                                    # 수동 모드로 전환 시 라인 추적 자동 정지
                                    if self.running and not self.manual_override:
                                        print("자동으로 라인 추적 정지")
                                        self.stop()
                            
                            # 디버깅용 상태 출력 (모드 변경 시에만)
                            print(f"Arduino: {response}")
                            break  # 모드 변경 메시지 처리 후 루프 탈출
                            
        except Exception as e:
            print(f"아두이노 응답 읽기 오류: {e}")
            # 응답 읽기 실패시 버퍼만 정리
            try:
                if self.arduino.in_waiting > 0:
                    self.arduino.read(self.arduino.in_waiting)
            except:
                pass
    
    def setup_web_routes(self):
        """웹 라우트 설정"""
        @self.app.route('/')
        def index():
            return render_template('index.html')
        
        @self.sock.route('/ws')
        def webcam_ws(ws):
            try:
                while True:
                    frame = self.get_processed_frame()
                    ok, buf = cv2.imencode('.jpg', frame)
                    if not ok:
                        continue
                    jpg_b64 = base64.b64encode(buf).decode()
                    
                    data = {
                        'image': jpg_b64,
                        'steering': self.current_steering,
                        'speed': self.current_speed,
                        'running': self.running,
                        'serial_connected': self.serial_connected,
                        'connection_stable': self.connection_stable,
                        'arduino_status': self.arduino_status,
                        'autonomous_mode': self.autonomous_mode,  # 아두이노 모드 상태 추가
                        'manual_override': self.manual_override  # 수동 오버라이드 상태 추가
                    }
                    ws.send(json.dumps(data))
                    time.sleep(0.03)
            except Exception as e:
                print("WebSocket 연결 종료:", e)
    
    def detect_line(self, frame):
        """검정색 라인 검출"""
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        
        height, width = gray.shape
        roi_y_start = height - self.roi_height - self.roi_offset
        roi_y_end = height - self.roi_offset
        roi = gray[roi_y_start:roi_y_end, :]
        
        blurred = cv2.GaussianBlur(roi, (5, 5), 0)
        _, binary = cv2.threshold(blurred, 160, 255, cv2.THRESH_BINARY_INV)
        
        kernel = np.ones((3, 3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        line_center = None
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 6400:
                M = cv2.moments(largest_contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    line_center = cx
            else:
                line_center = None
                #print("라인이 너무 작아 검출 실패")
        else :
            line_center = None
            #print("라인 검출 실패")
        return binary, line_center, roi_y_start
    
    def calculate_steering(self, line_center):
        """라인 중심에 따른 조향각 계산"""
        if line_center is None:
            return 1500
        
        error = self.image_center - line_center
        steering_adjustment = int(error * self.steering_sensitivity)
        steering_adjustment = max(-self.max_steering, min(self.max_steering, steering_adjustment))
        steering_pwm = 1500 + steering_adjustment
        
        return max(1000, min(2000, steering_pwm))
    
    def get_processed_frame(self):
        """처리된 프레임 반환 (웹 스트리밍용)"""
        frame = self.picam2.capture_array()
        binary, line_center, roi_y_start = self.detect_line(frame)
        
        overlay = frame.copy()
        height, width = frame.shape[:2]
        roi_y_end = height - self.roi_offset
        
        cv2.rectangle(overlay, (0, roi_y_start), (width, roi_y_end), (0, 255, 0), 2)
        cv2.line(overlay, (self.image_center, 0), (self.image_center, height), (255, 0, 0), 2)
        
        if line_center is not None:
            cv2.circle(overlay, (line_center, roi_y_start + self.roi_height//2), 10, (0, 0, 255), -1)
            cv2.line(overlay, (line_center, roi_y_start), (line_center, roi_y_end), (0, 0, 255), 2)
        
        binary_color = cv2.cvtColor(binary, cv2.COLOR_GRAY2RGB)
        overlay[roi_y_start:roi_y_start+binary.shape[0], :binary.shape[1]] = binary_color
        
        # 연결 상태 표시 개선
        if self.serial_connected and self.connection_stable:
            connection_status = "Stable"
            status_color = (0, 255, 0)
        elif self.serial_connected:
            connection_status = "Connected"
            status_color = (0, 255, 255)  # 노란색
        else:
            connection_status = "Disconnected"
            status_color = (0, 0, 255)
        
        # 모드 상태 표시 개선
        if self.manual_override:
            mode_text = "강제"
            mode_color = (0, 0, 255)  # 빨간색
        elif self.autonomous_mode:
            mode_text = "자율주행"
            mode_color = (0, 255, 0)  # 초록색
        else:
            mode_text = "수동"
            mode_color = (0, 255, 255)  # 노란색
        
        cv2.putText(overlay, f"Speed: {self.current_speed}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(overlay, f"Steering: {self.current_steering}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(overlay, f"Mode: {mode_text}", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, mode_color, 2)
        cv2.putText(overlay, f"Running: {self.running}", (10, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(overlay, f"Serial: {connection_status}", (10, 150), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        cv2.putText(overlay, f"Errors: {self.connection_error_count}", (10, 180), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 아두이노 상태 표시 (있는 경우)
        if self.arduino_status:
            cv2.putText(overlay, f"Arduino: {self.arduino_status[:30]}", (10, 210), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        return overlay
    
    def autonomous_drive(self):
        """자율주행 메인 루프"""
        if self.manual_override:
            print("강제 모드로 라인 추적 시작 (아두이노 모드 무시)")
        else:
            print("라인 추적 시스템 준비됨 (아두이노 모드에 따라 동작)")
        
        self.running = True
        
        while self.running:
            try:
                # 강제 모드이거나 (아두이노가 자율주행 모드이고 연결된 경우)
                should_track_line = (self.manual_override or 
                                   (self.autonomous_mode and self.serial_connected))
                
                if should_track_line:
                    frame = self.picam2.capture_array()
                    binary, line_center, _ = self.detect_line(frame)
                    steering_pwm = self.calculate_steering(line_center)
                    
                    if line_center is not None:
                        speed_pwm = self.base_speed
                    else:
                        speed_pwm = self.backword_speed
                    
                    self.current_speed = speed_pwm
                    self.current_steering = steering_pwm
                    
                    # 강제 모드이거나 자율주행 모드일 때 신호 전송
                    if self.serial_connected:
                        self.send_control_signal(speed_pwm, steering_pwm)
                    
                    if line_center is not None:
                        mode_text = "강제" if self.manual_override else "자동"
                        status = f"[{mode_text}] 라인 중심: {line_center}, 조향: {steering_pwm}, 속도: {speed_pwm}"
                        print(status)
                else:
                    # 수동 모드이거나 연결이 끊어진 경우
                    if not self.autonomous_mode and not self.manual_override:
                        # 수동 모드: 중립값 설정 (RC 신호 사용)
                        self.current_speed = 1500
                        self.current_steering = 1500
                    elif not self.serial_connected:
                        print("시리얼 연결 끊어짐 - 재연결 대기 중")
                
                # 아두이노 상태 메시지 읽기 (항상 수행)
                if self.arduino and self.arduino.in_waiting > 0:
                    try:
                        response = self.arduino.readline().decode('utf-8').strip()
                        if response:
                            self.arduino_status = response
                            # 강제 모드가 아닐 때만 모드 변경 감지
                            if not self.manual_override and "모드:" in response:
                                if "자율주행" in response and not self.autonomous_mode:
                                    self.autonomous_mode = True
                                    print("아두이노: 자율주행 모드 활성화")
                                elif "수동" in response and self.autonomous_mode:
                                    self.autonomous_mode = False
                                    print("아두이노: 수동 모드 활성화")
                    except:
                        pass
                
                time.sleep(0.05)
                
            except KeyboardInterrupt:
                print("라인 추적 중단")
                break
            except Exception as e:
                print(f"자율주행 오류: {e}")
                time.sleep(0.1)
        
        self.stop()
    
    def start(self):
        """라인 추적 시스템 시작 (모드 모니터링 포함)"""
        if not self.running:
            self.manual_override = False  # 수동 오버라이드 해제
            # 자율주행 모드이면 즉시 중립 신호 전송하여 타임아웃 방지
            if self.autonomous_mode and self.serial_connected:
                self.send_control_signal(1500, 1500)
                time.sleep(0.1)  # 아두이노가 신호를 처리할 시간 제공
            self.drive_thread = threading.Thread(target=self.autonomous_drive)
            self.drive_thread.daemon = True
            self.drive_thread.start()
    
    def start_manual(self):
        """수동으로 라인 추적 강제 시작"""
        if not self.running:
            self.manual_override = True  # 수동 오버라이드 설정
            print("수동으로 라인 추적 강제 시작")
            # 강제 시작 시 즉시 중립 신호 전송
            if self.serial_connected:
                self.send_control_signal(1500, 1500)
                time.sleep(0.1)  # 아두이노가 신호를 처리할 시간 제공
            self.drive_thread = threading.Thread(target=self.autonomous_drive)
            self.drive_thread.daemon = True
            self.drive_thread.start()
    
    def stop(self):
        """라인 추적 정지"""
        print("라인 추적 정지")
        self.running = False
        self.manual_override = False
        self.current_speed = 1500
        self.current_steering = 1500
        # 자율주행 모드일 때만 정지 신호 전송
        if self.autonomous_mode and self.serial_connected:
            self.send_control_signal(1500, 1500)
    
    def run_web_server(self):
        """웹 서버 실행"""
        self.app.run(host='0.0.0.0', port=5000, debug=False)
    
    def cleanup(self):
        """리소스 정리"""
        self.stop()
        if self.arduino:
            self.arduino.close()
        self.picam2.stop()

def main():
    line_follower = LineFollower()
    
    try:
        print("라인 추적 시스템 시작")
        print("웹 브라우저에서 http://라즈베리파이IP:5000 접속")
        print("시스템 시작: 's' 키 (아두이노 모드에 따라 자동 동작)")
        print("강제 시작: 'f' 키 (아두이노 모드 무시하고 강제 실행)")
        print("시스템 정지: 'q' 키")
        print("시리얼 재연결: 'r' 키")
        print("연결 상태 확인: 'c' 키")
        print("아두이노 상태 읽기: 'a' 키")
        print("")
        print("* 조종기 스위치로 자율주행/수동 모드가 자동 전환됩니다 *")
        
        web_thread = threading.Thread(target=line_follower.run_web_server)
        web_thread.daemon = True
        web_thread.start()
        
        while True:
            command = input("명령 입력 (s: 시작, f: 강제시작, q: 종료, r: 재연결, c: 상태확인, a: 아두이노상태): ").strip().lower()
            
            if command == 's':
                line_follower.start()
            elif command == 'f':
                line_follower.start_manual()
            elif command == 'r':
                line_follower.reconnect_serial()
            elif command == 'a':
                # 아두이노 상태 즉시 읽기
                print("아두이노 상태 확인 중...")
                line_follower.read_arduino_status()
            elif command == 'c':
                print(f"시리얼 연결: {line_follower.serial_connected}")
                print(f"연결 안정성: {line_follower.connection_stable}")
                print(f"오류 횟수: {line_follower.connection_error_count}")
                print(f"아두이노 모드: {'자율주행' if line_follower.autonomous_mode else '수동'}")
                print(f"라인 추적 실행: {line_follower.running}")
                print(f"수동 오버라이드: {line_follower.manual_override}")
                print(f"아두이노 상태: {line_follower.arduino_status}")
                if line_follower.arduino:
                    print(f"포트: {line_follower.arduino.port}")
                    print(f"포트 열림: {line_follower.arduino.is_open}")
                    print(f"입력 버퍼: {line_follower.arduino.in_waiting} bytes")
            elif command == 'q':
                break
            else:
                print("알 수 없는 명령입니다.")
    
    except KeyboardInterrupt:
        print("프로그램 중단")
    finally:
        line_follower.cleanup()

if __name__ == '__main__':
    main()