#include <Arduino.h>
#include <TaskScheduler.h>

// 출력 핀 구성
#define STEERING_PIN 5  // 서보 모터 (조향)
#define THROTTLE_PIN 6  // ESC (속도)

// 상수
#define MIN_PULSE 1000    // 최소 펄스 폭(μs)
#define MAX_PULSE 2000    // 최대 펄스 폭(μs)
#define NEUTRAL 1500      // 중립 위치
#define FRAME_INTERVAL 20000  // 50Hz (20ms)

// 변수
uint16_t pulseWidths[2] = {NEUTRAL, NEUTRAL};  // [조향, 속도]
volatile uint8_t currentChannel = 0;
volatile uint32_t lastFrameStart = 0;
Scheduler runner;

// 함수 프로토타입
void processCommands();
void printStatus();

Task taskCommand(50, TASK_FOREVER, &processCommands);
Task taskPrint(1000, TASK_FOREVER, &printStatus);

// 타이머 인터럽트 핸들러
void timerISR() {
  static bool pulseHigh = false;
  
  if (!pulseHigh) {
    // 펄스 시작
    digitalWrite(currentChannel == 0 ? STEERING_PIN : THROTTLE_PIN, HIGH);
    pulseHigh = true;
    OCR1A = TCNT1 + (pulseWidths[currentChannel] * 2);
  } else {
    // 펄스 종료
    digitalWrite(currentChannel == 0 ? STEERING_PIN : THROTTLE_PIN, LOW);
    pulseHigh = false;
    currentChannel = (currentChannel + 1) % 2;
    
    if (currentChannel == 0) {
      uint32_t elapsed = micros() - lastFrameStart;
      if (elapsed < FRAME_INTERVAL) {
        OCR1A = TCNT1 + ((FRAME_INTERVAL - elapsed) * 2);
      } else {
        lastFrameStart = micros();
        OCR1A = TCNT1 + 100;
      }
    } else {
      OCR1A = TCNT1 + 200;
    }
  }
}

// 타이머 설정
void setupTimer() {
  cli();
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11);
  TCNT1 = 0;
  OCR1A = 100;
  TIMSK1 = (1 << OCIE1A);
  sei();
}

// 시리얼 명령 처리
void processCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    
    // S{값}T{값} 형식의 명령 파싱
    int sIndex = command.indexOf('S');
    int tIndex = command.indexOf('T');
    
    if (sIndex != -1 && tIndex != -1) {
      // 조향값 파싱
      String steeringStr = command.substring(sIndex + 1, tIndex);
      int steeringVal = steeringStr.toInt();
      
      // 속도값 파싱
      String throttleStr = command.substring(tIndex + 1);
      int throttleVal = throttleStr.toInt();
      
      // 값 검증 및 적용
      if (steeringVal >= MIN_PULSE && steeringVal <= MAX_PULSE) {
        pulseWidths[0] = steeringVal;
      }
      
      if (throttleVal >= MIN_PULSE && throttleVal <= MAX_PULSE) {
        pulseWidths[1] = throttleVal;
      }
    }
  }
}

void printStatus() {
  Serial.print("Steering: ");
  Serial.print(pulseWidths[0]);
  Serial.print("μs, Throttle: ");
  Serial.print(pulseWidths[1]);
  Serial.println("μs");
}

void setup() {
  Serial.begin(9600);
  Serial.println("RC Car Controller Starting...");
  
  // 출력 핀 설정
  pinMode(STEERING_PIN, OUTPUT);
  pinMode(THROTTLE_PIN, OUTPUT);
  digitalWrite(STEERING_PIN, LOW);
  digitalWrite(THROTTLE_PIN, LOW);
  
  // 타이머 설정
  setupTimer();
  
  // 작업 스케줄러 초기화
  runner.init();
  runner.addTask(taskCommand);
  runner.addTask(taskPrint);
  taskCommand.enable();
  taskPrint.enable();
  
  lastFrameStart = micros();
  
  Serial.println("RC Car Controller Ready");
}

void loop() {
  runner.execute();
}

ISR(TIMER1_COMPA_vect) {
  timerISR();
}