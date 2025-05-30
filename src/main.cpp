#include <TaskScheduler.h>
#include "PinChangeInterrupt.h"

#define PWM_PINS_START 6  // 첫 번째 출력 핀 (6,7번 핀 사용)
#define RC_CH2_PIN 2      // RC 수신기 채널 2 (전후진 제어) - 수동 모드용
#define RC_CH4_PIN 3      // RC 수신기 채널 4 (방향 제어) - 수동 모드용
#define MODE_PIN 4        // 모드 스위치 핀 (HIGH: 자율주행, LOW: 수동)
#define NUM_CHANNELS 2
#define MIN_PULSE 1000    // 최소 펄스 폭(μs)
#define MAX_PULSE 2000    // 최대 펄스 폭(μs)
#define FRAME_INTERVAL 20000  // 50Hz 주파수

// PWM 출력 값
uint16_t outputPulseWidths[NUM_CHANNELS] = {1500, 1500}; // 속도, 조향
uint16_t rcPulseWidths[NUM_CHANNELS] = {1500, 1500};     // RC 수신 값
uint16_t serialPulseWidths[NUM_CHANNELS] = {1500, 1500}; // 시리얼 수신 값

// RC 수신기 관련 변수
volatile uint32_t rcStartTime[2] = {0, 0};
bool rcChannelActive[NUM_CHANNELS] = {true, true};

// PWM 생성 관련 변수
volatile uint8_t currentChannel = 0;
volatile uint32_t lastFrameStart = 0;

// 모드 관리
bool autonomousMode = false;
bool lastModeState = false;

// 시리얼 통신 관련
String serialBuffer = "";
unsigned long lastSerialUpdate = 0;
const unsigned long SERIAL_TIMEOUT = 1000; // 1초 시리얼 타임아웃

// 작업 스케줄러
Scheduler runner;

// 함수 선언
void printStatus();
void setPulseWidth(uint8_t ch, uint16_t width);
void updateMode();
void processSerialData();
void handleRCInput();
void parseSerialCommand(String command);

Task taskMode(50, TASK_FOREVER, &updateMode);
Task taskSerial(20, TASK_FOREVER, &processSerialData);
Task taskPrint(1000, TASK_FOREVER, &printStatus);

// RC 수신기 인터럽트 핸들러
void ch2Change() { 
  bool state = digitalRead(RC_CH2_PIN);
  if (state) rcStartTime[0] = micros();
  else {
    uint32_t pulseWidth = micros() - rcStartTime[0];
    if (pulseWidth >= MIN_PULSE && pulseWidth <= MAX_PULSE) {
      rcPulseWidths[0] = pulseWidth;
    }
  }
}

void ch4Change() { 
  bool state = digitalRead(RC_CH4_PIN);
  if (state) rcStartTime[1] = micros();
  else {
    uint32_t pulseWidth = micros() - rcStartTime[1];
    if (pulseWidth >= MIN_PULSE && pulseWidth <= MAX_PULSE) {
      rcPulseWidths[1] = pulseWidth;
    }
  }
}

// 타이머 인터럽트 핸들러
void timerISR() {
  static bool pulseHigh = false;
  
  if (!pulseHigh) {
    // 펄스 시작
    if (rcChannelActive[currentChannel]) {
      digitalWrite(PWM_PINS_START + currentChannel, HIGH);
    }
    pulseHigh = true;
    OCR1A = TCNT1 + (outputPulseWidths[currentChannel] * 2);
  } else {
    // 펄스 종료
    if (rcChannelActive[currentChannel]) {
      digitalWrite(PWM_PINS_START + currentChannel, LOW);
    }
    pulseHigh = false;
    currentChannel = (currentChannel + 1) % NUM_CHANNELS;
    
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

void updateMode() {
  // 모드 스위치 상태 읽기
  bool currentModeState = digitalRead(MODE_PIN);
  
  if (currentModeState != lastModeState) {
    autonomousMode = currentModeState;
    if (autonomousMode) {
      Serial.println("자율주행 모드 활성화");
    } else {
      Serial.println("수동 모드 활성화");
      // 수동 모드로 전환 시 안전을 위해 정지
      outputPulseWidths[0] = 1500;
      outputPulseWidths[1] = 1500;
    }
    lastModeState = currentModeState;
  }
  
  // 모드에 따른 출력 값 설정
  if (autonomousMode) {
    // 자율주행 모드: 시리얼 데이터 사용
    if (millis() - lastSerialUpdate < SERIAL_TIMEOUT) {
      outputPulseWidths[0] = serialPulseWidths[0]; // 속도
      outputPulseWidths[1] = serialPulseWidths[1]; // 조향
    } else {
      // 시리얼 통신 타임아웃 시 안전 정지
      outputPulseWidths[0] = 1500;
      outputPulseWidths[1] = 1500;
    }
  } else {
    // 수동 모드: RC 신호 사용
    outputPulseWidths[0] = map(rcPulseWidths[0], MIN_PULSE, MAX_PULSE, 1610, 1460);
    outputPulseWidths[1] = map(rcPulseWidths[1], MIN_PULSE, MAX_PULSE, MAX_PULSE, MIN_PULSE);
  }
}

void processSerialData() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        parseSerialCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
    }
  }
}

void parseSerialCommand(String command) {
  // 형식: "SPEED,STEERING" (예: "1550,1600")
  int commaIndex = command.indexOf(',');
  
  if (commaIndex > 0) {
    String speedStr = command.substring(0, commaIndex);
    String steeringStr = command.substring(commaIndex + 1);
    
    uint16_t speed = speedStr.toInt();
    uint16_t steering = steeringStr.toInt();
    
    // 유효성 검사
    if (speed >= MIN_PULSE && speed <= MAX_PULSE && 
        steering >= MIN_PULSE && steering <= MAX_PULSE) {
      serialPulseWidths[0] = speed;
      serialPulseWidths[1] = steering;
      lastSerialUpdate = millis();
      
      // 디버그 출력
      Serial.print("수신: 속도=");
      Serial.print(speed);
      Serial.print(", 조향=");
      Serial.println(steering);
    } else {
      Serial.println("잘못된 PWM 값");
    }
  } else {
    Serial.println("잘못된 명령 형식");
  }
}

void printStatus() {
  Serial.print("모드: ");
  Serial.print(autonomousMode ? "자율주행" : "수동");
  Serial.print(" | 출력 PWM: 속도=");
  Serial.print(outputPulseWidths[0]);
  Serial.print(", 조향=");
  Serial.print(outputPulseWidths[1]);
  
  if (autonomousMode) {
    Serial.print(" | 시리얼 타임아웃: ");
    Serial.print(millis() - lastSerialUpdate < SERIAL_TIMEOUT ? "OK" : "TIMEOUT");
  } else {
    Serial.print(" | RC 입력: 속도=");
    Serial.print(rcPulseWidths[0]);
    Serial.print(", 조향=");
    Serial.print(rcPulseWidths[1]);
  }
  Serial.println();
}

void setPulseWidth(uint8_t ch, uint16_t width) {
  if (ch < NUM_CHANNELS && width >= MIN_PULSE && width <= MAX_PULSE) {
    outputPulseWidths[ch] = width;
  }
}

void setup() {
  Serial.begin(9600);
  
  // 핀 설정
  pinMode(RC_CH2_PIN, INPUT);
  pinMode(RC_CH4_PIN, INPUT);
  pinMode(MODE_PIN, INPUT_PULLUP);
  
  // RC 수신기 인터럽트 설정
  attachInterrupt(digitalPinToInterrupt(RC_CH2_PIN), ch2Change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH4_PIN), ch4Change, CHANGE);

  // 출력 핀 설정
  for (int i = 0; i < NUM_CHANNELS; i++) {
    pinMode(PWM_PINS_START + i, OUTPUT);
    digitalWrite(PWM_PINS_START + i, LOW);
  }
  
  // 타이머1 설정 (CTC 모드, 8분주)
  cli();
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11);
  TCNT1 = 0;
  OCR1A = 100;
  TIMSK1 = (1 << OCIE1A);
  sei();
  
  // 작업 스케줄러 초기화
  runner.init();
  runner.addTask(taskMode);
  runner.addTask(taskSerial);
  runner.addTask(taskPrint);
  
  taskMode.enable();
  taskSerial.enable();
  taskPrint.enable();
  
  lastFrameStart = micros();
  
  Serial.println("라인 추적 PWM 제어기 준비 완료");
  Serial.println("모드 스위치: HIGH=자율주행, LOW=수동");
  Serial.println("시리얼 명령 형식: SPEED,STEERING (예: 1550,1600)");
}

void loop() {
  runner.execute();
}

ISR(TIMER1_COMPA_vect) {
  timerISR();
}