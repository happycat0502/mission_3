#include <Arduino.h>
#include <TaskScheduler.h>
#include "PinChangeInterrupt.h"

#define PWM_PINS_START 6  // 첫 번째 출력 핀 (5,6,7번 핀 사용)
#define RC_CH2_PIN 2    // RC 수신기 채널 2 (전후진 제어)
#define RC_CH4_PIN 3    // RC 수신기 채널 4 (방향 제어)
#define NUM_CHANNELS 2
#define MIN_PULSE 1000    // 최소 펄스 폭(μs)
#define MAX_PULSE 2000    // 최대 펄스 폭(μs)
#define FRAME_INTERVAL 20000  // 50Hz 주파수
uint16_t outputPulseWidths[NUM_CHANNELS] = {1500, 1500};
uint16_t inputPulseWidths[NUM_CHANNELS] = {1500, 1500};
bool channelActive[NUM_CHANNELS] = {true, true};
volatile uint32_t startTime[2] = {0, 0};
volatile uint8_t currentChannel = 0;
volatile uint32_t lastFrameStart = 0;

Scheduler runner;

void printStatus();
void setPulseWidth(uint8_t ch, uint16_t width);
void updateState();

Task taskUpdate(50, TASK_FOREVER, &updateState);
Task taskPrint(1000, TASK_FOREVER, &printStatus);


 void ch2Change() { 
   bool state = digitalRead(RC_CH2_PIN);
   if (state) startTime[0] = micros();   // 신호가 HIGH로 변경될 때 타이머 시작
   else inputPulseWidths[0] = micros() - startTime[0];  // 신호가 LOW로 변경될 때 펄스 폭 계산
 }
 

 void ch4Change() { 
   bool state = digitalRead(RC_CH4_PIN);
   if (state) startTime[1] = micros();   // 신호가 HIGH로 변경될 때 타이머 시작
   else inputPulseWidths[1] = micros() - startTime[1];  // 신호가 LOW로 변경될 때 펄스 폭 계산
 }

void timerISR() {
  static bool pulseHigh = false;
  
  if (!pulseHigh) {
    // 펄스 시작
    if (channelActive[currentChannel]) {
      digitalWrite(PWM_PINS_START + currentChannel, HIGH);
    }
    pulseHigh = true;
    OCR1A = TCNT1 + (outputPulseWidths[currentChannel] * 2);
  } else {
    // 펄스 종료
    if (channelActive[currentChannel]) {
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

void updateState(){
  setPulseWidth(0, map(inputPulseWidths[0], MIN_PULSE, MAX_PULSE, 1610, 1460));
  setPulseWidth(1, map(inputPulseWidths[1], MIN_PULSE, MAX_PULSE, MAX_PULSE, MIN_PULSE));
}

void printStatus() {
  Serial.print("input PWM: ");
  for (int i = 0; i < NUM_CHANNELS; i++) {
    Serial.print("CH");
    Serial.print(i+1);
    Serial.print("=");
    Serial.print(inputPulseWidths[i]);
    if (i < NUM_CHANNELS-1) Serial.print(" ");
  }
  Serial.println();
  Serial.print("output PWM: ");
  for (int i = 0; i < NUM_CHANNELS; i++) {
    Serial.print("CH");
    Serial.print(i+1);
    Serial.print("=");
    Serial.print(outputPulseWidths[i]);
    if (i < NUM_CHANNELS-1) Serial.print(" ");
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
  
  pinMode(RC_CH2_PIN, INPUT);
  pinMode(RC_CH4_PIN, INPUT);
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
  runner.addTask(taskUpdate);
  runner.addTask(taskPrint);
  taskUpdate.enable();
  taskPrint.enable();
  
  lastFrameStart = micros();
  Serial.println("PWM 생성기 준비 완료");
}

void loop() {
  runner.execute();
  
}

ISR(TIMER1_COMPA_vect) {
  timerISR();
}