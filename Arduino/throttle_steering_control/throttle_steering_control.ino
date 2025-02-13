#include <SoftwareSerial.h>
#include <Servo.h>
#include <util/atomic.h>

// DC 모터 엔코더 핀
#define ENCODER_A 2
#define ENCODER_B 5

// DC 모터 핀
#define ENA 6
#define IN1 7
#define IN2 8

// 블루투스 관련 핀 (app inventor <-> arduino)
#define BTRX 12
#define BTTX 13
SoftwareSerial BTSerial(BTRX, BTTX);

// 정지 상태에서 forward/backward 눌렀을 때의 속도
#define INIT_MOTORSPEED 35
//#define INIT_MAX_MOTORSPEED 100


// 모터 속도 갱신 주기
#define UPDATE_INTERVAL 100


// throttle 관련 상수
#define NO_CONTROL 0
#define FORWARD 1
#define BACKWARD 2
#define EMERGENCYSTOP 3
#define BRAKE 4





// dc모터 엔코더 관련
long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;

// 필터링 관련
#define numReads 10
int velocityValues[numReads];
int velocitySum = 0;



// 서보모터 관련
Servo myServo;
int servo_value_init = 90;
int servo_value = 90;


// 라즈베리파이에 권한 위임시 1
int raspcode = 0;

byte curr_throttle_value = 0;
byte last_throttle_value = 0;
unsigned long last_throttle_time = 0;
int motor_speed = 0;
unsigned long current_time = 0;
signed char steering_value = 0;

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600);

  myServo.attach(10);
  myServo.write(90);

  pinMode(ENCODER_A,INPUT);
  pinMode(ENCODER_B,INPUT);
  pinMode(ENA,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A),readEncoder,RISING);

  EmergencyStopMotor();
  Serial.println("Turned on");
}

void loop() {
  if (Serial.available()) raspcode = 1;

  // 모터 엔코더 관련 코드
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
  }
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity = (pos-posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;
  //Serial.println(velocity);
  delay(30);

  
  // 필터링 관련 코드
  for(int k=0; k<numReads-1; k++){
    velocityValues[k] = velocityValues[k+1];
  }
  velocityValues[numReads-1] = (int)velocity;
  velocitySum = 0;
  for(int k=0; k<numReads; k++){
    velocitySum += velocityValues[k];
  }
  int velocityAverage = velocitySum / numReads;  


      
  // 앱이 조종
  if (raspcode == 0) {
    // 앱의 조종 값 갱신
    if(BTSerial.available() >= 2) {

      // 1로 시작하면 throttle 데이터
      if (BTSerial.read() == 1) {
        while (!BTSerial.available()); // 얘는 왜 있는거지?

        curr_throttle_value = BTSerial.read();
        //if (curr_throttle_value != last_throttle_value) {
        last_throttle_time = 0;
          //last_throttle_value = curr_throttle_value;
        //}
        Serial.print("Throttle is ");
        Serial.println(curr_throttle_value);
      } 
      
      // 0으로 시작하면 steering 데이터
      else {
        while (!BTSerial.available());
        steering_value = BTSerial.read();
        Serial.print("Steering is ");
        Serial.println(steering_value);
      }
    }

    current_time = millis();
    
    // throttle 구현 (앱)
    if (current_time - last_throttle_time > (unsigned long)UPDATE_INTERVAL) {
      last_throttle_time = current_time;
      if (curr_throttle_value == NO_CONTROL) {
        if (motor_speed >= 0) {
          motor_speed -= 1;
          if (motor_speed < 0) motor_speed = 0;
          driveMotor(motor_speed);
        } else {
          motor_speed += 1;
          if (motor_speed > 0) motor_speed = 0;              
        
          driveMotor(motor_speed);
        }
      } else if (curr_throttle_value == FORWARD) {
        motor_speed += 1;
        //if (motor_speed < 0) motor_speed += 1;
        if ((INIT_MOTORSPEED > motor_speed) && (motor_speed > -INIT_MOTORSPEED)) motor_speed = INIT_MOTORSPEED;
        
        if (motor_speed > 255) motor_speed = 255;
        driveMotor(motor_speed);
      } else if (curr_throttle_value == BACKWARD) {
        motor_speed -= 1;
        if ((-INIT_MOTORSPEED < motor_speed) && (motor_speed < INIT_MOTORSPEED) ) motor_speed = -INIT_MOTORSPEED;
        if (motor_speed < -255) motor_speed = -255;
        driveMotor(motor_speed);
      } else if (curr_throttle_value == EMERGENCYSTOP) {
        EmergencyStopMotor();
      } else if (curr_throttle_value == BRAKE){
        if (motor_speed > 0) {
          motor_speed -= 3;
          if (motor_speed < 0) motor_speed = 0;
        }
        if (motor_speed<0) motor_speed = min(motor_speed+3,0);
        driveMotor(motor_speed);
      }
    }

    // steering 구현
    myServo.write(90+steering_value/2);

  } 

  // 라즈베리파이에 조종 권한 위임
  else {
    if (Serial.available()) {
      byte code = Serial.read();
      Serial.print(code); 
      if (code == 255) {
        raspcode = 0;
        clearBluetoothBuffer();
        return;
      } else if (code == 1) {
        while(!Serial.available());
        curr_throttle_value = Serial.read();
        last_throttle_time = 0;
      } else {
        while(!Serial.available());
        steering_value = Serial.read();
      }
    }
    current_time = millis();
    if (current_time - last_throttle_time > (unsigned long)UPDATE_INTERVAL) {
      last_throttle_time = current_time;
      if (curr_throttle_value == NO_CONTROL) {
        if (motor_speed >= 0) {
          motor_speed -= 1;
          if (motor_speed < 0) motor_speed = 0;
          driveMotor(motor_speed);
        } else {
          motor_speed += 1;
          if (motor_speed > 0) motor_speed = 0;              
        
          driveMotor(motor_speed);
        }
      } else if (curr_throttle_value == FORWARD) {
        motor_speed += 1;
        if ((INIT_MOTORSPEED > motor_speed) && (motor_speed > -INIT_MOTORSPEED)) motor_speed = INIT_MOTORSPEED;
        if (motor_speed > 255) motor_speed = 255;
        driveMotor(motor_speed);
      } else if (curr_throttle_value == BACKWARD) {
        motor_speed -= 1;
        if ((-INIT_MOTORSPEED < motor_speed) && (motor_speed < INIT_MOTORSPEED)) motor_speed = -INIT_MOTORSPEED;
        if (motor_speed < -255) motor_speed = -255;
        driveMotor(motor_speed);
      } else if (curr_throttle_value == EMERGENCYSTOP) {
        EmergencyStopMotor();
      } else if (curr_throttle_value == BRAKE){
        if (motor_speed>0) motor_speed = max(motor_speed-3,0);
        if (motor_speed<0) motor_speed = min(motor_speed+3,0);
        driveMotor(motor_speed);
      }
    }

    myServo.write(steering_value);
  }
}


void driveMotor(int speed){
  // speed : -255 ~ 255
  if (speed >= 0) {
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite(ENA,speed);
  } else {
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    analogWrite(ENA, -speed);
  }
}


void EmergencyStopMotor() {
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,HIGH);
  analogWrite(ENA,0);
  motor_speed = 0;
}


void readEncoder(){
  int b = digitalRead(ENCODER_B);
  int increment = 0;
  if(b>0){
    pos_i--;
  }
  else{
    pos_i++;
  }
}

void clearBluetoothBuffer(){
  delay(10);
  while(BTSerial.available()){
    BTSerial.read();
  }
}