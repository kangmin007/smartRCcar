#include <SoftwareSerial.h>
#include <Servo.h>

#define ENA 6
#define IN1 7
#define IN2 8

#define INIT_MOTORSPEED 50
#define INIT_MAX_MOTORSPEED 100


// 모터 속도 갱신 주기
#define UPDATE_INTERVAL 100


// throttle 관련 상수
#define NO_CONTROL 0
#define FORWARD 1
#define BACKWARD 2
#define STOP 3

#define BTRX 12
#define BTTX 13

void driveMotor(int);
void stopMotor();
void steerWheel(int);

// 블루투스 모듈 설정
SoftwareSerial BTSerial(BTRX, BTTX);

Servo myServo;
int servo_value_init = 90;
int servo_value = 90;


int raspcode = 0;

byte curr_throttle_value = 0;
byte last_throttle_value = 0;
unsigned long last_throttle_time = 0;
int motor_speed = 0;
int steering = 127;
unsigned long current_time = 0;
signed char steering_value = 0;

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600);

  myServo.attach(3);
  myServo.write(90);

  pinMode(ENA,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);


  stopMotor();
  Serial.println("Turned on");
}

void loop() {
      if (Serial.available()) raspcode = 1;
      
      // 앱이 조종
      if (raspcode == 0) {
        // 앱의 조종 값 갱신
        if(BTSerial.available()) {

          // 1로 시작하면 throttle 데이터
          if (BTSerial.read() == 1) {
            while (!BTSerial.available()); // 얘는 왜 있는거지?

            curr_throttle_value = BTSerial.read();
            if (curr_throttle_value != last_throttle_value) {
              last_throttle_time = 0;
              last_throttle_value = curr_throttle_value;
            }
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
        
        // throttle 구현
        if (current_time - last_throttle_time > (unsigned long)UPDATE_INTERVAL) {
          last_throttle_time = current_time;
          if (last_throttle_value == NO_CONTROL) {
            if (motor_speed >= 0) {
              motor_speed -= 1;
              if (motor_speed < 0) motor_speed = 0;
              driveMotor(motor_speed);
            } else {
              motor_speed += 1;
              if (motor_speed > 0) motor_speed = 0;              
            
              driveMotor(motor_speed);
            }
          } else if (last_throttle_value == FORWARD) {
            motor_speed += 1;
            if (INIT_MOTORSPEED > motor_speed) motor_speed = INIT_MOTORSPEED;
            if (motor_speed > 255) motor_speed = 255;
            driveMotor(motor_speed);
          } else if (last_throttle_value == BACKWARD) {
            motor_speed -= 1;
            if (-INIT_MOTORSPEED < motor_speed) motor_speed = -INIT_MOTORSPEED;
            if (motor_speed < -255) motor_speed = -255;
            driveMotor(motor_speed);
          } else {
            stopMotor();
          }
        }

        // steering 구현
        myServo.write(90+steering_value/2);

      } 

      // 라즈베리파이에 조종 권한 위임
      else {
        if (Serial.available()) {
          curr_throttle_value = Serial.read();
          if ((int)curr_throttle_value == 255) {
            raspcode = 0;
            return;
          }
          while (!Serial.available());
          if (Serial.read() == 0) {
            Serial.print("Throttle is ");
            Serial.println(curr_throttle_value);
            motor_speed = (int)curr_throttle_value;
            driveMotor(motor_speed);
          } else {
            Serial.print("Throttle is ");
            Serial.println(-curr_throttle_value);
            motor_speed = -(int)curr_throttle_value;
            driveMotor(motor_speed);          
          }
          while (!Serial.available());
          steering_value = Serial.read();
          Serial.print("Steering is ");
          Serial.println(steering_value);
          steering = (int)steering_value;
        }
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


void stopMotor() {
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,HIGH);
  analogWrite(ENA,0);
  motor_speed = 0;
}

void steerWheel(int angle) {
  (void)0;
}