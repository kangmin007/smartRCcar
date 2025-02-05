#include <util/atomic.h>
#define ENCODER_A 2
#define ENCODER_B 5
#define ENA 6
#define IN1 7
#define IN2 8

// dc모터 엔코더 관련
long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(ENCODER_A,INPUT);
  pinMode(ENCODER_B,INPUT);
  pinMode(ENA,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A),readEncoder,RISING);

}

void loop() {
  // put your main code here, to run repeatedly:

      int pwr = 100/1.5*micros()/1.0e6;
      int dir = 1;
      setMotor(dir,pwr,ENA,IN1,IN2);

      int pos = 0;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        pos = pos_i;
      }
      long currT = micros();
      float deltaT = ((float) (currT-prevT))/1.0e6;
      float velocity1 = (pos-posPrev)/deltaT;
      posPrev = pos;
      prevT = currT;
      Serial.println(velocity1);
      delay(100);

      //Serial.println(digitalRead(ENCODER_B));
  
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1, HIGH);
    digitalWrite(in2,LOW);
  }
}

void readEncoder(){
  int b = digitalRead(ENCODER_B);
  int increment = 0;
  if(b>0){
    increment = 1;
  }
  else{
    increment = -1;
  }
  pos_i = pos_i + increment;

}