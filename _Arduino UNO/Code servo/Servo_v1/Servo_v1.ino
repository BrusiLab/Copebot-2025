/*
  COLLEGAMENTO DRIVER:
  GND --> GND
  OE  --> PIN DIGITALE  (interrupt)
  SCL --> SCL
  SDA --> SDA
  VCC --> 5V

  COLLEGAMENTO SERVO:
  PWM --> PIN DIGITALE
  VCC --> ALIMENTAZIONE
  GND --> GND
*/

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver(0x40);

// posizione da 0 a 4095
#define angolo_min 125 
#define angolo_mid 375 //90 gradi
#define angolo_max 600 

#define ferma_servo 2

#define servo_inclinatore 0 //servo che inclina blocchi
#define servo_L           1 //servo che raccoglie blocchi
#define servo_ruota       2 //servo che sposta blocchi
#define servo_bodyguard   3 //servo che contiene blocchi

#define kfcaperto 5
#define kfcchiuso 6

int angolo_L = 0;

void interrompi(){
  if(millis() > 208000){
    digitalWrite(ferma_servo, HIGH);
  }
}

void setup() {

  Serial.begin(9600);
  servo.begin();
  servo.setOscillatorFrequency(27000000);
  servo.setPWMFreq(50); //da 24 a 1600

  pinMode(kfcchiuso, INPUT);
  pinMode(ferma_servo, OUTPUT);

  delay(10);
}

void apriL(){
  
  digitalWrite(ferma_servo, LOW);
  
  while (digitalRead(kfccaperto) == LOW) {
    pwm.setPWM(servo_L, 0, angolo_L);
    c
  }
  
  digitalWrite(ferma_servo, HIGH);
}

void chiudiL(){
  
  digitalWrite(ferma_servo, LOW);
  
  while (digitalRead(kfcchiuso) == LOW) {
    pwm.setPWM(servo_L, 0, angolo_L);
    if(angolo_L > 0){
      angolo_L--;
    }
  }
  
  digitalWrite(ferma_servo, HIGH);
}

void loop() {

  digitalWrite(ferma_servo, LOW);
  
  pwm.setPWM(servo_inclinatore, 0, angolo_max);
  delay(1000);
  pwm.setPWM(servo_inclinatore, 0, angolo_min);
  delay(1000);
  
  digitalWrite(ferma_servo, HIGH);

}
