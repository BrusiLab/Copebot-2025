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

Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver();

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
int fine = 0;

void interrompi() {
  if (millis() > 208000) {
    digitalWrite(ferma_servo, HIGH); //ferma tutti i servo
  }
}

void setup() {

  Serial.begin(9600);
  servo.begin();
  servo.setPWMFreq(60); //da 24 a 1600

  pinMode(kfcchiuso, INPUT);
  pinMode(ferma_servo, OUTPUT);

  delay(100);
}

void apri_L() {

  digitalWrite(ferma_servo, LOW);

  while (digitalRead(kfcaperto) == LOW && angolo_L < 4095) {
    servo.setPWM(servo_L, 0, angolo_L);
    angolo_L++;
  }

  digitalWrite(ferma_servo, HIGH);
}

void chiudi_L(int posizione) {

  digitalWrite(ferma_servo, LOW);

  if (posizione == 0) {
    fine = 0;
  } else {
    fine = 300; //non si chiude tutta --> blocchi sul cerchio esterno
  }

  while (digitalRead(kfcchiuso) == LOW && angolo_L > fine) {
    servo.setPWM(servo_L, 0, angolo_L);
    angolo_L--;
  }

  digitalWrite(ferma_servo, HIGH);
}

void loop() {

  digitalWrite(ferma_servo, LOW);

  servo.setPWM(servo_inclinatore, 0, angolo_max);
  delay(1000);
  servo.setPWM(servo_inclinatore, 0, angolo_min);
  delay(1000);

  digitalWrite(ferma_servo, HIGH);

  delay(1000);

}
