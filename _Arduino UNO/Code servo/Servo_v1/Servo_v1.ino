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
#include <Servo.h>
#include <Wire.h>

Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver(); //dichiaro oggetto driver
Servo servo_inclinatore;                                   //servo piccolo diverso (non attaccato a driver perché usa meno corrente)

//Direzione per servo sbloccato
#define orario 150      
#define fermo 385       
#define antiorario 650 

//Angoli per servo normali
#define chiuso 150  //0
#define retto 385   //90 
#define aperto 650  //180

#define ferma_servo 2      //interrompi corrente

#define pin_inclinatore 4  //pin servo piccolo che inclina blocchi
#define servo_L 0          //pos servo sbloccato che raccoglie blocchi
#define servo_ruota 1      //pos servo che sposta blocchi
#define servo_bodyguard 2  //pos servo che contiene blocchi

#define kfcaperto 5        //fine corsa per L aperta
#define kfcchiuso 6        //fine corsa per L chiusa

long t0 = 0;

void interrompi() {        //ferma tutti i servo se scaduto tempo
  if (millis() > 208000) {
    digitalWrite(ferma_servo, HIGH);  
  }
}

void setup() {

  Serial.begin(9600);

  servo.begin();
  servo.setPWMFreq(60);  //da 24 a 1600

  servo_inclinatore.attach(pin_inclinatore);

  pinMode(kfcchiuso, INPUT);
  pinMode(kfcaperto, INPUT);
  pinMode(ferma_servo, OUTPUT);

  delay(100);
}

void apri_L() {

  digitalWrite(ferma_servo, LOW);           //attiva corrente

  while (digitalRead(kfcaperto) == LOW) {   //finché non arrivi in fondo apri L
    servo.setPWM(servo_L, 0, antiorario);
  }

  servo.setPWM(servo_L, 0, fermo);          //ferma il servo sbloccato

  digitalWrite(ferma_servo, HIGH);          //blocca corrente
}

void chiudi_L(int posizione = 1) {

  digitalWrite(ferma_servo, LOW);           //attiva corrente

  if (posizione == 0) {                     //posizione = 0 se fino in fondo
    
    while (digitalRead(kfcchiuso) == LOW) { //finché non arrivi in fondo chiudi L
      servo.setPWM(servo_L, 0, orario);
    }

    servo.setPWM(servo_L, 0, fermo);        //ferma il servo sbloccato

  } else {                                  //posizione = 1 se lasci blocco esternamente
    
    t0 = millis();
    
    while (millis()-t0 <= 2500) {           //tempo (stima) per raggiungere posizione
      servo.setPWM(servo_L, 0, orario);
    }

    servo.setPWM(servo_L, 0, fermo);        //ferma il servo sbloccato

  }

  digitalWrite(ferma_servo, HIGH);          //blocca corrente
}

void loop() {

  digitalWrite(ferma_servo, LOW);

  servo.setPWM(servo_ruota, 0, chiuso);
  delay(1000);
  servo.setPWM(servo_ruota, 0, retto);
  delay(1000);
  servo.setPWM(servo_ruota, 0, aperto);
  delay(1000);
  servo.setPWM(servo_ruota, 0, retto);
  delay(2500);

  digitalWrite(ferma_servo, HIGH);
}
