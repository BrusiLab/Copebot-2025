#include "lidar.h"
#include "comunicazione.h"
#include "twoSteppers.h"

#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#include "Wire.h" 

Lidar lidar = Lidar();
Comunicazione arduino = Comunicazione();
Robot robot = Robot(); 
Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver();
Servo servo_inclinatore; //servo piccolo diverso (non attaccato a driver perché usa meno corrente)

// posizione da 0 a 4095
#define angolo_min 125
#define angolo_mid 375 //90 gradi
#define angolo_max 600

#define ferma_servo       2 //interrompi corrente

#define pin_inclinatore   4 //pin servo che inclina blocchi
#define servo_L           0 //pos servo che raccoglie blocchi
#define servo_ruota       1 //pos servo che sposta blocchi
#define servo_bodyguard   2 //pos servo che contiene blocchi

#define kfcaperto 5
#define kfcchiuso 6

String ricevuto = "";
int velocita_vai = 2000;
int velocita_gira = 2000;
int angolo_L = 0;
int fine = 0;

void setup() {

  Serial.begin(9600);
  while (!Serial) {}

  pinMode(LED_BUILTIN, OUTPUT); 
  pinMode(kfcchiuso, INPUT);
  pinMode(ferma_servo, OUTPUT);

  robot.set();

  servo.begin();
  servo.setPWMFreq(60); //da 24 a 1600

  servo_inclinatore.attach(pin_inclinatore);
}

void apri_L() {

  digitalWrite(ferma_servo, LOW); //attiva corrente

  while (digitalRead(kfcaperto) == LOW && angolo_L < 4095) {  //finché non arrivi in fondo e non hai raggiunto max apertura
    servo.setPWM(servo_L, 0, angolo_L);
    angolo_L++;
  }

  digitalWrite(ferma_servo, HIGH);  //blocca corrente
}

void chiudi_L(int posizione) {  

  digitalWrite(ferma_servo, LOW);

  if (posizione == 0) {   //posizione = 0 se fino in fondo
    fine = 0;
  } else {                //posizione = 1 se fino a metà cerchio
    fine = 300;           //non si chiude tutta --> blocchi sul cerchio esterno
  }

  while (digitalRead(kfcchiuso) == LOW && angolo_L > fine) {  //finché non arrivi a fine e non hai raggiunto min apertura
    servo.setPWM(servo_L, 0, angolo_L);
    angolo_L--;
  }

  digitalWrite(ferma_servo, HIGH);
}

void loop() {
  
  if (lidar.misura() <= 5){

    arduino.invia("rileva");
    ricevuto = arduino.ricevi();

  }

  if(ricevuto == "rosso"){
    apri_L();
    robot.vai(10, 1000, "avanti", "off");
    //inclina
    chiudi_L(0);
  }

}


