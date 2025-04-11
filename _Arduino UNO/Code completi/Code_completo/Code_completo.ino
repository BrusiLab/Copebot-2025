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

//----------------------------------------------------------------
//                            SERVO
//----------------------------------------------------------------

//Direzione per servo sbloccato
#define orario 150      
#define fermo 385       
#define antiorario 650 

#define ferma_servo 2      //interrompi corrente

#define pin_inclinatore 3  //pin servo piccolo che inclina blocchi
#define servo_L 0          //pos servo sbloccato che raccoglie blocchi
#define servo_ruota 1      //pos servo che sposta blocchi
#define servo_bodyguard 2  //pos servo che contiene blocchi

#define kfcaperto 4        //fine corsa per L aperta
#define kfcchiuso 5        //fine corsa per L chiusa

String ricevuto = "";
int velocita_vai = 2000;
int velocita_gira = 2000;
int angolo_L = 0;
int fine = 0;

void setup() {

  //Seriale
  Serial.begin(9600);
  while (!Serial) {}

  pinMode(LED_BUILTIN, OUTPUT); 

  robot.set();

  //servo
  servo.begin();
  servo.setPWMFreq(60);  //da 24 a 1600
  servo_inclinatore.attach(pin_inclinatore);

  pinMode(kfcchiuso, INPUT);
  pinMode(kfcaperto, INPUT);
  pinMode(ferma_servo, OUTPUT);

}

void blocco(){
  
  if (lidar.misura() <= 5){

    arduino.invia("rileva");
    ricevuto = arduino.ricevi();

  }

  if(ricevuto == "rosso"){
    raccogli_blocco();
  }
}

void loop() {
  
  robot.vai(400, velocita_vai, "avanti", "on");
  robot.gira(90, velocita_gira, "destra");
  robot.vai(900, velocita_vai, "avanti", "on");
  robot.gira(90, velocita_gira, "sinistra");
  robot.vai(400, velocita_vai, "avanti", "on");
  robot.vai_blocchi(950, velocita_vai, "avanti", "on");
  robot.gira(90, velocita_gira, "destra");
  robot.vai(350, velocita_vai, "avanti", "on");
  robot.gira(90, velocita_gira, "sinistra");

  scarica_blocchi();

}


