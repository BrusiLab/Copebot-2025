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

long t0 = 0;
int blocchi_raccolti = 0;

void interrompi_servo() {                   //ferma tutti i servo se scaduto tempo
  if (millis() > 208000) {
    digitalWrite(ferma_servo, HIGH);  
  }
}

float angolo(float ang){                    //trasforma angolo da gradi a valore da inserire nel driver
  
  if(ang == 90){
    ang = 385;
  } else {
    ang = map(ang, 0, 180, 150, 650);       //angoli di riferimento (150 = 0, 385 = 90, 650 = 180)
  }

  return ang;
}

void apri_L() { 

  digitalWrite(ferma_servo, LOW);           //attiva corrente

  while (digitalRead(kfcaperto) == LOW) {   //finché non arrivi in fondo apri L
    servo.setPWM(servo_L, 0, antiorario);
  }

  servo.setPWM(servo_L, 0, fermo);          //ferma il servo sbloccato

  digitalWrite(ferma_servo, HIGH);          //blocca corrente
}

void chiudi_L() {

  digitalWrite(ferma_servo, LOW);           //attiva corrente

  if (blocchi_raccolti > 4) {               //porta blocchi fino in fondo
    
    while (digitalRead(kfcchiuso) == LOW) { //finché non arrivi in fondo chiudi L
      servo.setPWM(servo_L, 0, orario);
    }

    servo.setPWM(servo_L, 0, fermo);        //ferma il servo sbloccato

  } else {                                  //lascia blocco esternamente
    
    t0 = millis();
    
    while (millis()-t0 <= 2500) {           //tempo (stima) per raggiungere posizione
      servo.setPWM(servo_L, 0, orario);
    }

    servo.setPWM(servo_L, 0, fermo);        //ferma il servo sbloccato

  }

  digitalWrite(ferma_servo, HIGH);          //blocca corrente
}

void raccogli_blocco(){
  
  blocchi_raccolti++;                       //incrementa numero blocchi raccolti
  
  servo_inclinatore.write(0);               //chiudi motori (per sicurezza)
  servo.setPWM(servo_ruota, 0, angolo(0));

  apri_L();
  robot.vai(130, velocita_vai, "avanti", "off");

  servo_inclinatore.write(100);             //inclina blocco

  chiudi_L();
  robot.vai(130, velocita_vai, "indietro", "off");

  servo.setPWM(servo_ruota, 0, angolo(90 - blocchi_raccolti*20));   //sposta blocco nella zona di raccolta
  servo.setPWM(servo_ruota, 0, angolo(0));                          //chiudi la ruota per non impedire passaggio

}

