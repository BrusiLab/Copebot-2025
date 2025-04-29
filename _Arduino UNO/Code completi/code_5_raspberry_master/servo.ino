// Direzione per servo sbloccato
#define orario 150
#define fermo 385
#define antiorario 650

// Angoli per servo normali
#define chiuso 150  //0
#define retto 385   //90
#define aperto_inclinatore 500
#define aperto 650  //180

// Servo indirizzi
#define servo_L 0            //pos servo sbloccato che raccoglie blocchi
#define servo_ruota 1        //pos servo che sposta blocchi
#define servo_bodyguard 2    //pos servo che contiene blocchi
#define servo_inclinatore 3  //pin servo piccolo che inclina blocchi

long t0 = 0;


// ========================================================
// ==================== FUNZIONI SERVO ====================
// ========================================================

// Inzializzazione comunicazione
void servo_set(){
  servo.begin();
  servo.setPWMFreq(60);  //da 24 a 1600
  delay(100);
}


void apri_L() {
  digitalWrite(ferma_servo, LOW);          //attiva corrente
  while (digitalRead(kfcaperto) == LOW) {  //finché non arrivi in fondo apri L
    servo.setPWM(servo_L, 0, orario);
  }
  servo.setPWM(servo_L, 0, fermo);  //ferma il servo sbloccato
  digitalWrite(ferma_servo, HIGH);  //blocca corrente
}


void chiudi_L(int posizione = 1) {
  digitalWrite(ferma_servo, LOW);            //attiva corrente
  if (posizione == 0) {                      //posizione = 0 se fino in fondo
    while (digitalRead(kfcchiuso) == LOW) {  //finché non arrivi in fondo chiudi L
      servo.setPWM(servo_L, 0, antiorario);
    }
    servo.setPWM(servo_L, 0, fermo);  //ferma il servo sbloccato
  } else {                            //posizione = 1 se lasci blocco esternamente
    t0 = millis();
    while (millis() - t0 <= 2500) {  //tempo (stima) per raggiungere posizione
      servo.setPWM(servo_L, 0, antiorario);
    }
    servo.setPWM(servo_L, 0, fermo);  //ferma il servo sbloccato
  }
  digitalWrite(ferma_servo, HIGH);  //blocca corrente
}


void chiudi_leva() {
  digitalWrite(ferma_servo, LOW);  //blocca corrente
  servo.setPWM(servo_inclinatore, 0, chiuso);
  delay(1000);
  digitalWrite(ferma_servo, HIGH);  //blocca corrente
}


void apri_leva() {
  digitalWrite(ferma_servo, LOW);  //blocca corrente
  servo.setPWM(servo_inclinatore, 0, aperto_inclinatore);
  delay(1000);
  digitalWrite(ferma_servo, HIGH);  //blocca corrente
}