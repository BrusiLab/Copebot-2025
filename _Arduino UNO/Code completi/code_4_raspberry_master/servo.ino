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
#define servo_inclinatore 3  //pos servo piccolo che inclina blocchi

long t0 = 0;
float gradi = 0;

// ========================================================
// ==================== FUNZIONI SERVO ====================
// ========================================================

// Inzializzazione comunicazione
void servo_set() {
  servo.begin();
  servo.setPWMFreq(60);  //da 24 a 1600
  delay(100);
}

float converti(int angolo) {
  gradi = map(angolo, 125, 650, 0, 180);
  return gradi;
}

void apri_L() {
  digitalWrite(ferma_servo, LOW);          //attiva corrente
  while (digitalRead(kfcaperto) == LOW) {  //finché non arrivi in fondo apri L
    servo.setPWM(servo_L, 0, orario);
  }
  servo.setPWM(servo_L, 0, fermo);  //ferma il servo sbloccato
  delay(100);
  digitalWrite(ferma_servo, HIGH);  //blocca corrente
}

void chiudi_L() {
  digitalWrite(ferma_servo, LOW);          //attiva corrente
  while (digitalRead(kfcchiuso) == LOW) {  //finché non arrivi in fondo chiudi L
    servo.setPWM(servo_L, 0, antiorario);
  }
  servo.setPWM(servo_L, 0, fermo);  //ferma il servo sbloccato
  delay(100);
  digitalWrite(ferma_servo, HIGH);  //blocca corrente
}

void chiudi_leva() {
  digitalWrite(ferma_servo, LOW);  //blocca corrente
  servo.setPWM(servo_inclinatore, 0, chiuso);
  delay(100);
  digitalWrite(ferma_servo, HIGH);  //blocca corrente
}

void apri_leva() {
  digitalWrite(ferma_servo, LOW);  //blocca corrente
  servo.setPWM(servo_inclinatore, 0, aperto_inclinatore);
  delay(100);
  digitalWrite(ferma_servo, HIGH);  //blocca corrente
}

void scarica() {
  digitalWrite(ferma_servo, LOW);  //blocca corrente

  servo.setPWM(servo_bodyguard, 0, retto);  //apri paratia
  delay(1000);

  for (i = 650; i >= 300; i--) {
    servo.setPWM(servo_ruota, 0, i);  //scarica
    delay(8);
  }

  delay(500);
  servo.setPWM(servo_ruota, 0, 650);  //chiudi ruota
  delay(500);
  servo.setPWM(servo_bodyguard, 0, chiuso);  //chiudi paratia
  blocchi_raccolti = 0;
  delay(500);

  digitalWrite(ferma_servo, HIGH);  //blocca corrente
}

void posiziona(int numero_blocchi) {
  digitalWrite(ferma_servo, LOW);

  if (numero_blocchi <= 3) {
    for (i = 650; i >= 550; i--) {
      servo.setPWM(servo_ruota, 0, i);  //scarica
      delay(5);
    }
    servo.setPWM(servo_ruota, 0, 650);  //chiudi la ruota
    delay(500);
  }

  digitalWrite(ferma_servo, HIGH);
}