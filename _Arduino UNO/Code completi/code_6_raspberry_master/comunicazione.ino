#define t1 200
int dt1, tp1 = 0;
unsigned long i = 0;


String ricevi() {
  int end = millis() + 5000;
  while (Serial.available() == 0 || millis()<end);  // Attendi che ci siano dati disponibili
  String data = Serial.readStringUntil('\n');  // Leggi fino a '\n'
  Serial.println("ok");  // Invia conferma a Raspberry Pi
  return data;
}

//UMCM

//****************************

void invia(String line) {
  int end = millis() + 10000;
  Serial.println(line);  // Invia il messaggio a Raspberry Pi
  while (millis()<end) {
    //Serial.println(line);
    while (Serial.available() > 0 && millis()<end) {
      String response = Serial.readStringUntil('\n');
      if (response == "ok") {
        return;  // Esci quando Raspberry Pi conferma la ricezione
      } else if (response =="1" && line=="ricevi") {//STARTMdB
        for (int count=0; count<10;count++) {
          blink(8);
        }
        return;
      }//ENDMdB
    }
  }
}

void blink(int pin) {
  dt1 = millis() - tp1;
  if (dt1 > t1) {
    digitalWrite(pin, (i % 2 == 0) ? HIGH : LOW);
    i++;
    tp1 = millis();
  }

}
