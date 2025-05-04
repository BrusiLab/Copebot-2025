#define t1 200
int dt1, tp1 = 0;
unsigned long i = 0;

String ricevi() {
  while (Serial.available() == 0);  // Attendi che ci siano dati disponibili
  String data = Serial.readStringUntil('\n');  // Leggi fino a '\n'
  Serial.println("ok");  // Invia conferma a Raspberry Pi
  return data;
}


void invia(String line) {
  Serial.println(line);  // Invia il messaggio a Raspberry Pi
  while (true) {
    while (Serial.available() > 0) {
      String response = Serial.readStringUntil('\n');
      if (response == "ok") {
        return;  // Esci quando Raspberry Pi conferma la ricezione
      }
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