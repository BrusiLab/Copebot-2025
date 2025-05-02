#define velocitaBassa 300   //non meno di 300
#define velocitaAlta 2000   //usata dovunque si può usare (e non solo), MASSIMA 2500
#define velocitaMedia 1200
#define velocitaGiro 900    //meno di 1000
unsigned int avanzaCM = 0;



// Funzione di interruzione
void interrompiTutto() {
  if (millis() >= durata) {
    Serial.println("finito");
    while (true) {
    }
  }
}

void raccogli() {
  delay(100);
  apri_leva();
  delay(100);
  apri_L();
  delay(100);
  robot.vai(100, velocitaBassa, "avanti", "off");
  delay(100);
  chiudi_leva();
  delay(100);
  chiudi_L();
  delay(100);
  apri_leva();
  delay(100);
  blocchi_raccolti ++;
  posiziona(blocchi_raccolti);
}

void inizio() {
  robot.vai(50, velocitaBassa, "avanti", "off");
  //LETTURA FAKE
  robot.vai(250, velocitaAlta, "avanti", "on");
  robot.gira(90, velocitaGiro, "destra");
  robot.vai(900, velocitaAlta, "avanti", "on");
  robot.giraRuote(-90, velocitaGiro);
  robot.vai(400, velocitaAlta, "avanti", "on");
}


void avanza(int distanzaAvanza) {
  int distanzaPercorsa = 0;

  while (distanzaPercorsa <= distanzaAvanza) {
    robot.vai(10, velocitaBassa, "avanti", "off");
    distanzaPercorsa += 10;

    int misuraCM = lidar.misura();

    if (misuraCM < 7) {
      robot.vai(10, velocitaBassa, "avanti", "off");  // distanza per portare la fotocamera in posizione
      invia("rileva");
      String colore = ricevi();

      if (colore == "r" || colore == "g") {
        digitalWrite(LED, HIGH);
        //invia("nothing");
        raccogli();
        digitalWrite(LED, LOW);
        delay(50);
      }

      robot.vai(50, velocitaBassa, "avanti", "off");
      distanzaPercorsa += 60;
    }
    //torna su
  }
}


void gira1() {  //al termina del primo avanza, scarica sulla prima postazione rossa e porta il robot in posizione per il secondo avanza
  robot.giraRuote(90, velocitaGiro);
  robot.vai(350, velocitaMedia, "avanti", "on");
  robot.giraRuote(-90, velocitaGiro);
  robot.vai(100, velocitaMedia, "avanti", "off");
  scarica();  //SCARICA
  robot.vai(300, velocitaAlta, "avanti", "on");
  robot.giraRuote(-90, velocitaGiro);
  robot.vai(800, velocitaAlta, "avanti", "on");
  robot.gira(-90, velocitaGiro, "sinistra");
  robot.vai(450, velocitaAlta, "avanti", "on");
}


void gira2() {  //al termine del secondo avanza, scarica sulle altre due postazioni rosse, poi riporta il robot sulla base di partenza
  robot.gira(90, velocitaGiro, "destra");
  robot.vai(400, velocitaAlta, "avanti", "on");
  robot.giraRuote(-90, velocitaGiro);
  robot.vai(50, velocitaMedia, "avanti", "off");
  scarica();
  robot.vai(300, velocitaAlta, "avanti", "on");
  robot.giraRuote(-90, velocitaGiro);
  robot.vai(300, velocitaAlta, "avanti", "on");
  robot.giraRuote(90, velocitaGiro);
  robot.vai(300, velocitaAlta, "avanti", "on");
  robot.gira(-90, velocitaGiro, "desta");
  scarica();
  robot.vai(250, velocitaAlta, "indietro", "on");
  robot.giraRuote(-90, velocitaGiro);
  robot.vai(200, velocitaAlta, "indietro", "on");
}


void killer(){
  robot.vai(50, velocitaBassa, "avanti", "off");
  //LETTURA FAKE
  robot.vai(250, velocitaBassa, "avanti", "on");
  delay(50000);
  robot.vai(300, velocitaBassa, "indietro", "on");
}