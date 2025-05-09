#ifndef lidar_h //max 11 cm
#define lidar_h

#define pinlidar 11 //pin a cui è collegato il sensore

int16_t tempo; //tempo di volo della luce
int distanza;  //distanza rilevata

class Lidar {

  public:
  Lidar();

  int misura();

};

Lidar::Lidar() {}

int Lidar::misura(){

  tempo = pulseIn(pinlidar, HIGH);    //rileva tempo di volo
  
  if(tempo > 1850 || tempo == 0){     //se intervallo non è ragionevole (1400)
    distanza = 100;
  } else {                            //se intervallo ragionevole
    distanza = (tempo - 1330) * 4/5;  //calcola la distanza (3/4)
    distanza = distanza/10;           //distanza in cm
  }

  return distanza;
  
}

#endif