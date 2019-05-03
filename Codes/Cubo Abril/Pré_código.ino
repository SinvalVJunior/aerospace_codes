#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include "TimerOne.h"
#include<Aerospace.h>


#define PRESSAONIVELDOMAR_HPA (1013.25)

SoftwareSerial serial1(3, 2); // RX, TX
SoftwareSerial serialGSM(10, 11); // RX, TX

bool temSMS = false;
String dataHoraSMS;
String mensagemSMS;
String comandoGSM = "";
String ultimoGSM = "";

Aerospace barometro;

int lastTime;
int barreira;
String telefone ="01531998184049"; 

bool callStatus = false;

void leGSM();
void enviaSMS(String telefone, String mensagem);
void configuraGSM();

TinyGPS gps1;

void setup() {
  Serial.begin(9600);
  serialGSM.begin(9600); 

  Serial.println("Sketch Iniciado!");
  serialGSM.listen();
  configuraGSM();
  
   serial1.begin(9600);
   Serial.println("Aguardando pelo sinal dos satelites...");
   
   barreira=0;
   serial1.listen();
   
}

void loop() {

  //--------------------GPS---------------------
  bool recebido = false;
  bool teste = false;
  
  while (serial1.available() && !teste) {
    //Serial.println("Entrou");
     char cIn = serial1.read();
     recebido = gps1.encode(cIn);
      //Serial.print(cIn);
 if (recebido) {
     Serial.println("----------------------------------------");
     teste=true;
     //Latitude e Longitude
     float latitude, longitude;
     unsigned long idadeInfo;
     gps1.f_get_position(&latitude, &longitude, &idadeInfo);     
/*
   if (latitude != TinyGPS::GPS_INVALID_F_ANGLE) {
        Serial.print("Latitude: ");
        Serial.println(float(latitude) , 6);
     }

     if (longitude != TinyGPS::GPS_INVALID_F_ANGLE) {
        Serial.print("Longitude: ");
        Serial.println(float(longitude), 6);
     }

     if (idadeInfo != TinyGPS::GPS_INVALID_AGE) {
        Serial.print("Idade da Informacao (ms): ");
        Serial.println(idadeInfo);
     }
*/

     //Dia e Hora
     int ano;
     byte mes, dia, hora, minuto, segundo, centesimo;
     long date,tie,age;
     gps1.crack_datetime(&ano, &mes, &dia, &hora, &minuto, &segundo, &centesimo, &idadeInfo);
    
/*
     Serial.print("Data (GMT): ");
     Serial.print(dia);
     Serial.print("/");
     Serial.print(mes);
     Serial.print("/");
     Serial.println(ano);

     Serial.print("Horario (GMT): ");
     Serial.print(hora);
     Serial.print(":");
     Serial.print(minuto);
     Serial.print(":");
     Serial.print(segundo);
     Serial.print(":");
     Serial.println(centesimo);
*/

     //altitude
     float altitudeGPS;
     altitudeGPS = gps1.f_altitude();
/*
     if ((altitudeGPS != TinyGPS::GPS_INVALID_ALTITUDE) && (altitudeGPS != 1000000)) {
        Serial.print("Altitude (cm): ");
        Serial.println(altitudeGPS);
     }
*/

     //velocidade
     float velocidade;
     //velocidade = gps1.speed();        //nós
     velocidade = gps1.f_speed_kmph();   //km/h
     //velocidade = gps1.f_speed_mph();  //milha/h
     //velocidade = gps1.f_speed_mps();  //milha/segundo

     //Serial.print("Velocidade (km/h): ");
     //Serial.println(velocidade, 2);  //Conversão de Nós para Km/h



     //sentito (em centesima de graus)
     unsigned long sentido;
     sentido = gps1.course();

     //Serial.print("Sentido (grau): ");
     //Serial.println(float(sentido) / 100, 2);

     
     
     //satelites e precisão
     unsigned short satelites;
     unsigned long precisao;
     satelites = gps1.satellites();
     precisao =  gps1.hdop();
/*
     if (satelites != TinyGPS::GPS_INVALID_SATELLITES) {
        Serial.print("Satelites: ");
        Serial.println(satelites);
     }

     if (precisao != TinyGPS::GPS_INVALID_HDOP) {
        Serial.print("Precisao (centesimos de segundo): ");
        Serial.println(precisao);
     }

*/

     //float distancia_entre;
     //distancia_entre = gps1.distance_between(lat1, long1, lat2, long2);
      int teste=dia;
     //float sentido_para;
     //sentido_para = gps1.course_to(lat1, long1, lat2, long2);
     /*"Data: "+dia+"/"+mes+"/"+ano+"\n"+
                   "Hora: "+hora+":"+minuto+":"+segundo;
     */



  int temperatura=barometro.BME_readTemperature();
  int umidade=barometro.BME_readHumidity();
  int pressao=barometro.BME_readPressure();
  int altitudeBME=barometro.BME_readAltitude(PRESSAONIVELDOMAR_HPA);
  Serial.println(umidade);
    char buffer[7];
    String lat = dtostrf(latitude, 1, 6, buffer);
    String longi = dtostrf(longitude, 1, 6, buffer);

     String msg="GPS funcionando\n Latitude:"+lat;//mudar para 6 caracteres depois da virgula
     msg=msg+"\n Longitude:"+longi+//mudar para 6 caracteres depois da virgula
                          "\n Idade da mensagem:"+idadeInfo+
                          "\n Altitude:"+altitudeGPS/1000+
                          "\n Velocidade:"+velocidade;
     
     serialGSM.listen();

     if(millis()>= 60000*barreira) {
      
      Serial.println(msg);
      enviaSMS(msg);
      barreira++;
      }
      serial1.listen();
  }
 // delay(2000);
   }
   
}
void enviaSMS(String mensagem) {
  
  serialGSM.print("AT+CMGS=\"" + telefone + "\"\n");
  serialGSM.print(mensagem + "\n");
  serialGSM.print((char)26); 
}

void configuraGSM() {
   serialGSM.print("AT+CMGF=1\n;AT+CNMI=2,2,0,0,0\n;ATX4\n;AT+COLP=1\n"); 
}
