#include <Aerospace.h>
#include <SoftwareSerial.h>
#include "TimerOne.h"
//-----GPRS-----
#define pinBotaoCall 12
String telefoneSMS;//Definir o numero
 String msg;
//--------------------

SoftwareSerial serialGPS(10, 11); // RX, TX

SoftwareSerial serialGSM(10, 11); // RX, TX
Aerospace aerospace;


void configuraGSM();
void enviaSMS(String telefone, String mensagem);

void setup() {
    Serial.begin(9600);
    //------------------GPRS-----------------------------
    serialGSM.begin(9600); 
    pinMode(pinBotaoCall, INPUT_PULLUP);
    Serial.println("Sketch Iniciado!");
    configuraGSM(); //funcao para receber sms
    /*----------------------------------GPS----------------------------------*/
    serialGPS.begin(9600);
    Serial.println("Aguardando sinal dos satélites...");

  //-------------------------Timer--------------------------------------
  Timer1.initialize(500000); // Inicializa o Timer1 e configura para um período de 0,5 segundos //acell
  Timer1.attachInterrupt(enviaSMS); // Configura a função callback() como a função para ser chamada a cada interrupção do Timer1

}

void loop() {
  bool recebido = false;
  if(!serialGPS.available())
          serialGPS.listen();
  
  else {
     char cIn = serialGPS.read();
     recebido = aerospace.GPS_encode(cIn);
     
 if (recebido) {
     Serial.println("----------------------------------------");
     //Latitude e Longitude
     long latitude, longitude;
     unsigned long idadeInfo;
     aerospace.GPS_get_position(&latitude, &longitude, &idadeInfo);     

     if (latitude != Aerospace::GPS_INVALID_F_ANGLE) {
        Serial.print("Latitude: ");
        Serial.println(float(latitude)/1000000, 6);
     }

     if (longitude != Aerospace::GPS_INVALID_F_ANGLE) {
        Serial.print("Longitude: ");
        Serial.println(float(longitude)/1000000, 6);
     }

     if (idadeInfo != Aerospace::GPS_INVALID_AGE) {
        Serial.print("Idade da Informacao (ms): ");
        Serial.println(idadeInfo);
     }


     //Dia e Hora
     int ano;
     byte mes, dia, hora, minuto, segundo, centesimo;
     long date,tie,age;
     aerospace.GPS_crack_datetime(&ano, &mes, &dia, &hora, &minuto, &segundo, &centesimo, &idadeInfo);
    

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


     //altitude
     float altitudeGPS;
     altitudeGPS = aerospace.GPS_f_altitude();

     if ((altitudeGPS != Aerospace::GPS_INVALID_ALTITUDE) && altitudeGPS!=1000000) {
        Serial.print("Altitude (cm): ");
        Serial.println(altitudeGPS);
     }


     //velocidade
     float velocidade;
     velocidade = aerospace.GPS_f_speed_kmph();   //km/h
     Serial.print("Velocidade (km/h): ");
     Serial.println(velocidade, 2);  //Conversão de Nós para Km/h
     msg="GPS funcionando\nLatitude:"+latitude/1000000;//mudar para 6 caracteres depois da virgula
     msg=msg+"\nLongitude:"+longitude/1000000+//mudar para 6 caracteres depois da virgula
                          "\nIdade da mensagem:"+idadeInfo+
                          "\nAltitude"+altitudeGPS/100+
                          "\nVelocidade"+velocidade;
                           
                          
    }
  }
  
}

void configuraGSM() {
   serialGSM.print("AT+CMGF=1\n;AT+CNMI=2,2,0,0,0\n;ATX4\n;AT+COLP=1\n"); 
}
void enviaSMS() {
  serialGSM.print("AT+CMGS=\"" + telefoneSMS + "\"\n");
  serialGSM.print(msg + "\n");
  //serialGSM.print((char)26); 
}
