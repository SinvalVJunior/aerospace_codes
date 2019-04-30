#include <Aerospace.h>
#include <SoftwareSerial.h>
#include "TimerOne.h"
//-----GPRS-----
String telefoneSMS = "992999511" ;//Definir o numero
 String msg;
//--------------------

SoftwareSerial serialGPS(10, 11); // TX, RX do modulo
SoftwareSerial serialGSM(3, 2); // RX, TX
Aerospace aerospace;
//-----------------------------------Barômetro---------------------------------
#define PRESSAONIVELDOMAR_HPA (1013.25)
unsigned long delayTime;

void configuraGSM();
void enviaSMS();

void setup() {
    Serial.begin(9600);
    //---------------------------------Barômetro------------------------------
    //Serial.println(F("Testando barometro..."));

    bool BME_status;

    //Verifica o sensor de barômetro
    BME_status = aerospace.BME_begin();
    if (!BME_status) {
      //Serial.println("Sensor de barômetro não encontrado, verifique a fiação!");
    }

    delayTime = 1000;
   // Serial.println();
    //------------------GPRS-----------------------------
    serialGSM.begin(9600); 
    Serial.println("Sketch Iniciado!");
    serialGSM.listen();
    configuraGSM(); //funcao para receber sms
    /*----------------------------------GPS----------------------------------*/
    serialGPS.begin(19200);
    Serial.println("Aguardando sinal dos satélites...");

    
   
  //-------------------------Timer--------------------------------------
  //Timer1.initialize(500000); // Inicializa o Timer1 e configura para um período de 0,5 segundos //acell
  //Timer1.attachInterrupt(enviaSMS); // Configura a função callback() como a função para ser chamada a cada interrupção do Timer1

}

void loop() {
  bool recebido = false;
  serialGPS.listen();
  char cIn = serialGPS.read();
  recebido = aerospace.GPS_encode(cIn);
     
 if (recebido) {
     Serial.println("----------------------------------------");
     //Latitude e Longitude
     long latitude, longitude;
     unsigned long idadeInfo;
     aerospace.GPS_get_position(&latitude, &longitude, &idadeInfo);     

     if (latitude != Aerospace::GPS_INVALID_F_ANGLE) {
       // Serial.print("Latitude: ");
        //Serial.println(float(latitude)/1000000, 6);
     }

     if (longitude != Aerospace::GPS_INVALID_F_ANGLE) {
        //Serial.print("Longitude: ");
        //Serial.println(float(longitude)/1000000, 6);
     }

     if (idadeInfo != Aerospace::GPS_INVALID_AGE) {
        //Serial.print("Idade da Informacao (ms): ");
        //Serial.println(idadeInfo);
     }


     //Dia e Hora
     int ano;
     byte mes, dia, hora, minuto, segundo, centesimo;
     long date,tie,age;
     aerospace.GPS_crack_datetime(&ano, &mes, &dia, &hora, &minuto, &segundo, &centesimo, &idadeInfo);
    
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
     altitudeGPS = aerospace.GPS_f_altitude();


  /* Problema na altitude:
     if ((altitudeGPS != Aerospace::GPS_INVALID_ALTITUDE) && altitudeGPS!=1000000) {
        //Serial.print("Altitude (cm): "); 
        //Serial.println(altitudeGPS);
     }
  */

  //------------------ Barometro--------------------------
  float pressao=aerospace.BME_readPressure() / 100.0F;
  float temperatura=aerospace.BME_readTemperature();
  float altitudeBar=aerospace.BME_readAltitude(PRESSAONIVELDOMAR_HPA);
  float humidade=aerospace.BME_readHumidity();
     //velocidade
     float velocidade;
     velocidade = aerospace.GPS_f_speed_kmph();   //km/h
     //Serial.print("Velocidade (km/h): ");
     //Serial.println(velocidade, 2);  //Conversão de Nós para Km/h
     msg=msg+"GPS funcionando\n Latitude:"+latitude;//mudar para 6 caracteres depois da virgula
     msg=msg+"\n Longitude:"+longitude+//mudar para 6 caracteres depois da virgula
                          "\n Idade da mensagem:"+idadeInfo+
                          "\n Altitude:"+altitudeGPS/1000+
                          "\n Velocidade:"+velocidade+
                          "\n Barometro:"
                          "\n Pressao:"+pressao+
                          "\n Temperatura:"+temperatura+
                          "\n Humidade:"+humidade;
                          delay(1000);

                          
     serialGSM.listen();
     if(serialGSM.available()) enviaSMS();                      
             delay(5000);             
    }
  
  
}

void configuraGSM() {
   serialGSM.print("AT+CMGF=1\n;AT+CNMI=2,2,0,0,0\n;ATX4\n;AT+COLP=1\n"); 
}
void enviaSMS() {
  serialGSM.print("AT+CMGS=\"" + telefoneSMS + "\"\n");
  serialGSM.print(msg + "\n");
  serialGSM.print((char)26); 
}
