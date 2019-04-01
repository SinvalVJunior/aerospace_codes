#include "Aerospace.h"
#include <SoftwareSerial.h>
#include "TimerOne.h"

SoftwareSerial serialGPS(10, 11); // RX, TX
Aerospace aerospace;

void setup()
{
   Serial.begin(9600);

   /*----------------------------------GPS----------------------------------*/
   serialGPS.begin(9600);
   Serial.println("Aguardando sinal dos satélites...");

   /*----------------------------------BME----------------------------------*/
   if(!aerospace.BME_begin()){
       Serial.println("Could not find a valid BME280 sensor");
   }


}

void loop()
{
   bool recebido = false;
   serialGPS.listen();

   while (serialGPS.available())
   {
      char cIn = serialGPS.read();
      recebido = aerospace.GPS_encode(cIn);

      if (recebido)
      {
         Serial.println("----------------------------------------");
         //Latitude e Longitude
         long latitude, longitude;
         unsigned long idadeInfo;
         aerospace.GPS_get_position(&latitude, &longitude, &idadeInfo);

         if (latitude != Aerospace::GPS_INVALID_F_ANGLE)
         {
            Serial.print("Latitude: ");
            Serial.println(float(latitude) / 1000000, 6);
         }

         if (longitude != Aerospace::GPS_INVALID_F_ANGLE)
         {
            Serial.print("Longitude: ");
            Serial.println(float(longitude) / 1000000, 6);
         }

         if (idadeInfo != Aerospace::GPS_INVALID_AGE)
         {
            Serial.print("Idade da Informacao (ms): ");
            Serial.println(idadeInfo);
         }

         //Dia e Hora
         int ano;
         byte mes, dia, hora, minuto, segundo, centesimo;
         long date, tie, age;
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

         if ((altitudeGPS != Aerospace::GPS_INVALID_ALTITUDE) && altitudeGPS != 1000000)
         {
            Serial.print("Altitude (cm): ");
            Serial.println(altitudeGPS);
         }

         //velocidade
         float velocidade;
         velocidade = aerospace.GPS_f_speed_kmph(); //km/h

         Serial.print("Velocidade (km/h): ");
         Serial.println(velocidade, 2); //Conversão de Nós para Km/h
      }
   }

//-------------------------Barômetro--------------------------------
  Serial.print("Temperatura = ");
  Serial.print(aerospace.BME_readTemperature());
  Serial.println(" *C");
//
  Serial.print("Pressão = ");
  Serial.print(aerospace.BME_readPressure() / 100.0F);
  Serial.println(" hPa");
//
  Serial.print("Aprox. Altitude barômetro= ");
  Serial.print(aerospace.BME_readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
//
  Serial.print("Humidade = ");
  Serial.print(aerospace.BME_readHumidity());
  Serial.println(" %");
//
  Serial.println();
  //delay(delayTime);
  


}
