#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

#include <Aerospace.h>
#include "TimerOne.h"

File dataFile;
Aerospace aerospace;
const int pinCS = 10; // Pin 10 - on Arduino Uno; Pin 53 - on Arduino Mega
int Xg,Yg,Zg;
int t,h;

void setup() {
  Serial.begin(9600);

  /*-----------------------------------SD----------------------------------*/
  Serial.print("Initializing SD card...");
  pinMode(pinCS, OUTPUT);

  if (!SD.begin(pinCS)) {  // check if SD card is present
    Serial.println("No SD Card present in module");
    return;
  }
  
  Serial.println("SD Card Ready");

  /*-----------------------------------ACEEL----------------------------------*/
  aerospace.accelero_begin();
  aerospace.accelero_calibrate();

  /*-------------------------TIMER-------------------------------------------*/
  Timer1.initialize(500000); // Inicializa o Timer1 e configura para um período de 0,5 segundos //acell
  Timer1.attachInterrupt(salvar); // Configura a função callback() como a função para ser chamada a cada interrupção do Timer1
}
void salvar() {

  dataFile = SD.open("accel.txt", FILE_WRITE);  // Open or Create file

  if (dataFile) {  // Check if file exist on SD Card
    dataFile.print(Xg);
    dataFile.print(":");
    dataFile.println(Yg);
    dataFile.print(":");
    dataFile.println(Zg);
    dataFile.close();  // Close file
  } else {
    Serial.println("error opening the accel file"); // if file not on SD Card
  }
  dataFile = SD.open("temp.txt", FILE_WRITE);  // Open or Create file

  if (dataFile) {  // Check if file exist on SD Card
    dataFile.println(t);
    dataFile.close();  // Close file
  } else {
    Serial.println("error opening the temp file"); // if file not on SD Card
  }
  dataFile = SD.open("umi.txt", FILE_WRITE);  // Open or Create file
  if (dataFile) {
    dataFile.println(h);
    dataFile.close();  // Close file
  }
  else {
    Serial.println("error opening umi the file"); // if file not on SD Card
  }
}
void loop() {
    //------------------------DHT---------------------------------
    h = aerospace.DHT_readHumidity();
    t = aerospace.DHT_readTemperature();
    // testa se retorno é valido, caso contrário algo está errado.
    if (isnan(t) || isnan(h)){
    Serial.println("Failed to read from DHT");
    }
  
    /*----------------------ACEEL--------------------------*/
    Xg=aerospace.accelero_getXAccel();
    Yg=aerospace.accelero_getYAccel();
    Zg=aerospace.accelero_getZAccel();

    Serial.print(Xg);
    Serial.print(":");
    Serial.print(Yg);
    Serial.print(":");
    Serial.print(Zg);
    Serial.print("\n");

    /*----------------------BME--------------------------*/
    bool status;
    status = aerospace.BME_begin();  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }

    Serial.print("Temperature = ");
    Serial.print(aerospace.BME_getTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(aerospace.BME_getPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(aerospace.BME_getAltitude(1013.25));
    
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(aerospace.BME_getHumidity());
    Serial.println(" %");
}
