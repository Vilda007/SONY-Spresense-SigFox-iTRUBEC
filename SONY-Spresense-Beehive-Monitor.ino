long lastJob1s = 0, lastJob5s = 0, lastJob10s = 0, lastJob30s = 0, lastJob1min = 0, lastJob5min = 0, lastJob10min = 0;
float myTemp, myPres, myHumi; //programove globalni promenne
char zprava[12]; //SigFox zpráva
int t1, t2, t3, v1, p1, n1; //SigFox proměnné

// libraries
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
// BME280 sensor address setup
#define BME280_ADDRESS (0x76)
// initialization of BME280 sensor
Adafruit_BME280 bme;
#include <SDHCI.h>

SDClass SD;
File myLogFile;

// SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-
void setup() {
  pinMode(LED0, OUTPUT); //BME280 in operation
  pinMode(LED1, OUTPUT); //SD card in operation
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  
  Serial.begin(115200);
  Serial.println("Setup");
  
  // initialization of communication with BME280 sensor
  if (!bme.begin(BME280_ADDRESS)) {
    Serial.println("BME280 sensor not found! Check wiring!");
    while (1);
  }
  
  // Check if Log file exists (initialize if does not exist yet)
  digitalWrite(LED1, HIGH);
  myLogFile = SD.open("iTrubecLog.csv");
  if (myLogFile) {
    Serial.println("Log file iTrubecLog.csv exists already.");
    myLogFile.close();
    digitalWrite(LED1, LOW);
  } else {
    // Log file does not exist - will be created and header row will be written into it
    myLogFile.close();
    Serial.println("Initializing log file iTrubecLog.csv.");
    myLogFile = SD.open("iTrubecLog.csv", FILE_WRITE);
    if (myLogFile) {
      Serial.print("Writing header row to log file iTrubecLog.csv on SD card...");
      myLogFile.println("Temperature;Humidity;Pressure");
      myLogFile.close();
      Serial.println("...done.");
    } else {
      /* If the file didn't open, print an error */
      Serial.println("error opening log file iTrubecLog.csv");
    }
  }digitalWrite(LED1, LOW);

} // setup konec

// LOOP-LOOP-LOOP-LOOP-LOOP-LOOP-LOOP-LOOP-LOOP-LOOP-LOOP-LOOP-LOOP-LOOP-LOOP-LOOP-LOOP-LOOP-LOOP-LOOP-LOOP-LOOP-LOOP-
void loop() {



  // LOOP-BLOCK-1s-LOOP-BLOCK-1s-LOOP-BLOCK-1s-LOOP-BLOCK-1s-LOOP-BLOCK-1s-LOOP-BLOCK-1s-LOOP-BLOCK-1s-LOOP-BLOCK-1s-LOOP-BLOCK-1s-LOOP-BLOCK-1s-LOOP-BLOCK-1s-LOOP-BLOCK-1s
  if (millis() > (1000 + lastJob1s))
  {
    // kód vykonaný každou 1 vteřinu (1000 ms)
    //Serial.println("1 s");

    lastJob1s = millis();
  } // 1s konec

  // LOOP-BLOCK-5s-LOOP-BLOCK-5s-LOOP-BLOCK-5s-LOOP-BLOCK-5s-LOOP-BLOCK-5s-LOOP-BLOCK-5s-LOOP-BLOCK-5s-LOOP-BLOCK-5s-LOOP-BLOCK-5s-LOOP-BLOCK-5s-LOOP-BLOCK-5s-LOOP-BLOCK-5s-
  if (millis() > (5000 + lastJob5s))
  {
    // kód vykonaný každých 5 vteřin (5000 ms)


    lastJob5s = millis();
  } // 5s konec

  // LOOP-BLOCK-10s-LOOP-BLOCK-10s-LOOP-BLOCK-10s-LOOP-BLOCK-10s-LOOP-BLOCK-10s-LOOP-BLOCK-10s-LOOP-BLOCK-10s-LOOP-BLOCK-10-LOOP-BLOCK-10s-LOOP-BLOCK-10s-LOOP-BLOCK-10s
  if (millis() > (10000 + lastJob10s))
  {
    // kód vykonaný každých 10 vteřin (10000 ms)
    //Serial.println("10 s");


    lastJob10s = millis();
  } // 10s konec

  // LOOP-BLOCK-30s-LOOP-BLOCK-30s-LOOP-BLOCK-30s-LOOP-BLOCK-30s-LOOP-BLOCK-30s-LOOP-BLOCK-30s-LOOP-BLOCK-30s-LOOP-BLOCK-30s-LOOP-BLOCK-30s-LOOP-BLOCK-30s-LOOP-BLOCK-30s-
  if (millis() > (30000 + lastJob30s))
  {
    // kód vykonaný každých 30 vteřin (30000 ms)


    lastJob30s = millis();
  } //30s konec

  // LOOP-BLOCK-1min-LOOP-BLOCK-1min-LOOP-BLOCK-1min-LOOP-BLOCK-1min-LOOP-BLOCK-1min-LOOP-BLOCK-1min-LOOP-BLOCK-1min-LOOP-BLOCK-1min-LOOP-BLOCK-1min-LOOP-BLOCK-1min
  if (millis() > (60000 + lastJob1min))
  {
    // kód vykonaný každou 1 minutu (60000 ms)
    //Serial.println("1min");
    // Read all data from BME280
    digitalWrite(LED0, HIGH);
    // Temperature
    Serial.print("Temperature  :   ");
    myTemp = bme.readTemperature();
    Serial.print(myTemp);
    Serial.println(" degrees of Celsius");
    // Humidity
    Serial.print("Rel. humidity:   ");
    myHumi = bme.readHumidity();
    Serial.print(myHumi);
    Serial.println(" %");
    // Atmospherical pressure
    Serial.print("Pressure     : ");
    myPres = bme.readPressure() / 100.0F;
    Serial.print(myPres);
    Serial.println(" hPa");
    Serial.println();
    digitalWrite(LED0, LOW);

    // initialization of SD card and writing the log
    digitalWrite(LED1, HIGH);
    myLogFile = SD.open("iTrubecLog.csv", FILE_WRITE);
    if (myLogFile) {
      Serial.print("Writing log to SD card - iTrubecLog.csv...");
      myLogFile.print(myTemp);
      myLogFile.print(";");
      myLogFile.print(myHumi);
      myLogFile.print(";");
      myLogFile.print(myPres);
      myLogFile.println("");
      myLogFile.close();
      Serial.println("...done.");
    } else {
      /* If the file didn't open, print an error */
      Serial.println("error opening log file iTrubecLog.csv");
    }
    digitalWrite(LED1, LOW);
    Serial.println();

    lastJob1min = millis();
  } // 1min konec

  // LOOP-BLOCK-10min-LOOP-BLOCK-10min-LOOP-BLOCK-10min-LOOP-BLOCK-10min-LOOP-BLOCK-10min-LOOP-BLOCK-10min-LOOP-BLOCK-10min-LOOP-BLOCK-10min-LOOP-BLOCK-10min-LOOP-BLOCK-10min-
  if (millis() > (300000 + lastJob10min))
  {
    // kód vykonaný každých 5 minut (300000 ms)


    lastJob10min = millis();
  } // 5min konec

  // LOOP-BLOCK-LOOP-BLOCK-LOOP-BLOCK-LOOP-BLOCK-LOOP-BLOCK-LOOP-BLOCK-LOOP-BLOCK-LOOP-BLOCK-LOOP-BLOCK-LOOP-BLOCK-LOOP-BLOCK-LOOP-BLOCK-LOOP-BLOCK-LOOP-BLOCK-LOOP-BLOCK-
} // loop konec
