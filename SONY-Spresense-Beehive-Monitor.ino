long lastJob1s = 0, lastJob5s = 0, lastJob10s = 0, lastJob30s = 0, lastJob1min = 0, lastJob5min = 0, lastJob10min = 0;
float myTemp, myPres, myHumi, myLon, myLat, myLastLon, myLastLat, myLonChange, myLatChange; //my variables
char zprava[12]; //SigFox message
char myDate[15], myTime[15]; // Date and Time
int t1, v1, p1, s1, lon1, lat1; //SigFox variables
int myZvuk = 0, myZvukSum = 0,  myZvukCount = 0, myZvukAVG = 0, myZvukMin = 1024, myZvukMax = 0;
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;

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
#include <GNSS.h>

SDClass SD;
File myLogFile;

const float myPosChangeTreshold = 0.00001; //If position changes more, alert is triggered

#define STRING_BUFFER_SIZE  128       /**< %Buffer size */
#define RESTART_CYCLE       (60 * 5)  /**< positioning test term */
static SpGnss Gnss;

enum ParamSat {
  eSatGps,            /**< GPS                     World wide coverage  */
  eSatGlonass,        /**< GLONASS                 World wide coverage  */
  eSatGpsSbas,        /**< GPS+SBAS                North America        */
  eSatGpsGlonass,     /**< GPS+Glonass             World wide coverage  */
  eSatGpsQz1c,        /**< GPS+QZSS_L1CA           East Asia & Oceania  */
  eSatGpsGlonassQz1c, /**< GPS+Glonass+QZSS_L1CA   East Asia & Oceania  */
  eSatGpsQz1cQz1S,    /**< GPS+QZSS_L1CA+QZSS_L1S  Japan                */
};

/* Set this parameter depending on your current region. */
static enum ParamSat satType =  eSatGpsGlonass;

/**
   @brief Turn on / off the LED0 for CPU active notification.
*/
static void Led_isActive(void)
{
  static int state = 1;
  if (state == 1)
  {
    ledOn(PIN_LED0);
    state = 0;
  }
  else
  {
    ledOff(PIN_LED0);
    state = 1;
  }
}

/**
   @brief Turn on / off the LED1 for positioning state notification.

   @param [in] state Positioning state
*/
static void Led_isPosfix(bool state)
{
  if (state)
  {
    ledOn(PIN_LED1);
  }
  else
  {
    ledOff(PIN_LED1);
  }
}

/**
   @brief Turn on / off the LED3 for error notification.

   @param [in] state Error state
*/
static void Led_isError(bool state)
{
  if (state)
  {
    ledOn(PIN_LED3);
  }
  else
  {
    ledOff(PIN_LED3);
  }
}

// SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-SETUP-
void setup() {
  pinMode(LED0, OUTPUT); //BME280 in operation
  pinMode(LED1, OUTPUT); //SD card in operation
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  int error_flag = 0;

  Serial.begin(115200);

  /* Wait HW initialization done. */
  sleep(3);

  Serial.println("Setup");
  // POSITIONING SYSTEM INI BEGIN
  /* Set Debug mode to Info */
  Gnss.setDebugMode(PrintInfo);

  int result;

  /* Activate GNSS device */
  result = Gnss.begin();

  if (result != 0)
  {
    Serial.println("Gnss begin error!!");
    error_flag = 1;
  }
  else
  {
    /* Setup GNSS
        It is possible to setup up to two GNSS satellites systems.
        Depending on your location you can improve your accuracy by selecting different GNSS system than the GPS system.
        See: https://developer.sony.com/develop/spresense/developer-tools/get-started-using-nuttx/nuttx-developer-guide#_gnss
        for detailed information.
    */
    switch (satType)
    {
      case eSatGps:
        Gnss.select(GPS);
        break;

      case eSatGpsSbas:
        Gnss.select(GPS);
        Gnss.select(SBAS);
        break;

      case eSatGlonass:
        Gnss.select(GLONASS);
        break;

      case eSatGpsGlonass:
        Gnss.select(GPS);
        Gnss.select(GLONASS);
        break;

      case eSatGpsQz1c:
        Gnss.select(GPS);
        Gnss.select(QZ_L1CA);
        break;

      case eSatGpsQz1cQz1S:
        Gnss.select(GPS);
        Gnss.select(QZ_L1CA);
        Gnss.select(QZ_L1S);
        break;

      case eSatGpsGlonassQz1c:
      default:
        Gnss.select(GPS);
        Gnss.select(GLONASS);
        Gnss.select(QZ_L1CA);
        break;
    }


    /* Start positioning */
    result = Gnss.start(COLD_START);
    if (result != 0)
    {
      Serial.println("Gnss start error!!");
      error_flag = 1;
    }
    else
    {
      Serial.println("Gnss setup OK");
    }
  }

  /* Turn off all LED:Setup done. */
  ledOff(PIN_LED0);
  ledOff(PIN_LED1);
  ledOff(PIN_LED2);
  ledOff(PIN_LED3);

  /* Set error LED. */
  if (error_flag == 1)
  {
    Led_isError(true);
    exit(0);
  }
  // POSITIONING SYSTEM INI END

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
      myLogFile.println("Date;Time;Temperature;Humidity;Atm.Pressure;Longitude;Latitude;AVG Sound level;MIN Sound level;MAX Sound level");
      myLogFile.close();
      Serial.println("...done.");
    } else {
      /* If the file didn't open, print an error */
      Serial.println("error opening log file iTrubecLog.csv");
    }
  } digitalWrite(LED1, LOW);

} // setup konec

static void get_date_time(SpNavData *pNavData)
{
  /* get date */
  snprintf(myDate, 15, "%04d/%02d/%02d ", pNavData->time.year, pNavData->time.month, pNavData->time.day);
  /* get time */
  snprintf(myTime, 15, "%02d:%02d:%02d.%06d, ", pNavData->time.hour, pNavData->time.minute, pNavData->time.sec, pNavData->time.usec);
}

/**
   @brief %Print position information.
*/
static void print_pos(SpNavData *pNavData)
{
  char StringBuffer[STRING_BUFFER_SIZE];

  /* print time */
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%04d/%02d/%02d ", pNavData->time.year, pNavData->time.month, pNavData->time.day);
  Serial.print(StringBuffer);

  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%02d:%02d:%02d.%06d, ", pNavData->time.hour, pNavData->time.minute, pNavData->time.sec, pNavData->time.usec);
  Serial.print(StringBuffer);

  /* print satellites count */
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "numSat:%2d, ", pNavData->numSatellites);
  Serial.print(StringBuffer);

  /* print position data */
  if (pNavData->posFixMode == FixInvalid)
  {
    Serial.print("No-Fix, ");
  }
  else
  {
    Serial.print("Fix, ");
  }
  if (pNavData->posDataExist == 0)
  {
    Serial.print("No Position");
    myLat = 0;
    myLon = 0;
    myLastLat = 0;
    myLastLon = 0;
  }
  else
  {
    Serial.print("Lat=");
    myLat = (pNavData->latitude, 6);
    Serial.print(myLat);
    Serial.print(", Lon=");
    myLon = (pNavData->longitude, 6);
    Serial.print(myLon);
  }

  Serial.println("");
}

/**
   @brief %Print satellite condition.
*/
static void print_condition(SpNavData *pNavData)
{
  char StringBuffer[STRING_BUFFER_SIZE];
  unsigned long cnt;

  /* Print satellite count. */
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "numSatellites:%2d\n", pNavData->numSatellites);
  Serial.print(StringBuffer);

  for (cnt = 0; cnt < pNavData->numSatellites; cnt++)
  {
    const char *pType = "---";
    SpSatelliteType sattype = pNavData->getSatelliteType(cnt);

    /* Get satellite type. */
    /* Keep it to three letters. */
    switch (sattype)
    {
      case GPS:
        pType = "GPS";
        break;

      case GLONASS:
        pType = "GLN";
        break;

      case QZ_L1CA:
        pType = "QCA";
        break;

      case SBAS:
        pType = "SBA";
        break;

      case QZ_L1S:
        pType = "Q1S";
        break;

      default:
        pType = "UKN";
        break;
    }

    /* Get print conditions. */
    unsigned long Id  = pNavData->getSatelliteId(cnt);
    unsigned long Elv = pNavData->getSatelliteElevation(cnt);
    unsigned long Azm = pNavData->getSatelliteAzimuth(cnt);
    float sigLevel = pNavData->getSatelliteSignalLevel(cnt);

    /* Print satellite condition. */
    snprintf(StringBuffer, STRING_BUFFER_SIZE, "[%2d] Type:%s, Id:%2d, Elv:%2d, Azm:%3d, CN0:", cnt, pType, Id, Elv, Azm );
    Serial.print(StringBuffer);
    Serial.println(sigLevel, 6);
  }
}

void SampleSound() {
  unsigned long startMillis = millis(); // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;

  // collect data for 50 mS
  while (millis() - startMillis < sampleWindow)
  {
    sample = analogRead(A0);
    if (sample < 1024)  // toss out spurious readings
    {
      if (sample > signalMax)
      {
        signalMax = sample;  // save just the max levels
      }
      else if (sample < signalMin)
      {
        signalMin = sample;  // save just the min levels
      }
    }
  }
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
  myZvuk = peakToPeak;
}

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
    static int LoopCount = 0;
    static int LastPrintMin = 0;

    /* Blink LED. */
    Led_isActive();

    /* Check update. */
    if (Gnss.waitUpdate(-1))
    {
      /* Get NaviData. */
      SpNavData NavData;
      Gnss.getNavData(&NavData);

      /* Set posfix LED. */
      bool LedSet = (NavData.posDataExist && (NavData.posFixMode != FixInvalid));
      Led_isPosfix(LedSet);

      /* Print satellite information every minute. */
      if (NavData.time.minute != LastPrintMin)
      {
        print_condition(&NavData);
        LastPrintMin = NavData.time.minute;
      }

      /* Print position information. */
      print_pos(&NavData);
      //update stored date and time
      get_date_time(&NavData);
    }
    else
    {
      /* Not update. */
      Serial.println("data not update");
    }

    /* Check loop count. */
    LoopCount++;
    if (LoopCount >= RESTART_CYCLE)
    {
      int error_flag = 0;

      /* Turn off LED0 */
      ledOff(PIN_LED0);

      /* Set posfix LED. */
      Led_isPosfix(false);

      /* Restart GNSS. */
      if (Gnss.stop() != 0)
      {
        Serial.println("Gnss stop error!!");
        error_flag = 1;
      }
      else if (Gnss.end() != 0)
      {
        Serial.println("Gnss end error!!");
        error_flag = 1;
      }
      else
      {
        Serial.println("Gnss stop OK.");
      }

      if (Gnss.begin() != 0)
      {
        Serial.println("Gnss begin error!!");
        error_flag = 1;
      }
      else if (Gnss.start(HOT_START) != 0)
      {
        Serial.println("Gnss start error!!");
        error_flag = 1;
      }
      else
      {
        Serial.println("Gnss restart OK.");
      }

      LoopCount = 0;

      /* Set error LED. */
      if (error_flag == 1)
      {
        Led_isError(true);
        exit(0);
      }
    }

    SampleSound();
    Serial.print("Sound level: ");
    Serial.println(myZvuk);
    myZvukSum = myZvukSum + myZvuk;
    myZvukCount = myZvukCount + 1;
    if (myZvuk < myZvukMin) {
      myZvukMin = myZvuk;
    }
    if (myZvuk > myZvukMax) {
      myZvukMax = myZvuk;
    }

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
    //Average level of sound
    myZvukAVG = myZvukSum / myZvukCount;
    myZvukSum = 0;
    myZvukCount = 0;
    // Read all data from BME280
    Serial.println();
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
    Serial.print("Sound lvl - AVG: ");
    Serial.print(myZvukAVG);
    Serial.print(", MIN: ");
    Serial.print(myZvukMin);
    Serial.print(", MAX: ");
    Serial.println(myZvukMax);
    Serial.println();
    digitalWrite(LED0, LOW);

    // initialization of SD card and writing the log
    digitalWrite(LED1, HIGH);
    myLogFile = SD.open("iTrubecLog.csv", FILE_WRITE);
    if (myLogFile) {
      Serial.print("Writing log to SD card - iTrubecLog.csv...");
      myLogFile.print(myDate);
      myLogFile.print(";");
      myLogFile.print(myTime);
      myLogFile.print(";");
      myLogFile.print(myTemp);
      myLogFile.print(";");
      myLogFile.print(myHumi);
      myLogFile.print(";");
      myLogFile.print(myPres);
      myLogFile.print(";");
      myLogFile.print(myLon);
      myLogFile.print(";");
      myLogFile.print(myLat);
      myLogFile.print(";");
      myLogFile.print(myZvukAVG);
      myLogFile.print(";");
      myLogFile.print(myZvukMin);
      myZvukMin = 1024;
      myLogFile.print(";");
      myLogFile.print(myZvukMax);
      myZvukMax = 0;
      myLogFile.println("");
      myLogFile.close();
      Serial.println("...done.");
    } else {
      /* If the file didn't open, print an error */
      Serial.println("error opening log file iTrubecLog.csv");
    }
    digitalWrite(LED1, LOW);
    Serial.println();

    //position change check
    myLonChange = abs(myLastLon - myLon);
    myLatChange = abs(myLastLat - myLat);
    myLastLat = myLat;
    myLastLon = myLon;
    //Debug print
    Serial.println();
    Serial.print("Lat: ");
    Serial.print(myLat);
    Serial.print(" ==>Absolute change: ");
    Serial.println(myLatChange);
    Serial.print("Lon: ");
    Serial.print(myLon);
    Serial.print(" ==>Absolute change: ");
    Serial.println(myLonChange);
    Serial.println();
    //Check if position was changed
    if ((myPosChangeTreshold < myLonChange) || (myPosChangeTreshold < myLatChange)) {
      //POSITION CHANGE ALERT!
      Serial.println();
      Serial.println("POSITION CHANGED");
      Serial.println();
    }

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
