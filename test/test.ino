#include <Wire.h> 
#define watchdog //enable this only on board with optiboot bootloader
#ifdef watchdog
#include <avr/wdt.h>
#endif

#include <OneWire.h>
#define ONE_WIRE_BUS A0
OneWire onewire(ONE_WIRE_BUS);  // pin for onewire DALLAS bus
#define dallasMinimal           //-956 Bytes
#ifdef dallasMinimal
#include <DallasTemperatureMinimal.h>
DallasTemperatureMinimal dsSensors(&onewire);
#else
#include <DallasTemperature.h>
DallasTemperature dsSensors(&onewire);
#endif
DeviceAddress tempDeviceAddress;
#ifndef NUMBER_OF_DEVICES
#define NUMBER_OF_DEVICES 4
#endif
DeviceAddress tempDeviceAddresses[NUMBER_OF_DEVICES];
unsigned int numberOfDevices              = 0; // Number of temperature devices found
unsigned long lastDsMeasStartTime         = 0;
bool dsMeasStarted                        = false;
float sensor[NUMBER_OF_DEVICES];
float tempDiffON                          = 5.0; //difference between room temperature and solar OUT (sensor 2 - sensor 1) to set relay ON
float tempDiffOFF                         = 2.0; //difference between room temperature and solar OUT (sensor 2 - sensor 1) to set relay OFF
//diferences in normal mode (power for pump is ready)
float tempDiffONNormal                    = tempDiffON;
float tempDiffOFFNormal                   = tempDiffOFF;
//diferences in power save mode (power for pump is OFF)
float tempDiffONPowerSave                 = 90.0; 
float tempDiffOFFPowerSave                = 50.0; 
unsigned long const dsMeassureInterval    = 750; //inteval between meassurements
unsigned long lastMeasTime                = 0;
unsigned long msDayON                     = 0;  //number of miliseconds in ON state per day
unsigned long lastOn                      = 0;     //ms posledniho behu ve stavu ON
unsigned long const delayAfterON          = 120000; //2 min
unsigned long lastOffOn                   = 0;
unsigned long lastOff                     = 0;  //ms posledniho vypnuti rele
unsigned long const dayInterval           = 43200000; //1000*60*60*12; //
unsigned long const delayON               = 120000; //1000*60*2; //po tento cas zustane rele sepnute bez ohledu na stav teplotnich cidel
unsigned long lastOn4Delay                = 0;
unsigned long const lastWriteEEPROMDelay  = 3600000; //in ms = 1 hod
unsigned long lastWriteEEPROM             = 0;
unsigned long totalEnergy                 = 0; //total enery in Ws. To kWh ->> totalEnergy/1000.f/3600.f
unsigned long totalSec                    = 0; //total time for pump ON in sec. To hours ->> totalSec/60/60
unsigned long msDiff                      = 0; //pocet ms ve stavu ON od posledniho ulozeni do EEPROM
unsigned int  power                       = 0; //actual power in W
unsigned int  maxPower                    = 0; //maximal power in W
unsigned long energyADay                  = 0; //energy a day in Ws
float energyDiff                          = 0.f; //difference in Ws
unsigned int const energyKoef             = 343; //Ws TODO - read from configuration
unsigned int pulseCount                   = 0; //
unsigned long consumption                 = 0;
unsigned long lastPulse                   = 0;
unsigned int cycles                       = 0;


//MODE
byte modeSolar                            = 0;
bool manualSetFromKeyboard                = false;
bool firstMeasComplete                    = false;
bool manualON                             = false;

float tIn                                 = 0; //input medium temperature to solar panel
float tOut                                = 0; //output medium temperature to solar panel
float tRoom                               = 0; //room temperature
float tBojler                             = 0; //boiler temperature
float tControl                            = 0; //temperature which is used as control temperature

//maximal temperatures
float tMaxIn                              = 0; //maximal input temperature (just for statistics)
float tMaxOut                             = 0; //maximal output temperature (just for statistics)
float tMaxBojler                          = 0; //maximal boiler temperature (just for statistics)

byte controlSensor                        = 0; //control sensor index

//int powerOff                            = 200;     //minimalni vykon, pokud je vykon nizssi, rele vzdy vypne
float safetyON                            = 80.0; //teplota, pri niz rele vzdy sepne


#define DS1307
#ifdef DS1307
#include <DS1307RTC.h>
#define time
#endif

#ifdef time
#include <Time.h>
#include <Streaming.h>        
#include <Time.h>             
bool parse=false;
bool config=false;
tmElements_t    tm;
const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};
bool isTime = true;
#endif

#include <LiquidCrystal_I2C.h>
#define LCDADDRESS   0x27
#define EN           2
#define RW           1
#define RS           0
#define D4           4
#define D5           5
#define D6           6
#define D7           7
#define BACKLIGHT    3
#define POL          POSITIVE
#define LCDROWS      4
#define LCDCOLS      20
LiquidCrystal_I2C lcd(LCDADDRESS,EN,RW,RS,D4,D5,D6,D7,BACKLIGHT,POL);  // set the LCD

#define RELAY                           6
#define LED                             8
#define BUZZER                          13
unsigned int const SERIAL_SPEED=9600;

bool relay                               = HIGH; 

#define STATUS_NORMAL0                        0
#define STATUS_NORMAL1                        1
#define STATUS_AFTER_START                    2
#define STATUS_WRITETOTALTOEEPROM_DELAY       3
#define STATUS_WRITETOTALTOEEPROM_ONOFF       4
#define STATUS_WRITETOTALTOEEPROM_MANUAL      5
#define STATUS_STARTAFTER_BROWNOUT            6
#define STATUS_STARTAFTER_POWERON             7
#define STATUS_STARTAFTER_WATCHDOGOREXTERNAL  8

byte status                                = STATUS_NORMAL0;

//EEPROM
//při výměně čipu nastavit default hodnoty odkomentovím následujícího defu a nastavením kWh a minut ve funkci setEEPROM()
//#define setEEPROM
#include <EEPROM.h>
byte const tempDiffONEEPROMAdrH           = 0;
byte const tempDiffONEEPROMAdrL           = 1;
byte const tempDiffOFFEEPROMAdrH          = 2; 
byte const tempDiffOFFEEPROMAdrL          = 3;
byte const totalEnergyEEPROMAdrH          = 4;
byte const totalEnergyEEPROMAdrM          = 5;
byte const totalEnergyEEPROMAdrS          = 6;
byte const totalEnergyEEPROMAdrL          = 7;
byte const controlSensorEEPROMAdr         = 8;
byte const totalSecEEPROMAdrH             = 9;
byte const totalSecEEPROMAdrM             = 10;
byte const totalSecEEPROMAdrS             = 11;
byte const totalSecEEPROMAdrL             = 12;
byte const backLightEEPROMAdr             = 13;



//SW name & version
float const   versionSW                   = 0.82;
char  const   versionSWString[]           = "Solar v"; 

//-------------------------------------------- S E T U P ------------------------------------------------------------------------------
void setup() {
#ifdef watchdog
  wdt_enable(WDTO_8S);
#endif
  Serial.begin(SERIAL_SPEED);
  Serial.print(versionSWString);
  Serial.println(versionSW);
  pinMode(LED, OUTPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(LED, LOW);
  digitalWrite(RELAY, HIGH);
  digitalWrite(BUZZER, HIGH);
  delay(250);
  digitalWrite(RELAY, LOW);
  digitalWrite(BUZZER, LOW);
  digitalWrite(LED, HIGH);
  
  lcd.begin(LCDCOLS,LCDROWS);               // initialize the lcd 
  lcd.setBacklight(255);
  lcd.clear();
  lcd.print(versionSWString);
  lcd.print(versionSW);
  
#ifdef DS1307
  if (RTC.read(tm)) {
/*    Serial.print("Ok, Time = ");
    print2digits(tm.Hour);
    Serial.write(':');
    print2digits(tm.Minute);
    Serial.write(':');
    print2digits(tm.Second);
    Serial.print(", Date (D/M/Y) = ");
    Serial.print(tm.Day);
    Serial.write('/');
    Serial.print(tm.Month);
    Serial.write('/');
    //Serial.print(tmYearToCalendar(tm.Year));
    Serial.print(tm.Year);
    Serial.println();*/
  } else {
    if (RTC.chipPresent()) {
/*      Serial.println("The DS1307 is stopped.  Please run the SetTime");
      Serial.println("example to initialize the time and begin running.");
      Serial.println();
    } else {
      Serial.println("DS1307 read error!  Please check the circuitry.");
      Serial.println();*/
    }
  }
  setSyncProvider(RTC.get);          // the function to get the time from the RTC
  
  dsInit();

  Wire.begin();        // join i2c bus (address optional for master)

#endif

}

//----------------------------------------L O O P ----------------------------------------------------------------------------------
void loop() {
#ifdef watchdog
  wdt_reset();
#endif
  
  tempMeas();
  calcPowerAndEnergy();
  mainControl();
  //lcdShow(); //show display
  
  /*if (lastOff > 0 && ((millis() - lastOff)>dayInterval)) {
      lastOff = 0;
  }
  */

 // communication();

//  keyBoard();
  
//  centralHeating();
  
} //loop


//--------------------------------------- F U N C T I O N S -----------------------------------------------------------------------------------
void mainControl() {
  //safety function
  if ((tOut || tIn) >= safetyON) {
    relay=LOW; //relay ON
  } else if (manualON) {
  } else {
    //pump is ON - relay ON = LOW
    if (relay==LOW) { 
      //save totalEnergy to EEPROM
      if ((millis() - lastWriteEEPROM) > lastWriteEEPROMDelay) {
        writeTotalEEPROM(STATUS_WRITETOTALTOEEPROM_DELAY);
      }
      //if (((tOut - tControl) < tempDiffOFF && (tIn < tOut) || ) /*|| (int)getPower() < powerOff)*/) { //switch pump ON->OFF
      if (((tOut - tControl) < tempDiffOFF) && (millis() - delayAfterON >= lastOffOn)) { //switch pump ON->OFF
#ifdef serial
        Serial.print("millis()=");
        Serial.print(millis());
        Serial.print(" delayAfterON=");
        Serial.print(delayAfterON);
        Serial.print(" lastOffOn=");
        Serial.print(lastOffOn);
        Serial.print(" tOut=");
        Serial.print(tOut);
        Serial.print("tControl=");
        Serial.println(tControl);
#endif
        relay=HIGH; //relay OFF = HIGH
        //digitalWrite(RELAY1PIN, relay);
        lastOff=millis();
        lastOn4Delay=0;
        //save totalEnergy to EEPROM
        writeTotalEEPROM(STATUS_WRITETOTALTOEEPROM_ONOFF);
      }
    } else { //pump is OFF - relay OFF = HIGH
      //if ((((tOut - tControl) >= tempDiffON) || ((tIn - tControl) >= tempDiffON))) { //switch pump OFF->ON
      if ((tOut - tControl) >= tempDiffON) { //switch pump OFF->ON
        relay=LOW; //relay ON = LOW
        //digitalWrite(RELAY1PIN, relay);
        lastOn = millis();
        lastOffOn = lastOn;
        if (lastOn4Delay==0) {
          lastOn4Delay = lastOn;
        }
        if ((millis()-lastOff)>=dayInterval) { //first ON in actual day
          lcd.clear();
          lastOff=millis();
          energyADay=0;
          //energyDiff=0.0;
          msDayON=0;
          tMaxOut=-128.0;
          tMaxIn=-128.0;
          tMaxBojler=-128.0;
        }
      }
    }
  }
  digitalWrite(RELAY, relay);
}


void dsInit(void) {
  dsSensors.begin();
  numberOfDevices = dsSensors.getDeviceCount();

  lcd.setCursor (0, 0);
  lcd.print(numberOfDevices);
  
  if (numberOfDevices==1)
    lcd.print(" sensor found");
  else
    lcd.print(" sensors found");
  delay(1000);
  
  // Loop through each device, print out address
  for (byte i=0;i<numberOfDevices; i++) {
      // Search the wire for address
    if (dsSensors.getAddress(tempDeviceAddress, i)) {
      memcpy(tempDeviceAddresses[i],tempDeviceAddress,8);
    }
  }
#ifndef dallasMinimal
  dsSensors.setResolution(12);
  dsSensors.setWaitForConversion(false);
#endif

}


void tempMeas() {
 if (!dsMeasStarted) {
    //start sampling
    dsMeasStarted=true;
    dsSensors.requestTemperatures(); 
    //digitalWrite(13,HIGH);
    lastDsMeasStartTime = millis();
  }
  else if (dsMeasStarted && (millis() - lastDsMeasStartTime>dsMeassureInterval)) {
    dsMeasStarted=false;
    firstMeasComplete=true;
    //digitalWrite(13,LOW);
    //saving temperatures into variables
    for (byte i=0;i<numberOfDevices; i++) {
      float tempTemp=-126;
      for (byte j=0;j<10;j++) { //try to read temperature ten times
        //tempTemp = dsSensors.getTempCByIndex(i);
        tempTemp = dsSensors.getTempC(tempDeviceAddresses[i]);
        if (tempTemp>=-55) {
          break;
        }
      }

      sensor[i] = tempTemp;
    } 
    tOut       = sensor[1];
    tIn         = sensor[2];
    tRoom     = sensor[3];
    tBojler   = sensor[0];
    tControl  = sensor[controlSensor];
    
    if (tOut>tMaxOut)       tMaxOut     = tOut;
    if (tIn>tMaxIn)         tMaxIn      = tIn;
    if (tBojler>tMaxBojler) tMaxBojler   = tBojler;
    //obcas se vyskytne chyba a vsechna cidla prestanou merit
    //zkusim restartovat sbernici
    bool reset=true;
    for (byte i=0; i<numberOfDevices; i++) {
      if (sensor[i]!=0.0) {
        reset=false;
      }
    }
    if (reset) {
      dsInit();
    }
  }
}

void calcPowerAndEnergy() {
  if (relay==LOW) {  //pump is ON
    if (tIn<tOut) {
      msDayON+=(millis()-lastOn);
      msDiff+=(millis()-lastOn);
      if (msDiff >= 1000) {
        totalSec+=msDiff/1000;
        msDiff=msDiff%1000;
      }
      power = getPower(); //in W
      if (power > maxPower) {
        maxPower = power;
      }
      energyDiff += (float)((millis()-lastOn))*(float)power/1000.f; //in Ws
      if (energyDiff >= 3600.f) { //Wh
        totalEnergy += (unsigned long)energyDiff;
        energyADay  += (unsigned long)energyDiff;
        energyDiff = energyDiff - (long)energyDiff;
      }
    } else {
      power=0;
    }
    lastOn = millis();
  } else {
    power=0;
  }
}

void writeTotalEEPROM(byte typ) {
  EEPROM.write(totalEnergyEEPROMAdrL, totalEnergy & 0xFF);
  EEPROM.write(totalEnergyEEPROMAdrS, (totalEnergy >> 8) & 0xFF);
  EEPROM.write(totalEnergyEEPROMAdrM, (totalEnergy >> 16) & 0xFF);
  EEPROM.write(totalEnergyEEPROMAdrH, (totalEnergy >> 24) & 0xFF); 
#ifdef serial
  Serial.print("Save totalEnergy to EEPROM:");
  Serial.print(totalEnergy);
  Serial.println("Ws");
#endif
  EEPROM.write(totalSecEEPROMAdrL, totalSec & 0xFF);
  EEPROM.write(totalSecEEPROMAdrS, (totalSec >> 8) & 0xFF);
  EEPROM.write(totalSecEEPROMAdrM, (totalSec >> 16) & 0xFF);
  EEPROM.write(totalSecEEPROMAdrH, (totalSec >> 24) & 0xFF); 
#ifdef serial
  Serial.print("Save totalSec to EEPROM:");
  Serial.print(totalSec);
  Serial.println("s");
#endif
  lastWriteEEPROM = millis();
  status=typ;
}

void readTotalEEPROM() {
  //read power from EEPROM
  int valueIH = EEPROM.read(totalEnergyEEPROMAdrH);
  int valueIM = EEPROM.read(totalEnergyEEPROMAdrM);
  int valueIS = EEPROM.read(totalEnergyEEPROMAdrS);
  int valueIL = EEPROM.read(totalEnergyEEPROMAdrL);
  totalEnergy = ((unsigned long)valueIH << 24) + ((unsigned long)valueIM << 16) + ((unsigned long)valueIS << 8) + ((unsigned long)valueIL);
#ifdef serial
  Serial.print("TotalEnergy from EEPROM:");
  Serial.print(totalEnergy);
  Serial.println("Ws");
#endif
  valueIH = EEPROM.read(totalSecEEPROMAdrH);
  valueIM = EEPROM.read(totalSecEEPROMAdrM);
  valueIS = EEPROM.read(totalSecEEPROMAdrS);
  valueIL = EEPROM.read(totalSecEEPROMAdrL);
  totalSec = ((unsigned long)valueIH << 24) + ((unsigned long)valueIM << 16) + ((unsigned long)valueIS << 8) + ((unsigned long)valueIL);
#ifdef serial
  Serial.print("TotalSec from EEPROM:");
  Serial.print(totalSec);
  Serial.println("s");
#endif
}

unsigned int getPower() {
  return (float)energyKoef*(tOut-tIn); //in W
}
