/*
--------------------------------------------------------------------------------------------------------------------------
SOLAR - control system for solar unit
Petr Fory pfory@seznam.cz
GIT - https://github.com/pfory/solar-heating
*/

#include "Configuration.h"

#define watchdog //enable this only on board with optiboot bootloader
#ifdef watchdog
#include <avr/wdt.h>
#endif

#define verbose
#ifdef verbose
 #define DEBUG_PRINT(x)         Serial.print (x)
 #define DEBUG_PRINTDEC(x)      Serial.print (x, DEC)
 #define DEBUG_PRINTHEX(x)      Serial.print(x, HEX);

 #define DEBUG_PRINTLN(x)       Serial.println (x)
 #define DEBUG_PRINTF(x, y)     Serial.printf (x, y)
 #define PORTSPEED 115200
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTDEC(x)
 #define DEBUG_PRINTLN(x)
 #define DEBUG_PRINTF(x, y)
#endif 

#include <Wire.h>
#include <avr/pgmspace.h>
unsigned long crc;
const PROGMEM uint32_t crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

#include <SoftwareSerial.h>
SoftwareSerial mySerial(RX, TX);

const unsigned int serialTimeout=2000;

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(LCDADDRESS,LCDCOLS,LCDROWS);  // set the LCD

#define DS1307
#ifdef DS1307
#include <DS1307RTC.h>
#include <Time.h>
#include <TimeLib.h>
#define time
#endif


#ifdef time
#include <Time.h>
//#include <Streaming.h>        
#include <Time.h>             
bool            parse                      = false;
bool            config                     = false;
tmElements_t    tm;
bool            isTime                     = true;
bool            zobrazNastavovaciHlasku    = false;
int             uhelKolektoru              = 0;
//uhly pro 49 viz http://www.reneslacal.eu/photogallery/software/VyskaSlunceWEB.xls
const byte uhelMesic[12]                   = {18,23,33,45,56,63,64,59,49,37,26,19};
#endif

#include <OneWire.h>
OneWire onewire(ONE_WIRE_BUS);  // pin for onewire DALLAS bus
#ifdef dallasMinimal
#include <DallasTemperatureMinimal.h>
DallasTemperatureMinimal dsSensors(&onewire);
#else
#include <DallasTemperature.h>
DallasTemperature dsSensors(&onewire);
#endif
DeviceAddress tempDeviceAddress;

DeviceAddress tempDeviceAddresses[NUMBER_OF_DEVICES];
unsigned int numberOfDevices              = 0; // Number of temperature devices found
unsigned long lastDsMeasStartTime         = 0;
bool dsMeasStarted                        = false;
float sensor[NUMBER_OF_DEVICES];
unsigned long msDayON                     = 0;  //number of miliseconds in ON state per day
unsigned long lastOn                      = 0;     //ms posledniho behu ve stavu ON
unsigned long lastOffOn                   = 0;
unsigned long lastOff                     = 0;  //ms posledniho vypnuti rele
unsigned long lastOn4Delay                = 0;
unsigned long lastWriteEEPROM             = 0;
unsigned long totalEnergy                 = 0; //total enery in Ws. To kWh ->> totalEnergy/1000.f/3600.f
unsigned long totalSec                    = 0; //total time for pump ON in sec. To hours ->> totalSec/60/60
unsigned long msDiff                      = 0; //pocet ms ve stavu ON od posledniho ulozeni do EEPROM
unsigned int  power                       = 0; //actual power in W
unsigned int  maxPower                    = 0; //maximal power in W
unsigned long energyADay                  = 0; //energy a day in Ws
unsigned long showInfo                    = 0; //zobrazeni 4 radky na displeji
float energyDiff                          = 0.f; //difference in Ws
//unsigned int const energyKoef             = 343; //Ws TODO - read from configuration
unsigned int pulseCount                   = 0; //
//unsigned long consumption                 = 0;
//unsigned long lastPulse                   = 0;
//unsigned int cycles                       = 0;
unsigned long lastSend                    = 0;  //ms posledniho poslani dat
unsigned long lastRunMin                  = 0;
unsigned long milisLastRunMinOld          = 0;

//MODE
//byte modeSolar                            = 0;
//bool manualSetFromKeyboard                = false;
bool firstMeasComplete                    = false;
bool manualON                             = false;

float tP2In                               = 0; //input medium temperature to solar panel roof
float tP2Out                              = 0; //output medium temperature to solar panel roof`
float tP1In                               = 0; //input medium temperature to solar panel drevnik
float tP1Out                              = 0; //output medium temperature to solar panel drevnik
float tRoom                               = 0; //room temperature
float tBojler                             = 0; //boiler temperature
float tControl                            = 0; //temperature which is used as control temperature
float tBojlerIn                           = 0; //boiler input temperature
float tBojlerOut                          = 0; //boiler output temperature

//maximal temperatures
float tMaxIn                              = 0; //maximal input temperature (just for statistics)
float tMaxOut                             = 0; //maximal output temperature (just for statistics)
float tMaxBojler                          = 0; //maximal boiler temperature (just for statistics)

//byte controlSensor                        = 0; //control sensor index

//int powerOff                            = 200;     //minimalni vykon, pokud je vykon nizssi, rele vzdy vypne

// enum mode {NORMAL, POWERSAVE};
// mode powerMode                            = NORMAL;

unsigned int display                      = 0;

enum modeDisplay                          {SETUP, INFO};
modeDisplay displayMode                   = INFO;


byte status                               = STATUS_NORMAL0;

//HIGH - relay OFF, LOW - relay ON
bool relay1                               = HIGH; 
bool relay2                               = HIGH;

#include <Keypad_I2C.h>
#include <Keypad.h>          // GDY120705
#include <Wire.h>

const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns
char keys[ROWS][COLS]                 = {
                                            {'1','2','3','A'},
                                            {'4','5','6','B'},
                                            {'7','8','9','C'},
                                            {'*','0','#','D'}
};
byte rowPins[ROWS] = {7,6,5,4}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {3,2,1,0}; //connect to the column pinouts of the keypad

//Keypad_I2C keypad = Keypad_I2C( makeKeymap(keys), rowPins, colPins, ROWS, COLS, I2CADDR );
Keypad_I2C keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS, I2CADDR); 
// #endif
 

#define flowSensor
#ifdef flowSensor
volatile int      numberOfPulsesFlow      = 0; // Measures flow sensor pulses
float             lMinCumul               = 0; // kumulovany prutok pro vypocet prumerneho
float             lMin                    = 0; // Calculated litres/min
byte              numberOfCyclesFlow      = 0; //pocet mereni prutoku pro prumerny prutok
unsigned char     flowsensor              = 2; // Sensor Input
unsigned long     cloopTime;

void flow () { // Interrupt function
   numberOfPulsesFlow++;
}
#endif

uint8_t MyRstFlags __attribute__ ((section(".noinit")));
void SaveResetFlags(void) __attribute__ ((naked))
                          __attribute__ ((section (".init0")));
void SaveResetFlags(void)
{
  __asm__ __volatile__ ("mov %0, r2\n" : "=r" (MyRstFlags) :);
}

#include <EEPROM.h>
struct StoreStruct {
  // This is for mere detection if they are your settings
  char            version[4];
  // The variables of your settings
  byte            tDiffON;     
  byte            tDiffOFF;
  byte            controlSensor;
  bool            backLight;
  unsigned long   totalEnergy;
  unsigned long   totalSec;
  byte            sensorOrder[NUMBER_OF_DEVICES];
  bool            controlSensorBojler;
#ifdef time
  tmElements_t    lastPumpRun;
#endif
} storage = {
  CONFIG_VERSION,
  5,
  2, 
  1,
  0,
  0,
  0,
  0,
  1
};

//-------------------------------------------- S E T U P ------------------------------------------------------------------------------
void setup() {
  Serial.begin(PORTSPEED);
  DEBUG_PRINT(F(SW_NAME));
  DEBUG_PRINT(F(" "));
  DEBUG_PRINTLN(F(VERSION));

#ifdef watchdog
  wdt_enable(WDTO_8S);
#endif
  printConfigVersion();
  testConfigChange();

  #ifdef DS1307
  if (RTC.read(tm)) {
    DEBUG_PRINT(F("RTC OK, Time = "));
    print2digits(tm.Hour);
    DEBUG_WRITE(':');
    print2digits(tm.Minute);
    DEBUG_WRITE(':');
    print2digits(tm.Second);
    DEBUG_PRINT(F(", Date (D/M/Y) = "));
    DEBUG_PRINT(tm.Day);
    DEBUG_WRITE('/');
    DEBUG_PRINT(tm.Month);
    DEBUG_WRITE('/');
    //DEBUG_PRINT(tmYearToCalendar(tm.Year));
    DEBUG_PRINT(tm.Year);
    DEBUG_PRINTLN();
  } else {
    if (RTC.chipPresent()) {
      DEBUG_PRINTLN(F("The DS1307 is stopped.  Please run the SetTime"));
      DEBUG_PRINTLN(F("example to initialize the time and begin running."));
      DEBUG_PRINTLN();
    } else {
      DEBUG_PRINTLN(F("DS1307 read error!  Please check the circuitry."));
      DEBUG_PRINTLN();
    }
  }
#endif

#ifdef time
  // Setup time library  
  DEBUG_PRINT(F("RTC Sync"));
  setSyncProvider(RTC.get);          // the function to get the time from the RTC
  if(timeStatus() == timeSet) {
    DEBUG_PRINTLN(F(" OK!"));
  } else {
    DEBUG_PRINTLN(F(" FAIL!"));
  }
#endif

  
  loadConfig();
 
  DEBUG_PRINT(F("tON:"));  
  DEBUG_PRINTLN(storage.tDiffON);
  DEBUG_PRINT(F("tOFF:"));  
  DEBUG_PRINTLN(storage.tDiffOFF);
  DEBUG_PRINT(F("Control:"));  
  DEBUG_PRINT(storage.controlSensor);
  if (storage.controlSensorBojler) {
    DEBUG_PRINTLN(" - Bojler");
  } else {
    DEBUG_PRINTLN(" - Room");
  }
  DEBUG_PRINT(F("TotalEnergy from EEPROM:"));
  DEBUG_PRINT(storage.totalEnergy);
  DEBUG_PRINTLN(F("Ws"));
  DEBUG_PRINT(F("TotalSec from EEPROM:"));
  DEBUG_PRINT(storage.totalSec);
  DEBUG_PRINTLN(F("s"));
  DEBUG_PRINT(F("backlight:"));
  DEBUG_PRINTLN(storage.backLight);

  lcd.begin();               // initialize the lcd 
  lcd.home();                   
  lcd.print(SW_NAME);  
  PRINT_SPACE
  lcd.print(VERSION);
  lcd.setCursor(0,1);
  lcd.print(F("tON:"));  
  lcd.print(storage.tDiffON);
  lcd.print(F(" tOFF:"));  
  lcd.print(storage.tDiffOFF);
  lcd.setCursor(0,2);
  lcd.print(F("Control:"));  
  lcd.print(storage.controlSensor);
  if (storage.controlSensorBojler) {
    lcd.print(" - Bojler");
  } else {
    lcd.print(" - Room");
  }

  keypad.begin();
  //keypad.addEventListener(keypadEvent); //add an event listener for this keypad  
  
  mySerial.begin(mySERIAL_SPEED);
  pinMode(LEDPIN,OUTPUT);
  
  dsInit();

#ifdef flowSensor
   pinMode(flowsensor, INPUT);
   digitalWrite(flowsensor, HIGH); // Optional Internal Pull-Up
   attachInterrupt(0, flow, RISING); // Setup Interrupt
   cloopTime = 0;
#endif
  
  pinMode(RELAY1PIN, OUTPUT);
  
  digitalWrite(RELAY1PIN, relay1);
  lastOn=millis();

#ifdef setEEPROM
  setEEPROMFunc();
#endif
  
  if (MyRstFlags==4) status = STATUS_STARTAFTER_BROWNOUT;
  if (MyRstFlags==5) status = STATUS_STARTAFTER_POWERON;
  if (MyRstFlags==8) status = STATUS_STARTAFTER_WATCHDOGOREXTERNAL;
  else status = STATUS_AFTER_START;

  DEBUG_PRINT(F("Status:"));
  DEBUG_PRINTLN(status);
  
  if (storage.backLight==true) {
    lcd.backlight();
  }
  else {
    lcd.noBacklight();
  }
  
  lcd.clear();
} //setup


//----------------------------------------L O O P ----------------------------------------------------------------------------------
void loop() {
#ifdef watchdog
  wdt_reset();
#endif
  
  if (numberOfDevices>0) {
    tempMeas();
    calcPowerAndEnergy();
    mainControl();
  }
#ifdef flowSensor  
  calcFlow();
#endif
  lcdShow();
  
  if (numberOfDevices>0) {
    if (millis() - lastSend >= SEND_DELAY) {
      sendDataSerial();
      lastSend = millis();
    }
  }
  keyBoard();
  
  if (millis() - milisLastRunMinOld > 60000) {
    milisLastRunMinOld = millis();
    lastRunMin += 1;
  }
    
} //loop


//--------------------------------------- F U N C T I O N S -----------------------------------------------------------------------------------
void mainControl() {
  //safety function
  if ((tP1In >= SAFETY_ON) || (tP1Out >= SAFETY_ON) || (tP2In >= SAFETY_ON) || (tP2Out >= SAFETY_ON)) {
    relay1=LOW; //relay ON
    //DEBUG_PRINTLN(F("SAFETY CONTROL!!!!"));
  } else if (manualON) {
    //DEBUG_PRINTLN(F("MANUAL CONTROL!!!!"));
  } else {
    //pump is ON - relay ON = LOW
    if (relay1==LOW) { 
      //save totalEnergy to EEPROM
      if ((millis() - lastWriteEEPROM) > LAST_WRITE_EEPROM_DELAY) {
        saveConfig();
        //writeTotalEEPROM(STATUS_WRITETOTALTOEEPROM_DELAY);
      }
      //if (((tP2Out - tControl) < tDiffOFF && (tP2In < tP2Out) || ) /*|| (int)getPower() < powerOff)*/) { //switch pump ON->OFF
      if (((tP2Out - tControl) < storage.tDiffOFF) && (millis() - DELAY_AFTER_ON >= lastOffOn)) { //switch pump ON->OFF
        DEBUG_PRINT(F("millis()="));
        DEBUG_PRINT(millis());
        DEBUG_PRINT(F(" delayAfterON="));
        DEBUG_PRINT(DELAY_AFTER_ON);
        DEBUG_PRINT(F(" lastOffOn="));
        DEBUG_PRINT(lastOffOn);
        DEBUG_PRINT(F(" tP2Out="));
        DEBUG_PRINT(tP2Out);
        DEBUG_PRINT(F("tControl="));
        DEBUG_PRINTLN(tControl);
        relay1=HIGH; //relay OFF = HIGH
        //digitalWrite(RELAY1PIN, relay1);
        lastOff=millis();
        lastOn4Delay=0;
        //save totalEnergy to EEPROM
        saveConfig();
        //writeTotalEEPROM(STATUS_WRITETOTALTOEEPROM_ONOFF);
      }
    } else { //pump is OFF - relay OFF = HIGH
      //if ((((tP2Out - tControl) >= tDiffON) || ((tP2In - tControl) >= tDiffON))) { //switch pump OFF->ON
      if ((tP1Out - tControl) >= storage.tDiffON || (tP2Out - tControl) >= storage.tDiffON) { //switch pump OFF->ON
        relay1=LOW; //relay ON = LOW
        //digitalWrite(RELAY1PIN, relay1);
        lastOn = millis();
        lastOffOn = lastOn;
        if (lastOn4Delay==0) {
          lastOn4Delay = lastOn;
        }
        if ((millis()-lastOff)>=DAY_INTERVAL) { //first ON in actual day
          lcd.clear();
          lastOff=millis();
          energyADay=0;
          //energyDiff=0.0;
          msDayON=0;
          tMaxOut=T_MIN;
          tMaxIn=T_MIN;
          tMaxBojler=T_MIN;
        }
      }
    }
  }
  digitalWrite(RELAY1PIN, relay1);
}

void tempMeas() {
 if (!dsMeasStarted) {
    //start sampling
    dsMeasStarted=true;
    dsSensors.requestTemperatures(); 
    //digitalWrite(13,HIGH);
    lastDsMeasStartTime = millis();
  }
  else if (dsMeasStarted && (millis() - lastDsMeasStartTime>DS_MEASSURE_INTERVAL)) {
    dsMeasStarted=false;
    firstMeasComplete=true;
    //digitalWrite(13,LOW);
    //saving temperatures into variables
    for (byte i=0;i<numberOfDevices; i++) {
      float tempTemp=T_MIN;
      for (byte j=0;j<10;j++) { //try to read temperature ten times
        //tempTemp = dsSensors.getTempCByIndex(i);
        tempTemp = dsSensors.getTempC(tempDeviceAddresses[i]);
        if (tempTemp>=-55) {
          break;
        }
      }

      sensor[i] = tempTemp;
    } 
    // tP1In       = sensor[7];    
    // tP1Out      = sensor[4];
    // tP2In       = sensor[2];
    // tP2Out      = sensor[1];
    // tBojlerIn   = sensor[5];
    // tBojlerOut  = sensor[3];
    // tRoom       = sensor[6];
    // tBojler     = sensor[0];
    tP1In       = sensor[storage.sensorOrder[0]];    
    tP1Out      = sensor[storage.sensorOrder[1]];
    tP2In       = sensor[storage.sensorOrder[2]];
    tP2Out      = sensor[storage.sensorOrder[3]];
    tBojlerIn   = sensor[storage.sensorOrder[4]];
    tBojlerOut  = sensor[storage.sensorOrder[5]];
    tRoom       = sensor[storage.sensorOrder[6]];
    tBojler     = sensor[storage.sensorOrder[7]];
    tControl    = sensor[storage.controlSensor];

    /*
    DEBUG_PRINT(F("P1 In:"));
    DEBUG_PRINTLN(tP1In);
    DEBUG_PRINT(F("P1 Out:"));
    DEBUG_PRINTLN(tP1Out);
    DEBUG_PRINT(F("P2 In:"));
    DEBUG_PRINTLN(tP2In);
    DEBUG_PRINT(F("P2 Out:"));
    DEBUG_PRINTLN(tP2Out);
    DEBUG_PRINT(F("Room:"));
    DEBUG_PRINTLN(tRoom);
    DEBUG_PRINT(F("Bojler:"));
    DEBUG_PRINTLN(tBojler);
    DEBUG_PRINT(F("Bojler In:"));
    DEBUG_PRINTLN(tBojlerIn);
    DEBUG_PRINT(F("Bojler Out:"));
    DEBUG_PRINTLN(tBojlerOut);
    DEBUG_PRINT(F("Control:"));
    DEBUG_PRINTLN(tControl);
*/
    
    if (tP2Out>tMaxOut)       tMaxOut      = tP2Out;
    if (tP2In>tMaxIn)         tMaxIn       = tP2In;
    if (tBojler>tMaxBojler)   tMaxBojler   = tBojler;
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
  float t1;
  float t2;
  
  if (relay1==LOW) {  //pump is ON
    if (storage.controlSensorBojler) {
      t1 = tBojlerIn;
      t2 = tBojlerOut;
    } else {
      t1 = tP2Out;
      t2 = tP1In;
    }
    if (t1>t2) {
      msDayON+=(millis()-lastOn);
      msDiff+=(millis()-lastOn);
      if (msDiff >= 1000) {
        totalSec+=msDiff/1000;
        msDiff=msDiff%1000;
      }
      power = getPower(t1, t2); //in W
      //DEBUG_PRINTLN(F(power);
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

//take care of some special events
void keypadEvent(KeypadEvent key){
  lcd.setCursor(0,1);
  lcd.print(key);
  // DEBUG_PRINTLN(F(key);
  // DEBUG_PRINTLN(F(keypad.getState());
  switch (keypad.getState()){
    case PRESSED:
      switch (key){
        case '#': break;
        case '*': 
          //digitalWrite(ledPin,!digitalRead(ledPin));
        break;
      }
    break;
    case RELEASED:
      switch (key){
        case '*': 
          //digitalWrite(ledPin,!digitalRead(ledPin));
          //blink = false;
        break;
      }
    break;
    case HOLD:
      switch (key){
        case '*': 
        //blink = true;
        break;
      }
    break;
  }
}


void keyBoard() {
  char key = keypad.getKey();
  if (key!=NO_KEY){
    lcd.clear();
    //DEBUG_PRINTLN(key);
    /*
    Keyboard layout
    -----------
    | 1 2 3 A |
    | 4 5 6 B |
    | 7 8 9 C |
    | * 0 # D |
    -----------
    SETUP - vstup do režimu *, dopredu B, dozadu A, nahoru D, dolu C, uložení hodnot a výstup #*/
    #define MAXSETUP      111
    #define MINSETUP      100
    /*
    100  - TDiffON           - nastavení teploty                     - klávesy C,D
    101  - TDiffOFF          - nastavení teploty                     - klávesy C,D
    102  - Panel1 vstup      - výběr čidla                           - klávesy C,D
    103  - Panel1 výstup     - výběr čidla                           - klávesy C,D
    104  - Panel2 vstup      - výběr čidla                           - klávesy C,D
    105  - Panel2 výstup     - výběr čidla                           - klávesy C,D
    106  - Bojler vstup      - výběr čidla                           - klávesy C,D
    107  - Bojler výstup     - výběr čidla                           - klávesy C,D
    108  - Bojler            - výběr čidla                           - klávesy C,D
    109  - Teplota místnost  - výběr čidla                           - klávesy C,D 
    110  - Control sensor    - výběr čidla podle kterého se spíná    - klávesy C,D
    111  - Control sensor    - výběr čidla bojler/místnost           - klávesy C,D
    
    INFO
    1 - total energy
    2 - TDiffON
    3 - TDiffOFF
    4 - prutok
    A - BACKLIGHT ON/OFF
    5 - Max IN OUT temp
    6 - Max bojler
    B - Save total energy to EEPROM
    7 - Max power today
    8 - Control sensor
    9 - total time
    C - RESET
    * - SETUP
    0 - main display
    D - manual/auto
    */
    
    //SETUP MODE
    if (displayMode==SETUP) {
      if (key=='#') {
        displayMode=INFO;
        display = 0;
        saveConfig();
      }
      if (key=='B') {
        display++;
        //lcd.clear();
        if (display>MAXSETUP) {
          display = MINSETUP;
        }
      }
      if (key=='A') {
        display--;
        //lcd.clear();
        if (display<MINSETUP) {
          display = MAXSETUP;
        }
      }
      if (key=='D') {
        if (display==DISPLAY_T_DIFF_ON_SETUP) {
          storage.tDiffON++;
        } else if (display==DISPLAY_T_DIFF_OFF_SETUP) {
          storage.tDiffOFF++;
        } else if (display==DISPLAY_P1OUT_SETUP) {
        } else if (display==DISPLAY_P2IN_SETUP) {
        } else if (display==DISPLAY_P2OUT_SETUP) {
        } else if (display==DISPLAY_BOJLERIN_SETUP) {
        } else if (display==DISPLAY_BOJLEROUT_SETUP) {
        } else if (display==DISPLAY_BOJLER_SETUP) {
        } else if (display==DISPLAY_ROOM_SETUP) {
        } else if (display==DISPLAY_CONTROL_SENSOR_SETUP) {
          if (storage.controlSensor>numberOfDevices) {
            storage.controlSensor=0;
          } else {
            storage.controlSensor++;
          }
        } else if (display==DISPLAY_CONTROL_SENSOR_SETUP_TEXT) {
          if (storage.controlSensorBojler) {
            storage.controlSensorBojler=false;
          } else {
            storage.controlSensorBojler=true;
          }
        }
      }
      if (key=='C') {
        if (display==DISPLAY_T_DIFF_ON_SETUP) {
          storage.tDiffON--;
          if (storage.tDiffON<0) {
            storage.tDiffON=0;
          }
        } else if (display==DISPLAY_T_DIFF_OFF_SETUP) {
          storage.tDiffOFF--;
          if (storage.tDiffOFF<0) {
            storage.tDiffOFF=0;
          }
        } else if (display==DISPLAY_P1OUT_SETUP) {
        } else if (display==DISPLAY_P2IN_SETUP) {
        } else if (display==DISPLAY_P2OUT_SETUP) {
        } else if (display==DISPLAY_BOJLERIN_SETUP) {
        } else if (display==DISPLAY_BOJLEROUT_SETUP) {
        } else if (display==DISPLAY_BOJLER_SETUP) {
        } else if (display==DISPLAY_ROOM_SETUP) {
        } else if (display==DISPLAY_CONTROL_SENSOR_SETUP) {
          if (storage.controlSensor==0) {
            storage.controlSensor=numberOfDevices;
          } else {
            storage.controlSensor--;
          }
        } else if (display==DISPLAY_CONTROL_SENSOR_SETUP_TEXT) {
          if (storage.controlSensorBojler) {
            storage.controlSensorBojler=false;
          } else {
            storage.controlSensorBojler=true;
          }
        }
      }
    //INFO MODE
    } else {
      if (key=='*') {
        displayMode=SETUP;
        display = MINSETUP;
      }
      if (key=='D') {
        manualON = !manualON;
        if (manualON) {
          relay1=LOW;
        } else {
          relay1=HIGH;
        }
      }
      if (key=='C') {
        saveConfig();
        asm volatile ("  jmp 0");  
      }
      else if (key=='A') {
        storage.backLight=!storage.backLight;
        if (storage.backLight==true) {
          lcd.backlight();
        }
        else {
          lcd.noBacklight();
        }
        saveConfig();
      }
      else if (key=='0') { 
        display=DISPLAY_MAIN;
      }
      else if (key=='1') { 
        display=DISPLAY_TOTAL_ENERGY;
      }
      else if (key=='2') { 
        display=DISPLAY_T_DIFF_ON;
      }
      else if (key=='3') { 
        display=DISPLAY_T_DIFF_OFF;
      }
      else if (key=='4') {
        display=DISPLAY_FLOW;
      }
      else if (key=='5') { 
        display=DISPLAY_MAX_IO_TEMP;
      }
      else if (key=='6') { 
        display=DISPLAY_MAX_BOJLER;
      }
      else if (key=='7') { 
        display=DISPLAY_MAX_POWER_TODAY;
      }
      else if (key=='8') { 
        display=DISPLAY_CONTROL_SENSOR;
      }
      else if (key=='9') { 
        display=DISPLAY_TOTAL_TIME;
      }
      else if (key=='B') { //Save total energy to EEPROM
        saveConfig();
        lcd.setCursor(0,3);
        lcd.print(F("Energy "));
        lcd.print(enegyWsTokWh(storage.totalEnergy));
        lcd.print(F(" kWh"));
      }
    }
    key = ' ';
  }
}

void displayTemp(int x, int y, float value, bool des) {
  /*
  value     des=true   des=false
            0123       0123
  89.3      89.3       89
  10.0      10.0       10
   9.9       9.9        9
   1.1       1.1        1
   0.9       0.9        0
   0.1       0.0        0
   0.0       0.0        0
  -0.1      -0.1       -0
  -0.9      -0.9       -0
  -1.0      -1.0       -1
  -9.9      -9.9       -9
 -10.0      -10        -10
 -25.2      -25        -25  
   */
  lcd.setCursor(x,y);
  
  //DEBUG_PRINTLN(F(value);
  
  if (value<10.f && value>=0.f) {
    //DEBUG_PRINT(F("_"));
    lcd.print(F(" "));
  } else if (value<0.f && value>-10.f) {
    //DEBUG_PRINT(F("_"));
    lcd.print(F("-"));
  } else if (value<-10.f) {
    des = false;
    //DEBUG_PRINT("-"));
  }
  
  if (value>=100.f) {
    value=value-100.f;
  }
 
  lcd.print(abs((int)value));
  if (des) {
    lcd.print(F("."));
    lcd.print(abs((int)(value*10)%10));
  }
  lcd.print(F(" "));
}

#ifdef time
//display time on LCD
void lcd2digits(int number) {
  if (number >= 0 && number < 10) {
    lcd.write('0');
  }
  lcd.print(number);
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    DEBUG_WRITE('0');
  }
  DEBUG_PRINT(number);
}

void displayTime() {
  lcd.setCursor(TIMEX, TIMEY); //col,row
  if (RTC.read(tm)) {
    lcd2digits(tm.Hour);
    lcd.write(':');
    lcd2digits(tm.Minute);
    lcd.write(':');
    lcd2digits(tm.Second);
  } else {
    lcd.write('        ');
  }
  //zobrazeni hlasky o zmene uhlu kolektoru
}
#endif

#ifdef flowSensor
void calcFlow() {
  // Every second, calculate and print litres/hour
  if (millis() >= (cloopTime + 5000)) {
    // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min.
    lMin = numberOfPulsesFlow / (7.5f * ((float)(millis() - cloopTime) / 1000.f));
    cloopTime = millis(); // Updates cloopTime
    lMinCumul += lMin;
    numberOfCyclesFlow++;
    DEBUG_PRINT(F("Pulsu: "));
    DEBUG_PRINTLN(numberOfPulsesFlow);
    Serial.print(lMin, DEC); // Print litres/min
    DEBUG_PRINTLN(F(" L/min"));
    numberOfPulsesFlow = 0; // Reset Counter
  }
}
#endif

void dsInit(void) {
  dsSensors.begin();
  numberOfDevices = dsSensors.getDeviceCount();

  lcd.setCursor(0,3);
  lcd.print(numberOfDevices);
  
  if (numberOfDevices==1)
    lcd.print(F(" sensor found"));
  else
    lcd.print(F(" sensors found"));
  
  DEBUG_PRINT(F("Sensor(s):"));
  DEBUG_PRINTLN(numberOfDevices);

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

//show relay's status in column 15
void displayRelayStatus(void) {
  lcd.setCursor(RELAY1X,RELAY1Y);
  if (manualON) {
    lcd.print(F("M"));
  } else {
    if (relay1==LOW)
      lcd.print(F("T"));
    else
      lcd.print(F("N"));
  }
  if (manualON) {
    //DEBUG_PRINTLN(F("Manual"));
  } else {
  }
/*  lcd.setCursor(RELAY2X,RELAY2Y);
  if (relay2==LOW)
    lcd.print(F("T"));
  else
    lcd.print(F("N"));
*/
}

unsigned int getPower(float t1, float t2) {
  /*
  Q	0,00006	m3/s
  K	4184000	
  t vstup	56,4	°C
  t vystup	63,7	
  P = Q x K x (t1 - t2)	1832,592	W
  */
  return (lMin / 1000.0 / 60.0) * 4184000.0 * (t1 - t2);
  //return (float)energyKoef*(tBojlerOut-tBojlerIn); //in W
}

void lcdShow() {
  if (display>=100) { 
    lcd.setCursor(POZ0X,POZ0Y);
  }
  if (millis() > SHOW_INFO_DELAY + showInfo) {
    showInfo = millis();
    lcd.setCursor(0,3);
    for (byte i=0;i<14;i++) {
      PRINT_SPACE;
    }
  }
  
  if (display==DISPLAY_MAIN) {
    //    012345678901234567890
    //00  10  9  0  -0 -9.1  T    
    //01   555W   12.3kWh 124m                    
    //02   2.1l/m  55 45 48  0
    //03                123456
    #ifdef time
      displayTime();
    #endif
    displayTemp(TEMP1X,TEMP1Y, tP1In, false);
    displayTemp(TEMP2X,TEMP2Y, tP1Out, false);
    displayTemp(TEMP3X,TEMP3Y, tP2In, false);
    displayTemp(TEMP4X,TEMP4Y, tP2Out, false);
    displayTemp(TEMP5X,TEMP5Y, tControl, true);
    displayTemp(TEMP6X,TEMP6Y, tBojlerIn, false);
    displayTemp(TEMP7X,TEMP7Y, tBojlerOut, false);
    displayTemp(TEMP8X,TEMP8Y, tBojler, false);
    if ((millis()-lastOff)>=DAY_INTERVAL) {
      lcd.setCursor(0,1);
      lcd.print(F("Bez slunce "));
      lcd.print((millis() - lastOff)/1000/3600);
      lcd.print(F(" h"));
    } else {
      //zobrazeni okamziteho vykonu ve W
      //zobrazeni celkoveho vykonu za den v kWh
      //zobrazeni poctu minut behu cerpadla za aktualni den
      //0123456789012345
      // 636 0.1234 720T
      unsigned int p=(int)power;
      lcd.setCursor(POWERX,POWERY);
      if (p<10000) PRINT_SPACE
      if (p<1000) PRINT_SPACE
      if (p<100) PRINT_SPACE
      if (p<10) PRINT_SPACE
      if (power<=65534) {
        lcd.print(p);
        lcd.print(F("W"));
      }
      
      lcd.setCursor(ENERGYX,ENERGYY);
      lcd.print(enegyWsTokWh(energyADay)); //Ws -> kWh (show it in kWh)
      lcd.print(F("kWh"));
      
      lcd.setCursor(TIMEX,TIMEY);
      p=(int)(msDayON/1000/60);
      if (p<100) PRINT_SPACE
      if (p<10) PRINT_SPACE
      if (p<=999) {
        lcd.print(p); //ms->min (show it in minutes)
        lcd.print(F("m"));
      }
      lcd.setCursor(FLOWX,FLOWY);
      lcd.print(lMin);
      lcd.print(F("l/m"));
    }
    displayRelayStatus();
    lcd.setCursor(STATUSX, STATUSY);
    lcd.print(status);
    lcd.setCursor(MINRUNX, MINRUNY);
    if (lastRunMin<100000) PRINT_SPACE
    if (lastRunMin<10000) PRINT_SPACE
    if (lastRunMin<1000) PRINT_SPACE
    if (lastRunMin<100) PRINT_SPACE
    if (lastRunMin<10) PRINT_SPACE
    lcd.print(lastRunMin);
  } else if (display==DISPLAY_TOTAL_ENERGY) {
    displayInfoValue('Total energy', enegyWsTokWh(totalEnergy), 'kWh');
  } else if (display==DISPLAY_T_DIFF_ON) {
    displayInfoValue('TDiffON', storage.tDiffON, 'C');
  } else if (display==DISPLAY_T_DIFF_OFF) { 
    displayInfoValue('TDiffOFF', storage.tDiffOFF, 'C');
  } else if (display==DISPLAY_FLOW) {
    displayInfoValue('Flow', lMin, 'l/min');
  } else if (display==DISPLAY_MAX_IO_TEMP) {
    lcd.setCursor(POZ0X,POZ0Y);
    lcd.print(F("Max IN:"));
    lcd.print(tMaxIn);
    lcd.print(F("     "));
    lcd.setCursor(0,1);
    lcd.print(F("Max OUT:"));
    lcd.print(tMaxOut);
    lcd.print(F("     "));
  } else if (display==DISPLAY_MAX_BOJLER) {
    displayInfoValue('Max bojler', tMaxBojler, 'C');
  } else if (display==DISPLAY_MAX_POWER_TODAY) { 
    displayInfoValue('Max power today', maxPower, 'W');
  } else if (display==DISPLAY_CONTROL_SENSOR) {
    lcd.setCursor(POZ0X,POZ0Y);
    lcd.print(F("Control sensor"));
    lcd.setCursor(0,1);
    lcd.print(F(" ["));
    lcd.print(sensor[storage.controlSensor]);
    lcd.print(F("]   "));
    lcd.setCursor(0,2);
    if (storage.controlSensorBojler) {
      lcd.print(F("Bojler"));
    } else {
      lcd.print(F("Room"));
    }
  } else if (display==DISPLAY_TOTAL_TIME) { 
    displayInfoValue('Total time', totalSec/60/60, 'hours');
    
  } else if (display==DISPLAY_T_DIFF_ON_SETUP) {
    displayInfoValue('tDiffON', storage.tDiffON, 'C');
  } else if (display==DISPLAY_T_DIFF_OFF_SETUP) {
    displayInfoValue('tDiffOFF', storage.tDiffOFF, 'C');
  } else if (display==DISPLAY_P1IN_SETUP) {
    displayInfoValue('Panel1 IN', tP1In, 'C');
  } else if (display==DISPLAY_P1OUT_SETUP) {
    displayInfoValue('Panel1 OUT', tP1Out, 'C');
  } else if (display==DISPLAY_P2IN_SETUP) {
    displayInfoValue('Panel2 IN', tP2In, 'C');
  } else if (display==DISPLAY_P2OUT_SETUP) {
    displayInfoValue('Panel2 OUT', tP2Out, 'C');
  } else if (display==DISPLAY_BOJLERIN_SETUP) {
    displayInfoValue('Bojler IN', tBojlerIn, 'C');
  } else if (display==DISPLAY_BOJLEROUT_SETUP) {
    displayInfoValue('Bojler OUT', tBojlerOut, 'C');
  } else if (display==DISPLAY_BOJLER_SETUP) {
    displayInfoValue('Bojler', tBojler, 'C');
  } else if (display==DISPLAY_ROOM_SETUP) {
    displayInfoValue('Room', tRoom, 'C');
  } else if (display==DISPLAY_CONTROL_SENSOR_SETUP) {
    displayInfoValue('Control sensor', tControl, 'C');
  }
}

#ifdef setEEPROM
void setEEPROMFunc() {
  storage.totalEnergy = 162091 * 3600;  //Wh
  storage.totalSec    = 5820180;        //sec
  //writeTotalEEPROM(STATUS_WRITETOTALTOEEPROM_MANUAL);
  //readTotalEEPROM();
  saveConfig();
}
#endif

float enegyWsTokWh(float e) {
  return e/3600.f/1000.f;
}


void sendDataSerial() {
  //send to ESP8266 unit via UART
  //data sended:
  //S tempIN Panel1
  //T tempOUT Panel1
  //I tempIN Panel2
  //O tempOUT Panel2
  //M room temp
  //B bojler temp
  //C bojler IN
  //D bojler OUT
  //Q prutok
  //R relay status

  //data sended:
  //#B;25.31#M;25.19#S;25.10#T;50.5#I;25.10#O;50.5#C;50.5#D;40.5#Q;10.2#R;1$3600177622*

  if (firstMeasComplete==false) return;
  DEBUG_PRINT(F("DATA:"));
  digitalWrite(LEDPIN,HIGH);
  crc = ~0L;
  send(START_BLOCK);
  send('B');
  send(DELIMITER);
  send(tBojler);

  send(START_BLOCK);
  send('M');
  send(DELIMITER);
  send(tRoom);

  send(START_BLOCK);
  send('S');
  send(DELIMITER);
  send(tP1In);

  send(START_BLOCK);
  send('T');
  send(DELIMITER);
  send(tP1Out);

  send(START_BLOCK);
  send('I');
  send(DELIMITER);
  send(tP2In);

  send(START_BLOCK);
  send('O');
  send(DELIMITER);
  send(tP2Out);

  //bojler vstup
  send(START_BLOCK);
  send('C');
  send(DELIMITER);
  send(tBojlerIn);

  //bojler vystup
  send(START_BLOCK);
  send('D');
  send(DELIMITER);
  send(tBojlerOut);

  
  send(START_BLOCK);
  send('Q');
  send(DELIMITER);
  send(lMinCumul / (float)numberOfCyclesFlow);

  lMinCumul = 0;
  numberOfCyclesFlow = 0;

  send(START_BLOCK);
  send('R');
  send(DELIMITER);
  if (relay1==LOW)
    send('1');
  else
    send('0');

  send(END_BLOCK);

  //myDEBUG_PRINT(crc);
  send(END_TRANSMITION);
  mySerial.flush();
  DEBUG_PRINTLN();
}

void send(char s) {
  send(s, ' ');
}


void send(char s, char type) {
  if (type=='X') {
    DEBUG_PRINTHEX(s);
    mySerial.print(s, HEX);
  }
  else {
    DEBUG_PRINT(s);
    mySerial.print(s);
  }
  crc_string(byte(s));
}

void send(byte s) {
  send(s, ' ');
}

void send(byte s, char type) {
  if (type=='X') {
    DEBUG_PRINTHEX(s);
    mySerial.print(s, HEX);
  }
  else {
    DEBUG_PRINT(s);
    mySerial.print(s);
  }
  crc_string(s);
}

void crc_string(byte s)
{
  crc = crc_update(crc, s);
  crc = ~crc;
}

unsigned long crc_update(unsigned long crc, byte data)
{
    byte tbl_idx;
    tbl_idx = crc ^ (data >> (0 * 4));
    crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
    tbl_idx = crc ^ (data >> (1 * 4));
    crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
    return crc;
}


void send(unsigned long s) {
  DEBUG_PRINT(s);
  mySerial.print(s);
}

void send(unsigned int s) {
  DEBUG_PRINT(s);
  mySerial.print(s);
}

void send(float s) {
  char tBuffer[8];
  dtostrf(s,0,2,tBuffer);
  for (byte i=0; i<8; i++) {
    if (tBuffer[i]==0) break;
    send(tBuffer[i]);
  }
}

void loadConfig() {
  DEBUG_PRINTLN(F("Load config from EEPROM"));
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (!isConfigVersionChanged()) {
    for (unsigned int t=0; t<sizeof(storage); t++) {
      *((char*)&storage + t) = EEPROM.read(CONFIG_START + t);
      DEBUG_PRINTLN(EEPROM.read(CONFIG_START + t));
    }
  }
}

void saveConfig() {
  for (unsigned int t=0; t<sizeof(storage); t++) {
    EEPROM.update(CONFIG_START + t, *((char*)&storage + t));
  }
  lastWriteEEPROM = millis();
  lcd.setCursor(0,3);
  lcd.print(F("Setup saved"));
  showInfo = millis();
}

void testConfigChange() {
  DEBUG_PRINT(F("Test config change - "));
  if (!isConfigVersionChanged()) {
      DEBUG_PRINTLN(F("NO change."));
  } else {
    DEBUG_PRINTLN(F("change."));
    if (EEPROM.read(CONFIG_START + 0) == 'v' &&
        EEPROM.read(CONFIG_START + 1) == '0' &&
        EEPROM.read(CONFIG_START + 2) == '3') {
      DEBUG_PRINTLN(F("Zmena konfigurace na verzi v04"));
      storage.sensorOrder[0] = 7;
      storage.sensorOrder[1] = 4;
      storage.sensorOrder[2] = 2;
      storage.sensorOrder[3] = 1;
      storage.sensorOrder[4] = 5;
      storage.sensorOrder[5] = 3;
      storage.sensorOrder[6] = 6;
      storage.sensorOrder[7] = 0;
      storage.controlSensorBojler=false;
      storage.version[0]='v';
      storage.version[1]='0';
      storage.version[2]='4';
      saveConfig();
    }
  }
}

bool isConfigVersionChanged() {
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
    EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
    EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2])
    return false;
  else
    return true;
}

#ifdef verbose
void printConfigVersion() {
  DEBUG_PRINT(F("Config version:"));
  Serial.write(EEPROM.read(CONFIG_START + 0));
  Serial.write(EEPROM.read(CONFIG_START + 1));
  Serial.write(EEPROM.read(CONFIG_START + 2));
  DEBUG_PRINTLN();
}
#endif

#ifdef time
int getAngle() {
  if (RTC.read(tm)) {
    if (!zobrazNastavovaciHlasku) {
      if (tm.Day==1) {
        zobrazNastavovaciHlasku = true;
        uhelKolektoru = uhelMesic[tm.Month - 1];
      }
    }
  }
}
#endif


void displayInfoValue(char text1, float value, char text2) {
  lcd.setCursor(POZ0X,POZ0Y);
  lcd.print(text1);
  lcd.setCursor(POZ0X,POZ1Y);
  lcd.print(value);
  lcd.print(" ");
  lcd.print(text2);
  lcd.print("          ");
}