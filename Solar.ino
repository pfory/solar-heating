/*
--------------------------------------------------------------------------------------------------------------------------

               SOLAR - control system for solar unit

Petr Fory pfory@seznam.cz
SVN  - https://code.google.com/p/solar-heating/

Version history:
0.76 - 7.8.2014
0.75 - 31.7.2014  funkcni pocitani totalSec a totalPower
0.74 - 30.7.2014  add delay after ON, prevent cyclic switch ON-OFF-ON....
0.73 - 29.7.2014  add totalSec, oprava posilani totalPower
0.72 - 14.6.2013  zmena spinani teplot
0.71 - 5.6.2014   optiboot, watchdog
0.70 - 21.5.2014
0.69 - 21.5.2014
0.68 - 20.5.2014
0.67 - 15.6.2014
0.66 - 6.4.2014
0.65 - 26.3.2014
0.60 - 16.3.2014
0.50 - 1.12.2013
0.41 - 20.10.2013

--------------------------------------------------------------------------------------------------------------------------
HW
Pro Mini 328 data are sent via serial line to comunication unit
I2C display
2 Relays module
DALLAS
keyboard

Pro Mini 328 Layout
------------------------------------------
A0 						 - DALLAS temperature sensors
A1 						 - relay 1
A2             - relay 2
A3             - free
A4 (D20 MEGA)  - I2C display SDA
A5 (D21 MEGA)  - I2C display SCL
D0 						 - Rx
D1 						 - Tx
D2 						 - keyboard
D3 						 - keyboard
D4 						 - keyboard
D5 						 - keyboard
D6 						 - keyboard
D7 						 - keyboard
D8 						 - keyboard
D9 						 - keyboard
D10 					 - free
D10 					 - free
D11 					 - free
D12 					 - free
D13 					 - free
--------------------------------------------------------------------------------------------------------------------------
*/

#include <Wire.h> 
#define watchdog //enable this only on board with UNO bootloader
#ifdef watchdog
#include <avr/wdt.h>
#endif

#define serial //serial monitor
unsigned int const SERIAL_SPEED=9600;

#include <avr/pgmspace.h>
unsigned long crc;
static PROGMEM prog_uint32_t crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

#define START_BLOCK 			'#'
#define DELIMITER 				';'
#define END_BLOCK 				'$'
#define END_TRANSMITION 	'*'

#define LEDPIN 13

#include <SoftwareSerial.h>
#define RX 10
#define TX 11
SoftwareSerial mySerial(RX, TX);

const unsigned int serialTimeout=2000;

#include <LiquidCrystal_I2C.h>
#define LCDADDRESS 0x20
#define EN 					2
#define RW 					1
#define RS 					0
#define D4 					4
#define D5 					5
#define D6 					6
#define D7 					7
#define BACKLIGHT 	3
#define POL 				POSITIVE
#define LCDROWS			2
#define LCDCOLS			16
LiquidCrystal_I2C lcd(LCDADDRESS,EN,RW,RS,D4,D5,D6,D7,BACKLIGHT, POL);  // set the LCD

// Create a set of new characters
/*const uint8_t charBitmap[][8] = {
   { 0xc, 0x12, 0x12, 0xc, 0, 0, 0, 0 },
   { 0x6, 0x9, 0x9, 0x6, 0, 0, 0, 0 },
   { 0x0, 0x6, 0x9, 0x9, 0x6, 0, 0, 0x0 },
   { 0x0, 0xc, 0x12, 0x12, 0xc, 0, 0, 0x0 },
   { 0x0, 0x0, 0xc, 0x12, 0x12, 0xc, 0, 0x0 },
   { 0x0, 0x0, 0x6, 0x9, 0x9, 0x6, 0, 0x0 },
   { 0x0, 0x0, 0x0, 0x6, 0x9, 0x9, 0x6, 0x0 },
   { 0x0, 0x0, 0x0, 0xc, 0x12, 0x12, 0xc, 0x0 }
   
};
*/

#include <OneWire.h>
#define ONE_WIRE_BUS A0
OneWire onewire(ONE_WIRE_BUS); // pin for onewire DALLAS bus
#define dallasMinimal //-956 Bytes
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
float tempDiffON                          = 25.0; //difference between room temperature and solar OUT (sensor 2 - sensor 1) to set relay ON
float tempDiffOFF                         = 15.0; //difference between room temperature and solar OUT (sensor 2 - sensor 1) to set relay OFF
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

//MODE
byte modeSolar                            = 0;
bool manualSetFromKeyboard                = false;
bool firstMeasComplete                    = false;
bool manualON                             = false;

float tIn		                              = 0; //input medium temperature to solar panel
float tOut	                              = 0; //output medium temperature to solar panel
float tRoom	                              = 0; //room temperature
float tBojler                             = 0; //boiler temperature
float tControl                            = 0; //temperature which is used as control temperature

//maximal temperatures
float tMaxIn			                        = 0; //maximal input temperature (just for statistics)
float tMaxOut			                        = 0; //maximal output temperature (just for statistics)
float tMaxBojler 	                        = 0; //maximal boiler temperature (just for statistics)

byte controlSensor                          = 0; //control sensor index

//int powerOff                            = 200;     //minimalni vykon, pokud je vykon nizssi, rele vzdy vypne
float safetyON                            = 80.0; //teplota, pri niz rele vzdy sepne

enum mode {NORMAL, POWERSAVE};
mode powerMode                            = NORMAL;

byte display                              = 0;
//status  0/1 - normal
//        2 - after start
//        3 - write total to EEPROM - delay
//        4 - write total to EEPROM - ON->OFF
//        5 - write total to EEPROM - manual
byte status                               = 0;

//0123456789012345
//15.6 15.8 15.8 V
//1234 0.12 624
#define TEMP1X 		                          0
#define TEMP1Y 		                          0
#define TEMP2X 		                          5
#define TEMP2Y 		                          0
#define TEMP3X 		                          10
#define TEMP3Y 		                          0
//#define TEMP4X 	                          9
//#define TEMP4Y 	                          1
#define POWERX 		                          0
#define POWERY 		                          1
#define ENERGYX 	                          7
#define ENERGYY 	                          1
#define TIMEX 		                          12
#define TIMEY 		                          1
                          
                          
#define RELAY1X 	                          15
#define RELAY1Y 	                          0
/*#define RELAY2X                           15
#define RELAY2Y 	                          1
*/                          
                          
#define RELAY1PIN                           A1
#define RELAY2PIN                           A2

//HIGH - relay OFF, LOW - relay ON
bool relay1                               = HIGH; 
bool relay2                               = HIGH;

#define keypad
#ifdef keypad
#include <Keypad.h>
const byte ROWS                           = 4; //four rows
const byte COLS                           = 4; //four columns
//define the symbols on the buttons of the keypads
char hexaKeys[ROWS][COLS]                 = {
                                            {'1','4','7','*'},
                                            {'2','5','8','0'},
                                            {'3','6','9','#'},
                                            {'A','B','C','D'}
};
byte rowPins[ROWS]                        = {5,4,3,2}; //connect to the row pinouts of the keypad
byte colPins[COLS]                        = {9,8,7,6}; //connect to the column pinouts of the keypad

//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 
#endif

//EEPROM
#include <EEPROM.h>
byte const tempDiffONEEPROMAdrH	        	= 0;
byte const tempDiffONEEPROMAdrL	        	= 1;
byte const tempDiffOFFEEPROMAdrH        	= 2; 
byte const tempDiffOFFEEPROMAdrL        	= 3;
byte const totalEnergyEEPROMAdrH        	= 4;
byte const totalEnergyEEPROMAdrM        	= 5;
byte const totalEnergyEEPROMAdrS        	= 6;
byte const totalEnergyEEPROMAdrL        	= 7;
byte const controlSensorEEPROMAdr	          = 8;
byte const totalSecEEPROMAdrH	            = 9;
byte const totalSecEEPROMAdrM	            = 10;
byte const totalSecEEPROMAdrS	            = 11;
byte const totalSecEEPROMAdrL	            = 12;

//SW name & version
float const   versionSW                   = 0.76;
char  const   versionSWString[]           = "Solar v"; 

//--------------------------------------------------------------------------------------------------------------------------
void setup() {
#ifdef watchdog
	wdt_enable(WDTO_8S);
#endif

	lcd.begin(LCDCOLS,LCDROWS);               // initialize the lcd 
  // Switch on the backlight
  pinMode(BACKLIGHT, OUTPUT);
  digitalWrite(BACKLIGHT, HIGH);
	mySerial.begin(SERIAL_SPEED);
	pinMode(LEDPIN,OUTPUT);
	
#ifdef serial
  Serial.begin(SERIAL_SPEED);
	Serial.print(versionSWString);
  Serial.println(versionSW);
#endif
  lcd.home();                   // go home
  lcd.print(versionSWString);  
  lcd.print(" ");
  lcd.print (versionSW);
  delay(1000);
  lcd.clear();

  dsInit();

	lcd.clear();

  pinMode(RELAY1PIN, OUTPUT);
  
  digitalWrite(RELAY1PIN, relay1);
  lastOn=millis();

  readAndSetONOFFFromEEPROM();
  readTotalEEPROM();

  //#define setTE
#ifdef setTE
  setTE();
#endif

  readAndSetControlSensorFromEEPROM();
 
  status = 2;

} //setup

void loop() {
#ifdef watchdog
	wdt_reset();
#endif
  
  tempMeas();
  calcPowerAndEnergy();
  mainControl();
  lcdShow(); //show display
  
  if (lastOff > 0 && (millis() - lastOff>dayInterval)) {
      lastOff = 0;
  }

  communication();

  keyBoard();
} //loop


void mainControl() {
  //safety function
  if ((tOut || tIn) >= safetyON) {
    relay1=LOW; //relay ON
  } else if (manualON) {
  } else {
    //pump is ON - relay ON = LOW
    if (relay1==LOW) { 
      //save totalEnergy to EEPROM
      if ((millis() - lastWriteEEPROM) > lastWriteEEPROMDelay) {
        writeTotalEEPROM(3);
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
        relay1=HIGH; //relay OFF = HIGH
        //digitalWrite(RELAY1PIN, relay1);
        lastOff=millis();
        lastOn4Delay=0;
        //save totalEnergy to EEPROM
        writeTotalEEPROM(4);
      }
    } else { //pump is OFF - relay OFF = HIGH
      //if ((((tOut - tControl) >= tempDiffON) || ((tIn - tControl) >= tempDiffON))) { //switch pump OFF->ON
      if ((tOut - tControl) >= tempDiffON) { //switch pump OFF->ON
        relay1=LOW; //relay ON = LOW
        //digitalWrite(RELAY1PIN, relay1);
        lastOn = millis();
        lastOffOn = lastOn;
        if (lastOn4Delay==0) {
          lastOn4Delay = lastOn;
        }
        if (lastOff==0) { //first ON in actual day
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
		tOut 	    = sensor[1];
		tIn	 	    = sensor[2];
		tRoom     = sensor[3];
		tBojler   = sensor[0];
    tControl  = sensor[controlSensor];
		
		if (tOut>tMaxOut) 			tMaxOut 		= tOut;
		if (tIn>tMaxIn) 				tMaxIn  		= tIn;
		if (tBojler>tMaxBojler) tMaxBojler 	= tBojler;
/*#ifdef serial
		Serial.print("tOut:");
    Serial.print(tOut);
		Serial.print(" tIn:");
    Serial.print(tIn);
		Serial.print(" tRoom:");
    Serial.print(tRoom);
		Serial.print(" tBojler:");
    Serial.print(tBojler);
		Serial.print(" Ridici teplota:");
    Serial.println(tControl);
#endif*/
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
  if (relay1==LOW) {  //pump is ON
    if (tIn<tOut) {
      msDayON+=(millis()-lastOn);
      msDiff+=(millis()-lastOn);
#ifdef serial
      //Serial.print("msDiff=");
      //Serial.println(msDiff);
#endif
      if (msDiff >= 1000) {
#ifdef serial
        //Serial.print("totalSec (before)=");
        //Serial.println(totalSec);
#endif
        totalSec+=msDiff/1000;
        msDiff=msDiff%1000;
#ifdef serial
        //Serial.print("totalSec (after)=");
        //Serial.println(totalSec);
        //Serial.print("msDiff (zbytek)=");
        //Serial.println(msDiff);
#endif
      }
      power = getPower(); //in W
      if (power > maxPower) {
        maxPower = power;
      }
      energyDiff += (float)((millis()-lastOn))*(float)power/1000.f; //in Ws
#ifdef serial
      //Serial.print("energyDiff=");
      //Serial.println(energyDiff);
#endif
      if (energyDiff >= 3600.f) { //Wh
#ifdef serial
        //Serial.print("TE (before)=");
        //Serial.println(totalEnergy);
        //Serial.print("energyDiff (before)=");
        //Serial.println(energyDiff);
#endif
        totalEnergy += (unsigned long)energyDiff;
        energyADay  += (unsigned long)energyDiff;
        energyDiff = energyDiff - (long)energyDiff;
#ifdef serial
        //Serial.print("TE (after)=");
        //Serial.println(totalEnergy);
        //Serial.print("energyDiff (after)=");
        //Serial.println(energyDiff);
#endif
      }
    } else {
      power=0;
    }
    lastOn = millis();
  } else {
    power=0;
  }

/*#ifdef serial
  Serial.print("Power:");
  Serial.print(power);
  Serial.println("[W]");
  Serial.print("Energy:");
  Serial.print(energyADay/1000.f/3600.f);
  Serial.println("[kWh]");
  Serial.print("Pump ON:");
  Serial.print((int)(msDayON/1000));
  Serial.println("[s]");
#endif*/
}

void keyBoard() {
#ifdef keypad
  char customKey = customKeypad.getKey();
  if (customKey){
		/*
		Keyboard layout
		-----------
		| 1 2 3 A |
		| 4 5 6 B |
		| 7 8 9 C |
		| * 0 # D |
		-----------
    1 - total energy
    2 - TempDiffON
    3 - TempDiffOFF
    A - BACKLIGHT OFF
    4 - Energy koef
    5 - Max IN OUT temp
    6 - Max bojler
    B - BACKLIGHT ON
    7 - Max power today
    8 - Control sensor
    9 - total time
    C - DISPLAY CLEAR
    * - Save total energy to EEPROM
    0 -
    # - Select control sensor
    D - manual/auto
		*/
		if (customKey=='D') {
			manualON = !manualON;
      if (manualON) {
        relay1=LOW;
        manualSetFromKeyboard = true;
        //digitalWrite(RELAY1PIN, LOW);
      } else {
        relay1=HIGH;
        manualSetFromKeyboard = false;
        //relay1=HIGH;
        //digitalWrite(RELAY1PIN, HIGH);
      }
		}
		if (customKey=='C') {
			lcd.begin(LCDCOLS,LCDROWS);               // reinitialize the lcd 
		}
		else if (customKey=='B') {
		  digitalWrite(BACKLIGHT, HIGH);
		}
		else if (customKey=='A') {
		  digitalWrite(BACKLIGHT, LOW);
		}
		else if (customKey=='0') { //main display
      lcd.clear();
			display=0;
    }
		else if (customKey=='1') { //total energy or save control sensor to EEPROM
      lcd.clear();
      if (display>=200 && display<300) {
        controlSensor=3; //ROOM
        EEPROM.write(controlSensorEEPROMAdr, controlSensor);
        display=200-display;
      } else {
        display=1;
      }
    }
		else if (customKey=='2') { //TempDiffON or save control sensor to EEPROM
      lcd.clear();
      if (display>=200 && display<300) {
        controlSensor=0; //Bojler
        EEPROM.write(controlSensorEEPROMAdr, controlSensor);
        display=200-display;
      } else {
        display=2;
      }
    }
		else if (customKey=='3') { //TempDiffOFF
      lcd.clear();
      display=3;
    }
		else if (customKey=='4') { //Energy koef
      lcd.clear();
      display=4;
    }
		else if (customKey=='5') { //Max IN OUT temp
      lcd.clear();
      display=5;
    }
		else if (customKey=='6') { //Max bojler
      lcd.clear();
      display=6;
    }
		else if (customKey=='7') { //Max power today
      lcd.clear();
      display=7;
    }
		else if (customKey=='8') { //Control sensor
      lcd.clear();
      display=8;
    }
		else if (customKey=='9') { //Toatal time
      lcd.clear();
      display=9;
    }
		else if (customKey=='*') { //Save total energy to EEPROM
      writeTotalEEPROM(5);
      display=100 + display;
    }
		else if (customKey=='#') { //Select control sensor
      display=200 + display;
    }
  }
#endif
}

void communication() {
  char req=dataRequested();
	if (req=='R') { //if data were requested from central unit then send data
		sendDataSerial();
	} else if (req=='S') { //setup
		readDataSerial();
	} else if (req=='P') { //power down, power save mode set
		if (powerMode!=POWERSAVE) {
			tempDiffONNormal   = tempDiffON;
			tempDiffOFFNormal  = tempDiffOFF;
			tempDiffON         = tempDiffONPowerSave;
			tempDiffOFF        = tempDiffOFFPowerSave;
			powerMode=POWERSAVE;
		}
		mySerial.print("OK");
  } else if (req=='N') { //power up, normal mode 
    powerMode=NORMAL;
    tempDiffON      = tempDiffONNormal;
    tempDiffOFF     = tempDiffOFFNormal;
		mySerial.print("OK");
  }
}

void displayTemp(int x, int y, float value) {
  /*
  012345
  -25.3
  -5.3
  -0.1
   0.1
   5.3
  25.3
   0.5 //100.5
  */
	lcd.setCursor(x,y);
  int cela=(int)value;
	if (cela>100) {
		cela=cela-100;
	}
	if (cela<10 && cela>=0) {
		lcd.print(" ");
	}
 
	int desetina=abs((int)(value*10)%10);
  lcd.print(cela);
  lcd.print(".");
  lcd.print(desetina);
  
  if (cela>-10) {
    lcd.print(" ");
  }
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
  
#ifdef serial
  Serial.print("Sensor(s):");
  Serial.println(numberOfDevices);
#endif

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
    lcd.print("M");
  } else {
    if (relay1==LOW)
      lcd.print("T");
    else
      lcd.print("N");
  }
#ifdef serial
  //Serial.print("R1:");
#endif
  if (manualON) {
    //Serial.println("Manual");
  } else {
#ifdef serial
    //if (relay1==LOW)
      //Serial.println("ON");
    //else
      //Serial.println("OFF");
#endif
  }
/*  lcd.setCursor(RELAY2X,RELAY2Y);
  if (relay2==LOW)
    lcd.print("T");
  else
    lcd.print("N");
*/
}

void sendDataSerial() {
  if (firstMeasComplete==false) return;

#ifdef serial
  Serial.print("DATA:");
#endif
	//data sended:
	//#0;25.31#1;25.19#2;5.19#N;25.10#F;15.50#R;1#S;0#P;0.00#E;0.00#T0.00;#V;0.69#M;0#C;123456;#A;0$3600177622*
	digitalWrite(LEDPIN,HIGH);
	crc = ~0L;
  for (byte i=0;i<numberOfDevices; i++) {
		send(START_BLOCK);
		send(i);
		send(DELIMITER);
		send(sensor[i]);
	}
	send(START_BLOCK);
	send('N');
	send(DELIMITER);
	send(tempDiffON);

	send(START_BLOCK);
	send('F');
	send(DELIMITER);
	send(tempDiffOFF);

	send(START_BLOCK);
	send('R');
	send(DELIMITER);
	if (relay1==LOW)
		send('1');
  else
		send('0');

	send(START_BLOCK);
	send('S');
	send(DELIMITER);
	if (relay2==LOW)
		send('1');
  else
		send('0');

	//Power
	send(START_BLOCK);
	send('P');
	send(DELIMITER);
	send(power);
	
	//Energy a day
	send(START_BLOCK);
	send('E');
	send(DELIMITER);
	send(enegyWsTokWh(energyADay));
	
	//Energy Total
	send(START_BLOCK);
	send('T');
	send(DELIMITER);
	send(enegyWsTokWh(totalEnergy));
	
	send(START_BLOCK);
	send('V');
	send(DELIMITER);
	send(versionSW);
	
	send(START_BLOCK);
	send('M');
	send(DELIMITER);
	send(modeSolar);

  //total time in minutes
	send(START_BLOCK);
	send('C');
	send(DELIMITER);
	send(totalSec/60);

	send(START_BLOCK);
	send('A');
	send(DELIMITER);
	send(status);
  
  if (status>0) {
    status=0;
  }

	
	send(END_BLOCK);
#ifdef serial
	Serial.print(crc);
	Serial.println(END_TRANSMITION);
#endif	
	mySerial.print(crc);
	mySerial.print(END_TRANSMITION);
	mySerial.flush();
	digitalWrite(LEDPIN,LOW);
}

void readDataSerial() {
	float setOn=tempDiffON;
	float setOff=tempDiffOFF;
  byte setModeSolar=modeSolar;
	unsigned long timeOut = millis();
	char b[4+1];
  crc = ~0L;
  char crcBuffer[10+1]; //long = 10digits
  byte crcPointer=0;
  bool startCRC = false;
#ifdef serial
	Serial.print("Setup req.:");
#endif
	//#ON (4digits, only >=0) OFF (4digits (ex 25.1...), only >=0) MODE 1 digit $CRC
	//#25.115.50$541458114*
  char incomingByte;
	digitalWrite(LEDPIN,HIGH);
	do {
		incomingByte = mySerial.read();
		if (incomingByte=='#') {
      crc_string('#');
			//ON
			mySerial.readBytes(b,4);
			b[4]='\0';
			setOn=atof(b);
      crc_string(setOn);
#ifdef serial
			Serial.print("ON=");
			Serial.print(setOn);
#endif
      //OFF
			mySerial.readBytes(b,4);
			b[4]='\0';
			setOff=atof(b);
      crc_string(setOff);
#ifdef serial
			Serial.print(",OFF=");
			Serial.print(setOff);
#endif
			mySerial.readBytes(b,1);
      //MODE 0 - auto, 1 - ON, 2 - OFF
      setModeSolar = b[0]-48;
      crc_string(setModeSolar);
#ifdef serial
			Serial.print(",Mode=");
			Serial.print(modeSolar);
#endif
		}

    if (startCRC) {
#ifdef serial
      Serial.print(incomingByte);
#endif
      crcBuffer[crcPointer++]=incomingByte;
      crcBuffer[crcPointer]='\0';
    }
    
    if (incomingByte=='$') {
      startCRC = true;
#ifdef serial
      Serial.print(" CRC-");
#endif
    }

  } while ((char)incomingByte!='*' && millis() < (timeOut + serialTimeout));

  //validation with CRC
#ifdef serial
  Serial.print(" CRC=");
  Serial.println(crcBuffer);
#endif
  //if (crc==atol(crcBuffer)) {
  if (false) {
    //data valid
    //if any change -> save setup to EEPROM
    if (tempDiffON!=setOn) {
      tempDiffON=setOn;
  #ifdef serial
      Serial.print("The new value of tempDiffON was written to EEPROM:");
      Serial.println(tempDiffON);
  #endif
      EEPROM.write(tempDiffONEEPROMAdrH, (char)tempDiffON);
      EEPROM.write(tempDiffONEEPROMAdrL, (int)(tempDiffON * 10) % 10);
    }
    if (tempDiffOFF!=setOff) {
      tempDiffOFF=setOff;
  #ifdef serial
      Serial.print("The new value of tempDiffOFF was written to EEPROM:");
      Serial.println(tempDiffOFF);
  #endif
      EEPROM.write(tempDiffOFFEEPROMAdrH, (char)tempDiffOFF);
      EEPROM.write(tempDiffOFFEEPROMAdrL, (int)(tempDiffOFF * 10) % 10);
    }

    if (setModeSolar!=modeSolar) {
      if (manualSetFromKeyboard) { //keyboard setup has a high priority as internet setup
        if (setModeSolar==0) {
          manualON = false;
        } else {
          if (setModeSolar==1) {
            manualON = true;
            relay1=LOW;
          } else if (setModeSolar==2) {
            manualON = true;
            relay1=HIGH;
          }
        }
      }
    }
  }

	digitalWrite(LEDPIN,LOW);
}

void send(char s) {
	send(s, ' ');
}


void send(char s, char type) {
	if (type=='X') {
#ifdef serial
		Serial.print(s, HEX);
#endif
		mySerial.print(s, HEX);
	}
	else {
#ifdef serial
		Serial.print(s);
#endif
		mySerial.print(s);
	}
	crc_string(byte(s));
}

void send(byte s) {
	send(s, ' ');
}

void send(byte s, char type) {
	if (type=='X') {
#ifdef serial
		Serial.print(s, HEX);
#endif
		mySerial.print(s, HEX);
	}
	else {
#ifdef serial
		Serial.print(s);
#endif
		mySerial.print(s);
	}
	crc_string(s);
}

void send(unsigned long s) {
#ifdef serial
  Serial.print(s);
#endif
	mySerial.print(s);
}

void send(unsigned int s) {
#ifdef serial
  Serial.print(s);
#endif
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

char dataRequested() {
	char incomingByte=0;
	if (mySerial.available() > 0) {
    incomingByte = (char)mySerial.read();
#ifdef serial
		Serial.print("Data req-");
		Serial.println(incomingByte);
#endif
  }
	return incomingByte;
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

void crc_string(byte s)
{
  crc = crc_update(crc, s);
  crc = ~crc;
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
#ifdef serial
	/*Serial.print("H:");
	Serial.println(valueIH); //18
	Serial.print("M:");
	Serial.println(valueIM); //252
	Serial.print("S:");
	Serial.println(valueIS);  //143
	Serial.print("L:");
	Serial.println(valueIL);  //0
	*/
#endif
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

void readAndSetONOFFFromEEPROM() {
  //25.5 -> 25 5
  int valueIH = EEPROM.read(tempDiffONEEPROMAdrH);
  int valueIL = EEPROM.read(tempDiffONEEPROMAdrL);
  float valueF = (float)valueIH + (float)valueIL / 10;
  if (valueF<200) {
    tempDiffON = valueF;
  }
  else {} //use default value=25.0 see variable initialization
  valueIH = EEPROM.read(tempDiffOFFEEPROMAdrH);
  valueIL = EEPROM.read(tempDiffOFFEEPROMAdrL);
  valueF = (float)valueIH + (float)valueIL / 10;
  if (valueF<200) {
    tempDiffOFF = valueF;
  }
  else {} //use default value=15.0 see variable initialization
}

unsigned int getPower() {
  return (float)energyKoef*(tOut-tIn); //in W
}

void lcdShow() {
		if (display==0) { //main display
			//display OUT  IN  ROOM
			displayTemp(TEMP1X,TEMP1Y, tOut);
			displayTemp(TEMP2X,TEMP2Y, tIn);
			displayTemp(TEMP3X,TEMP3Y, tControl);
			//zobrazeni okamziteho vykonu ve W
			//zobrazeni celkoveho vykonu za den v kWh
			//zobrazeni poctu minut behu cerpadla za aktualni den
			//0123456789012345
			// 636 0.1234 720T
			unsigned int p=(int)power;
			lcd.setCursor(POWERX,POWERY);
			if (p<10000) lcd.print(" ");
			if (p<1000) lcd.print(" ");
			if (p<100) lcd.print(" ");
			if (p<10) lcd.print(" ");
			if (power<=99999) {
				lcd.print(p);
			}
			
			lcd.setCursor(ENERGYX,ENERGYY);
			lcd.print(enegyWsTokWh(energyADay)); //Ws -> kWh (show it in kWh)
			
			lcd.setCursor(TIMEX,TIMEY);
			p=(int)(msDayON/1000/60);
			if (p<100) lcd.print(" ");
			if (p<10) lcd.print(" ");
			if (p<=999) {
				lcd.print(p); //ms->min (show it in minutes)
			}
      displayRelayStatus();
		} else if (display==1) { //total Energy
      //lcd.clear();
			lcd.setCursor(0,0);
      lcd.print("Total energy");
			lcd.setCursor(0,1);
      lcd.print(enegyWsTokWh(totalEnergy));
      lcd.print(" kWh     ");
    } else if (display==2) { //TempDiffON
      //lcd.clear();
			lcd.setCursor(0,0);
      lcd.print("TempDiffON");
			lcd.setCursor(0,1);
      lcd.print(tempDiffON);
      lcd.print("     ");
    } else if (display==3) { //TempDiffOFF
      //lcd.clear();
			lcd.setCursor(0,0);
      lcd.print("TempDiffOFF");
			lcd.setCursor(0,1);
      lcd.print(tempDiffOFF);
      lcd.print("     ");
    } else if (display==4) { //Energy koef
      //lcd.clear();
			lcd.setCursor(0,0);
      lcd.print("Energy koef.");
			lcd.setCursor(0,1);
      lcd.print(energyKoef);
      lcd.print(" W/K    ");
    } else if (display==5) { //Max IN OUT temp
			lcd.setCursor(0,0);
      //lcd.clear();
      lcd.print("Max IN:");
      lcd.print(tMaxIn);
      lcd.print("     ");
			lcd.setCursor(0,1);
      lcd.print("Max OUT:");
      lcd.print(tMaxOut);
      lcd.print("     ");
    } else if (display==6) { //Max bojler
			lcd.setCursor(0,0);
      //lcd.clear();
      lcd.print("Max bojler");
			lcd.setCursor(0,1);
      lcd.print(tMaxBojler);
      lcd.print("     ");
    } else if (display==7) { //Max power today
			lcd.setCursor(0,0);
      //lcd.clear();
      lcd.print("Max power today");
			lcd.setCursor(0,1);
      lcd.print(maxPower);
      lcd.print(" W     ");
    } else if (display==8) { //Control sensor
			lcd.setCursor(0,0);
      //lcd.clear();
      lcd.print("Control sensor");
			lcd.setCursor(0,1);
      if (controlSensor==3) {
        lcd.print("Room");
      } else if (controlSensor==0) {
        lcd.print("Bojler");
      } else {
        lcd.print("Unknown");
			}
      lcd.print(" [");
      lcd.print(sensor[controlSensor]);
      lcd.print("]   ");
		} else if (display==9) { //total time
      //lcd.clear();
			lcd.setCursor(0,0);
      lcd.print("Total time");
			lcd.setCursor(0,1);
      lcd.print(totalSec/60/60));
      lcd.print(" hours   ");
    } else if (display>=100 && display<200) { //Save energy to EEPROM
			lcd.setCursor(0,0);
      //lcd.clear();
      lcd.print("Energy saved!  ");
			lcd.setCursor(0,1);
      lcd.print(enegyWsTokWh(totalEnergy));
      lcd.print(" Ws       ");
      delay(500);
      lcd.clear();
      display = display - 100;
    } else if (display>=200 && display<300) { //Vyber ridiciho cidla
			lcd.setCursor(0,0);
      //lcd.clear();
      lcd.print("Control sensor");
			lcd.setCursor(0,1);
      lcd.print("Room=1 Bojler=2");
    }
    
    
    //1234567890123456
    //Save Energy EEPR
    //Yes = 1

}

#ifdef setTE
void setTE() {
  totalEnergy = 315530 * 3600;
  totalSec = 1134000;
  writeTotalEEPROM(5);
  readTotalEEPROM();
}
#endif

void readAndSetControlSensorFromEEPROM() {
  controlSensor = EEPROM.read(controlSensorEEPROMAdr);
  if (controlSensor!=0 || controlSensor!=3) {
    controlSensor=0;
    EEPROM.write(controlSensorEEPROMAdr, controlSensor);
  }
}

float enegyWsTokWh(float e) {
  return e/3600.f/1000.f;
}