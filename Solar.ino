//HW
//Pro Mini 328 data are sent via serial line to comunication unit
//I2C display
//2 Relays module
//DALLAS
//keyboard

// A0 						- DALLAS temperature sensors
// A1 						- relay 1
// A2             - relay 2
// A3             - 
// A4 (D20 MEGA) 	- I2C display SDA
// A5 (D21 MEGA) 	- I2C display SCL
// D0 						- Rx
// D1 						- Tx
// D2 						- keyboard
// D3 						- keyboard
// D4 						- keyboard
// D5 						- keyboard
// D6 						- keyboard
// D7 						- keyboard
// D8 						- keyboard
// D9 						- keyboard
// D10 						- 
// D11 						- 
// D12 						- 
// D13 						- 

//spina rele pro čerpadlo v zavislosti na rozdilu teplot z cidla 0,1 a 2. 

//TODO
//spina ventil(y) pro rizeni natapeni bojleru nebo radiatoru v zavislosti na konfiguraci zjistene pres internet

//#include <limits.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h> 

//#define watchdog
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
SoftwareSerial mySerial(10, 11); // RX, TX

const unsigned int serialTimeout=2000;

//LiquidCrystal_I2C lcd(0x20);  // Set the LCD I2C address
//LiquidCrystal_I2C lcd(0x20,6,5,4);  // set the LCD address to 0x20 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x20,2,1,0,4,5,6,7,3, POSITIVE);  // set the LCD address to 0x20 for a 16 chars and 2 line display
#define BACKLIGHT_PIN     0


//LiquidCrystal_I2C lcd(0x38, BACKLIGHT_PIN, POSITIVE);  // Set the LCD I2C address


// Creat a set of new characters
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
#define NUMBER_OF_DEVICES 3
#endif
DeviceAddress tempDeviceAddresses[NUMBER_OF_DEVICES];
//int  resolution = 12;
unsigned int numberOfDevices; // Number of temperature devices found
unsigned long lastDsMeasStartTime;
bool dsMeasStarted=false;
float sensor[NUMBER_OF_DEVICES];
float tempDiffON            = 25.0; //difference between room temperature and solar OUT (sensor 2 - sensor 1) to set relay ON
float tempDiffOFF           = 15.0; //difference between room temperature and solar OUT (sensor 2 - sensor 1) to set relay OFF
//diferences in normal mode (power for pump is ready)
float tempDiffONNormal      = tempDiffON;
float tempDiffOFFNormal     = tempDiffOFF;
//diferences in power save mode (power for pump is OFF)
float tempDiffONPowerSave   = 90.0; 
float tempDiffOFFPowerSave  = 50.0; 
unsigned long const dsMeassureInterval=750; //inteval between meassurements
unsigned long lastMeasTime=0;
unsigned long msDayON = 0;  //kolik ms uz jsem za aktualni den byl ve stavu ON
unsigned long lastOn=0;     //ms posledniho behu ve stavu ON
unsigned long lastOff = 0;  //ms posledniho vypnuti rele
unsigned long const dayInterval=43200000; //1000*60*60*12; //
unsigned long const delayON=120000; //1000*60*2; //po tento cas zustane rele sepnute bez ohledu na stav teplotnich cidel
unsigned long lastOn4Delay = 0;

enum mode {NORMAL, POWERSAVE};
mode powerMode=NORMAL;

float power = 0; //actual power in W
float energy = 0.0; //energy a day in kWh
float const energyKoef = 283.5; //Ws
float tIn=0;
float tOut=0;
float tRoom=0;

//0123456789012345
//15.6 15.8 15.8 V
//1234 0.12 624
#define TEMP1X 0
#define TEMP1Y 0
#define TEMP2X 5
#define TEMP2Y 0
#define TEMP3X 10
#define TEMP3Y 0
//#define TEMP4X 9
//#define TEMP4Y 1
#define POWERX 0
#define POWERY 1
#define ENERGYX 7
#define ENERGYY 1
#define TIMEX 12
#define TIMEY 1


#define RELAY1X 15
#define RELAY1Y 0
/*#define RELAY2X 15
#define RELAY2Y 1
*/

#define RELAY1PIN A1
#define RELAY2PIN A2

//HIGH - relay OFF, LOW - relay ON
bool relay1=HIGH; 
bool relay2=HIGH;

//#define keypad
#ifdef keypad
#include <Keypad.h>
const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
//define the symbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'D','#','0','*'},
  {'C','9','8','7'},
  {'B','6','5','4'},
  {'A','3','2','1'}
};
byte rowPins[ROWS] = {5,4,3,2}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {9,8,7,6}; //connect to the column pinouts of the keypad

//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 
#endif

//EEPROM
#include <EEPROM.h>
byte const tempDiffONEEPROMAdrH=0;
byte const tempDiffONEEPROMAdrL=1;
byte const tempDiffOFFEEPROMAdrH=2;
byte const tempDiffOFFEEPROMAdrL=3;

float const   versionSW=0.57;
char  const   versionSWString[] = "Solar v"; //SW name & version

void setup() {
	lcd.begin(16,2);               // initialize the lcd 
  // Switch on the backlight
  pinMode(BACKLIGHT_PIN, OUTPUT);
  digitalWrite(BACKLIGHT_PIN, HIGH);
	mySerial.begin(SERIAL_SPEED);
	pinMode(LEDPIN,OUTPUT);
	
#ifdef serial
  Serial.begin(SERIAL_SPEED);
	Serial.print("Solar v.");
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

#ifdef watchdog
	wdt_enable(WDTO_8S);
#endif

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

void loop() {
#ifdef watchdog
	wdt_reset();
#endif
  if (!dsMeasStarted) {
    //start sampling
    dsMeasStarted=true;
    dsSensors.requestTemperatures(); 
    //digitalWrite(13,HIGH);
    lastDsMeasStartTime = millis();
  }
  else if (dsMeasStarted && (millis() - lastDsMeasStartTime>dsMeassureInterval)) {
    dsMeasStarted=false;
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
		tOut 	= sensor[0];
		tIn	 	= sensor[1];
		tRoom = sensor[2];
#ifdef serial
		Serial.print("tOut:");
    Serial.print(tOut);
		Serial.print(" tIn:");
    Serial.print(tIn);
		Serial.print(" tRoom:");
    Serial.println(tRoom);
#endif
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

		//display OUT  IN  ROOM
		displayTemp(TEMP1X,TEMP1Y, tOut);
    displayTemp(TEMP2X,TEMP2Y, tIn);
    displayTemp(TEMP3X,TEMP3Y, tRoom);
		
    if (relay1==LOW) {  //relay is ON
			if (tIn<tOut) {
				msDayON+=(millis()-lastOn);
				power = energyKoef*(tOut-tIn); //in W
				energy+=((float)(millis()-lastOn)*power/1000.f); //in Ws
			}
      lastOn = millis();
    }
		else {
			power=0;
		}

    //zobrazeni okamziteho vykonu ve W
    //zobrazeni celkoveho vykonu za den v kWh
    //zobrazeni poctu minut behu cerpadla za aktualni den
    //0123456789012345
    // 636 0.1234 720T
    lcd.setCursor(POWERX,POWERY);
		unsigned int p=(int)power;
		if (p<10000) lcd.print(" ");
		if (p<1000) lcd.print(" ");
		if (p<100) lcd.print(" ");
		if (p<10) lcd.print(" ");
    lcd.print(p);
   
    lcd.setCursor(ENERGYX,ENERGYY);
    lcd.print(energy/1000.f/3600.f); //Wh -> kWh (show it in kWh)
    
    lcd.setCursor(TIMEX,TIMEY);
    p=(int)(msDayON/1000/60);
		if (p<100) lcd.print(" ");
		if (p<10) lcd.print(" ");
		lcd.print(p); //ms->min (show it in minutes)
    
#ifdef serial
    Serial.print("Power:");
    Serial.print(power);
    Serial.println("[W]");
    Serial.print("Energy:");
    Serial.print(energy/1000.f/3600.f);
    Serial.println("[kWh]");
    Serial.print("Pump ON:");
    Serial.print((int)(msDayON/1000));
    Serial.println("[s]");
#endif
 
    //change relay 1 status
   if (relay1==LOW) { //switch ON->OFF
      //if (millis() - lastOn4Delay >= delayON) {
        if ((tOut - tRoom) < tempDiffOFF) {
          relay1=HIGH; ///relay OFF
          digitalWrite(RELAY1PIN, relay1);
          lastOff=millis();
					lastOn4Delay=0;
        }
      //}
    }
	
    if (relay1==HIGH) { //switch OFF->ON
      if (((tOut - tRoom) >= tempDiffON) | ((tIn - tRoom) >= tempDiffON)) {
        relay1=LOW; //relay ON
        digitalWrite(RELAY1PIN, relay1);
        lastOn = millis();
				if (lastOn4Delay==0) {
					lastOn4Delay = lastOn;
				}
        if (lastOff==0) { //first ON in actual day
          energy=0.0;
          msDayON=0;
        }
      }
    }
 
    displayRelayStatus();
#ifdef serial		
		Serial.print("tempDiffON=");
		Serial.println(tempDiffON);
		Serial.print("tempDiffOFF=");
		Serial.println(tempDiffOFF);
#endif
    
    if (lastOff > 0 && (millis() - lastOff>dayInterval)) {
        lastOff = 0;
    }
  }

  //if data requested from central unit, send data
  char req=dataRequested();
	if (req=='R') { //send data
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
 
#ifdef keypad
  char customKey = customKeypad.getKey();
  if (customKey){
    lcd.setCursor(0,0);
    lcd.print(customKey);
  }
#endif
}


void displayTemp(int x, int y, float value) {
	lcd.setCursor(x,y);
  int cela=(int)value;
	if (cela<10 && cela>=0) {
		lcd.print(" ");
	}
	if (cela>100) {
		cela=cela-100;
	}
	int desetina=abs((int)(value*10)%10);
  lcd.print(cela);
  lcd.print(".");
  lcd.print(desetina);
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
  if (relay1==LOW)
    lcd.print("T");
  else
    lcd.print("N");
    
#ifdef serial
  Serial.print("R1:");
  if (relay1==LOW)
    Serial.println("ON");
  else
    Serial.println("OFF");
#endif
/*  lcd.setCursor(RELAY2X,RELAY2Y);
  if (relay2==LOW)
    lcd.print("T");
  else
    lcd.print("N");
*/
}

void sendDataSerial() {
	//data sended:
	//#0;25.31#1;25.19#2;5.19#N;25.10#F;15.50#R;1#S;0$3600177622*
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
	unsigned long timeOut = millis();
	char b[4+1];
	//Serial.println("Data req.");
	//#ON (4digits, only >=0) OFF (4digits, only >=0) $CRC
	//#25.115.5$541458114*
  char incomingByte;
	digitalWrite(LEDPIN,HIGH);
	do {
//		if (Serial1.available() > 0) {
		incomingByte = mySerial.read();
		if (incomingByte=='#') {
			//ON
			mySerial.readBytes(b,4);
			b[4]='\0';
			setOn=atof(b);
#ifdef serial
			Serial.print("ON=");
			Serial.println(setOn);
#endif
			mySerial.readBytes(b,4);
			b[4]='\0';
			setOff=atof(b);
#ifdef serial
			Serial.print("OFF=");
			Serial.println(setOff);
#endif
		}
		//TODO validation with CRC
    
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
    
	} while ((char)incomingByte!='*' && millis() < (timeOut + serialTimeout));
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