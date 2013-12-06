//HW
//Arduino Mega 2560
//Ethernet shield
//I2C display
//2 Relays module
//DALLAS
//keyboard

// A0 						- DALLAS temperature sensors
// A1 						- relay 1
// A2 - relay 2
// A3 - keyboard
// A4 (D20 MEGA) 	- I2C display SDA
// A5 (D21 MEGA) 	- I2C display SCL
// D0 						- Rx
// D1 						- Tx
// D2 						- keyboard
// D3 						- keyboard
// D4 						- SD card on Ethernet shield
// D5 						- keyboard
// D6 						- keyboard
// D7 						- keyboard
// D8 						- keyboard
// D9 						- keyboard
// D10 						- Ethernet shield
// D11 						- Ethernet shield
// D12 						- Ethernet shield
// D13 						- Ethernet shield



//spina rele pro čerpadlo v zavislosti na rozdilu teplot z cidla 0,1 a 2. 

//TODO
//Rozdil teplot lze nastavit v konfiguraci pres internet
//spina ventil(y) pro rizeni natapeni bojleru nebo radiatoru v zavislosti na konfiguraci zjistene pres internet

#include <limits.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h> 

//#define watchdog
#ifdef watchdog
#include <avr/wdt.h>
#endif

#ifndef dummy //this section prevent from error while program is compiling without Ethernetdef
char a[0]; //do not delete this dummy variable
#endif

#define verbose
#define serial

#ifdef verbose
#define serial
#endif

#ifdef serial
int incomingByte = 0;   // for incoming serial data
#endif


#define ethernet
#ifdef ethernet
//#include <SPI.h>
#include <Ethernet.h>
#include <HttpClient.h>
#include <Xively.h>

byte mac[] = { 0x00, 0xE0, 0x07D, 0xCE, 0xC6, 0x6E};
//IPAddress dnServer(192, 168, 1, 1);
//IPAddress gateway(192, 168, 1, 1);
//IPAddress subnet(255, 255, 255, 0);
//IPAddress ip(192, 168, 1,89);
// Your Xively key to let you upload data
char xivelyKey[] = 			"azCLxsU4vKepKymGFFWVnXCvTQ6Ilze3euIsNrRKRRXuSPO8";
char xivelyKeySetup[] = "xabE5tkgkDbMBSn6k60NUqCP4WGpVvp2AMqsL36rWSx6y3Bv";
//your xively feed ID
#define xivelyFeed 				538561447
#define xivelyFeedSetup 	2020049288
//data feed
char VersionSolarID[] = "V";
char StatusSolarID[] = "S";
char TempOUTID[] = "OUT";
char TempINID[] = "IN";
char TempROOMID[] = "ROOM";
char TempDiffONID[] = "DiffON";
char TempDiffOFFID[] = "DiffOFF";
char StatusID[] = "Status";

//setup feed
char setTempDiffONID[] = "setDiffON";
char setTempDiffOFFID[] = "setDiffOFF";

bool status=0;

XivelyDatastream datastreams[] = {
XivelyDatastream(VersionSolarID, 		strlen(VersionSolarID), 	DATASTREAM_FLOAT),
XivelyDatastream(StatusSolarID, 		strlen(StatusSolarID), 		DATASTREAM_INT),
XivelyDatastream(TempOUTID, 				strlen(TempOUTID), 				DATASTREAM_FLOAT),
XivelyDatastream(TempINID, 					strlen(TempINID), 				DATASTREAM_FLOAT),
XivelyDatastream(TempROOMID, 				strlen(TempROOMID), 			DATASTREAM_FLOAT),
XivelyDatastream(TempDiffONID, 			strlen(TempDiffONID), 		DATASTREAM_FLOAT),
XivelyDatastream(TempDiffOFFID, 		strlen(TempDiffOFFID), 		DATASTREAM_FLOAT),
XivelyDatastream(StatusID, 					strlen(StatusID), 				DATASTREAM_INT)
};

XivelyDatastream datastreamsSetup[] = {
XivelyDatastream(setTempDiffONID, 	strlen(setTempDiffONID), 	DATASTREAM_FLOAT),
XivelyDatastream(setTempDiffOFFID, 	strlen(setTempDiffOFFID), DATASTREAM_FLOAT)
};

XivelyFeed feed(xivelyFeed, 						datastreams, 			8 /* number of datastreams */);
XivelyFeed feedSetup(xivelyFeedSetup, 	datastreamsSetup, 2 /* number of datastreams */);

EthernetClient client;
XivelyClient xivelyclient(client);
XivelyClient xivelyclientSetup(client);
#endif

unsigned long lastSendTime;
unsigned long lastUpdateTime;

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
unsigned int const sendTimeDelay=5000; //to send to cosm.com
unsigned int const updateTimeDelay=60000; //to send to cosm.com
float tempDiffON = 25.0; //difference between room temperature and solar OUT (sensor 2 - sensor 1) to set relay ON
float tempDiffOFF = 15.0; //difference between room temperature and solar OUT (sensor 2 - sensor 1) to set relay OFF
unsigned long const dsMeassureInterval=750; //inteval between meassurements
unsigned long lastMeasTime=0;
unsigned long msDayON = 0;  //kolik ms uz jsem za aktualni den byl ve stavu ON
unsigned long lastOn=0;     //ms posledniho behu ve stavu ON
unsigned long lastOff = 0;  //ms posledniho vypnuti rele
unsigned long const dayInterval=1000*60*60*12; //
unsigned long const delayON=1000*60*2; //po tento cas zustane rele sepnute bez ohledu na stav teplotnich cidel
unsigned long lastOn4Delay = 0;

float power = 0; //actual power in W
float energy = 0.0; //energy a day in kWh
float energyKoef = 283.5; //Ws
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
//#define RELAY2PIN A2

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
byte rowPins[ROWS] = {5,A3,3,2}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {9,8,7,6}; //connect to the column pinouts of the keypad

//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 
#endif


float versionSW=0.51;
char versionSWString[] = "Solar v"; //SW name & version

void dsInit(void);
void displayRelayStatus(void);

void setup() {
	lcd.begin(16,2);               // initialize the lcd 
  // Switch on the backlight
  pinMode(BACKLIGHT_PIN, OUTPUT);
  digitalWrite(BACKLIGHT_PIN, HIGH);

#ifdef serial
  Serial.begin(9600);
#endif
#ifdef verbose
	Serial.print("Solar v.");
  Serial.println(versionSW);
#endif
#ifdef ethernet
  datastreams[0].setFloat(versionSW);
#endif
  lcd.home();                   // go home
  lcd.print(versionSWString);  
  lcd.print(" ");
  lcd.print (versionSW);
  delay(1000);
  lcd.clear();

  dsInit();

#ifdef verbose
  Serial.println("waiting for net connection...");
#endif
	lcd.setCursor(0,0);
  lcd.print("waiting for net");

#ifdef ethernet
	//Ethernet.begin(mac, ip, dnServer, gateway, subnet);
  while (Ethernet.begin(mac) != 1)
  {
#ifdef verbose
    Serial.println("Error getting IP address via DHCP, trying again...");
#endif
    delay(2000);
  }
#endif

   // for ( int i = 0; i < charBitmapSize; i++ )
   // {
      // lcd.createChar ( i, (uint8_t *)charBitmap[i] );
   // }


#ifdef ethernet
  lcd.setCursor(0,1);
	lcd.print("IP:");
  lcd.print(Ethernet.localIP());
#endif
  delay(1000);

#ifdef ethernet	
#ifdef verbose
  Serial.println("EthOK");
  Serial.print("\nIP:");
  Serial.println(Ethernet.localIP());
  Serial.print("Mask:");
  Serial.println(Ethernet.subnetMask());
  Serial.print("Gateway:");
  Serial.println(Ethernet.gatewayIP());
  Serial.print("DNS:");
  Serial.println(Ethernet.dnsServerIP());
  Serial.println();
#endif
#endif
  
  lastSendTime = lastUpdateTime = lastMeasTime = millis();
  lcd.clear();
  
  pinMode(RELAY1PIN, OUTPUT);
  
  digitalWrite(RELAY1PIN, relay1);
  lastOn=millis();
#ifdef ethernet	
	lcd.setCursor(0,0);
  lcd.print("reading Xively");
	lcd.setCursor(0,1);
  lcd.print("feed:");
	lcd.print(xivelyFeedSetup);
	readData();
#endif
#ifdef watchdog
	wdt_enable(WDTO_8S);
#endif
	lcd.clear();
}

void loop() {
#ifdef serial
	checkSerial();
#endif
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
      /*Serial.print("S");
      Serial.print(i);
      Serial.print(":");*/
      Serial.println(sensor[i]);
      //Serial.print(" C ");
			tOut 	= sensor[0];
			tIn	 	= sensor[1];
			tRoom = sensor[2];
    } 
		//obcas se vyskytne chyba a vsechna cidla prestanou merit
		//zkusim restartovat sbernici
		bool reset=true;
		for (byte i=0; i<numberOfDevices; i++) {
			if (sensor[i]!=0.0) {
				reset=false;
			}
		}
		if (reset) {
#ifdef ethernet	
			status=2;
#endif
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
    
#ifdef verbose
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
    
    if (lastOff > 0 && (millis() - lastOff>dayInterval)) {
        lastOff = 0;
    }
  }


#ifdef ethernet
  if(!client.connected() && (millis() - lastSendTime > sendTimeDelay)) {
    lastSendTime = millis();
		readDataUART();
    sendData();
  }
  if(!client.connected() && (millis() - lastUpdateTime > updateTimeDelay)) {
    lastUpdateTime = millis();
    readData();
  }
#endif
 
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
	if (cela<10 & cela>=0) {
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
  
#ifdef verbose
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
    
#ifdef verbose
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

void readDataUART() {
  //read data from UART
/*  #ifdef serial
	if (Serial.available() > 0) {
    incomingByte = Serial.read();
  }
	#endif
  */
}


#ifdef ethernet
void sendData() {
  datastreams[1].setInt(status);  
  if (status==0) status=1; else status=0;
  datastreams[2].setFloat(tOut);
  datastreams[3].setFloat(tIn);  
  datastreams[4].setFloat(tRoom);  
  datastreams[5].setFloat(tempDiffON);  
  datastreams[6].setFloat(tempDiffOFF);  
  
  if (relay1==LOW)
    datastreams[7].setInt(1);  
  else
    datastreams[7].setInt(0);  

#ifdef verbose
  Serial.println("Uploading it to Xively");
#endif
#ifdef watchdog
	wdt_disable();
#endif

  int ret = xivelyclient.put(feed, xivelyKey);
	
#ifdef watchdog
	wdt_enable(WDTO_8S);
#endif

#ifdef verbose
  Serial.print("xivelyclient.put returned ");
  Serial.println(ret);
#endif
}


void readData() {
#ifdef watchdog
	wdt_disable();
#endif

	int ret = xivelyclientSetup.get(feedSetup, xivelyKeySetup);

#ifdef watchdog
	wdt_enable(WDTO_8S);
#endif

#ifdef verbose
  Serial.print("xivelyclient.get returned ");
  Serial.println(ret);
#endif
  if (ret > 0)
  {
		float _tempDiffON = tempDiffON;
		float _tempDiffOFF = tempDiffOFF;
		tempDiffON=datastreamsSetup[0].getFloat();
		tempDiffOFF=datastreamsSetup[1].getFloat();
#ifdef verbose
		if (tempDiffOFF!=_tempDiffOFF || tempDiffON!=_tempDiffON) {
			lcd.clear();
			lcd.setCursor(0,0);
			lcd.print("Change settings");
			Serial.println("ON is...");
			Serial.println(tempDiffON);
			Serial.print("OFF is... ");
			Serial.println(tempDiffOFF);
	#endif	
			lcd.setCursor(0,1);
			lcd.print("Z:");
			lcd.print(tempDiffON);
			lcd.print(" V:");
			lcd.print(tempDiffOFF);
			delay(2000);
			lcd.clear();
		}
  }
}
#endif

#ifdef serial
void checkSerial() {
	if (Serial.available() > 0) {
		incomingByte = Serial.read();
		Serial.print("I received: ");
		Serial.println(incomingByte, DEC);
#ifdef watchdog
		if (incomingByte=='R') {
			Serial.println("\n\nRESET signal present, RESET will be in 2 second.");
			wdt_enable(WDTO_2S);
			wdt_reset();
			for (;;) {}
		}
#endif
	}
}
#endif