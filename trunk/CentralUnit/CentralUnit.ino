//HW
//Arduino Mega 2560
//Ethernet shield


// A0 			- 230V present
// A1 			- 
// A2 			- 
// A3 			- 
// A4         	- 
// A5        	- 
// A6        	- 
// A7        	- 
// A8        	- 
// A9        	- 
// A10        	- 
// A11        	- 
// A12        	- 
// A13        	- 
// A14        	- 
// A15        	- 
// D0 			- Serial monitor (Serial 0 Rx)
// D1 			- Serial monitor (Serial 0 Tx)
// D2 			- 
// D3 			- 
// D4 			- 
// D5 			- 
// D6 			- 
// D7 			- 
// D8 			- 
// D9 			- 
// D10 			- Ethernet shield
// D11 			- Ethernet shield
// D12 			- Ethernet shield
// D13 			- Ethernet shield
// D14          - Alarm (Serial 3 Tx)
// D15          - Alarm (Serial 3 Rx)
// D16          - Temperature (Serial 2 Tx)
// D17          - Temperature (Serial 2 Rx)
// D18          - Solar (Serial 1 Tx)
// D19          - Solar (Serial 1 Rx)
// D20          - (SDA)
// D21          - (SCL)
// D22-D52      - reserved for Alarm sensors
// D53			- SD card on Ethernet shield

#ifndef dummy //this section prevent from error while program is compiling without Ethernetdef
char a[0]; //do not delete this dummy variable
#endif

//TODO
//support for data storage on SD card

//Solar system variables
#ifndef NUMBER_OF_DEVICES
#define NUMBER_OF_DEVICES 3
#endif
#define verbose
#define SDdef

float sensor[NUMBER_OF_DEVICES];
float tempDiffON = 25.0; //difference between room temperature and solar OUT (sensor 2 - sensor 1) to set relay ON
float tempDiffOFF = 15.0; //difference between room temperature and solar OUT (sensor 2 - sensor 1) to set relay OFF
bool relay1=HIGH; 
bool relay2=HIGH; 
float power = 0.0;
float energy = 0.0;
float energyTotal = 0.0;

unsigned long       lastReadSolarTime;
unsigned long       lastSendSolarTime;
unsigned long       lastUpdateSolarTime;
unsigned long 			lastReadTemperatureTime;
unsigned long				lastSendHouseTime;
unsigned int const  readDataSolarDelay          = 5000; //read data from solar unit
unsigned int const  sendTimeSolarDelay          = 5000; //to send to xively.com
unsigned int const  updateTimeSolarDelay        = 60000; //to send to xively.com
unsigned int const	readDataTemperatureDelay  	= 15000; //read data from temperature satelite
unsigned int const 	sendTimeHouseDelay					= 15000; //to send to xively.com

float tIn   				= 0;
float tOut  				= 0;
float tRoom 				= 0;

float tBedRoomOld   = 0;
float tBedRoomNew   = 0;
float tBojler   		= 0;
float tHall   			=	0;
float tLivingRoom   = 0;
float tCorridor   	= 0;
float tWorkRoom   	= 0;
float versionSolar;


unsigned int const SERIAL_SPEED=9600;

#define START_BLOCK 			'#'
#define DELIMITER 				';'
#define END_BLOCK 				'$'
#define END_TRANSMITION 	'*'

// #define testOnProMini
// #ifdef testOnProMini
// #include <SoftwareSerial.h>
// SoftwareSerial mySerial(10, 11); // RX, TX
// #endif

int ethOK=false;

//Ethernet
#include <Ethernet.h>
#include <HttpClient.h>
#include <Time.h> 
#include <EthernetUdp.h>

byte mac[] = { 0x00, 0xE0, 0x07D, 0xCE, 0xC6, 0x6E};
//IPAddress dnServer(192, 168, 1, 1);
//IPAddress gateway(192, 168, 1, 1);
//IPAddress subnet(255, 255, 255, 0);
//IPAddress ip(192, 168, 1,89);

//XIVELY
#include <Xively.h>
char xivelyKeySolar[] 			= "azCLxsU4vKepKymGFFWVnXCvTQ6Ilze3euIsNrRKRRXuSPO8";
char xivelyKeySetupSolar[] 	= "xabE5tkgkDbMBSn6k60NUqCP4WGpVvp2AMqsL36rWSx6y3Bv";

#define xivelyFeedSolar 				538561447
#define xivelyFeedSetupSolar 		2020049288

char VersionSolarID[]	 	= "V";
char StatusSolarID[]	 	= "S";
char TempOUTID[]			 	= "OUT";
char TempINID[] 				= "IN";
char TempROOMID[] 			= "ROOM";
char TempDiffONID[] 		= "DiffON";
char TempDiffOFFID[] 		= "DiffOFF";
char StatusID[] 				= "Status";
char PowerID[] 					= "Power";
char EnergyID[] 				= "EnergyAday";
char EnergyTotalID[] 		= "EnergyTotal";

//setup feed
char setTempDiffONID[] = "setDiffON";
char setTempDiffOFFID[] = "setDiffOFF";

bool statusSolar=0;
bool statusHouse=0;

XivelyDatastream datastreamsSolar[] = {
	XivelyDatastream(VersionSolarID, 		strlen(VersionSolarID), 	DATASTREAM_FLOAT),
	XivelyDatastream(StatusSolarID, 		strlen(StatusSolarID), 		DATASTREAM_INT),
	XivelyDatastream(TempOUTID, 				strlen(TempOUTID), 				DATASTREAM_FLOAT),
	XivelyDatastream(TempINID, 					strlen(TempINID), 				DATASTREAM_FLOAT),
	XivelyDatastream(TempROOMID, 				strlen(TempROOMID), 			DATASTREAM_FLOAT),
	XivelyDatastream(TempDiffONID, 			strlen(TempDiffONID), 		DATASTREAM_FLOAT),
	XivelyDatastream(TempDiffOFFID, 		strlen(TempDiffOFFID), 		DATASTREAM_FLOAT),
	XivelyDatastream(StatusID, 					strlen(StatusID), 				DATASTREAM_INT),
	XivelyDatastream(PowerID, 					strlen(PowerID), 					DATASTREAM_FLOAT),
	XivelyDatastream(EnergyID, 					strlen(EnergyID), 				DATASTREAM_FLOAT),
	XivelyDatastream(EnergyTotalID, 		strlen(EnergyTotalID), 		DATASTREAM_FLOAT)
};

XivelyDatastream datastreamsSolarSetup[] = {
	XivelyDatastream(setTempDiffONID, 	strlen(setTempDiffONID), 	DATASTREAM_FLOAT),
	XivelyDatastream(setTempDiffOFFID, 	strlen(setTempDiffOFFID), DATASTREAM_FLOAT)
};

XivelyFeed feedSolar(xivelyFeedSolar, 			datastreamsSolar, 			11);
XivelyFeed feedSetup(xivelyFeedSetupSolar, 	datastreamsSolarSetup, 	2);

EthernetClient client;
XivelyClient xivelyclientSolar(client);
XivelyClient xivelyclientSetup(client);

char xivelyKeyHouse[] 			= "I88WA1Y8x01WFUthoFJjhk5PD2xZIsTh1XMzAN6YeAA46teR";
#define xivelyFeedHouse 				740319992

char VersionHouseID[] 			= "V";
char StatusHouseID[] 				= "S";
char TempBedRoomNewID[] 		= "BedRoomNew";
char TempBedRoomOldID[] 		= "BedRoomOld";
char TempBojlerID[] 				= "Bojler";
char TempCorridorID[] 			= "Corridor";
char TempHallID[] 					= "Hall";
char TempLivingRoomID[] 		= "LivingRoom";
char TempWorkRoomID[] 			= "WorkRoom";


XivelyDatastream datastreamsHouse[] = {
	XivelyDatastream(VersionHouseID, 		strlen(VersionHouseID), 	DATASTREAM_FLOAT),
	XivelyDatastream(StatusHouseID, 		strlen(StatusHouseID), 		DATASTREAM_INT),
	XivelyDatastream(TempBedRoomNewID,	strlen(TempBedRoomNewID), DATASTREAM_FLOAT),
	XivelyDatastream(TempBedRoomOldID,	strlen(TempBedRoomOldID),	DATASTREAM_FLOAT),
	XivelyDatastream(TempBojlerID, 			strlen(TempBojlerID),			DATASTREAM_FLOAT),
	XivelyDatastream(TempCorridorID, 		strlen(TempCorridorID), 	DATASTREAM_FLOAT),
	XivelyDatastream(TempHallID,				strlen(TempHallID), 			DATASTREAM_FLOAT),
	XivelyDatastream(TempLivingRoomID, 	strlen(TempLivingRoomID), DATASTREAM_FLOAT),
	XivelyDatastream(TempWorkRoomID, 		strlen(TempWorkRoomID), 	DATASTREAM_FLOAT),
};

XivelyFeed feedHouse(xivelyFeedHouse, 						datastreamsHouse, 			9);

XivelyClient xivelyclientHouse(client);
 
IPAddress timeServer(132, 163, 4, 101); // time-a.timefreq.bldrdoc.gov
// IPAddress timeServer(132, 163, 4, 102); // time-b.timefreq.bldrdoc.gov
// IPAddress timeServer(132, 163, 4, 103); // time-c.timefreq.bldrdoc.gov
const int timeZone = 1;     // Central European Time
//const int timeZone = -5;  // Eastern Standard Time (USA)
//const int timeZone = -4;  // Eastern Daylight Time (USA)
//const int timeZone = -8;  // Pacific Standard Time (USA)
//const int timeZone = -7;  // Pacific Daylight Time (USA)
EthernetUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets

#define DATE_DELIMITER "."
#define TIME_DELIMITER ":"
#define DATE_TIME_DELIMITER " "

#include <avr/pgmspace.h>
unsigned long crc;
static PROGMEM prog_uint32_t crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

#ifdef SDdef
#include <SD.h>
const int chipSelect = 4;
File myFile;
// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;
bool bCardOK = false;
unsigned long lastSaveTime;
#endif

unsigned long getNtpTime();
void sendNTPpacket(IPAddress &address);

float versionSW=0.03;
char versionSWString[] = "CentralUnit v"; //SW name & version


void setup() {
  Serial.begin(SERIAL_SPEED);
	Serial.println("CentralUnit START");
	Serial1.begin(SERIAL_SPEED);
	Serial2.begin(SERIAL_SPEED);
	
	datastreamsHouse[0].setFloat(0.02);
	
#ifdef verbose
  Serial.println("waiting for net connection...");
#endif
	//lcd.setCursor(0,0);
  //lcd.print("waiting for net");
	//Ethernet.begin(mac, ip, dnServer, gateway, subnet);
  byte cyklus=0;
  while (ethOK==false && cyklus++<10) {
    if (Ethernet.begin(mac) == 1) {
      ethOK = true;
    }
#ifdef verbose
    Serial.println("Error getting IP address via DHCP, trying again...");
#endif
    delay(2000);
  }


#ifdef verbose
  if (ethOK) {
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
  } else {
    Serial.println("No internet!");
  }
#endif
	if (ethOK) {
		if (readDataXivelySolar()) {  //read setup from xively for Solar
      sendDataSolar(); //send setup data to Solar unit
    }
	}

	lastSendSolarTime = lastUpdateSolarTime = lastReadSolarTime = lastReadTemperatureTime = millis();
	
#ifdef SDdef
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(53, OUTPUT);
  digitalWrite(53,HIGH);

  Serial.print("Initializing SD card...");

  bCardOK = true;
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("card failed, or not present");
    bCardOK = false;
  }
  else {
    Serial.println("card initialized.");
    cardInfo();
  }
#endif

  Udp.begin(localPort);
  Serial.print("waiting 20s for time sync...");
  setSyncProvider(getNtpTime);

  unsigned long lastSetTime=millis();
  while(timeStatus()==timeNotSet && millis()<lastSetTime+20000); // wait until the time is set by the sync provider, timeout 20sec
  Serial.println("Time sync interval is set to 3600 second.");
  setSyncInterval(3600); //sync each 1 hour
  
  Serial.print("Now is ");
  printDateTime();
  Serial.println(" UTC.");

}

void loop() {
   if(millis() - lastReadSolarTime > readDataSolarDelay) {
    lastReadSolarTime = millis();
    readDataSolar(); //read data from solar
  } 
  if(millis() - lastReadTemperatureTime > readDataTemperatureDelay) {
    lastReadTemperatureTime = millis();
    readDataTemperature(); //read data from temperature satellite
  }   
  if (ethOK) {
    if(!client.connected() && (millis() - lastSendSolarTime > sendTimeSolarDelay)) {
      lastSendSolarTime = millis();
      sendDataSolarXively();
    }
    if(!client.connected() && (millis() - lastUpdateSolarTime > updateTimeSolarDelay)) {
      lastUpdateSolarTime = millis();
      if (readDataXivelySolar()) {
        sendDataSolar(); //send setup data to Solar unit
      }
    }
		if(!client.connected() && (millis() - lastSendHouseTime > sendTimeHouseDelay)) {
      lastSendHouseTime = millis();
      sendDataHouseXively();
    }

  }
}

void sendDataSolar() {
	//#ON (4digits, only >=0) OFF (4digits, only >=0) $CRC
	//#25.115.5$541458114*
	crc = ~0L;
	send('S');
	send(START_BLOCK);
	send(tempDiffON);
	send(tempDiffOFF);
	send(END_BLOCK);
	Serial1.print(crc);
	Serial1.println("*");
	Serial1.flush();
}

//send data to xively
void sendDataSolarXively() {
	if (versionSolar==0)
		versionSolar=0.59;
	datastreamsSolar[0].setFloat(versionSolar);
  datastreamsSolar[1].setInt(statusSolar);  
  if (statusSolar==0) statusSolar=1; else statusSolar=0;
  datastreamsSolar[2].setFloat(tOut);
  datastreamsSolar[3].setFloat(tIn);  
  datastreamsSolar[4].setFloat(tRoom);  
  datastreamsSolar[5].setFloat(tempDiffON);  
  datastreamsSolar[6].setFloat(tempDiffOFF);  
  datastreamsSolar[8].setFloat(power);  
  datastreamsSolar[9].setFloat(energy);  
  datastreamsSolar[10].setFloat(energyTotal);  
  
  if (relay1==LOW)
    datastreamsSolar[7].setInt(1);  
  else
    datastreamsSolar[7].setInt(0);  

#ifdef verbose
  Serial.println("Uploading solar data to Xively");
#endif
#ifdef watchdog
	wdt_disable();
#endif

  int ret = xivelyclientSolar.put(feedSolar, xivelyKeySolar);
	
#ifdef watchdog
	wdt_enable(WDTO_8S);
#endif

#ifdef verbose
  Serial.print("xivelyclientSolar.put returned ");
  Serial.println(ret);
#endif
}


void sendDataHouseXively() {
  datastreamsHouse[1].setInt(statusHouse);  
  if (statusHouse==0) statusHouse=1; else statusHouse=0;
  datastreamsHouse[2].setFloat(tBedRoomNew);  
  datastreamsHouse[3].setFloat(tBedRoomOld);
  datastreamsHouse[4].setFloat(tBojler);  
  datastreamsHouse[5].setFloat(tCorridor);  
  datastreamsHouse[6].setFloat(tHall);  
  datastreamsHouse[7].setFloat(tLivingRoom);  
  datastreamsHouse[8].setFloat(tWorkRoom);  

  
#ifdef verbose
  Serial.println("Uploading temperature to Xively");
#endif
#ifdef watchdog
	wdt_disable();
#endif

  int ret = xivelyclientHouse.put(feedHouse, xivelyKeyHouse);
	
#ifdef watchdog
	wdt_enable(WDTO_8S);
#endif

#ifdef verbose
  Serial.print("xivelyclientHouse.put returned ");
  Serial.println(ret);
#endif

}

void readDataSolar() {
  //read data from Solar unit UART1
	unsigned long timeOut = millis();
	Serial.println("Reading data from solar unit...");
	Serial1.flush();
	Serial1.println("R");
	Serial.println("Data req.");
	char b[10];
	byte i=0;
	char flag=' ';
	byte status=0;
	//#0;25.31#1;25.19#2;5.19#N;25.00#F;15.00#R;1#S;0$3600177622*
	char incomingByte = 0;   // for incoming serial data
	do {
		incomingByte = Serial1.read();
		if (incomingByte > 0) {
			Serial.print(incomingByte);
			if (status==1) {
				flag=incomingByte;
				status++; //2
			}
			
			if (incomingByte==START_BLOCK && status==0) {
				status++; //1
			}
			else if (incomingByte==DELIMITER && status==2) {
				i=0;
				status++; //3
			}
			else if ((incomingByte==START_BLOCK || incomingByte==END_BLOCK) && status==3) {
				b[i]='\0';
				if (flag=='0') { //sensor0 tOut
					sensor[0]=atof(b);
					tOut=sensor[0];
				}
				if (flag=='1') { //sensor1 tOIn
					sensor[1]=atof(b);
					tIn=sensor[1];
				}
				if (flag=='2') { //sensor2 tRoom
					sensor[2]=atof(b);
					tRoom=sensor[2];
				}
				if (flag=='N') { //temperature ON
					tempDiffON=atof(b);
				}
				if (flag=='F') { //temperature OFF
					tempDiffOFF=atof(b);
				}
				if (flag=='R') { //relay 1 status
					if (atoi(b)==0)
						relay1=HIGH; 
					else
						relay1=LOW; 
				}
				if (flag=='S') { //relay 2 status
					if (atoi(b)==0)
						relay2=HIGH; 
					else
						relay2=LOW; 
				}
				if (flag=='P') { //Power
					power=atof(b);
				}
				if (flag=='E') { //Energy a day
					energy=atof(b);
				}
				if (flag=='T') { //Total Energy
					energyTotal=atof(b);
					//energyTotal=82.5; 
				}
				if (flag=='V') { //Version
					versionSolar=atof(b);
				}

				status=1;
			}
			else {
				b[i++]=incomingByte;
			}
		}
	} while ((char)incomingByte!='*' && millis() < (timeOut + 2000));

	/*Serial.println("\nDATA:");
	Serial.print("tOut=");
	Serial.println(sensor[0]);*/
	/*Serial.print("tIn=");
	Serial.println(sensor[1]);*/
	/*Serial.print("tRoom=");
	Serial.println(sensor[2]);*/
	/*Serial.print("tON=");
	Serial.println(tempDiffON);
	Serial.print("tOFF=");
	Serial.println(tempDiffOFF);
	Serial.print("R1=");
	Serial.println(relay1);
	Serial.print("R2=");
	Serial.println(relay2);
	Serial.println("Data end");
	*/
#ifdef SDdef
    saveDataToSD('S');
#endif

}

void readDataTemperature() {
	//Reading data from temperature satelite UART2
	//#0;28E8B84104000016;21.25#1;28A6B0410400004E;7.56#2;28CEB0410400002C;5.81#3;28C9B84104000097;4.19#4;285DF3CF0200007E;46.63$4140078876*
	unsigned long timeOut = millis();
	Serial.println("Reading data from temperature satellite...");
	Serial2.flush();
	Serial2.println("R");
	Serial2.println("Data req.");
	char b[10];
	byte i=0;
	char flag=' ';
	char incomingByte = 0;   // for incoming serial data
	byte status=0;
	do {
		incomingByte = Serial2.read();
		if (incomingByte > 0) {
			Serial.print(incomingByte);
			if (status==1) {
				flag=incomingByte;
				status++; //2
			}
			
			if (incomingByte==START_BLOCK && status==0) {
				status++; //1
			}
			else if (incomingByte==DELIMITER && status==2) {
				//sensor id
				status++; //3
			}
			else if (incomingByte==DELIMITER && status==3) {
				i=0;
				status++; //4
			}
			else if ((incomingByte==START_BLOCK || incomingByte==END_BLOCK) && status==4) {
				b[i]='\0';
				if (flag=='0') { //tBedRoomNew
					tBedRoomNew=atof(b);
				}
				if (flag=='1') { //tBedRoomOld
					tBedRoomOld=atof(b);
				}
				if (flag=='2') { //tLivingRoom
					tLivingRoom=atof(b);
				}
				if (flag=='3') { //tWorkRoom
					tWorkRoom=atof(b);
				}
				if (flag=='4') { //tCorridor
					tCorridor=atof(b);
				}
				if (flag=='5') { //tBojler
					tBojler=atof(b);
				}
				status=1;
			}
			else {
				b[i++]=incomingByte;
			}
		}
	} while ((char)incomingByte!='*' && millis() < (timeOut + 2000));

	Serial.println("\nDATA:");
	Serial.print("tBedRoomNew=");
	Serial.println(tBedRoomNew);
	Serial.print("tBedRoomOld=");
	Serial.println(tBedRoomOld);
	Serial.print("tLivingRoom=");
	Serial.println(tLivingRoom);
	Serial.print("tWorkRoom=");
	Serial.println(tWorkRoom);
	Serial.print("tCorridor=");
	Serial.println(tCorridor);
	Serial.print("tBojler=");
	Serial.println(tBojler);
	Serial.println("Data end");
#ifdef SDdef
    saveDataToSD('T');
#endif
	
}

unsigned long crc_update(unsigned long crc, byte data) {
    byte tbl_idx;
    tbl_idx = crc ^ (data >> (0 * 4));
    crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
    tbl_idx = crc ^ (data >> (1 * 4));
    crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
    return crc;
}

void crc_string(byte s) {
  crc = crc_update(crc, s);
  crc = ~crc;
}

bool readDataXivelySolar() {
	Serial.println("Read setup data from Xively...");
  bool change=false;
#ifdef watchdog
	wdt_disable();
#endif

	int ret = xivelyclientSetup.get(feedSetup, xivelyKeySetupSolar);

#ifdef watchdog
	wdt_enable(WDTO_8S);
#endif

#ifdef verbose
  Serial.print("xivelyclientSolar.get returned ");
  Serial.println(ret);
#endif
  if (ret > 0) {
		float _tempDiffON = tempDiffON;
		float _tempDiffOFF = tempDiffOFF;
		tempDiffON=datastreamsSolarSetup[0].getFloat();
		tempDiffOFF=datastreamsSolarSetup[1].getFloat();
#ifdef verbose
		if (tempDiffOFF!=_tempDiffOFF || tempDiffON!=_tempDiffON) {
      change = true;
			Serial.print("ON is...");
			Serial.println(tempDiffON);
			Serial.print("OFF is... ");
			Serial.println(tempDiffOFF);
	#endif	
		}
  }
  return change;
}

void send(char s) {
	send(s, ' ');
}


void send(char s, char type) {
	if (type=='X') {
		Serial1.print(s, HEX);
	} else {
		Serial1.print(s);
	}
	crc_string(byte(s));
}

void send(byte s) {
	send(s, ' ');
}

void send(byte s, char type) {
	if (type=='X') {
		Serial1.print(s, HEX);
	}
	else {
		Serial1.print(s);
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

#ifdef SDdef
void cardInfo() {
  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card is inserted?");
    Serial.println("* Is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    return;
  } else {
   Serial.println("Wiring is correct and a card is present."); 
  }

  // print the type of card
  Serial.print("\nCard type: ");
  switch(card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    return;
  }


  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("\nVolume type is FAT");
  Serial.println(volume.fatType(), DEC);
  Serial.println();
  
  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize *= 512;                            // SD card blocks are always 512 bytes
  Serial.print("Volume size (bytes): ");
  Serial.println(volumesize);
  Serial.print("Volume size (Kbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print("Volume size (Mbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);

  
  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
  root.openRoot(volume);
  
  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);
  Serial.println();
  Serial.println();
}
#endif

#ifdef SDdef
//save data to SD card
void saveDataToSD(char rep) {
  String tMonth = "";
  String tDay = "";
  byte temp = month();
  if (temp<10) tMonth = "0";
  tMonth += String(month());
  temp = day();
  if (temp<10) tDay = "0";
  tDay += String(day());
    
  String fileName = String(year());
  if (month()<10) fileName+="0";
  fileName+=String(month());
  if (day()<10) fileName+="0";
  fileName+=String(day());
  fileName+=".csv";

  Serial.println();
  printDateTime();
  Serial.print("\nSaving data to file:");
  Serial.print(fileName);
  Serial.print("...");
  
  char cFileName[13];
  fileName.toCharArray(cFileName, 13);    
  File dataFile = SD.open(cFileName, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(day());
    dataFile.print(DATE_DELIMITER);
    dataFile.print(month());
    dataFile.print(DATE_DELIMITER);
    dataFile.print(year());
    dataFile.print(DATE_TIME_DELIMITER);
    if(hour() < 10)
      dataFile.print('0');
    dataFile.print(hour());
    dataFile.print(TIME_DELIMITER);
    if(minute() < 10)
      dataFile.print('0');
    dataFile.print(minute());
    dataFile.print(TIME_DELIMITER);
    if(second() < 10)
      dataFile.print('0');
    dataFile.print(second());

    dataFile.print(";");
    dataFile.print(tIn);
    dataFile.print(";");
    dataFile.print(tOut);
    dataFile.print(";");
    dataFile.print(tRoom);
    dataFile.print(";");
		dataFile.print(tempDiffON);
    dataFile.print(";");
		dataFile.print(tempDiffOFF);
    dataFile.print(";");
		dataFile.print(relay1);
    dataFile.print(";");
		dataFile.print(relay2);
    dataFile.print(";");
		dataFile.print(power);
    dataFile.print(";");
		dataFile.print(energy);
    dataFile.print(";");
		dataFile.print(energyTotal);
    dataFile.print(";");
    dataFile.print(versionSolar);
		
    dataFile.print(tBedRoomOld);
    dataFile.print(";");
    dataFile.print(tBedRoomNew);
    dataFile.print(";");
    dataFile.print(tBojler);
    dataFile.print(";");
    dataFile.print(tHall);
    dataFile.print(";");
    dataFile.print(tLivingRoom);
    dataFile.print(";");
    dataFile.print(tCorridor);
    dataFile.print(";");
    dataFile.print(tWorkRoom);
    dataFile.print(";");

   
    dataFile.print("\n");
      
    dataFile.close();
    Serial.println("data saved.");
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.print("error opening ");
    Serial.println(fileName);
    Serial.println("Try SD card reinit.");
    SD.begin(chipSelect);
    if (!rep) {
      saveDataToSD(true);
    }
  } 

  #ifdef LCDdef
  lcd.setCursor(15, 1);
  lcd.print(" ");
  #endif 
}
#endif

void printDateTime() {
	Serial.print(day());
	Serial.print(DATE_DELIMITER);
	Serial.print(month());
	Serial.print(DATE_DELIMITER);
	Serial.print(year());
	Serial.print(DATE_TIME_DELIMITER);
	printDigits(hour());
	Serial.print(TIME_DELIMITER);
	printDigits(minute());
	Serial.print(TIME_DELIMITER);
	printDigits(second());
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  if(digits < 10) {
    Serial.print('0');
  } else {
    Serial.print(digits);
  }
}


/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

unsigned long getNtpTime() {
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}


/* not implemented yet
#ifdef serialMonitor
void readDataUART() {
  //read data from UART
	byte flag=0;
	char[2+1] tempNo;
	char[8+1] sensorId;
	char[6+1] tempValue;
	byte p=0;
	bool resetP=false;
	unsigned long timeOut = millis();
	Serial1.flush();
	Serial1.println("R");
	Serial.println("Data req.");
	//#0;28422E8104000097;21.44;#1;28EA676B05000089;21.38;$541458114*
	do {
//		if (Serial1.available() > 0) {
			incomingByte = Serial1.read();
			if (incomingByte=='#') {
				flag=1;
				resetP=true;
			} else if (incomingByte==';') {
				flag=++;
				resetP=true;
			}
			else {
				if (flag==1) { //read temp #
					if (resetP) p=0;
					tempNo[p++]=incomingByte;
				}
				else if (flag==2) { //read sensor id
					if (resetP) {
						tempNo[p]=0;
						p=0;
					}
					sensorId[p++]=incomingByte;
				}
				else if (flag==3) { //read temp value
					if (resetP) {
						sensorId[p]=0;
						p=0;
					}
					tempValue[p++]=incomingByte;
				}
				else if (flag==4) {
					if (resetP) {
						tempValue[p]=0;
						p=0;
					}
				}
			}

			if (incomingByte > 0) {
				Serial.print((char)incomingByte);
			}
	} while ((char)incomingByte!='*' && millis() < (
	timeOut + 2000));
	Serial.println("\nData end");
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
*/