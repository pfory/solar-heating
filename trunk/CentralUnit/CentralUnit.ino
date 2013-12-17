//HW
//Arduino Mega 2560
//Ethernet shield
//I2C display

// A0 						- 
// A1 						- 
// A2 						- 
// A3 						- 
// A4 (D20 MEGA) 	- I2C display SDA
// A5 (D21 MEGA) 	- I2C display SCL
// D0 						- Rx
// D1 						- Tx
// D2 						- 
// D3 						- 
// D4 						- SD card on Ethernet shield
// D5 						- 
// D6 						- 
// D7 						- 
// D8 						- 
// D9 						- 
// D10 						- Ethernet shield
// D11 						- Ethernet shield
// D12 						- Ethernet shield
// D13 						- Ethernet shield

#ifndef NUMBER_OF_DEVICES
#define NUMBER_OF_DEVICES 3
#endif
#define verbose

float sensor[NUMBER_OF_DEVICES];
float tempDiffON = 25.0; //difference between room temperature and solar OUT (sensor 2 - sensor 1) to set relay ON
float tempDiffOFF = 15.0; //difference between room temperature and solar OUT (sensor 2 - sensor 1) to set relay OFF
bool relay1=HIGH; 
bool relay2=HIGH; 

//#include <SPI.h>
#include <Ethernet.h>
#include <HttpClient.h>
#include <Xively.h>

int ethOK=false;
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

unsigned long lastSendTime;
unsigned long lastUpdateTime;
unsigned int const sendTimeDelay=5000; //to send to cosm.com
unsigned int const updateTimeDelay=60000; //to send to cosm.com

float tIn=0;
float tOut=0;
float tRoom=0;


#include <avr/pgmspace.h>
unsigned long crc;
static PROGMEM prog_uint32_t crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // RX, TX

float versionSW=0.01;
char versionSWString[] = "CentralUnit v"; //SW name & version


void setup() {
  Serial.begin(9600);
	Serial.println("CentralUnit START");
	mySerial.begin(9600);
	
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

  //lcd.setCursor(0,1);
  if (ethOK) {
    //lcd.print("IP:");
    //lcd.print(Ethernet.localIP());
  }
  else {
    //lcd.print("No internet!!!");
  }
  delay(1000);

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
  }
  else
  {
    Serial.println("No internet!!!");
  }
#endif
	if (ethOK) {
		/*lcd.setCursor(0,0);
		lcd.print("reading Xively");
		lcd.setCursor(0,1);
		lcd.print("feed:");
		lcd.print(xivelyFeedSetup);*/
		readData();
	}

	lastSendTime = lastUpdateTime = millis();

  datastreams[0].setFloat(versionSW);
}

void loop() {
	readDataSolar(); //read data from solar
  if (ethOK) {
    if(!client.connected() && (millis() - lastSendTime > sendTimeDelay)) {
      lastSendTime = millis();
      sendData();
    }
    if(!client.connected() && (millis() - lastUpdateTime > updateTimeDelay)) {
      lastUpdateTime = millis();
      readData();
    }
  }
	delay(5000);
}

//send data to xively
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


void readDataSolar() {
  //read data from UART
	unsigned long timeOut = millis();
	mySerial.flush();
	mySerial.println("R");
	Serial.println("Data req.");
	char b[10];
	byte i=0;
	char flag=' ';
	byte status=0;
	//#0;25.31#1;25.19#2;5.19#N;25.00#F;15.00#R;1#S;0$3600177622*
	int incomingByte = 0;   // for incoming serial data
	do {
		incomingByte = mySerial.read();
		if (incomingByte > 0) {
			//Serial.print(incomingByte);
			if (status==1) {
				flag=incomingByte;
				status++; //2
			}
			
			if (incomingByte=='#' && status==0) {
				status++; //1
			}
			else if (incomingByte==';' && status==2) {
				i=0;
				status++; //3
			}
			else if ((incomingByte=='#' || incomingByte=='$') && status==3) {
				b[i]='\0';
				if (flag=='0') { //sensor0 tOut
					sensor[0]=atof(b);
				}
				if (flag=='1') { //sensor1 tOIn
					sensor[1]=atof(b);
				}
				if (flag=='2') { //sensor2 tRoom
					sensor[2]=atof(b);
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
				status=1;
			}
			else {
				b[i++]=incomingByte;
			}
		}
	} while ((char)incomingByte!='*' && millis() < (timeOut + 2000));

	Serial.print("TOut=");
	Serial.println(sensor[0]);
	Serial.print("TIn=");
	Serial.println(sensor[1]);
	Serial.print("TRoom=");
	Serial.println(sensor[2]);
	Serial.print("TON=");
	Serial.println(tempDiffON);
	Serial.print("TOFF=");
	Serial.println(tempDiffOFF);
	Serial.print("R1=");
	Serial.println(relay1);
	Serial.print("R2=");
	Serial.println(relay2);

	
	Serial.println("Data end");
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
  if (ret > 0) {
		float _tempDiffON = tempDiffON;
		float _tempDiffOFF = tempDiffOFF;
		tempDiffON=datastreamsSetup[0].getFloat();
		tempDiffOFF=datastreamsSetup[1].getFloat();
#ifdef verbose
		if (tempDiffOFF!=_tempDiffOFF || tempDiffON!=_tempDiffON) {
			/*lcd.clear();
			lcd.setCursor(0,0);
			lcd.print("Change settings");
			*/
			Serial.print("ON is...");
			Serial.println(tempDiffON);
			Serial.print("OFF is... ");
			Serial.println(tempDiffOFF);
	#endif	
			/*lcd.setCursor(0,1);
			lcd.print("Z:");
			lcd.print(tempDiffON);
			lcd.print(" V:");
			lcd.print(tempDiffOFF);
			delay(2000);
			lcd.clear();*/
		}
  }
}

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