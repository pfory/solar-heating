//HW
//Arduino Mega 2560
//Ethernet shield


// A0 						- 230V present
// A1 						- 
// A2 						- 
// A3 						- 
// A4            	- 
// A5           	- 
// A6           	- 
// A7           	- 
// A8           	- 
// A9           	- 
// A10           	- 
// A11           	- 
// A12           	- 
// A13           	- 
// A14           	- 
// A15           	- 
// D0 						- Serial monitor (Serial 0 Rx)
// D1 						- Serial monitor (Serial 0 Tx)
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
// D14            - Alarm (Serial 3 Tx)
// D15            - Alarm (Serial 3 Rx)
// D16            - Temperature (Serial 2 Tx)
// D17            - Temperature (Serial 2 Rx)
// D18            - Solar (Serial 1 Tx)
// D19            - Solar (Serial 1 Rx)
// D20            - (SDA)
// D21            - (SCL)
// D22-D53        - reserved for Alarm sensors


//Solar system variables
#ifndef NUMBER_OF_DEVICES
#define NUMBER_OF_DEVICES 3
#endif
#define verbose

float sensor[NUMBER_OF_DEVICES];
float tempDiffON = 25.0; //difference between room temperature and solar OUT (sensor 2 - sensor 1) to set relay ON
float tempDiffOFF = 15.0; //difference between room temperature and solar OUT (sensor 2 - sensor 1) to set relay OFF
bool relay1=HIGH; 
bool relay2=HIGH; 

unsigned long       lastReadSolarTime;
unsigned long       lastSendSolarTime;
unsigned long       lastUpdateSolarTime;
unsigned int const  readDataSolarDelay          =5000; //read data from solar unit
unsigned int const  sendTimeSolarDelay          =5000; //to send to cosm.com
unsigned int const  updateTimeSolarDelay        =60000; //to send to cosm.com

float tIn   =0;
float tOut  =0;
float tRoom =0;


//Ethernet
#include <Ethernet.h>
#include <HttpClient.h>
#include <Xively.h>

int ethOK=false;
byte mac[] = { 0x00, 0xE0, 0x07D, 0xCE, 0xC6, 0x6E};
//IPAddress dnServer(192, 168, 1, 1);
//IPAddress gateway(192, 168, 1, 1);
//IPAddress subnet(255, 255, 255, 0);
//IPAddress ip(192, 168, 1,89);

//XIVELY
// Your Xively key to let you upload data
char xivelyKeySolar[] = 			"azCLxsU4vKepKymGFFWVnXCvTQ6Ilze3euIsNrRKRRXuSPO8";
char xivelyKeySetupSolar[] = "xabE5tkgkDbMBSn6k60NUqCP4WGpVvp2AMqsL36rWSx6y3Bv";
//your xively feed ID
#define xivelyFeedSolar 				538561447
#define xivelyFeedSetupSolar 	2020049288
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

XivelyDatastream datastreamsSolar[] = {
XivelyDatastream(VersionSolarID, 		strlen(VersionSolarID), 	DATASTREAM_FLOAT),
XivelyDatastream(StatusSolarID, 		strlen(StatusSolarID), 		DATASTREAM_INT),
XivelyDatastream(TempOUTID, 				strlen(TempOUTID), 				DATASTREAM_FLOAT),
XivelyDatastream(TempINID, 					strlen(TempINID), 				DATASTREAM_FLOAT),
XivelyDatastream(TempROOMID, 				strlen(TempROOMID), 			DATASTREAM_FLOAT),
XivelyDatastream(TempDiffONID, 			strlen(TempDiffONID), 		DATASTREAM_FLOAT),
XivelyDatastream(TempDiffOFFID, 		strlen(TempDiffOFFID), 		DATASTREAM_FLOAT),
XivelyDatastream(StatusID, 					strlen(StatusID), 				DATASTREAM_INT)
};

XivelyDatastream datastreamsSolarSetup[] = {
XivelyDatastream(setTempDiffONID, 	strlen(setTempDiffONID), 	DATASTREAM_FLOAT),
XivelyDatastream(setTempDiffOFFID, 	strlen(setTempDiffOFFID), DATASTREAM_FLOAT)
};

XivelyFeed feed(xivelyFeedSolar, 						datastreamsSolar, 			8 /* number of datastreamsSolar */);
XivelyFeed feedSetup(xivelyFeedSetupSolar, 	datastreamsSolarSetup, 2 /* number of datastreamsSolarSolar */);

EthernetClient client;
XivelyClient xivelyclient(client);
XivelyClient xivelyclientSetup(client);


#include <avr/pgmspace.h>
unsigned long crc;
static PROGMEM prog_uint32_t crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

float versionSW=0.01;
char versionSWString[] = "CentralUnit v"; //SW name & version


void setup() {
  Serial.begin(9600);
	Serial.println("CentralUnit START");
	Serial1.begin(9600);
	
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
  //delay(1000);

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
		if (readDataXivelySolar()) {  //read setup from xively for Solar
      sendDataSolar(); //send setup data to Solar unit
    }
	}

	lastSendSolarTime = lastUpdateSolarTime = lastReadSolarTime = millis();

  datastreamsSolar[0].setFloat(versionSW);
}

void loop() {
  if(millis() - lastReadSolarTime > readDataSolarDelay) {
    lastReadSolarTime = millis();
    readDataSolar(); //read data from solar
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
  }
	delay(5000);
}

void sendDataSolar() {
}

//send data to xively
void sendDataSolarXively() {
  datastreamsSolar[1].setInt(status);  
  if (status==0) status=1; else status=0;
  datastreamsSolar[2].setFloat(tOut);
  datastreamsSolar[3].setFloat(tIn);  
  datastreamsSolar[4].setFloat(tRoom);  
  datastreamsSolar[5].setFloat(tempDiffON);  
  datastreamsSolar[6].setFloat(tempDiffOFF);  
  
  if (relay1==LOW)
    datastreamsSolar[7].setInt(1);  
  else
    datastreamsSolar[7].setInt(0);  

#ifdef verbose
  Serial.println("Uploading it to Xively");
#endif
#ifdef watchdog
	wdt_disable();
#endif

  int ret = xivelyclient.put(feed, xivelyKeySolar);
	
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
	Serial1.flush();
	Serial1.println("R");
	Serial.println("Data req.");
	char b[10];
	byte i=0;
	char flag=' ';
	byte status=0;
	//#0;25.31#1;25.19#2;5.19#N;25.00#F;15.00#R;1#S;0$3600177622*
	int incomingByte = 0;   // for incoming serial data
	do {
		incomingByte = Serial1.read();
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

bool readDataXivelySolar() {
  bool change=false;
#ifdef watchdog
	wdt_disable();
#endif

	int ret = xivelyclientSetup.get(feedSetup, xivelyKeySetupSolar);

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
		tempDiffON=datastreamsSolarSetup[0].getFloat();
		tempDiffOFF=datastreamsSolarSetup[1].getFloat();
#ifdef verbose
		if (tempDiffOFF!=_tempDiffOFF || tempDiffON!=_tempDiffON) {
      change = true;
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
  return change;
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