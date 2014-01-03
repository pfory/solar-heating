#include <avr/wdt.h>

#define debug

#include <OneWire.h>
#define ONE_WIRE_BUS A0
OneWire onewire(ONE_WIRE_BUS); // pin for onewire DALLAS bus
#include <DallasTemperatureMinimal.h>
DallasTemperatureMinimal dsSensors(&onewire);
DeviceAddress tempDeviceAddress;
#ifndef NUMBER_OF_DEVICES
#define NUMBER_OF_DEVICES 10
#endif
DeviceAddress tempDeviceAddresses[NUMBER_OF_DEVICES];
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


#include <avr/pgmspace.h>
unsigned long crc;
static PROGMEM prog_uint32_t crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

#define START_BLOCK send('#');
#define DELIMITER send(';');
#define END_BLOCK send('$');

int incomingByte;
#define LEDPIN 13

#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // RX, TX

float versionSW=0.02;
char versionSWString[] = "Temerature satelite v"; //SW name & version

//#define debug

void setup() {
	pinMode(LEDPIN, OUTPUT);
	#ifdef debug
	Serial.begin(9600);
	Serial.print(versionSWString);
	Serial.println(versionSW);
	#endif
	mySerial.begin(9600);
	dsInit();

	WDTCSR |= _BV(WDCE) | _BV(WDE);
	WDTCSR = 0;
}


void loop() {
	if (!dsMeasStarted) {
			//start sampling
			dsMeasStarted=true;
			dsSensors.requestTemperatures(); 
			//digitalWrite(13,HIGH);
			lastDsMeasStartTime = millis();
	}
  else if (dsMeasStarted && (millis() - lastDsMeasStartTime>dsMeassureInterval)) {
		digitalWrite(LEDPIN,HIGH);
    dsMeasStarted=false;
    Serial.println();
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
      Serial.print("S");
      Serial.print(i);
      Serial.print(":");
      Serial.print(sensor[i]);
      Serial.println(" C ");
    } 
		//obcas se vyskytne chyba a vsechna cidla prestanou merit
		//zkusim restartovat sbernici
		if (numberOfDevices>1) {
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
		digitalWrite(LEDPIN,LOW);

		char req=dataRequested();
		if (req=='R') {
			sendDataSerial();
			mySerial.flush();
		}	else if (req=='W') {
#ifdef debug
			Serial.println("WATCHDOG");
#endif
			WDTCSR = _BV(WDE);
			while (1); // 16 ms
		}
		//sendDataSerial();
	}
}


void sendDataSerial() {
	//#0;25.31#1;25.19#2;5.19$3600177622*
	crc = ~0L;
  for (byte i=0;i<numberOfDevices; i++) {
		START_BLOCK
		send(i);
		DELIMITER
		for (byte j=0; j<8; j++) {
			if (tempDeviceAddresses[i][j]<9) send('0');
			send(tempDeviceAddresses[i][j], 'X');
		}
		DELIMITER
		send(sensor[i]);
		//DELIMITER
	}
	END_BLOCK
#ifdef debug
	Serial.print(crc);
	Serial.print("*");
#endif	
	mySerial.print(crc);
	mySerial.print("*");
}

void send(char s) {
	send(s, ' ');
}

void send(char s, char type) {
	if (type=='X') {
#ifdef debug
		Serial.print(s, HEX);
#endif
		mySerial.print(s, HEX);
	}
	else {
#ifdef debug
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
#ifdef debug
		Serial.print(s, HEX);
#endif
		mySerial.print(s, HEX);
	}
	else {
#ifdef debug
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
#ifdef debug
		Serial.print("Data req-");
		Serial.println(incomingByte);
#endif
  }
	return incomingByte;
}

void dsInit(void) {
  dsSensors.begin();
  numberOfDevices = dsSensors.getDeviceCount();
#ifdef debug
  Serial.print("Sensor(s):");
  Serial.println(numberOfDevices);
#endif
  // Loop through each device, print out address
  for (byte i=0;i<numberOfDevices; i++) {
      // Search the wire for address
    if (dsSensors.getAddress(tempDeviceAddress, i)) {
      memcpy(tempDeviceAddresses[i],tempDeviceAddress,8);
    }
		digitalWrite(LEDPIN,HIGH);
		delay(500);
		digitalWrite(LEDPIN,LOW);
		delay(500);
  }
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
