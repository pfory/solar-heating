int incomingByte = 0;   // for incoming serial data
byte tempNoB=0;
#define NUMBER_OF_DEVICES 3
float tempValueF[NUMBER_OF_DEVICES];
typedef uint8_t DeviceAddress[8];
DeviceAddress tempDeviceAddresses[NUMBER_OF_DEVICES];

#include <avr/pgmspace.h>
unsigned long crc;
static PROGMEM prog_uint32_t crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

void setup() {
  Serial.begin(9600);
}

void loop() {
	readDataUART();
}

void readDataUART() {
  //read data from UART
	byte flag=0;
	char tempNo[2+1];
	DeviceAddress sensorId;
	char tempValue[6+1];
	byte p=0;
	bool resetP=false;
	unsigned long timeOut = millis();
	short numberOfSensors=-1;
	crc = ~0L;
	char buff[2+1];
		
	//Serial.println("START");
	//#0;28422E8104000097;21.44;#1;28EA676B05000089;21.38;$541458114*
	do {
		incomingByte = Serial.read();
		if (incomingByte > 0) {
			if (incomingByte=='#') {
				flag=1;
				resetP=true;
			} else if (incomingByte==';') {
				flag++;
				resetP=true;
			}
			
			//Serial.print("flag-");
			//Serial.println(flag);
			if (flag==1) { //read temp #
				if (resetP) {
					p=0;
				} else {
					tempNo[p++]=incomingByte;
					crc_string(incomingByte);
				}
			}
			else if (flag==2) { //read sensor id
				if (resetP) {
					tempNo[p]='\0';
					//Serial.print("tempNo-");
					tempNoB=atoi(tempNo);
					if (tempNoB>numberOfSensors) numberOfSensors=tempNoB;
					p=0;
				} else {
					if (p%2==0) {
						buff[0]=(char)incomingByte;
					}
					else {
						Serial.print('(');
						for(byte i=0;i<3;i++) {
							Serial.print(buff[i]);
						}
						Serial.print(')');
						buff[1]=(char)incomingByte;
						buff[2]='\0';
						Serial.print(atoi(buff));
						Serial.print(';');
						sensorId[p/2]=atoi(buff);;
					}
					p++;
				}
			}
			else if (flag==3) { //read temp value
				if (resetP) {
					for (byte i=0;i<8;i++) {
						if (sensorId[i]<9)
							crc_string('0');
						crc_string(sensorId[i]);
					}
					sensorId[p]='\0';
					//Serial.print("sensorId-");
		      memcpy(tempDeviceAddresses[tempNoB],sensorId,16);
					p=0;
				} else {
					tempValue[p++]=incomingByte;
					crc_string(incomingByte);
				}
			}
			else if (flag==4) {
				if (resetP) {
					tempValue[p]='\0';
					//Serial.print("tempValue-");
					tempValueF[tempNoB]=atof(tempValue);
					p=0;
				}
			}
			resetP=false;
		}
	} while ((char)incomingByte!='*' && millis() < (timeOut + 2000));
	
	if (numberOfSensors>-1) {
		for (byte i=0; i<=numberOfSensors; i++) {
			for (byte j=0; j<8; j++) {
				if (tempDeviceAddresses[i][j]<9)
					Serial.print('0');
				Serial.print(tempDeviceAddresses[i][j], 'X');
			}
			Serial.print("-");
			Serial.println(tempValueF[i]);
		}
		Serial.println();
		Serial.println(crc);
	}
	
	//Serial.println("END");

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