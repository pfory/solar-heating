#include <Wire.h>
#include <OneWire.h>
#define ONE_WIRE_BUS A0
OneWire onewire(ONE_WIRE_BUS);  // pin for onewire DALLAS bus
//#define dallasMinimal           //-956 Bytes
#include <DallasTemperature.h>
DallasTemperature dsSensors(&onewire);
DeviceAddress tempDeviceAddress;
#ifndef NUMBER_OF_DEVICES
#define NUMBER_OF_DEVICES 4
#endif
DeviceAddress tempDeviceAddresses[NUMBER_OF_DEVICES];
unsigned int numberOfDevices              = 0; // Number of temperature devices found
unsigned long lastDsMeasStartTime         = 0;
bool dsMeasStarted                        = false;
float sensor[NUMBER_OF_DEVICES];
unsigned long const dsMeassureInterval    = 750; //inteval between meassurements
bool firstMeasComplete                    = false;

#include <LiquidCrystal_I2C.h>

//-------------------------------------------- S E T U P ------------------------------------------------------------------------------
void setup() {
  Wire.begin();
  Serial.begin(9600);

  dsInit();
  
} //setup


//----------------------------------------L O O P ----------------------------------------------------------------------------------
void loop() {
  if (numberOfDevices>0) {
    tempMeas();
  }
} //loop


//--------------------------------------- F U N C T I O N S -----------------------------------------------------------------------------------
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
      Serial.println(tempTemp);
    } 
  }
}

void dsInit(void) {
  dsSensors.begin();
  numberOfDevices = dsSensors.getDeviceCount();


  Serial.print("Sensor(s):");
  Serial.println(numberOfDevices);

  // Loop through each device, print out address
  for (byte i=0;i<numberOfDevices; i++) {
      // Search the wire for address
    if (dsSensors.getAddress(tempDeviceAddress, i)) {
      memcpy(tempDeviceAddresses[i],tempDeviceAddress,8);
    }
  }
}

