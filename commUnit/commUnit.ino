#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

const char *ssid = "Datlovo";
const char *password = "Nu6kMABmseYwbCoJ7LyG";

#define AIO_SERVER      "192.168.1.56"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "datel"
#define AIO_KEY         "hanka12"

WiFiClient client;

byte heartBeat                    = 12;
String received                   = "";

#define pinLed                    2

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/
Adafruit_MQTT_Publish version             = Adafruit_MQTT_Publish(&mqtt, "/home/Corridor/esp07/VersionSWSolar");
Adafruit_MQTT_Publish hb                  = Adafruit_MQTT_Publish(&mqtt, "/home/Corridor/esp07/HeartBeat");
Adafruit_MQTT_Publish tP1INSolar          = Adafruit_MQTT_Publish(&mqtt, "/home/Corridor/esp07/tP1IN");
Adafruit_MQTT_Publish tP1OUTSolar         = Adafruit_MQTT_Publish(&mqtt, "/home/Corridor/esp07/tP1OUT");
Adafruit_MQTT_Publish tP2INSolar          = Adafruit_MQTT_Publish(&mqtt, "/home/Corridor/esp07/tP2IN");
Adafruit_MQTT_Publish tP2OUTSolar         = Adafruit_MQTT_Publish(&mqtt, "/home/Corridor/esp07/tP2OUT");
Adafruit_MQTT_Publish qSolar              = Adafruit_MQTT_Publish(&mqtt, "/home/Corridor/esp07/prutok");
Adafruit_MQTT_Publish sPumpSolar          = Adafruit_MQTT_Publish(&mqtt, "/home/Corridor/esp07/sPumpSolar/status");
Adafruit_MQTT_Publish tRoom               = Adafruit_MQTT_Publish(&mqtt, "/home/Corridor/esp07/tRoom");
Adafruit_MQTT_Publish tBojler             = Adafruit_MQTT_Publish(&mqtt, "/home/Corridor/esp07/tBojler");
Adafruit_MQTT_Publish tBojlerIN           = Adafruit_MQTT_Publish(&mqtt, "/home/Corridor/esp07/tBojlerIN");
Adafruit_MQTT_Publish tBojlerOUT          = Adafruit_MQTT_Publish(&mqtt, "/home/Corridor/esp07/tBojlerOUT");

#define SERIALSPEED 9600

void MQTT_connect(void);

float versionSW                   = 0.61;
String versionSWString            = "Solar v";

void setup() {
  Serial.begin(SERIALSPEED);
  Serial.print(versionSWString);
  Serial.print(versionSW);
  
  WiFi.begin(ssid, password);

	// Wait for connection
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}

	Serial.println("");
	Serial.print("Connected to ");
	Serial.println(ssid);
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());
  pinMode(pinLed,OUTPUT); 
  digitalWrite(pinLed,LOW);
  delay(1000);
  digitalWrite(pinLed,HIGH);
}

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  if (Serial.available()>0) { 
    received=Serial.readStringUntil('*');
    Serial.println(received);
    
    float _tP1INSolar, _tP1OUTSolar, _tP2INSolar, _tP2OUTSolar, _tBojler, _tBojlerIN, _tBojlerOUT, _tRoom, _qSolar;
    int _pumpStatus=0;
    
    bool emptyData=false;
    Serial.println(received);
    
    //received="#B;39.44#M;21.19#I;45.50#O;43.44#R;0$*

    received.trim();
    if (received!="") {
      digitalWrite(pinLed,LOW);
      byte i=1;
      while (i<=11) {
        String val = getValue(received, '#', i);
        if (val.substring(0,1)=="B") {
          _tBojler=val.substring(2).toFloat(); 
        }
        if (val.substring(0,1)=="C") {
          _tBojlerIN=val.substring(2).toFloat(); 
        }
        if (val.substring(0,1)=="D") {
          _tBojlerOUT=val.substring(2).toFloat(); 
        }
        if (val.substring(0,1)=="M") {
          _tRoom=val.substring(2).toFloat(); 
        }
        if (val.substring(0,1)=="I") {
          _tP2INSolar=val.substring(2).toFloat(); 
        }
        if (val.substring(0,1)=="O") {
          _tP2OUTSolar=val.substring(2).toFloat(); 
        }
        if (val.substring(0,1)=="S") {
          _tP1INSolar=val.substring(2).toFloat(); 
        }
        if (val.substring(0,1)=="T") {
          _tP1OUTSolar=val.substring(2).toFloat(); 
        }
        if (val.substring(0,1)=="Q") {
          _qSolar=val.substring(2).toFloat(); 
        }
        if (val.substring(0,1)=="R") {
          _pumpStatus=val.substring(2).toInt(); 
        }
        i++;
      }

      Serial.println("I am sending data from Solar unit to HomeAssistant");
    
      MQTT_connect();
      if (! tBojler.publish(_tBojler)) {
        Serial.println("failed");
      } else {
        Serial.println("OK!");
      }
      if (! tBojlerIN.publish(_tBojlerIN)) {
        Serial.println("failed");
      } else {
        Serial.println("OK!");
      }
      if (! tBojlerOUT.publish(_tBojlerOUT)) {
        Serial.println("failed");
      } else {
        Serial.println("OK!");
      }
      if (! tRoom.publish(_tRoom)) {
        Serial.println("failed");
      } else {
        Serial.println("OK!");
      }
      if (! tP1INSolar.publish(_tP1INSolar)) {
        Serial.println("failed");
      } else {
        Serial.println("OK!");
      }
      if (! tP1OUTSolar.publish(_tP1OUTSolar)) {
        Serial.println("failed");
      } else {
        Serial.println("OK!");
      }
      if (! tP2INSolar.publish(_tP2INSolar)) {
        Serial.println("failed");
      } else {
        Serial.println("OK!");
      }
      if (! tP2OUTSolar.publish(_tP2OUTSolar)) {
        Serial.println("failed");
      } else {
        Serial.println("OK!");
      }
      if (! qSolar.publish(_qSolar)) {
        Serial.println("failed");
      } else {
        Serial.println("OK!");
      }
      if (! sPumpSolar.publish(_pumpStatus)) {
        Serial.println("failed");
      } else {
        Serial.println("OK!");
      }
     
      if (! version.publish(versionSW)) {
        Serial.println("failed");
      } else {
        Serial.println("OK!");
      }
      if (! hb.publish(heartBeat++)) {
        Serial.println("failed");
      } else {
        Serial.println("OK!");
      }
      if (heartBeat>1) {
        heartBeat=0;
      }
      digitalWrite(pinLed,HIGH);
    } else {
        emptyData=true;
        Serial.println("empty data");
    }
  }
  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  /*
  if(! mqtt.ping()) {
    mqtt.disconnect();
  }
  */

}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}

String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}