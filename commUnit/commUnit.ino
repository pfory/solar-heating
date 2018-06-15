#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <WiFiManager.h> 

#define AIO_SERVER      "192.168.1.56"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "datel"
#define AIO_KEY         "hanka12"

#define verbose
#ifdef verbose
 #define DEBUG_PRINT(x)         Serial.print (x)
 #define DEBUG_PRINTDEC(x)      Serial.print (x, DEC)
 #define DEBUG_PRINTLN(x)       Serial.println (x)
 #define DEBUG_PRINTF(x, y)     Serial.printf (x, y)
 #define PORTSPEED 115200
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTDEC(x)
 #define DEBUG_PRINTLN(x)
 #define DEBUG_PRINTF(x, y)
#endif 

WiFiClient client;
WiFiManager wifiManager;

uint32_t heartBeat                    = 12;
String received                       = "";
unsigned long milisLastRunMinOld      = 0;

IPAddress _ip           = IPAddress(192, 168, 1, 108);
IPAddress _gw           = IPAddress(192, 168, 1, 1);
IPAddress _sn           = IPAddress(255, 255, 255, 0);


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

float versionSW                   = 0.62;
String versionSWString            = "Solar v";

void setup() {
  Serial.begin(SERIALSPEED);
  DEBUG_PRINT(versionSWString);
  DEBUG_PRINT(versionSW);
  
  
  DEBUG_PRINTLN(ESP.getResetReason());
  if (ESP.getResetReason()=="Software/System restart") {
    heartBeat=1;
  } else if (ESP.getResetReason()=="Power on") {
    heartBeat=2;
  } else if (ESP.getResetReason()=="External System") {
    heartBeat=3;
  } else if (ESP.getResetReason()=="Hardware Watchdog") {
    heartBeat=4;
  } else if (ESP.getResetReason()=="Exception") {
    heartBeat=5;
  } else if (ESP.getResetReason()=="Software Watchdog") {
    heartBeat=6;
  } else if (ESP.getResetReason()=="Deep-Sleep Wake") {
    heartBeat=7;
  }
  
  //WiFi.config(ip); 
  wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn);
  if (!wifiManager.autoConnect("AutoConnectAP", "password")) {
    DEBUG_PRINTLN("failed to connect, we should reset as see if it connects");
    delay(3000);
    ESP.reset();
    delay(5000);
  }

	// Wait for connection
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		DEBUG_PRINT(".");
	}

	DEBUG_PRINTLN("");
	DEBUG_PRINT("IP address: ");
	DEBUG_PRINTLN(WiFi.localIP());
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
    DEBUG_PRINTLN(received);
    
    float _tP1INSolar, _tP1OUTSolar, _tP2INSolar, _tP2OUTSolar, _tBojler, _tBojlerIN, _tBojlerOUT, _tRoom, _qSolar;
    int _pumpStatus=0;
    
    bool emptyData=false;
    DEBUG_PRINTLN(received);
    
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

      DEBUG_PRINTLN("I am sending data from Solar unit to HomeAssistant");
    
      MQTT_connect();
      if (! tBojler.publish(_tBojler)) {
        DEBUG_PRINTLN("failed");
      } else {
        DEBUG_PRINTLN("OK!");
      }
      if (! tBojlerIN.publish(_tBojlerIN)) {
        DEBUG_PRINTLN("failed");
      } else {
        DEBUG_PRINTLN("OK!");
      }
      if (! tBojlerOUT.publish(_tBojlerOUT)) {
        DEBUG_PRINTLN("failed");
      } else {
        DEBUG_PRINTLN("OK!");
      }
      if (! tRoom.publish(_tRoom)) {
        DEBUG_PRINTLN("failed");
      } else {
        DEBUG_PRINTLN("OK!");
      }
      if (! tP1INSolar.publish(_tP1INSolar)) {
        DEBUG_PRINTLN("failed");
      } else {
        DEBUG_PRINTLN("OK!");
      }
      if (! tP1OUTSolar.publish(_tP1OUTSolar)) {
        DEBUG_PRINTLN("failed");
      } else {
        DEBUG_PRINTLN("OK!");
      }
      if (! tP2INSolar.publish(_tP2INSolar)) {
        DEBUG_PRINTLN("failed");
      } else {
        DEBUG_PRINTLN("OK!");
      }
      if (! tP2OUTSolar.publish(_tP2OUTSolar)) {
        DEBUG_PRINTLN("failed");
      } else {
        DEBUG_PRINTLN("OK!");
      }
      if (! qSolar.publish(_qSolar)) {
        DEBUG_PRINTLN("failed");
      } else {
        DEBUG_PRINTLN("OK!");
      }
      if (! sPumpSolar.publish(_pumpStatus)) {
        DEBUG_PRINTLN("failed");
      } else {
        DEBUG_PRINTLN("OK!");
      }
     
     
      if (millis() - milisLastRunMinOld > 60000) {
        milisLastRunMinOld = millis();
        if (! hb.publish(heartBeat)) {
          DEBUG_PRINTLN("Send HB failed");
        } else {
          DEBUG_PRINTLN("Send HB OK!");
        }
        heartBeat++;
        if (! version.publish(versionSW)) {
          DEBUG_PRINT(F("Send verSW failed!"));
        } else {
          DEBUG_PRINT(F("Send verSW OK!"));
        }
      }
  
      digitalWrite(pinLed,HIGH);
    } else {
        emptyData=true;
        DEBUG_PRINTLN("empty data");
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

  DEBUG_PRINT("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       DEBUG_PRINTLN(mqtt.connectErrorString(ret));
       DEBUG_PRINTLN("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  DEBUG_PRINTLN("MQTT Connected!");
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