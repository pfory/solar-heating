//sensors 0.solar OUT temperature
//        1.solar IN temperature
//        2.room temperature

//HW
//Arduino 2009
//Ethernet shield
//I2C display
//2 Relays module
//DALLAS
//keyboard

// A0 - DALLAS temperature sensors
// A1 - relay 1
// A2 - relay 2
// A3 - keyboard
// A4 - I2C display SDA
// A5 - I2C display SCL
// D0 - Rx
// D1 - Tx
// D2 - keyboard
// D3 - keyboard
// D4 - SD card on ethernet shield
// D5 - keyboard
// D6 - keyboard
// D7 - keyboard
// D8 - keyboard
// D9 - keyboard
// D10 - ethernet shield
// D11 - ethernet shield
// D12 - ethernet shield
// D13 - ethernet shield



//spina rele pro cerpadlo v zavislosti na rozdilu teplot z cidla 0 a 2. 

//TODO
//umazat znaky za teplotami
//Rozdil teplot lze nastavit v konfiguraci pres internet
//posila teploty ze vsech 3 cidel na cosm.com
//spina ventil(y) pro rizeni natapeni bojleru nebo radiatoru v zavislosti na konfiguraci zjistene pres internet
//cidlo 2 slouzi k zjistovani vykonu podle rozdilu teplot

#include <limits.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h> 

#ifndef dummy //this section prevent from error while program is compiling without Ethernetdef
char a[0]; //do not delete this dummy variable
#endif


//#define ethernet
// #ifdef ethernet
// #include <Ethernet.h>
// byte mac[] = { 0x00, 0xE0, 0x07D, 0xCE, 0xC6, 0x70};
// EthernetClient client;
// char server[] = "api.cosm.com";   // name address for cosm API COSM
// #define APIKEY         "" // your cosm api key
// #define FEEDID         0 // your feed ID
// #define USERAGENT      "Solar Arduino" // user agent is the project name
// char dataString[280];

// unsigned long lastSendTime;
// #endif


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
#include <DallasTemperature.h>
#define ONE_WIRE_BUS A0
OneWire onewire(ONE_WIRE_BUS); // pin for onewire DALLAS bus
DallasTemperature dsSensors(&onewire);
DeviceAddress tempDeviceAddress;
#ifndef NUMBER_OF_DEVICES
#define NUMBER_OF_DEVICES 3
#endif
DeviceAddress tempDeviceAddresses[NUMBER_OF_DEVICES];
//int  resolution = 12;
unsigned int numberOfDevices; // Number of temperature devices found
unsigned long lastDisplayTempTime;
unsigned long lastDsMeasStartTime;
bool dsMeasStarted=false;
float sensor[NUMBER_OF_DEVICES];
unsigned int sample=0;
bool checkConfigFlag = false;
unsigned int const sendTimeDelay=5000; //to send to cosm.com
float tempDiffON = 25.0; //difference between room temperature and solar OUT (sensor 2 - sensor 1) to set relay ON
float tempDiffOFF = 10.0; //difference between room temperature and solar OUT (sensor 2 - sensor 1) to set relay OFF
int sensorReading = INT_MIN;
unsigned int const dsMeassureInterval=750; //inteval between meassurements
unsigned long lastMeasTime=0;
unsigned long msDayON = 0;  //kolik ms uz jsem za aktualni den byl ve stavu ON
unsigned long lastOn=0;     //ms posledniho behu ve stavu ON
unsigned long lastOff = 0;  //ms posledniho vypnuti rele
unsigned long const dayInterval=1000*60*60*12; //
unsigned int power = 0; //actual power in W
float energy = 0; //energy a day in kWh
float energyKoef = 283.0; //Ws

/*#define TEMP1X 0
#define TEMP1Y 0
#define TEMP2X 6
#define TEMP2Y 0
#define TEMP3X 0
#define TEMP3Y 1
//#define TEMP4X 9
//#define TEMP4Y 1
*/

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

#include <Keypad.h>
const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
//define the cymbols on the buttons of the keypads
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

char versionSW[]="0.11";
char versionSWString[] = "Solar v"; //SW name & version

void dsInit(void);
void displayRelayStatus(void);

void setup() {
  // start serial port:
  Serial.begin(115200);
//  Serial.println();
  Serial.println(versionSW);

  //  Serial.print("waiting for net connection...");
  #ifdef ethernet
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed using DHCP");
    // DHCP failed, so use a fixed IP address:
  }

  Serial.println("EthOK");
  
//  Serial.print("\nIP:");
//  Serial.println(Ethernet.localIP());
  /*Serial.print("Mask:");
  Serial.println(Ethernet.subnetMask());
  Serial.print("Gateway:");
  Serial.println(Ethernet.gatewayIP());
  Serial.print("DNS:");
  Serial.println(Ethernet.dnsServerIP());
  Serial.println();
  */
  lastSendTime = lastMeasTime = millis();
  #endif

  
  //int charBitmapSize = (sizeof(charBitmap ) / sizeof (charBitmap[0]));

  // Switch on the backlight
  pinMode ( BACKLIGHT_PIN, OUTPUT );
  digitalWrite ( BACKLIGHT_PIN, HIGH );
  
  lcd.begin(16,2);               // initialize the lcd 

   // for ( int i = 0; i < charBitmapSize; i++ )
   // {
      // lcd.createChar ( i, (uint8_t *)charBitmap[i] );
   // }

  lcd.home ();                   // go home
  lcd.print(versionSWString);  
  lcd.print(" ");
  lcd.print (versionSW);
  delay (3000);
  lcd.clear();
  
  dsInit();
  lastDisplayTempTime = millis();
  dsSensors.requestTemperatures(); 

  /*lcd.setCursor(0,0);
  lcd.print("1:");
  lcd.setCursor(8,0);
  lcd.print("2:");
  lcd.setCursor(0,1);
  lcd.print("3:");
  lcd.setCursor(8,1);
  lcd.print("4:");
  */
  
  lcd.clear();

  pinMode(RELAY1PIN, OUTPUT);
  //pinMode(RELAY2PIN, OUTPUT);
  
  digitalWrite(RELAY1PIN, relay1);
  //digitalWrite(RELAY2PIN, relay2);
  //displayRelayStatus();
}

void loop() {
  if (!dsMeasStarted) {
    //start sampling
    dsMeasStarted=true;
    dsSensors.requestTemperatures(); 
    lastDsMeasStartTime = millis();
  }
  else if (dsMeasStarted && (millis() - lastDsMeasStartTime>dsMeassureInterval)) {
    dsMeasStarted=false;
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
      Serial.println(sensor[i]);
    } 


    //show temperatures on display (first row)
    lcd.setCursor(0,0);
    lcd.print((int)sensor[0]);
    lcd.print(".");
    lcd.print(abs((int)(sensor[0]*10)%10));

    //lcd.setCursor(TEMP2X,TEMP2Y);
    lcd.print(" ");
    lcd.print((int)sensor[1]);
    lcd.print(".");
    lcd.print(abs((int)(sensor[1]*10)%10));

    //lcd.setCursor(TEMP3X,TEMP3Y);
    lcd.print(" ");
    lcd.print((int)sensor[2]);
    lcd.print(".");
    lcd.print(abs((int)(sensor[2]*10)%10));

    lcd.print("    "); //smaze znaky za teplotami
    
    
    if (relay1==LOW) {  //relay is ON
      msDayON+=(millis()-lastOn);
      energy+=(((millis()-lastOn)/1000)*energyKoef*(sensor[0]-sensor[1])); //in Ws
      lastOn = millis();
    }

    //zobrazeni okamziteho vykonu ve W
    //zobrazeni celkoveho vykonu za den v kWh
    //zobrazeni poctu minut behu cerpadla za aktualni den
    //0123456789012345
    // 636 0.1234 720T
    lcd.setCursor(0,1);
    lcd.print((int)power);
   
    lcd.setCursor(5,1);
    lcd.print(energy/1000/3600); //Wh -> kWh (show it in kWh)
    
    lcd.setCursor(12,1);
    lcd.print((int)msDayON/1000/60); //ms->min (show it in minutes)
    
    
    //change relay 1 status
    if (relay1==HIGH) { //switch OFF->ON
      if (sensor[0] - sensor[2] >= tempDiffON) {
        relay1=LOW; //relay ON
        digitalWrite(RELAY1PIN, relay1);
        lastOn = millis();
        if (lastOff==0) { //first ON in actual day
          energy=0.0;
          msDayON=0;
        }
      }
    }
    if (relay1==LOW) { //switch ON->OFF
      if (sensor[0] - sensor[2] < tempDiffOFF) {
        relay1=HIGH; ///relay OFF
        digitalWrite(RELAY1PIN, relay1);
        lastOff=millis();
      }
    }
    displayRelayStatus();
    
    if (lastOff > 0 && (millis() - lastOff>dayInterval)) {
        lastOff = 0;
    }
  }


  #ifdef ethernet
  if (sample==2) {
    client.stop();
  }

  if (sample==5 && checkConfigFlag == false) {
    //checkConfig();
  }

  if (sample==8) {
    checkConfigFlag = false;
  }

  if(!client.connected() && (millis() - lastSendTime > sendTimeDelay)) {
    lastSendTime = millis();
    //digitalWrite(ledPin, HIGH);
    sendData();
    //digitalWrite(ledPin, LOW);
    sample=0;
  }
  #endif
 
  char customKey = customKeypad.getKey();
  if (customKey){
    lcd.setCursor(0,0);
    lcd.print(customKey);
  }
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
    
  
  // Loop through each device, print out address
  for (byte i=0;i<numberOfDevices; i++) {
      // Search the wire for address
    if (dsSensors.getAddress(tempDeviceAddress, i)) {
      memcpy(tempDeviceAddresses[i],tempDeviceAddress,8);
    }
  }
}

//show relay's status in column 15
void displayRelayStatus(void) {
  lcd.setCursor(RELAY1X,RELAY1Y);
  if (relay1==LOW)
    lcd.print("T");
  else
    lcd.print("N");
/*  lcd.setCursor(RELAY2X,RELAY2Y);
  if (relay2==LOW)
    lcd.print("T");
  else
    lcd.print("N");
*/
}

#ifdef ethernet
void sendData() {

  //Serial.println("sending data");
  
  //prepare data to send
  #ifdef stringdef
  dataString1="";
  char buffer[3];
  #endif

  //temperature from DALLAS
  //00 01 02 03 04 05 06 07
  //-----------------------
  //28 C9 B8 41 04 00 00 97

  
  int n; //data length
  sprintf(dataString,"V,%s\n",versionSW);
  
  for(byte i=0;i<numberOfDevices; i++) {
    sprintf(dataString,"%sT",dataString);
    
    for (byte j=0; j<8; j++) {
      if (tempDeviceAddresses[i][j]<16) {
        sprintf(dataString,"%s0",dataString);
      }
    }

    int t = (int)(sensor[i]*10);
    sprintf(dataString,"%s,",dataString);

    if (t<0&&t>-10) {
      sprintf(dataString,"%s-",dataString);
    }
    sprintf(dataString,"%s%d.%u\n",dataString,t/10,abs(t%10));
  }

  // if there's a successful connection:
  if (client.connect(server, 80)) {
    Serial.println("connected");
    // send the HTTP PUT request:
    client.print("PUT /v2/feeds/");
    client.print(FEEDID);
    client.println(".csv HTTP/1.1");
    client.println("Host: api.cosm.com");
    client.print("X-ApiKey: ");
    client.println(APIKEY);
    client.print("User-Agent: ");
    client.println(USERAGENT);
    client.print("Content-Length: ");
    client.println(n);

    // last pieces of the HTTP PUT request:
    client.println("Content-Type: text/csv");
    client.println("Connection: close");
    client.println();

    // here's the actual content of the PUT request:
    client.print(dataString);
  } 
  else {
    // if you couldn't make a connection:
    Serial.println("failed");
    client.stop();
  }
 
  Serial.println(dataString);
}
#endif //ethernet