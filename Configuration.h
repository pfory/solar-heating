#ifndef CONFIGURATION_H
#define CONFIGURATION_H

//SW name & version
#define     VERSION                      "1.31"
#define     SW_NAME                      "Solar"

/*
Version history:
1.31 - change display to 4x20
1.1  - 13.9.2017  pridany cidla teploty bojler vstup/vystup
1.01 - 28.8.2017  pridani cidla prutoku
1.00 - 28.8.2017  vypinani pouze na zaklade vystupni teploty e solaru na strese 
0.99 - 24.8.2017  mereni z obou panelu
0.95 - 19.6.2016  i2c keypad
0.90 - 05.05.2016 komunikace s jednokou ESP8266
0.82 - 27.1.2016  opraveno zobrazeni teplot od 0 do -0.9 na displeji
0.80 - 9.9.2015   I2C komunikace s powerMeter unit
0.79 - 24.10.2014 zachyceni stavu po resetu
0.78 - 27.9.2014  pridano zobrazeni dnu bez slunce, zap/vyp podsviceni
0.77 - 14.8.2014
0.76 - 7.8.2014
0.75 - 31.7.2014  funkcni pocitani totalSec a totalPower
0.74 - 30.7.2014  add delay after ON, prevent cyclic switch ON-OFF-ON....
0.73 - 29.7.2014  add totalSec, oprava posilani totalPower
0.72 - 14.6.2013  zmena spinani teplot
0.71 - 5.6.2014   optiboot, watchdog
0.70 - 21.5.2014
0.69 - 21.5.2014
0.68 - 20.5.2014
0.67 - 15.6.2014
0.66 - 6.4.2014
0.65 - 26.3.2014
0.60 - 16.3.2014
0.50 - 1.12.2013
0.41 - 20.10.2013
TODO - odladit CRC kod
--------------------------------------------------------------------------------------------------------------------------
HW
Pro Mini 328 with Optiboot!!!! data are sent via serial line to comunication unit
I2C display
2 Relays module
DALLAS
keyboard
Pro Mini 328 Layout
------------------------------------------
A0              - DALLAS temperature sensors
A1              - relay 1
A2              - relay 2
A3              - free
A4              - I2C display SDA 0x20, keypad 0x27
A5              - I2C display SCL 0x20, keypad 0x27
D0              - Rx
D1              - Tx
D2              - flow sensor
D3              - free
D4              - free
D5              - free
D6              - free
D7              - free
D8              - free
D9              - free
D10             - Rx 
D11             - Tx
D12             - free
D13             - free
--------------------------------------------------------------------------------------------------------------------------
*/

#define START_BLOCK       '#'
#define DELIMITER         ';'
#define END_BLOCK         '$'
#define END_TRANSMITION   '*'

#define LEDPIN 13

//pins for softwareserial
#define RX 10
#define TX 11

//display
#define LCDADDRESS   0x27
#define LCDROWS      4
#define LCDCOLS      20

//one wire bus
#define ONE_WIRE_BUS A0

//#define dallasMinimal           //-956 Bytes
#ifndef NUMBER_OF_DEVICES
#define NUMBER_OF_DEVICES 10
#endif


#define STATUS_NORMAL0                        0
#define STATUS_NORMAL1                        1
#define STATUS_AFTER_START                    2
#define STATUS_WRITETOTALTOEEPROM_DELAY       3
#define STATUS_WRITETOTALTOEEPROM_ONOFF       4
#define STATUS_WRITETOTALTOEEPROM_MANUAL      5
#define STATUS_STARTAFTER_BROWNOUT            6
#define STATUS_STARTAFTER_POWERON             7
#define STATUS_STARTAFTER_WATCHDOGOREXTERNAL  8
#endif


//0123456789012345
//15.6 15.8 15.8 V
//1234 0.12 624
#define TEMP1X                               0  //P1 In
#define TEMP1Y                               0
#define TEMP2X                               3  //P1 Out
#define TEMP2Y                               0
#define TEMP3X                               6  //P2 In
#define TEMP3Y                               0
#define TEMP4X                               9  //P2 Out
#define TEMP4Y                               0
#define TEMP5X                              12  //Control
#define TEMP5Y                               0
#define TEMP6X                              12  //Bojler in
#define TEMP6Y                               2
#define TEMP7X                              16  //Bojler out
#define TEMP7Y                               2
#define POWERX                               0
#define POWERY                               1
#define ENERGYX                              8
#define ENERGYY                              1
#define TIMEX                               16
#define TIMEY                                1
#define FLOWX                                1
#define FLOWY                                2
                          
                          
#define RELAY1X                             19
#define RELAY1Y                              0
/*#define RELAY2X                           15
#define RELAY2Y                              1
*/                          
                          
#define RELAY1PIN                           A1
#define RELAY2PIN                           A2

//keypad i2c address
#define I2CADDR                             0x20

//EEPROM config
#define CONFIG_START 0
#define CONFIG_VERSION "v01"


#define DS_MEASSURE_INTERVAL                750 //inteval between meassurements
#define DELAY_AFTER_ON                      120000 //1000*60*2; //po tento cas zustane rele sepnute bez ohledu na stav teplotnich cidel

#define DAY_INTERVAL                        43200000 //1000*60*60*12; //

#define LAST_WRITE_EEPROM_DELAY             3600000 //in ms = 1 hod
#define SAFETY_ON                           80.0 //teplota, pri niz rele vzdy sepne

#define SEND_DELAY                          10000  //prodleva mezi poslanim dat v ms
