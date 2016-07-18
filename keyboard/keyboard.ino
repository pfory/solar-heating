#include <Wire.h>

int address = 0x27;
uint8_t data;
int error;
char key                                  = ' ';
char keyOld                               = ' ';
unsigned int repeatAfterMs                = 300;
unsigned int repeatCharSec                = 10;
unsigned long lastKeyPressed              = 0;

const byte ROWS                           = 4; //four rows
const byte COLS                           = 4; //four columns
char hexaKeys[ROWS][COLS]                 = {
                                            {'*','0','#','D'},
                                            {'7','8','9','C'},
                                            {'4','5','6','B'},
                                            {'1','2','3','A'}
};


byte i=0;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.print("Keyboard I2C test at address ");
  Serial.println(address);
}

void loop() {
  keyPressed();
  //delay(500);
}


uint8_t read8() {
  Wire.beginTransmission(address);
  Wire.requestFrom(39, 1);
  data = Wire.read();
  error = Wire.endTransmission();
  //Serial.println(error);
  return data;
}
/*
uint8_t read(uint8_t pin) {
  read8();
  return (data & (1<<pin)) > 0;
}
*/
void write8(uint8_t value) {
  Wire.beginTransmission(address);
  //data = value;
  Wire.write(value);
  error = Wire.endTransmission();
}
/*
void write(uint8_t pin, uint8_t value) {
  read8();
  if (value == LOW) {
    data &= ~(1<<pin);
  }else{
    data |= (1<<pin);
  }
  write8(data); 
}
*/
void keyPressed() {
  byte row=0;
  byte col=255;
  byte b=127;
  if (millis() - lastKeyPressed > repeatAfterMs) {
    keyOld = 0;
  }
  //write8(1);
  //Serial.println(read8());
  
  for (byte i=0; i<4; i++) {
    row=i;
    b=~(255&(1<<i+4));
    //Serial.println(b);
    write8(b);
    //Serial.println(read8());
    col = colTest(read8(), b);
    //Serial.println(col);
    if (col<255) {
      key = hexaKeys[row][col];
      //Serial.print(key);
      if (key!=keyOld) {
        lastKeyPressed = millis();
        keyOld=hexaKeys[row][col];
        Serial.print(row);
        Serial.print(",");
        Serial.print(col);
        Serial.print("=");
        Serial.println((char)key);
      }
      break;
    }
  }
}

byte colTest(byte key, byte b) {
  if (key==b-8) return 0;
  else if (key==b-4) return 1;
  else if (key==b-2) return 2;
  else if (key==b-1) return 3;
  else return 255;
}