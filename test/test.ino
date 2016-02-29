float f;
char charBuf[13];

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println("test");
  f=10.2;
  Serial.println(floatToString(f));
  f=0.2;
  Serial.println(floatToString(f));
  f=-10.2;
  Serial.println(floatToString(f));
  f=-0.2;
  Serial.println(floatToString(f));
  f=100.2;
  Serial.println(floatToString(f));
  f=-100.2;
  Serial.println(floatToString(f));
  f=10.22;
  Serial.println(floatToString(f));
  f=0.222;
  Serial.println(floatToString(f));
  f=123456789.01;
  Serial.println(floatToString(f));
  delay(100000);
}


char* floatToString(float f) {
  dtostrf(f, 1, 2, charBuf);
  return charBuf;
}
