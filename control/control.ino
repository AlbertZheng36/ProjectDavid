void setup() {
  Serial.begin(115200);
}

char senddata = '1';

void loop() {
  Serial.print(senddata);
  delay(1000);
  Serial.print(senddata);
  delay(1000);
  Serial.print(senddata);
  delay(1000);
  Serial.print(senddata);
  delay(1000);
  Serial.print(senddata);
  delay(1000);
  Serial.print(senddata);
  delay(1000);
  Serial.print(senddata);
  delay(1000);
  Serial.print(senddata);
  delay(1000);
  Serial.print(senddata);
  delay(1000);
  Serial.print(senddata);
  delay(1000);
  Serial.print(senddata);
  delay(1000);
  Serial.print(senddata);
  delay(1000);
  Serial.print(senddata);
  delay(1000);
  Serial.print(senddata);
  delay(1000);
  Serial.print(senddata);
  delay(1000);
  Serial.print(senddata);
  delay(1000);
  senddata = (senddata + 1 - 49)%5 +49;
}
