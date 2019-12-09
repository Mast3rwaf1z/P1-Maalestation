int MQ7_setup(){
}
int MQ7_read(){
  delay(200);
  int value = analogRead(0);
  Serial.print("|| CO value = "); Serial.print(value); Serial.print("ppm");
}
