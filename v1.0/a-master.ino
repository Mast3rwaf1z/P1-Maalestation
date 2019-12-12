#include "Wire.h" // For I2C
#include "LCD.h" // For LCD
#include "LiquidCrystal_I2C.h" // Added library*
//Set the pins on the I2C chip used for LCD connections
//ADDR,EN,R/W,RS,D4,D5,D6,D7
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7); // 0x27 is the default I2C bus address
//RTC
//COx
#define COpin A0
float RS_gas = 0;
float ratio = 0;
float sensorValueCO = 0;
float sensorValueCO2 = 0;
float sensor_voltage = 0;
float R0 = 7200.0;
#define sensorIn A1
float ppm = 0;
float concentration = 0;
//DHT
#include <DHT.h>
const int DHTpin = 3;
DHT dht (DHTpin, DHT22);
float tempdeclutter;
float humiditydeclutter;
float Heatindexdeclutter;
float humidity = 0;
float temperature = 0;
float HeatIndex = 0;
//GP2Y10
int measurePin = 6;
int ledPower = 12;
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;
//RTC
#include <Wire.h>
#include "RTClib.h"
RTC_DS1307 RTC;

void setup() {
  Serial.begin(9600);
  lcd.begin(20, 4); // 16 x 2 LCD module
  lcd.setBacklightPin(3, POSITIVE); // BL, BL_POL
  lcd.setBacklight(HIGH);
  pinMode(sensorIn, INPUT);
  analogReference(DEFAULT);
  //GP2Y10
  pinMode(ledPower, OUTPUT);
  //DHT22
  dht.begin();
  //RTC
  Wire.begin();
  RTC.begin();

}

void printToLCD() {
  DateTime now = RTC.now();
  lcd.clear();
  //Print CO value
  lcd.setCursor(0, 0);
  lcd.print("CO: "); 
  lcd.print(ppm); 
  lcd.print("ppm");
  //Print CO2 value
  lcd.setCursor(0, 1);
  lcd.print("CO2: "); 
  lcd.print(concentration); 
  lcd.print("ppm");
  //Print PM value
  lcd.setCursor(0, 2);
  lcd.print("PM: "); 
  lcd.print(dustDensity); 
  lcd.print("ppm");
  //Print humidity percentage
  lcd.setCursor(0, 3);
  lcd.print("Humidity: "); 
  lcd.print(humidity); 
  lcd.print("%");
  delay(2000); //Wait 2 seconds to display second screen
  lcd.clear();
  //Print temperature °C 
  lcd.setCursor(0, 0);
  lcd.print("Temperature: "); 
  lcd.print(temperature); 
  lcd.print("°C");
  //Print heat index
  lcd.setCursor(0, 1);
  lcd.print("Heat index: "); 
  lcd.print(HeatIndex);
  //Print year, month, day
  lcd.setCursor(0, 2);
  lcd.print(now.day(), DEC); 
  lcd.print("/"); 
  lcd.print(now.month(), DEC); 
  lcd.print("/"); 
  lcd.print(now.year(), DEC);
  //Print hour, minute, second
  lcd.setCursor(0, 3);
  lcd.setCursor(now.hour(), DEC); 
  lcd.print(":"); 
  lcd.print(now.minute(), DEC); 
  lcd.print(":"); 
  lcd.print(now.second(), DEC);
  delay(2000); //Wait 2 seconds before looping around
  /*
    reading to display with name and unit:
    reading     || name               || unit
    CO          || ppm                || ppm
    CO2         || concentration      || ppm
    PM          || dustDensity        || ppm
    Humidity    || humidity           || %
    Temperature || temperature        || °C
    Heat Index  || HeatIndex          ||
    year        || now.year           || dd/mm/yyyy
    month       || now.month          ||
    date        || now.day            ||
    hour        || now.hour           || hh:mm:ss
    minute      || now.minute         ||
    second      || now.second         ||
  */

}

float CO() {
  sensorValueCO = analogRead(COpin);
  sensor_voltage = sensorValueCO / 1024 * 5.0;
  RS_gas = (5.0 - sensor_voltage) / sensor_voltage;
  ratio = RS_gas / R0; //Replace R0 with the value found using the sketch above
  float x = 1538.46 * ratio;
  ppm = pow(x, -1.709);
  return ppm;
}

float CO2() {
  int sensorValueCO2 = analogRead(sensorIn);
  float voltage = sensorValueCO2 * (5000 / 1024.0);
  if (voltage == 0) {
    concentration = 0;
    return concentration;
  }

  else if (voltage < 400) {
    concentration = 0;
    return concentration;
  }

  else {
    int voltage_difference = voltage - 400;
    concentration = voltage_difference * 50.0 / 16.0;
    return concentration;
  }
}

float thomas() {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature() - 2;
  HeatIndex = dht.computeHeatIndex(temperature, humidity, false);
  if (isnan(humidity) || isnan(temperature)) {
    return humidity = 0;
    return temperature = 0;
    return HeatIndex = 0;
  }
  else if (temperature != tempdeclutter || humidity != humidity || Heatindexdeclutter != HeatIndex) {
    return humidity; //%
    return temperature; //°C
    return HeatIndex;
    tempdeclutter = temperature, humiditydeclutter = humidity, Heatindexdeclutter = HeatIndex;
  }
}

float PM() {
  digitalWrite(ledPower, LOW); //power on the internal LED
  delayMicroseconds(samplingTime);
  voMeasured = analogRead(measurePin); //read PM level
  delayMicroseconds(deltaTime);
  digitalWrite(ledPower, HIGH);
  delayMicroseconds(sleepTime);

  //0 - 3.3V mapped to 0 - 1023 integer values
  // recover voltage
  calcVoltage = voMeasured * (5.0 / 1024);

  // calculate dust density
  dustDensity = 0.17 * calcVoltage - 0.1;
  return dustDensity;
}

void printToSerial() {
  DateTime now = RTC.now();
  /*
    reading to display with name and unit:
    reading     || name               || unit
    CO          || ppm                || ppm
    CO2         || concentration      || ppm
    PM          || dustDensity        || ppm
    Humidity    || humidity           || %
    Temperature || temperature        || °C
    Heat Index  || HeatIndex          ||
    year        || now.year           || dd/mm/yyyy
    month       || now.month          ||
    date        || now.day            ||
    hour        || now.hour           || hh:mm:ss
    minute      || now.minute         ||
    second      || now.second         ||
  */
  //print year, month and date
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.println(now.day(), DEC);
  //print hour, minute and seconds
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC); 
  Serial.print(":"); 
  Serial.println(now.second(), DEC);
  //Print CO value
  Serial.print("CO: "); 
  Serial.print(ppm); 
  Serial.println("ppm");
  //Print CO2 value
  Serial.print("CO2: "); 
  Serial.print(concentration); 
  Serial.println("ppm");
  //Print PM value
  Serial.print("Particulate matter: "); 
  Serial.print(dustDensity); 
  Serial.println("ppm");
  //Print humidity percentage
  Serial.print("Humidity: "); 
  Serial.print(humidity); 
  Serial.println("%");
  //Print temperature °C
  Serial.print("Temperature: "); 
  Serial.print(temperature); 
  Serial.println("°C");
  //Print heat index
  Serial.print("Heat Index: "); 
  Serial.println(HeatIndex);
  //Print spaces between measurements
  Serial.println("-----------------------");
  Serial.println("");
  //Wait 4 seconds
  delay(4000);
}

void loop() {
  CO();
  CO2();
  PM();
  thomas();
  //printToLCD();
  printToSerial();
}
