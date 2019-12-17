//Set the pins on the I2C chip used for LCD connections
//ADDR,EN,R/W,RS,D4,D5,D6,D7
//RTC
//COx
const int COpin = A0;
float RS_gas = 0;
float ratio = 0;
float sensorValueCO = 0;
float sensorValueCO2 = 0;
float sensor_voltage = 0;
float R0 = 7200.0;
const int sensorIn = A1;
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
const int measurePin = 2;
int ledPower = 2;
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;
//RTC
#include <Wire.h>
#include <RTClib.h>
RTC_DS1307 RTC; //DS1307 Class

void setup() {
  Serial.begin(9600);
  pinMode(sensorIn, INPUT);
  analogReference(DEFAULT);
  pinMode(ledPower, OUTPUT);  // initialize GP2Y10
  dht.begin();  // initialize DHT22
  Wire.begin();
  RTC.begin(); // initialize RTC
  RTC.adjust(DateTime(__DATE__,__TIME__)); // Sets time to now from computer's time index
  //Only needed once, then it keeps track of time itself 
  }


float CO() {
  sensorValueCO = analogRead(COpin);
  sensor_voltage = sensorValueCO / 1024 * 5.0;
  RS_gas = (5.0 - sensor_voltage) / sensor_voltage;
  ratio = RS_gas / R0; //R0 = 7200
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

float humidityF() {
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
  DateTime now = RTC.now(); //Print the Date
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
  Serial.print(now.year(), DEC); // Print the year
  Serial.print("/");
  Serial.print(now.month(), DEC); // Print the month
  Serial.print("/");
  Serial.println(now.day(), DEC); // Print the date of the month
  //print hour, minute and seconds
  Serial.print(now.hour(), DEC); // Print the hour
  Serial.print(":");
  Serial.print(now.minute(), DEC); // Print the minute
  Serial.print(":"); 
  Serial.println(now.second(), DEC); // Print the second
  //Print CO value
  //Serial.print("CO: "); 
  //Serial.print(ppm); 
  //Serial.println("ppm");
  //Print CO2 value
  Serial.print("CO2: "); 
  Serial.print(concentration); 
  Serial.println("ppm");
  //Print PM value
  Serial.print("Particulate matter: "); 
  Serial.print(dustDensity); 
  Serial.println("mg/m3");
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
  //CO();
  CO2();
  PM();
  humidityF();
  printToSerial();
  delay(1000);
}
