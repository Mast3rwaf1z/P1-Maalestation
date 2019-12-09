/*
 * pin 1 på modulet er forbundet til Vcc 5V
 * pin 2 på modulet er forbundet til pin 2 på arduinoen
 * pin 3 på modulet er forbundet til GND
 * Den følgende kode er hentet fra library'et DHT.h's README fil og bliver derfor krediteret til https://github.com/adafruit/DHT-sensor-library
 * note: luftfugtigheden og temperaturen er testet og virker som det skal.
 */
#include <DHT.h>

#define DHTPIN 24
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
void setup() {
  Serial.begin(9600);
  Serial.println(F("DHT22 test!"));
  dht.begin();
}

void loop() {
  delay(2000);

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);

  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  float hif = dht.computeHeatIndex(f, h);
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
  Serial.print(F("°F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
  Serial.print(hif);
  Serial.println(F("°F"));
}
