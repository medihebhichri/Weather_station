#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DHT.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>

const int hallEffectPin = A0;
const float sensorDistance = 0.02;
const float kmhConversionFactor = 3.6;

unsigned long lastPrintTime = 0;

#define DHTPIN 2
#define DHTTYPE DHT11
#define LIGHTPIN A1;

Adafruit_BME280 bme;
DHT dht(DHTPIN, DHTTYPE);
ThreeWire myWire(4, 5, 3);
RtcDS1302<ThreeWire> Rtc(myWire);

const unsigned long intervalBME280 = 5000;
const unsigned long intervalDHT11 = 1000;

unsigned long previousBME280Millis = 0;
unsigned long previousDHT11Millis = 0;

// Rotary Encoder Inputs
#define CLK 7
#define DT 6

int counter = 0;
int degrees = 0; // Variable to store accumulated degrees
int currentStateCLK;
int lastStateCLK;
int lastEncoderValue = 0;
int countsPer360Degrees = 20; // Adjust this based on your calibration

unsigned long previousLoopMillis = 0;
const unsigned long loopInterval = 100;  // Adjust the interval as needed

const float SEALEVELPRESSURE_HPA = 1013.25; // Adjust to your location's value

void setup() {
  Serial.begin(9600);

  Wire.begin();
  Rtc.Begin();

  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  dht.begin();

  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);

  lastStateCLK = digitalRead(CLK);
}


  dht.begin();

  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);

  lastStateCLK = digitalRead(CLK);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousLoopMillis >= loopInterval) {
    previousLoopMillis = currentMillis;

    RtcDateTime now = Rtc.GetDateTime();
    printDateTime(now);
    Serial.print(" | ");

    float temperatureBME = bme.readTemperature();
    float pressureBME = bme.readPressure() / 100.0;
    float humidityBME = bme.readHumidity();
    float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

    Serial.print("BME280 - Temp: ");
    if (!isnan(temperatureBME)) {
      Serial.print(temperatureBME);
    } else {
      Serial.print("N/A");
    }
    Serial.print(" | Pres: ");
    if (!isnan(pressureBME)) {
      Serial.print(pressureBME);
    } else {
      Serial.print("N/A");
    }
    Serial.print(" | Hum: ");
    if (!isnan(humidityBME)) {
      Serial.print(humidityBME);
    } else {
      Serial.print("N/A");
    }
    Serial.print(" | Alt: ");
    if (!isnan(altitude)) {
      Serial.print(altitude);
    } else {
      Serial.print("N/A");
    }
    Serial.println(" | ");

    float temperatureDHT = dht.readTemperature();
    float humidityDHT = dht.readHumidity();

    Serial.print("DHT11 - Temp: ");
    if (!isnan(temperatureDHT)) {
      Serial.print(temperatureDHT);
    } else {
      Serial.print("N/A");
    }
    Serial.print(" | Hum: ");
    if (!isnan(humidityDHT)) {
      Serial.print(humidityDHT);
    } else {
      Serial.print("N/A");
    }
    Serial.println(" | ");

    currentStateCLK = digitalRead(CLK);

    if (currentStateCLK != lastStateCLK && currentStateCLK == 1) {
      int encoderDTState = digitalRead(DT);
      if (encoderDTState != currentStateCLK) {
        counter--;
      } else {
        counter++;
      }

      if (counter < 0) {
        counter = countsPer360Degrees - 1;
      } else if (counter >= countsPer360Degrees) {
        counter = 0;
      }

      degrees = (counter * 360) / countsPer360Degrees;

      lastEncoderValue = degrees;

      // Print the angle
      Serial.print("Angle: ");
      Serial.println(degrees);
    } else {
      // Print the last known angle value
      Serial.print("Angle: ");
      Serial.println(lastEncoderValue);
    }

    lastStateCLK = currentStateCLK;

    int lightIntensity = analogRead(LIGHTPIN);
    Serial.print("Light - Intensity: ");
    Serial.println(lightIntensity);

    int RanValue = digitalRead(RanPin);
    if (RanValue == 1) {
      Serial.println("Ran value: True");
    } else {
      Serial.println("Ran value: False");
    }
  }
}

void printDateTime(const RtcDateTime& dt) {
  char datestring[20];
  snprintf_P(datestring, sizeof(datestring), PSTR("%02u:%02u:%02u"),
             dt.Hour(), dt.Minute(), dt.Second());
  Serial.print("RTC - ");
  Serial.print(datestring);
}
