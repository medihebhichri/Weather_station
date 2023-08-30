#include <PinChangeInterrupt.h> // Include the PinChangeInterrupt library
#include <DHT.h>
/* bme */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25) // Change this to your local sea level pressure
Adafruit_BME280 bme; // I2C

/* bme def */
struct SensorData {
  float temperature;
  float humidity;
  float pressure;
  float altitude;
};



#define DHTPIN 2      // Pin where the DHT11 sensor is connected
#define DHTTYPE DHT11 // DHT11 sensor type

DHT dht(DHTPIN, DHTTYPE);

#define LightPin A1
#define RanPin A2


#define CLK 6
#define DT 7
#define SW 8
#define WindPin A0

int angle = 0;
int counter = 0;
int currentState;
int initState;
float lastvalue=0;
unsigned long debounceDelay = 0;

bool magnetDetected = false;
unsigned long startTime = 0;
unsigned long endTime = 0;

enum State {
  IDLE,
  DETECTING,
  TIMING
};

State currentStateWind = IDLE;
const int numSamples = 50;
unsigned long samples[numSamples];
int sampleIndex = 0;

void setup() {
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(SW, INPUT_PULLUP);
  pinMode(WindPin, INPUT);

  pinMode(LightPin,INPUT);
  pinMode(RanPin,INPUT);

  Serial.begin(9600);
  dht.begin();
  initState = digitalRead(CLK);

  attachInterrupt(0, encoder_value, CHANGE);
  attachInterrupt(1, encoder_value, CHANGE);
  attachPCINT(digitalPinToPCINT(SW), button_press, CHANGE);

  /* setup */
   if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}


// dht11

float readHumidity() {
  float humidity = dht.readHumidity();
  return humidity;
}

float readTemperature() {
  float temperature = dht.readTemperature();
  return temperature;
}

// ###############################################



SensorData readSensorData() {
  SensorData data;

  data.temperature = bme.readTemperature();
  data.humidity = bme.readHumidity();
  data.pressure = bme.readPressure() / 100.0F; // Convert Pa to hPa
  data.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  return data;
}


void loop() {
  windSpeedCalculator();
  SensorData sensorData = readSensorData();
  int currentAngle = encoder_value();
  
  // Do something with the currentAngle value
  // print bme values
  float humidityValue = readHumidity();
  float temperatureValue = readTemperature();
  float light=analogRead(LightPin);
  int ran =digitalRead(RanPin);
  light=(light*100)/1024;

  if(ran>0){
  Serial.print("0 /");
  }
  
  Serial.print(sensorData.temperature);
  Serial.print(" /");

  Serial.print(temperatureValue);
  Serial.print(" /");

  Serial.print((humidityValue*1)/100.0);
  Serial.print(" /");

  Serial.print(String(lastvalue)+String(" /"));

  Serial.print(String(angle) + " /" );

  Serial.print("16.00 /");

  Serial.print((light*10)/1024);
  Serial.print(" /");

  Serial.println(sensorData.pressure);
 

  

 

 
 

 
 
 


  Serial.println();
  
}

void button_press() {
  int buttonVal = digitalRead(SW);
  if (buttonVal == LOW) {
    if (millis() - debounceDelay > 200) {
      Serial.println("Button pressed!");
    }
    debounceDelay = millis();
  }
}

int encoder_value() {
  currentState = digitalRead(CLK);
  if (currentState != initState && currentState == 1) {
    if (digitalRead(DT) != currentState) {
      counter++;
    } else {
      counter--;
    }

    if (counter < 0) {
      counter = 19;
    } else if (counter > 19) {
      counter = 0;
    }

    angle = counter * (360 / 20);
    
  }
  initState = currentState;
  return angle;
}






void windSpeedCalculator() {
  int windPinValue = analogRead(WindPin);

  switch (currentStateWind) {
    case IDLE:
      if (windPinValue < 5) {
        currentStateWind = DETECTING;
      }
      break;

    case DETECTING:
      if (windPinValue > 5) {
        startTime = millis();
        currentStateWind = TIMING;
      }
      break;

    case TIMING:
      if (windPinValue < 5) {
        endTime = millis();
        unsigned long elapsedTime = endTime - startTime;

        samples[sampleIndex] = elapsedTime;
        sampleIndex++;

        if (sampleIndex >= numSamples) {
          unsigned long targetTime = samples[numSamples / 2];
          unsigned long closestDiff = abs(samples[0] - targetTime);
          unsigned long sumClosest = samples[0];
          for (int i = 1; i < numSamples; i++) {
            unsigned long diff = abs(samples[i] - targetTime);
            if (diff < closestDiff) {
              closestDiff = diff;
              sumClosest = samples[i];
            }
          }
          float windSpeedMPerS = (2.0 * 3.14159265358979323846 * 0.14) / ((float)sumClosest / numSamples) ;
          float windSpeedKmPerH = windSpeedMPerS * 3.6;

          lastvalue=windSpeedKmPerH;
      
         

          sampleIndex = 0;
          currentStateWind = IDLE;
        }
      }
      break;
  }
  delay(1);
}
