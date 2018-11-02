#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RtcDS3231.h>
#include <Adafruit_ADS1015.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define ONE_WIRE_BUS 10
#define countof(a) (sizeof(a) / sizeof(a[0]))
#define BAUD_RATE 9600

Adafruit_BME280 bme;
BH1750 lightMeter;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;
RtcDS3231<TwoWire> Rtc(Wire);
Adafruit_ADS1115 ads;
SoftwareSerial radio(13, 12, false, 256);

const int analogPin = A0;

void setup() {
    pinMode(analogPin, OUTPUT);

    Serial.begin(BAUD_RATE);
    radio.begin(BAUD_RATE);

    initSensors();
}

void loop() {
    sendValues();
    delay(500);
}

void printDateTime()
{
    RtcDateTime dt = Rtc.GetDateTime();

    char datestring[20];

    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%04u-%02u-%02u %02u:%02u:%02u"),
            dt.Year(),
            dt.Month(),
            dt.Day(),            
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    Serial.println(datestring);
}

void initSensors() {
    Rtc.Begin();

    if (!Rtc.IsDateTimeValid()) 
    {
        Serial.println("RTC lost confidence in the DateTime!");
    }

    if (!Rtc.GetIsRunning())
    {
        Rtc.SetIsRunning(true);
    }

    Rtc.Enable32kHzPin(false);
    Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone); 

    ads.setGain(GAIN_ONE);
    ads.begin();    

    // locate devices on the onowire bus
    Serial.print("Locating devices...");
    sensors.begin();
    Serial.print("Found ");
    Serial.print(sensors.getDeviceCount(), DEC);
    Serial.println(" devices.");

    if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 

    sensors.setResolution(insideThermometer, 9);

    bool status = bme.begin();
    
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
    }

    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,
                    Adafruit_BME280::SAMPLING_X16,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5);

    lightMeter.begin(BH1750_CONTINUOUS_LOW_RES_MODE);

    digitalWrite(analogPin, HIGH);
}

void sendValues() {
    printDateTime();

    float temp = bme.readTemperature();

    float pressure = bme.readPressure() / 100.0F;

    float humidity = bme.readHumidity();

    uint16_t lux = lightMeter.readLightLevel();
    delay(16);

    sensors.requestTemperatures();
    float tempWater = sensors.getTempC(insideThermometer);

    int16_t rain = ads.readADC_SingleEnded(0);
    int16_t groundHum = ads.readADC_SingleEnded(1);

    Serial.print("Temperature = ");
    Serial.print(temp);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.println(" hPa");

    Serial.print("Humidity = ");
    Serial.print(humidity);
    Serial.println(" %");

    Serial.print("Light: ");
    Serial.print(lux);
    Serial.println(" lx");

    Serial.print("Wather temperature = ");
    Serial.print(tempWater);
    Serial.println(" *C");

    Serial.print("Rain = ");
    Serial.println(rain);

    Serial.print("Ground humidity = ");
    Serial.println(groundHum);
    Serial.println("");

    const size_t bufferSize = JSON_OBJECT_SIZE(7);
    DynamicJsonBuffer jsonBuffer(bufferSize);

    JsonObject& root = jsonBuffer.createObject();
    root["time"] = Rtc.GetDateTime().Epoch32Time();
    root["temp"] = temp;
    root["pressure"] = pressure;
    root["humidity"] = humidity;
    root["light"] = lux;
    root["watherTemp"] = tempWater;
    root["rain"] = rain;
    root["groundHum"] = groundHum;

    String json;

    root.printTo(json);

    radio.println(json);
}