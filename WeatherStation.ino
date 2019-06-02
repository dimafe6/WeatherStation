#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RtcDS3231.h>
#include <Adafruit_ADS1015.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define ONE_WIRE_BUS 10
#define BAUD_RATE 9600

Adafruit_BME280 bme;
BH1750 lightMeter;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;
RtcDS3231<TwoWire> Rtc(Wire);
Adafruit_ADS1115 ads;
long prevSendDataMillis = millis();
const int analogPin = A0;

void setup()
{
    pinMode(analogPin, OUTPUT);

    Serial.begin(BAUD_RATE);
    Serial.set_tx(2);
    //Serial.println("AT+P6");

    initSensors();
}

void loop()
{
    if (millis() - prevSendDataMillis > 2500)
    {
        prevSendDataMillis = millis();        
        sendValues();
    }
}

void initSensors()
{
    Rtc.Begin();

    if (!Rtc.GetIsRunning())
    {
        Rtc.SetIsRunning(true);
    }

    Rtc.Enable32kHzPin(false);
    Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone);

    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);

    if (!Rtc.IsDateTimeValid())
    {
        Rtc.SetDateTime(compiled);
    }

    if (!Rtc.GetIsRunning())
    {
        Rtc.SetIsRunning(true);
    }

    RtcDateTime now = Rtc.GetDateTime();
    if (now < compiled)
    {
        Rtc.SetDateTime(compiled);
    }

    ads.setGain(GAIN_ONE);
    ads.begin();
    sensors.begin();
    sensors.setResolution(insideThermometer, 9);

    bool status = bme.begin();

    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,
                    Adafruit_BME280::SAMPLING_X16,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5);

    lightMeter.begin(BH1750_CONTINUOUS_LOW_RES_MODE);

    digitalWrite(analogPin, HIGH);
}

void sendValues()
{
    int temp = bme.readTemperature();
    int pressure = bme.readPressure() / 100.0F;
    int humidity = bme.readHumidity();
    uint16_t lux = lightMeter.readLightLevel();
    delay(16);
    sensors.requestTemperatures();
    int tempWater = sensors.getTempC(insideThermometer);
    int16_t rain = ads.readADC_SingleEnded(0);
    int16_t groundHum = ads.readADC_SingleEnded(1);
    char json[200];
    sprintf(json, "{\"t\":%d,\"p\":%d,\"h\":%d,\"l\":%d,\"wt\":%d,\"r\":%d,\"gh\":%d}", temp, pressure, humidity, lux, tempWater, rain, groundHum);
    Serial.println(json);
}