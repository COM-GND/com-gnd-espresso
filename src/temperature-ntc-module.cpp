
#include "temperature-ntc-module.h"

TemperatureNtcModule::TemperatureNtcModule(unsigned char sensorPin, int acFreqHz = 60)
{
    adc.attach(sensorPin);
    acFreq = acFreqHz;
    rawTemperatureRange = maxRawTemperature - minRawTemperature;
}

TemperatureNtcModule::~TemperatureNtcModule()
{
    if (xHandle != NULL)
    {
        vTaskDelete(xHandle);
    }
}

/**
 * Reduce ADC noise with multisampling
 */
void TemperatureNtcModule::watchTemperatureTask(void *instance)
{
    Serial.println("watchTemperatureTask started");

    TemperatureNtcModule *myself = (TemperatureNtcModule *)instance;

    // The pump stroke oscillates at the AC mains frequency. As it cycles
    // the temperature varies with the pump stroke. Here we take a full-cycle's
    // worth of samples to get the avg temperature for one full stroke.
    // e.g. 60hz = 1000ms/60 = 16.66ms

    int sampleCount = 5;
    //hold moving average of 4 * 1/60th = 2/60 = 1/30th; 3/60th = 1/20th; 6/60 = 1/10th
    const int readingCount = 6;
    int readings[readingCount] = {};
    int readingsIndex = 0;

    for (;;)
    {
        float oldTemperature = 0;
        float newTemperature = 0;

        oldTemperature = newTemperature;
        for (int i = 0; i < sampleCount; i++)
        {
            newTemperature += myself->adc.readMiliVolts();
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        newTemperature /= (float)sampleCount;

        readings[readingsIndex] = newTemperature;
        readingsIndex++;
        if(readingsIndex >= readingCount) {
            readingsIndex = 0;
        }
        float readingsAvg = 0;
        for (int i = 0; i < readingCount; i++) {
            readingsAvg += readings[i];
        }
        readingsAvg /= readingCount;

        myself->_setTemperatureMv(round(readingsAvg));
        vTaskDelay(100 / portTICK_PERIOD_MS);
        // https://learn.adafruit.com/thermistor/using-a-thermistor
        Serial.println("temp r1: " + String(readingsAvg) + " : " + String(10000.0 / (3300.0 / readingsAvg)));
        Serial.println("temp r2: " + String((10000.0 * readingsAvg) / (3300.0 - readingsAvg) ));
    }

    vTaskDelete(NULL);
}

void TemperatureNtcModule::begin()
{
    Serial.println("TemperatureNtcModule Begin");

    xTaskCreate(
        &TemperatureNtcModule::watchTemperatureTask, // Function that should be called
        "read temperature multisample",            // Name of the task (for debugging)
        8000,                                  // Stack size (bytes)
        this,                                   // Parameter to pass
        3,                                      // Task priority
        &xHandle                                // Task handle
    );
}

void TemperatureNtcModule::_setTemperatureMv(int temperatureMv)
{
    rawTemperature = temperatureMv;
}

int TemperatureNtcModule::getTemperatureMv(void)
{
    return rawTemperature;
}

float TemperatureNtcModule::getTemperatureResistance(void)
{
    float ohms = (10000.0 * rawTemperature) / (3300.0 - rawTemperature);
    return ohms;
}

float TemperatureNtcModule::getTemperaturePercent()
{
    int temperatureMv = getTemperatureMv();
    int normalizeRawTemperature = temperatureMv - minRawTemperature;
    float temperaturePercent = (float)((float)normalizeRawTemperature / (float)rawTemperatureRange);
    return temperaturePercent;
}

float TemperatureNtcModule::getTemperatureC()
{
    float temperatureMv = getTemperatureMv();
    float r = 10000 / temperatureMv;
    // Calculate the bar temperature, rounding two 3 decimals
    //float barTemperature = roundf(temperaturePerc * 10.0 * 1000.0) / 1000.0;
    return r;
}