
#include "temperature-ntc-module.h"

TemperatureNtcModule::TemperatureNtcModule(unsigned char sensorPin)
{
    adc.attach(sensorPin);
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

    int sampleCount = 5;

    for (;;)
    {
        float newTemperature = 0;
        for (int i = 0; i < sampleCount; i++)
        {
            newTemperature += myself->adc.readMiliVolts();
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        newTemperature /= (float)sampleCount;

        myself->_setTemperatureMv(round(newTemperature));

        vTaskDelay(100 / portTICK_PERIOD_MS);
        // https://learn.adafruit.com/thermistor/using-a-thermistor
        //Serial.println("temp r1: " + String(newTemperature) + " : " + String(10000.0 / (3300.0 / newTemperature)));
        //Serial.println("temp r2: " + String((10000.0 * newTemperature) / (3300.0 - newTemperature) ));
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


float TemperatureNtcModule::getTemperatureC()
{
    float r = getTemperatureResistance();
    float steinhart = log(r / ro);
    steinhart /= b25;
    steinhart += 1.0 / (25.0 + 273.15);
    steinhart = (1.0 / steinhart) - 273.15;
    return steinhart;
}