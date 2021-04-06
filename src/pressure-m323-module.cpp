
#include "pressure-m323-module.h"

PressureM323Module::PressureM323Module(unsigned char sensorPin, int acFreqHz = 60)
{
    adc.attach(sensorPin);
    acFreq = acFreqHz;
    rawPressureRange = maxRawPressure - minRawPressure;
}

PressureM323Module::~PressureM323Module()
{
    if (xHandle != NULL)
    {
        vTaskDelete(xHandle);
    }
}

/**
 * Reduce ADC noise with multisampling
 */
void PressureM323Module::watchPressureTask(void *instance)
{
    Serial.println("watchPressureTask started");

    PressureM323Module *myself = (PressureM323Module *)instance;

    // The pump stroke oscillates at the AC mains frequency. As it cycles
    // the pressure varies with the pump stroke. Here we take a full-cycle's
    // worth of samples to get the avg pressure for one full stroke.
    // e.g. 60hz = 1000ms/60 = 16.66ms

    int sampleCount = round(1000.0 / (myself->acFreq));
    //hold moving average of 4 * 1/60th = 2/60 = 1/30th; 3/60th = 1/20th; 6/60 = 1/10th
    const int readingCount = 6;
    int readings[readingCount] = {};
    int readingsIndex = 0;

    for (;;)
    {
        float oldPressure = 0;
        float newPressure = 0;

        oldPressure = newPressure;
        for (int i = 0; i < sampleCount; i++)
        {
            newPressure += myself->adc.readMiliVolts();
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        newPressure /= (float)sampleCount;
        // int smoothPressure = round((newPressure + oldPressure) / 2.0);
        // Serial.println("pressure" + String(newPressure));
        readings[readingsIndex] = newPressure;
        readingsIndex++;
        if(readingsIndex >= readingCount) {
            readingsIndex = 0;
        }
        float readingsAvg = 0;
        for (int i = 0; i < readingCount; i++) {
            readingsAvg += readings[i];
        }
        readingsAvg /= readingCount;

        myself->_setPressureMv(round(readingsAvg));
        // vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void PressureM323Module::begin()
{
    Serial.println("PressureM323Module Begin");

    xTaskCreate(
        &PressureM323Module::watchPressureTask, // Function that should be called
        "read pressure multisample",            // Name of the task (for debugging)
        10000,                                  // Stack size (bytes)
        this,                                   // Parameter to pass
        5,                                      // Task priority
        &xHandle                                // Task handle
    );
}

void PressureM323Module::_setPressureMv(int pressureMv)
{
    rawPressure = pressureMv;
}

int PressureM323Module::getPressureMv(void)
{
    return rawPressure;
}

float PressureM323Module::getPressurePercent()
{
    int pressureMv = getPressureMv();
    int normalizeRawPressure = pressureMv - minRawPressure;
    float pressurePercent = (float)((float)normalizeRawPressure / (float)rawPressureRange);
    return pressurePercent;
}

float PressureM323Module::getPressureBars()
{
    float pressurePerc = getPressurePercent();
    // Calculate the bar pressure, rounding two 3 decimals
    float barPressure = roundf(pressurePerc * 10.0 * 1000.0) / 1000.0;
    return barPressure;
}