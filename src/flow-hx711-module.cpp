#include "flow-hx711-module.h"

FlowHx711Module::FlowHx711Module(uint8_t dout, uint8_t sck)
{
    doutPin = dout;
    sckPin = sck;
}

FlowHx711Module::~FlowHx711Module()
{
    if (xHandle != NULL)
    {
        vTaskDelete(xHandle);
    }
}

void FlowHx711Module::begin()
{
    scale.begin(doutPin, sckPin);
    scale.tare();
    Serial.println("FlowHx711Module Begin");

    return;

    xTaskCreate(
        &FlowHx711Module::watchFlowTask,  // Function that should be called
        "read_hx711_flow_rate",           // Name of the task (for debugging)
        2000,                             // Stack size (bytes)
        this,                             // Parameter to pass
        3,                                // Task priority
        &xHandle                          // Task handle
    );
}

void FlowHx711Module::watchFlowTask(void *instance)
{
    // Credits for deciphering sensor I2C api:
    // https://github.com/OPEnSLab-OSU/eDNA/blob/master/Hardware/Electrical%20Components/Sensor%20-%20Flow/flowsensor.ino

    Serial.println("watchFlowTask started");

    FlowHx711Module *myself = (FlowHx711Module *)instance;

    HX711 scale = myself->scale;

    unsigned int highwater = 0;

    long lastWeight = 0;
    TickType_t lastReadingEndTime = 0;
    TickType_t xLastWakeTime;

    for (;;)
    {

        if (scale.is_ready())
        {

            TickType_t newReadingStartTime = xTaskGetTickCount();
            long weight = myself->getWeight();
            TickType_t newReadingEndTime = xTaskGetTickCount();
            
            TickType_t readingDuration = newReadingEndTime - newReadingStartTime;
            TickType_t ticksSinceLastReading = newReadingEndTime - lastReadingEndTime;
            lastReadingEndTime = newReadingEndTime;
            long weightChange = weight - lastWeight;
            // get flow rate in MS
            long flowRate = weightChange / (ticksSinceLastReading / portTICK_PERIOD_MS);
            myself->_setFlowRateMgPerMs(flowRate);

            vTaskDelayUntil(&xLastWakeTime, ((100 - readingDuration) / portTICK_PERIOD_MS));
        };
    }

    vTaskDelete(NULL);
}

void FlowHx711Module::tare()
{
    scale.tare();
}

void FlowHx711Module::_setFlowRateMgPerMs(float flowRate)
{
    flowRateMgPerMs = flowRate;
}

long FlowHx711Module::getWeight()
{
    if (scale.is_ready())
    {
        long reading = scale.get_value(2);
        return reading;
    } else {
        return -1;
    }
}