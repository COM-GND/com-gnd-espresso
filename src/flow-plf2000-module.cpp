#include "flow-plf2000-module.h"

/*
Pins
1: Vdd
2: Gnd
3: Out
4: SDA
5: SCL 
6: N/C
*/
FlowPlf2000Module::FlowPlf2000Module(TwoWire *twoWire)
{
    xHandle = NULL;
    I2C = twoWire;

    I2C->beginTransmission(PLF2000_I2C_ADDR);
    byte error = I2C->endTransmission();
    Serial.println("i2C test: " + String(error));
}

FlowPlf2000Module::~FlowPlf2000Module()
{
    if (xHandle != NULL)
    {
        vTaskDelete(xHandle);
    }
}

void FlowPlf2000Module::watchFlowTask(void *instance)
{
    // Credits for deciphering sensor I2C api:
    // https://github.com/OPEnSLab-OSU/eDNA/blob/master/Hardware/Electrical%20Components/Sensor%20-%20Flow/flowsensor.ino

    Serial.println("watchFlowTask started");

    FlowPlf2000Module *myself = (FlowPlf2000Module *)instance;

    TwoWire *I2C = myself->I2C;

    unsigned int highwater = 0;

    for (;;)
    {

        I2C->beginTransmission(PLF2000_I2C_ADDR);

        vTaskDelay(20 / portTICK_PERIOD_MS);

        I2C->requestFrom(PLF2000_I2C_ADDR, 5);

        vTaskDelay(1 / portTICK_PERIOD_MS);

        if (I2C->available())
        {

            uint8_t checksum = I2C->read();
            uint8_t myMSB = I2C->read();
            uint8_t myLSB = I2C->read();
            // clear the two remaining "reserved" bytes.
            uint8_t byte3 = I2C->read();
            uint8_t byte4 = I2C->read();

            // uint16_t sum = myMSB + myLSB + byte3 + byte4;
            // unit8_t =

            int sensorValue = ((myMSB << 8) + myLSB);
            myself->_setRawFlowRate(sensorValue);

            highwater = uxTaskGetStackHighWaterMark(NULL);

            vTaskDelay(50 / portTICK_PERIOD_MS);
        };
    }

    vTaskDelete(NULL);
}

void FlowPlf2000Module::begin()
{
    Serial.println("FlowPlf2000Module Begin");

    xTaskCreate(
        &FlowPlf2000Module::watchFlowTask, // Function that should be called
        "read_flow_rate",                  // Name of the task (for debugging)
        2000,                              // Stack size (bytes)
        this,                              // Parameter to pass
        3,                                 // Task priority
        &xHandle                           // Task handle
    );
}

int FlowPlf2000Module::readSensor(void)
{
    I2C->beginTransmission(PLF2000_I2C_ADDR);
    // send a bit asking for register one (as specified by the pdf)
    I2C->write(1);
    byte sendResult = I2C->endTransmission();

    if (sendResult != 0)
    {
        Serial.println("Error reading I2C: " + String(sendResult));
    }

    delay(20);

    I2C->requestFrom(PLF2000_I2C_ADDR, 2);

    unsigned int sensorValue = 0;

    while (I2C->available() == 0)
    {
        delay(1);
    };

    if (2 <= I2C->available())
    { // if two bytes were received

        uint8_t myMSB = I2C->read(); // receive high byte (overwrites previous reading)

        uint8_t myLSB = I2C->read(); // receive low byte as lower 8 bits

        sensorValue = myMSB;
        sensorValue = sensorValue << 8;
        sensorValue = sensorValue + myLSB;

        Serial.print(myMSB, BIN);
        Serial.print(" ");
        Serial.print(myLSB, BIN);
        Serial.println();
        Serial.println(sensorValue, BIN);
        Serial.println(sensorValue); // print the reading
    }

    // _setRawFlowRate(sensorValue);

    delay(20);
}

float FlowPlf2000Module::getFlowRateMlPerMin(void)
{
    // Use sensor data lookup-table to interpolate the flow rate.
    FlowRateData lowerDataPoint = {0.0, 0.50, 409};
    FlowRateData upperDataPoint = {702.0, 3.35, 2715};
    int flowCount = rawFlowRate;
    if (flowCount < 409)
    {
        flowCount = 409;
    }
    else if (flowCount > 2715)
    {
        flowCount = 2715;
    }

    for (int i = 0; i < 9; i++)
    {
        if (flowCount >= flowRateTable[i].count && flowCount <= flowRateTable[i + 1].count)
        {
            lowerDataPoint = flowRateTable[i];
            upperDataPoint = flowRateTable[i + 1];
            break;
        }
    }

    float flowRate = map(
        flowCount,
        lowerDataPoint.count,
        upperDataPoint.count,
        lowerDataPoint.flow,
        upperDataPoint.flow);

    return flowRate;
}

TwoWire *FlowPlf2000Module::getI2cInstance(void)
{
    return I2C;
}

void FlowPlf2000Module::_setRawFlowRate(int newRawFlowRate)
{
    //Serial.println("_setRawFlowRate: " + String(newRawFlowRate));

    // sensor can sometimes return invalid value (e.g 0xFFFF)
    if (newRawFlowRate <= 5000)
    {
        rawFlowRate = newRawFlowRate;
    }
}

int FlowPlf2000Module::getRawFlowRate(void)
{
    return rawFlowRate;
}