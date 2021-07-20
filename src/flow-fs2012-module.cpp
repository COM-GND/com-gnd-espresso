#include "flow-fs2012-module.h"

FlowFs2012Module::FlowFs2012Module(TwoWire *twoWire)
{
    xHandle = NULL;
    I2C = twoWire;

    I2C->beginTransmission(FS2012_I2C_ADDR);
    byte error = I2C->endTransmission();
    Serial.println("i2C test: " + String(error));
}

FlowFs2012Module::~FlowFs2012Module()
{
    if (xHandle != NULL)
    {
        vTaskDelete(xHandle);
    }
}

void FlowFs2012Module::watchFlowTask(void *instance)
{
    // Credits for deciphering sensor I2C api:
    // https://github.com/OPEnSLab-OSU/eDNA/blob/master/Hardware/Electrical%20Components/Sensor%20-%20Flow/flowsensor.ino

    Serial.println("watchFlowTask started");

    FlowFs2012Module *myself = (FlowFs2012Module *)instance;

    TwoWire *I2C = myself->I2C;

    unsigned int highwater = 0;

    // Note that the sensor only update it's value every ~.7s
    // so no need to have fast sensor update rate here.
    for (;;)
    {

        I2C->beginTransmission(FS2012_I2C_ADDR);
        // send a bit asking for register one (as specified by the pdf)
        I2C->write(1);
        I2C->endTransmission();

        vTaskDelay(20 / portTICK_PERIOD_MS);

        I2C->requestFrom(FS2012_I2C_ADDR, 2);

        vTaskDelay(1 / portTICK_PERIOD_MS);

        if (I2C->available())
        {

            uint8_t myMSB = I2C->read();
            uint8_t myLSB = I2C->read();

            int sensorValue = ((myMSB << 8) + myLSB);
            myself->_setRawFlowRate(sensorValue);

            highwater = uxTaskGetStackHighWaterMark(NULL);

            vTaskDelay(80 / portTICK_PERIOD_MS);
        };
    }

    vTaskDelete(NULL);
}

void FlowFs2012Module::begin()
{
    Serial.println("FlowFs2012Module Begin");

    xTaskCreate(
        &FlowFs2012Module::watchFlowTask, // Function that should be called
        "read_flow_rate",                 // Name of the task (for debugging)
        2000,                             // Stack size (bytes)
        this,                             // Parameter to pass
        3,                                // Task priority
        &xHandle                          // Task handle
    );
}

int FlowFs2012Module::readSensor(void)
{
    I2C->beginTransmission(FS2012_I2C_ADDR);
    // send a bit asking for register one (as specified by the pdf)
    I2C->write(1);
    byte sendResult = I2C->endTransmission();

    if (sendResult != 0)
    {
        Serial.println("Error reading I2C: " + String(sendResult));
    }

    delay(20);

    I2C->requestFrom(FS2012_I2C_ADDR, 2);

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

float FlowFs2012Module::getFlowRateMlPerMin(void)
{
    if (rawFlowRate > 0)
    {
        float mlPerMin = (float)rawFlowRate / 10.0;
        return mlPerMin;
    }

    return 0.0;
}

TwoWire *FlowFs2012Module::getI2cInstance(void)
{
    return I2C;
}

void FlowFs2012Module::_setRawFlowRate(int newRawFlowRate)
{
    Serial.println("_setRawFlowRate: " + String(newRawFlowRate));

    // sensor can sometimes return invalid value (e.g 0xFFFF)
    if(newRawFlowRate <= 5000) {
        rawFlowRate = newRawFlowRate;
    }
   
}

int FlowFs2012Module::getRawFlowRate(void)
{
    return rawFlowRate;
}