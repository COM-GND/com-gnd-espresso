#include "flow-fs2012-module.h"


FlowFs2012Module::FlowFs2012Module(TwoWire *twoWire)
{
    xHandle = NULL;
    I2C = twoWire;
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
    TwoWire *I2C = myself->getI2cInstance(); 

    unsigned int highwater = 0;

    for (;;)
    {
        I2C->beginTransmission(FS2012_I2C_ADDR);
        // send a bit asking for register one (as specified by the pdf)
        I2C->write(1); 
        I2C->endTransmission();

        vTaskDelay(20 / portTICK_PERIOD_MS);

        I2C->requestFrom(FS2012_I2C_ADDR, 2);
        
        while(Wire.available() == 0){
            vTaskDelay(1 / portTICK_PERIOD_MS);
        };
        byte myMSB = I2C->read(); 
        byte LSB = I2C->read(); 

        int sensorValue = (( myMSB << 8 ) + LSB);
        myself->_setRawFlowRate(sensorValue);

        highwater = uxTaskGetStackHighWaterMark(NULL);

        vTaskDelay(80 / portTICK_PERIOD_MS);

    }

    vTaskDelete(NULL);
}

void FlowFs2012Module::begin()
{
     Serial.println("FlowFs2012Module Begin");

    xTaskCreate(
        &FlowFs2012Module::watchFlowTask, // Function that should be called
        "read_flow_rate",                       // Name of the task (for debugging)
        2000,                                  // Stack size (bytes)
        this,                                   // Parameter to pass
        3,                                      // Task priority
        &xHandle                                // Task handle
    );
}

float FlowFs2012Module::getFlowRateMlPerMin(void)
{
    float mlPerMin = (float)rawFlowRate / 10.0;
    return mlPerMin;

}

TwoWire * FlowFs2012Module::getI2cInstance(void) {
    return I2C;
}

void FlowFs2012Module::_setRawFlowRate(int newRawFlowRate) {
    rawFlowRate = newRawFlowRate;
}

int FlowFs2012Module::getRawFlowRate(void) {
    return rawFlowRate;
}