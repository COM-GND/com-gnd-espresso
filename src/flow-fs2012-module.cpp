#include "flow-fs2012-module.h"


FlowFs2012Module::FlowFs2012Module(TwoWire *twoWire)
{
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

    Serial.println("watchFlowTask started");

    FlowFs2012Module *myself = (FlowFs2012Module *)instance;

    for (;;)
    {

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void FlowFs2012Module::begin()
{

}

int FlowFs2012Module::getFlowRate(void)
{
}