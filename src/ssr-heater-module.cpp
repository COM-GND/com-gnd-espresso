#include "ssr-heater-module.h"

SsrHeaterModule::SsrHeaterModule(unsigned char ssrControlPin, unsigned int cyclePeriodMs)
{
    xHandle = NULL;
    setControlPin(ssrControlPin);
    cyclePeriod = cyclePeriodMs;
    pinMode(ssrControlPin, OUTPUT);
    digitalWrite(ssrControlPin, LOW);
}

SsrHeaterModule::~SsrHeaterModule()
{
}

void SsrHeaterModule::begin()
{
    Serial.println("SSR Heater Module Begin");
    xTaskCreate(
        &SsrHeaterModule::ssrHeaterModuleTask,
        "ssrHeater",
        2000, /* bytes */
        this,
        10,
        &xHandle);
}

void SsrHeaterModule::setControlPin(unsigned char pin)
{
    ssrControlPin = pin;
}

unsigned char SsrHeaterModule::getControlPin()
{
    return ssrControlPin;
}

void SsrHeaterModule::setDutyCyclePercent(float percent) {
    if(percent < 0) {
        percent = 0;
    }
    if(percent > 1) {
        percent = 1;
    }

    unsigned int dutyTime = (unsigned int)(percent * (float)cyclePeriod);
    // Serial.println("setDutyCyclePercent: " + String(percent));

    setDutyCycleMs(dutyTime);
}

void SsrHeaterModule::setDutyCycleMs(unsigned int ms)
{
    // Serial.println("setDutyCycleMs: " + String(ms));
    dutyPeriod = ms;
}

unsigned int SsrHeaterModule::getDutyCycleMs()
{
    return dutyPeriod;
}

unsigned int SsrHeaterModule::getCyclePeriodMs()
{
    return cyclePeriod;
}

void SsrHeaterModule::setSsrHigh() {
    Serial.println("setSsrHigh");
    digitalWrite(ssrControlPin, HIGH);
}

void SsrHeaterModule::setSsrLow() {
    Serial.println("setSsrLow");
    digitalWrite(ssrControlPin, LOW);
}

void SsrHeaterModule::ssrHeaterModuleTask(void *pSelf)
{
    SsrHeaterModule *myself = (SsrHeaterModule *)pSelf;
    unsigned char controlPin = myself->getControlPin();
    unsigned int counter = 0;
    unsigned int periodMs = myself->getCyclePeriodMs();
    const unsigned int testFreqMs = 100;

    TickType_t xLastWakeTime;
    // run at 1hz
    //const TickType_t xFrequency = testFreqMs / portTICK_PERIOD_MS;
    unsigned int highwater = 0;

    for (;;)
    {
        unsigned int dutyCycleMs = myself->getDutyCycleMs();

        TickType_t onTime = dutyCycleMs > 0 ? (dutyCycleMs / portTICK_PERIOD_MS) : 0;
        TickType_t offTime = (periodMs - dutyCycleMs) / portTICK_PERIOD_MS;

        if(onTime > 0){
            Serial.println("on hw: "  + String(onTime) + " m: " + String(highwater));
            myself->setSsrHigh();
            vTaskDelayUntil(&xLastWakeTime, onTime);
        }
        if(offTime > 0){
            Serial.println("off hw: " + String(offTime) + " m: " + String(highwater));
            myself->setSsrLow();
            vTaskDelayUntil(&xLastWakeTime, offTime);
        }
        highwater = uxTaskGetStackHighWaterMark(NULL);
    }
}