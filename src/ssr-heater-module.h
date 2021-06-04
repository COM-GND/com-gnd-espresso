
#ifndef SSRHEATERMODULE_H
#define SSRHEATERMODULE_H
#include <Arduino.h>

class SsrHeaterModule
{
private:
    TaskHandle_t xHandle;
    unsigned char ssrControlPin;
    unsigned int cyclePeriod = 2000;
    unsigned int dutyPeriod = 1000;

public:
    SsrHeaterModule(unsigned char ssrControlPin, unsigned int);
    ~SsrHeaterModule();
    void begin();
    void setDutyCycleMs(unsigned int ms);
    unsigned int getDutyCycleMs();
    unsigned int getCyclePeriodMs();
    void setDutyCyclePercent(float);
    void setControlPin(unsigned char pin);
    unsigned char getControlPin();
    void setSsrHigh();
    void setSsrLow();
    static void ssrHeaterModuleTask(void *self);

};

#endif