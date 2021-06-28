#ifndef INFLOWSCALE_H
#define INFLOWSCALE_H
#include <Arduino.h>
#include "HX711.h"

class FlowHx711Module
{
private:
    uint8_t doutPin;
	uint8_t sckPin;
    float flowRateMgPerMs = 0;
    HX711 scale;
    TaskHandle_t xHandle;
    void _setFlowRateMgPerMs(float flowRate);

public:
    FlowHx711Module(uint8_t dout, uint8_t sck);
    ~FlowHx711Module();
    static void watchFlowTask(void *);
    void begin(void);
    void tare(void);
    long getWeight(void);
};

#endif