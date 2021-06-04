#ifndef FLOWFS2012_H
#define FLOWFS2012_H
#include <Arduino.h>
#include <Wire.h>

class FlowFs2012Module
{
private:
    TaskHandle_t xHandle;
    void _setRawFlowRate(int);
    TwoWire *I2C;

public:
    const unsigned char fs2012I2cAddress = 0x07;
    FlowFs2012Module(TwoWire *twoWire);
    static void watchFlowTask(void *);
    void begin(void);
    int getFlowRate(void);
};

#endif