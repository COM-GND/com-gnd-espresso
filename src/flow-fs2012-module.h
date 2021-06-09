#ifndef FLOWFS2012_H
#define FLOWFS2012_H
#include <Arduino.h>
#include <Wire.h>

#define FS2012_I2C_ADDR 0x07

class FlowFs2012Module
{
private:
    TaskHandle_t xHandle;
    void _setRawFlowRate(int);
    TwoWire *I2C;
    int rawFlowRate = 0;

public:
    FlowFs2012Module(TwoWire *twoWire);
    ~FlowFs2012Module();
    static void watchFlowTask(void *);
    void begin(void);
    int getRawFlowRate(void);
    int readSensor(void);
    float getFlowRateMlPerMin(void);
    TwoWire * getI2cInstance(void);
};

#endif