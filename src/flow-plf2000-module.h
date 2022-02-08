#ifndef FLOWPLF2000_H
#define FLOWPLF2000_H
#include <Arduino.h>
#include <Wire.h>

#define PLF2000_I2C_ADDR 0x50

class FlowPlf2000Module
{
private:
    TaskHandle_t xHandle;
    void _setRawFlowRate(int);
    TwoWire *I2C;
    int rawFlowRate = 0;

public:
    typedef struct
    {
        float flow;
        float v;
        int count;

    } FlowRateData;

    FlowRateData flowRateTable[10] = {
        {0.0, 0.50, 409},
        {85.0, 1.71, 1362},
        {100.0, 1.78, 1403},
        {150.0, 1.96, 1572},
        {202.0, 2.20, 1761},
        {303.0, 2.60, 2103},
        {402.0, 2.92, 2353},
        {503.0, 3.15, 2535},
        {602.0, 3.27, 2650},
        {702.0, 3.35, 2715}};

    FlowPlf2000Module(TwoWire *twoWire);
    ~FlowPlf2000Module();
    static void watchFlowTask(void *);
    void begin(void);
    int getRawFlowRate(void);
    int readSensor(void);
    float getFlowRateMlPerMin(void);
    TwoWire *getI2cInstance(void);
};

#endif