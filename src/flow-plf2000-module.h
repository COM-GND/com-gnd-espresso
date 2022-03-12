#ifndef FLOWPLF2000_H
#define FLOWPLF2000_H
#include <Arduino.h>
#include <Wire.h>
#include <Smoothed.h>

#define PLF2000_I2C_ADDR 0x50
#define FLOW_RATE_TABLE_SIZE 11
class FlowPlf2000Module
{
private:
    TaskHandle_t xHandle;
    void _setRawFlowRate(uint16_t);
    TwoWire *I2C;
    uint16_t rawFlowRate = 0;
    Smoothed<uint16_t> rawFlowRateSmoother;

public:
    typedef struct
    {
        float flow;
        float v;
        int count;

    } FlowRateData;

    FlowRateData flowRateTable[11] = {
        {0.0, 0.50, 409},
        {85.0, 1.71, 1362},
        {100.0, 1.78, 1403},
        {150.0, 1.96, 1572},
        {202.0, 2.20, 1761},
        {303.0, 2.60, 2103},
        {402.0, 2.92, 2353},
        {503.0, 3.15, 2535},
        {602.0, 3.27, 2650},
        {702.0, 3.35, 2715},
        {802.0, 3.43, 3686}};

    FlowPlf2000Module(TwoWire *twoWire);
    ~FlowPlf2000Module();
    void printRawData(uint16_t val, uint8_t checksum, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4, uint8_t check);
    static void watchFlowTask(void *);
    void begin(void);
    uint16_t getRawFlowRate(void);
    uint16_t readCalibrated(void);
    uint16_t readUncalibrated(void);
    int readSensor(void);
    float getFlowRateMlPerMin(void);
    TwoWire *getI2cInstance(void);
};

#endif