#ifndef M323MODULE_H
#define M323MODULE_H

#include <ESP32AnalogRead.h>

class PressureM323Module
{
private:
    TaskHandle_t xHandle;
    void _setPressureMv(int);

public:
    int acFreq;
    ESP32AnalogRead adc;
    int rawPressure; // must be static so that the RTOS task can change it
    const int minRawPressure = 37;   // .37v = 0bar
    const int maxRawPressure = 3300; // 3.3v = ~10bar
    int rawPressureRange;
    PressureM323Module(unsigned char, int);
    ~PressureM323Module();
    static void watchPressureTask(void *);
    void begin(void);
    int getPressureMv(void);
    float getPressurePercent(void);
    float getPressureBars(void);
};

#endif