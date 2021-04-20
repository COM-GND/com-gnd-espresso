#ifndef TEMPNTCMODULE_H
#define TEMPNTCMODULE_H

#include <ESP32AnalogRead.h>


class TemperatureNtcModule
{
private:
    TaskHandle_t xHandle;
    void _setTemperatureMv(int);

public:
    int acFreq;
    ESP32AnalogRead adc;
    int rawTemperature; // must be static so that the RTOS task can change it
    /*
    Based on NTC sensor with range of -20°C to +150°C
    */
    const int minRawTemperature = 0;   
    const int maxRawTemperature = 3300; // 3.3v = ~10bar
    int rawTemperatureRange;
    TemperatureNtcModule(unsigned char, int);
    ~TemperatureNtcModule();
    static void watchTemperatureTask(void *);
    void begin(void);
    int getTemperatureMv(void);
    float getTemperaturePercent(void);
    float getTemperatureResistance(void);
    float getTemperatureC(void);
};

#endif