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
    /*
    * Sensor range is .5 to 4.5V
    * voltage divider on sensor ouput is: 1.2k : 3.3k
    * Dropping the voltage from 4.5v to 3.3v and .5 to .37v
    * New output range .37v to 3.3v -> ~370 to 3300 mv 
    */
    const int minRawPressure = 320;   // .37v = 0bar (should be 370, but reducing to 320 to compensate for error in resistor - adjust as needed)
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