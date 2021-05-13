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

    const int minRawTemperature = 0;   
    const int maxRawTemperature = 3300; // 3.3v

    // thermistor specs
    // https://www.mouser.com/ProductDetail/vishay/ntcalug02a103f/?qs=AtFvwFU%2F1FnuJdiGNBrReQ%3D%3D&countrycode=US&currencycode=USD
    int b25 = 3984;  
    int ro = 10000; // 10k Ohm

    TemperatureNtcModule(unsigned char);
    ~TemperatureNtcModule();
    static void watchTemperatureTask(void *);
    void begin(void);
    int getTemperatureMv(void);
    float getTemperatureResistance(void);
    float getTemperatureC(void);
};

#endif