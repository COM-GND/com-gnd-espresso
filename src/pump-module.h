
#ifndef PUMPMODULE_H
#define PUMPMODULE_H
#include <dimmable_light_linearized.h>
// #include "pump-module-callbacks.h"

class PumpModuleCallbacks;

class PumpModule
{
private:
    TaskHandle_t xHandle;
    int zeroCrossPin;
    int pumpMin;
    int pumpMax;
    int pumpRange;
    int pumpLevel;
    int oldPowerIsOn;
    int powerIsOn;
    PumpModuleCallbacks *callbacks = nullptr;
    DimmableLightLinearized pump;

public:
    PumpModule(int, int);
    ~PumpModule();
    static void watchPumpPowerTask(void *pump);
    void setZeroCrossPin(int);
    int getZeroCrossPin();
    void begin();
    void setCallbacks(PumpModuleCallbacks *callbacks);
    bool getPowerIsOn();
    void _setPowerIsOn(bool);
    void setPumpMin(int min);
    int getPumpMin();
    void setPumpMax(int max);
    int getPumpMax();
    int getPumpRange();
    void setPumpLevel(int);
    int getPumpLevel();
    float getPumpPercent();
    void setPumpPercent(float);
};

class PumpModuleCallbacks
{
public:
    virtual void onPowerOn(PumpModule *module);
    virtual void onPowerOff(PumpModule *module);

private:
};

#endif