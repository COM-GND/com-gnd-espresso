
#ifndef PUMPMODULE_H
#define PUMPMODULE_H
#include <Arduino.h>
#include "utils.h"

class PumpModuleCallbacks;

class PumpModule
{
private:
    TaskHandle_t xHandle;
    uint8_t zeroCrossPin;
    uint8_t ctrlPin;
    uint8_t pumpMin;
    uint8_t pumpMax;
    uint8_t pumpRange;
    volatile uint8_t pumpLevel;
    uint8_t oldPowerIsOn;
    uint8_t powerIsOn;
    uint8_t psmIndex = 0;                     // the current psm pulse index
    volatile uint8_t psmMaxPeriodCounts = 12; // the max number of counts in a psm period
    uint8_t currPsmPeriodCounts = 10;         // the number of counts in the current psm period
    uint8_t currPsmOnCounts = 10;             // the number of counts within the current period that the pump is on
    uint8_t nextPsmPeriodCounts = 10;         // the number of counts in next psm period
    uint8_t nextPsmOnCounts = 10;             // the number of counts within the next period that the pump is on

    float_t acFqMs = 16.66; // (1/60hz) * 1000 - duration in MS of a single peek-to-peek ac oscillation
    PumpModuleCallbacks *callbacks = nullptr;
    void computeVPsm();
    void computeDitheredVPsm();

public:
    PumpModule(uint8_t, uint8_t);
    ~PumpModule();
    static void watchPumpPowerTask(void *pump);
    void IRAM_ATTR doVariablePeriodPsm();
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

extern PumpModule *_pumpModule;

#endif