
// #pragma once
#include <Arduino.h>
#include <AceButton.h>
#include <driver/gpio.h>
#include <ESP32Encoder.h>

using namespace ace_button;

class RotaryEncoderModuleCallbacks;

class RotaryEncoderModule : public IEventHandler
{
private:
    ESP32Encoder encoder;
    RotaryEncoderModuleCallbacks *callbacks = nullptr;
    TaskHandle_t xHandle = NULL;
    int oldPosition;

public:
    RotaryEncoderModule(unsigned char, unsigned char, unsigned char);
    ~RotaryEncoderModule();
    unsigned char encoderPin1;
    unsigned char encoderPin2;
    unsigned char encoderButtonPin;
    void begin();
    static void watchEncoderTask(void *RotaryEncoderModule);
    void setPosition(int position);
    int getPosition();
    void setPercent(float perc);
    float getPercent();
    void setCallbacks(RotaryEncoderModuleCallbacks *callbacks);
    AceButton button;
    void handleEvent(AceButton * /* button */, uint8_t eventType,
                     uint8_t buttonState);
    void _handleEncoderChange(int position);
};

class RotaryEncoderModuleCallbacks
{
public:
    virtual void onButtonDown(RotaryEncoderModule *module);
    virtual void onButtonUp(RotaryEncoderModule *module);
    virtual void onEncoderChange(RotaryEncoderModule *module);

private:
};

// #pragma once
