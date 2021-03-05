
#include <AceButton.h>
#include <Arduino.h>
#include "rotary-encoder-module.h"

using namespace ace_button;

/**
 * Rotary Encoder Globals
 */

// The numder of pulses produced by one full revolution
const int encoderPulsesPerRev = 40;
// The number of revolutions required for full output range
const int encoderFullRangeRevs = 2;
// initial encoder value - set to max for initializing at full pressure
volatile int encoderCount = encoderFullRangeRevs * encoderPulsesPerRev;
// Mode flag to set if the encoder controls the target Pressure or directly sets the pump power output

// An object that handles the event.
// class ButtonHandler : public IEventHandler
// {
// public:
//     void handleEvent(AceButton * /* button */, uint8_t eventType,
//                      uint8_t buttonState) override
//     {

//         // Print out a message for all events.
//         Serial.print(F("handleEvent(): eventType: "));
//         Serial.print(eventType);
//         Serial.print(F("; buttonState: "));
//         Serial.println(buttonState);

//         // Control the LED only for the Pressed and Released events.
//         // Notice that if the MCU is rebooted while the button is pressed down, no
//         // event is triggered and the LED remains off.
//         switch (eventType)
//         {
//         case AceButton::kEventPressed:
//             //   digitalWrite(LED_PIN, LED_ON);
//             break;
//         case AceButton::kEventReleased:
//             //   digitalWrite(LED_PIN, LED_OFF);
//             break;
//         }
//     }
// };

RotaryEncoderModule::RotaryEncoderModule(unsigned char encoderPin1, unsigned char encoderPin2, unsigned char encoderButtonPin)
{
    encoder.attachHalfQuad(encoderPin1, encoderPin2);
    encoder.clearCount();
    AceButton button(encoderButtonPin);
    ButtonConfig *buttonConfig = button.getButtonConfig();
    // ButtonHandler handleButtonEvent;
    buttonConfig->setIEventHandler(this);

    Serial.println("Encoder Configured");
}
RotaryEncoderModule::~RotaryEncoderModule()
{
    if (xHandle != NULL)
    {
        vTaskDelete(xHandle);
    }
}

void RotaryEncoderModule::watchEncoderTask(void *rotaryEncoder)
{
    RotaryEncoderModule *myself = (RotaryEncoderModule *)rotaryEncoder;
    myself->button.check();

    int position = myself->getPosition();
    int oldPosition = 0;

    for (;;)
    {
        oldPosition = position;
        position = myself->getPosition();
        if (oldPosition != position)
        {
            myself->_handleEncoderChange(position);
        }
        delay(4);
    }
}

void RotaryEncoderModule::begin()
{
    Serial.println("RotaryEncoderModule Begin");

    xTaskCreate(&RotaryEncoderModule::watchEncoderTask, "btnMon", 10000, this, 1, &xHandle);
}

void RotaryEncoderModule::handleEvent(AceButton *button, uint8_t eventType,
                                      uint8_t buttonState)
{
    Serial.println("Button Event " + eventType);

    switch (eventType)
    {
    case AceButton::kEventPressed:
        if (callbacks)
        {
            callbacks->onButtonDown(this);
        }
        break;
    case AceButton::kEventReleased:
        if (callbacks)
        {
            callbacks->onButtonUp(this);
        }
        break;
    }
}

void RotaryEncoderModule::_handleEncoderChange(int position)
{
    if (callbacks)
    {
        callbacks->onEncoderChange(this);
    }
}

int RotaryEncoderModule::getPosition()
{
    int encoderCount = encoder.getCount();
    if (encoderCount < 0)
    {
        encoder.clearCount();
        encoderCount = 0;
    }
    else if (encoderCount > encoderFullRangeRevs * encoderPulsesPerRev)
    {
        encoder.setCount(encoderFullRangeRevs * encoderPulsesPerRev);
        encoderCount = encoderFullRangeRevs * encoderPulsesPerRev;
    }
    return encoderCount;
}

float RotaryEncoderModule::getPercent()
{
    int pos = getPosition();
    float perc = pos / float(encoderCount * encoderPulsesPerRev);
    return perc;
}

void RotaryEncoderModule::setCallbacks(RotaryEncoderModuleCallbacks *pCallbacks)
{
    callbacks = pCallbacks;
}
