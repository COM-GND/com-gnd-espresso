
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

RotaryEncoderModule::RotaryEncoderModule(unsigned char pin1, unsigned char pin2, unsigned char buttonPin)
{
    xHandle = NULL;
    encoderPin1 = pin1;
    encoderPin2 = pin2;
    encoderButtonPin = buttonPin;
    pinMode(encoderButtonPin, INPUT);
    encoder.attachHalfQuad(encoderPin1, encoderPin2);
    encoder.clearCount();
    button.init(buttonPin);
    ButtonConfig *buttonConfig = button.getButtonConfig();
    buttonConfig->setIEventHandler(this);
    buttonConfig->setFeature(ButtonConfig::kFeatureClick);


    Serial.println("Encoder Configured - Button Pin: " + String(encoderButtonPin));
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
    int position = myself->getPosition();
    int oldPosition = 0;
    
    //Serial.println("ace " + String( myself->button.getPin() ) + " var "  + String(myself->encoderButtonPin));
    
    for (;;)
    {
        myself->button.check();
        oldPosition = position;
        position = myself->getPosition();
        if (oldPosition != position)
        {
            myself->_handleEncoderChange(position);
        }
        vTaskDelay(4 / portTICK_PERIOD_MS);
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
    // Serial.println("Button Event " + eventType);

    switch (eventType)
    {
    case AceButton::kEventPressed:
        if (callbacks)
        {
            callbacks->onButtonDown(this);
        }
        break;
    case AceButton::kEventClicked:
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

void RotaryEncoderModule::setPosition(int position)
{
    encoder.setCount(position);
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

void RotaryEncoderModule::setPercent(float perc)
{
    int pos = round(perc * (float)(encoderFullRangeRevs * encoderPulsesPerRev));
    setPosition(pos);
}

float RotaryEncoderModule::getPercent()
{
    int pos = getPosition();
    float perc = (float)pos / (float)(encoderFullRangeRevs * encoderPulsesPerRev);
    return perc;
}

void RotaryEncoderModule::setCallbacks(RotaryEncoderModuleCallbacks *pCallbacks)
{
    callbacks = pCallbacks;
}
