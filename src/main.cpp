#include <Arduino.h>
#include <RBDdimmer.h>

// https://www.arduino.cc/en/Reference/ArduinoBLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define PUMPCTLPIN 27
#define ZEROCROSSPIN 14

// https://btprodspecificationrefs.blob.core.windows.net/assigned-values/16-bit%20UUID%20Numbers%20Document.pdf
/**
 * GATT Unit 0x2780 pressure (bar)
 * GATT Unit 0x272F Celsius temperature (degree Celsius)
 * GATT Characteristic and Object Type 0x2A1C Temperature Measurement
 * GATT Characteristic and Object Type 0x2A1D Temperature Type
 */
#define SERVICE_UUID "8fc1ceca-b162-4401-9607-c8ac21383e4e"
#define PRESSURE_CHARACTERISTIC_UUID "2780" // standard characteristic for pressure in bars

int encoderPin1 = 2;
int encoderPin2 = 3;
int encoderSwitchPin = 4; //push button switch

volatile int rawPressure = 0;
volatile float barPressure = 0;
int minRawPressure = 100;  // 1bar
int maxRawPressure = 1012; // ~10bar
int rawPressureRange = maxRawPressure - minRawPressure;

volatile int lastEncoded = 0;
volatile float encoderValue = 0;
volatile float lastEncoderValue = 0;

int lastMSB = 0;
int lastLSB = 0;

float encoderResolution = .5;
int pumpMin = 35;

bool deviceConnected = false;


dimmerLamp dimmer(PUMPCTLPIN, ZEROCROSSPIN); //initialase port for dimmer for ESP8266, ESP32, Arduino due boards

// https://github.com/nkolban/ESP32_BLE_Arduino/blob/master/examples/BLE_notify/BLE_notify.ino
class ComGndServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("BLE Server connected");
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
       Serial.println("BLE Server Disconnected");
      deviceConnected = false;
    }
};
//https://learn.sparkfun.com/tutorials/esp32-thing-plus-hookup-guide/arduino-example-esp32-ble
class PressureCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() > 0) {
        Serial.println("*********");
        Serial.print("received value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
      }
      pCharacteristic->setValue("received " + value);
    }
};

void setup()
{
  // put your setup code here, to run once:
  dimmer.begin(NORMAL_MODE, ON); //dimmer initialisation: name.begin(MODE, STATE)
  Serial.begin(115200);
  Serial.println("Ready to begin");

  // attachInterrupt(encoderPin1, updateEncoder, CHANGE);
  // attachInterrupt(encoderPin2, updateEncoder, CHANGE);

  BLEDevice::init("COM-GND Espresso");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ComGndServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pPressureCharacteristic = pService->createCharacteristic(
      PRESSURE_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_NOTIFY | 
      BLECharacteristic::PROPERTY_INDICATE);

  pPressureCharacteristic->setCallbacks(new PressureCallbacks());
  pPressureCharacteristic->setValue("Hell World");
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void loop()
{
  // put your main code here, to run repeatedly:

  if (lastEncoderValue != encoderValue)
  {
    Serial.println(encoderValue);
    lastEncoderValue = encoderValue;
    // TriacDimmer::setBrightness(channel_1, percVal);
    delay(500);
  }
}

void updateEncoder()
{

  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB;         //converting the 2 pin value to single number
  int sum = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
  {
    encoderValue += encoderResolution;
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
  {
    encoderValue -= encoderResolution;
  }
  lastEncoded = encoded; //store this value for next time

  if (encoderValue < pumpMin)
  {
    encoderValue = pumpMin;
  }
  else if (encoderValue > 99)
  {
    encoderValue = 99;
  }
}