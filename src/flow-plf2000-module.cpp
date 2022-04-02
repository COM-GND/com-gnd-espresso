#include "flow-plf2000-module.h"

/*
Pins
1: Vdd
2: Gnd
3: Out
4: SDA
5: SCL
6: N/C
*/
FlowPlf2000Module::FlowPlf2000Module(TwoWire *twoWire)
{
    xHandle = NULL;
    I2C = twoWire;

    I2C->beginTransmission(PLF2000_I2C_ADDR);
    byte error = I2C->endTransmission();
    Serial.println("i2C test: " + String(error));
}

FlowPlf2000Module::~FlowPlf2000Module()
{
    if (xHandle != NULL)
    {
        vTaskDelete(xHandle);
    }
}

void FlowPlf2000Module::watchFlowTask(void *instance)
{

    Serial.println("watchFlowTask started");

    FlowPlf2000Module *myself = (FlowPlf2000Module *)instance;

    TwoWire *I2C = myself->I2C;

    unsigned int highwater = 0;

    // try to sync with AC
    // 60hz = ~16.6ms pr cycle
    // sample at half that to get aprox min and max
    for (;;)
    {
        uint8_t multiSamples = 4;
        uint8_t sampleCount = 0;
        uint16_t multiSampleSum = 0;
        int8_t byteCount = 5;
        uint8_t bytes[byteCount];

        for (uint8_t sampleIndex = 0; sampleIndex < multiSamples; sampleIndex++)
        {

            I2C->beginTransmission(PLF2000_I2C_ADDR);
            vTaskDelay(2 / portTICK_PERIOD_MS);

            I2C->requestFrom(PLF2000_I2C_ADDR, byteCount);

            // plf2000 min sample time is 5ms.
            // 5ms delay will give about 200 sps.
            vTaskDelay(6 / portTICK_PERIOD_MS);

            uint8_t sum = 0;
            uint8_t i = 0;
            if (I2C->available())
            {
                for (i = 0; i < byteCount; i++)
                {

                    bytes[i] = I2C->read();

                    if (i > 0)
                    {
                        sum += bytes[i];
                    }
                }
            }

            if (i < byteCount)
            {
                Serial.println("Error reading I2C value (not enough bytes): " + String(i));
                break;
            }

            uint16_t sensorValue = (uint16_t)(((uint16_t)bytes[1] << 8) | bytes[2]);
            uint8_t check = 0x01 + ~(sum);

            if (bytes[0] != check)
            {
                Serial.println("Checksum does not match. Expected: " + String(bytes[0]) + " Actual: " + String(check));
                break;
            }

            multiSampleSum += sensorValue;
            sampleCount++;
        }

        if (sampleCount == multiSamples)
        {
            uint16_t avgValue = multiSampleSum / multiSamples;
            myself->_setRawFlowRate(avgValue);
        }
        else
        {
            Serial.println("Not enough flow samples. Expected: " + String(multiSamples) + " Actual: " + String(sampleCount));
        }

        // I2C->beginTransmission(PLF2000_I2C_ADDR);

        // vTaskDelay(20 / portTICK_PERIOD_MS);

        // I2C->requestFrom(PLF2000_I2C_ADDR, 5);

        // vTaskDelay(5 / portTICK_PERIOD_MS);

        // if (I2C->available())
        // {

        //     uint8_t checksum = I2C->read();
        //     uint8_t myMSB = I2C->read();
        //     uint8_t myLSB = I2C->read();
        //     // clear the two remaining "reserved" bytes.
        //     uint8_t byte3 = I2C->read();
        //     uint8_t byte4 = I2C->read();

        //     uint16_t sum = myMSB + myLSB + byte3 + byte4;
        //     uint8_t check = 0x01 + ~(sum);

        //     uint16_t sensorValue = (uint16_t)(((uint16_t)myMSB << 8) | myLSB);

        //     myself->printRawData(sensorValue, checksum, myMSB, myLSB, byte3, byte4, check);

        //     myself->_setRawFlowRate(sensorValue);

        //     highwater = uxTaskGetStackHighWaterMark(NULL);

        //     vTaskDelay(100 / portTICK_PERIOD_MS);
        // };
    }

    vTaskDelete(NULL);
}

void FlowPlf2000Module::printRawData(uint16_t val, uint8_t checksum, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4, uint8_t check)
{
    Serial.println("raw: " +
                   String(val) + " - " +
                   String(checksum) + " | " +
                   String(byte1) + ", " +
                   String(byte2) + ", " +
                   String(byte3) + ", " +
                   String(byte4) + " | " +
                   String(check));
}

void FlowPlf2000Module::begin()
{
    Serial.println("FlowPlf2000Module Begin");

    rawFlowRateSmoother.begin(SMOOTHED_EXPONENTIAL, 10);
    xTaskCreate(
        &FlowPlf2000Module::watchFlowTask, // Function that should be called
        "read_flow_rate",                  // Name of the task (for debugging)
        2000,                              // Stack size (bytes)
        this,                              // Parameter to pass
        3,                                 // Task priority
        &xHandle                           // Task handle
    );
}

/**
 * Read the Calibrated Data from the sensor
 * The output range is 409 to 3686
 */
uint16_t FlowPlf2000Module::readCalibrated(void)
{
    int8_t byteCount = 5;
    uint8_t bytes[byteCount];

    I2C->beginTransmission(PLF2000_I2C_ADDR);
    delay(5);

    I2C->requestFrom(PLF2000_I2C_ADDR, byteCount);

    delay(5);

    uint8_t sum = 0;
    uint8_t i = 0;
    if (I2C->available())
    {
        for (i = 0; i < byteCount; i++)
        {

            bytes[i] = I2C->read();

            if (i > 0)
            {
                sum += bytes[i];
            }
        }
    }

    if (i < byteCount)
    {
        Serial.println("Error reading I2C value (not enough bytes): " + String(i));
        return 0;
    }

    uint8_t sensorValue = (uint16_t)(((uint16_t)bytes[1] << 8) | bytes[2]);
    uint8_t check = 0x01 + ~(sum);

    if (bytes[0] != check)
    {
        Serial.println("Checksum does not match. Expected: " + String(bytes[0]) + " Actual: " + String(check));
        return 0;
    }

    return sensorValue;
}

/**
 * Read the uncalibrated data from the sensor.
 * The output range is specified as 30000 to 45000 counts.
 */
uint16_t FlowPlf2000Module::readUncalibrated(void)
{
    uint8_t byteCount = 6;
    uint8_t bytes[byteCount];

    I2C->beginTransmission(PLF2000_I2C_ADDR);
    I2C->write(0xD0);
    byte sendResult = I2C->endTransmission();

    if (sendResult != 0)
    {
        Serial.println("Error writing I2C: " + String(sendResult));
        return 0;
    }

    delay(5);

    I2C->requestFrom(PLF2000_I2C_ADDR, byteCount);

    while (I2C->available() == 0)
    {
        delay(1);
    }

    uint8_t sum = 0;
    uint8_t i = 0;
    for (i = 0; i < byteCount && I2C->available(); i++)
    {
        bytes[i] = I2C->read();
        if (i > 0)
        {
            sum += bytes[i];
        }
    }

    if (i < byteCount - 1)
    {
        Serial.println("Error reading I2C value (not enough bytes)");
        return -1;
    }

    uint16_t sensorValue = (uint16_t)(((uint16_t)bytes[1] << 8) | bytes[2]);
    uint16_t temperatureValue = (uint16_t)(((uint16_t)bytes[4] << 8) | bytes[5]);
    uint8_t check = 0x01 + ~(sum);

    if (bytes[0] != check)
    {
        Serial.println("Checksum does not match. Expected: " + String(bytes[0]) + " Actual: " + String(check));
        return -1;
    }
    //     Serial.print(sensorValue, BIN);

    return sensorValue;
}

float FlowPlf2000Module::getFlowRateMlPerMin(void)
{
    // Use sensor data lookup-table to interpolate the flow rate.
    FlowRateData lowerDataPoint = flowRateTable[0];
    FlowRateData upperDataPoint = flowRateTable[FLOW_RATE_TABLE_SIZE - 1];
    int flowCount = getRawFlowRate();
    if (flowCount < lowerDataPoint.count)
    {
        flowCount = lowerDataPoint.count;
    }
    else if (flowCount > upperDataPoint.count)
    {
        flowCount = upperDataPoint.count;
    }

    for (int i = 0; i < FLOW_RATE_TABLE_SIZE - 1; i++)
    {
        if (flowCount >= flowRateTable[i].count && flowCount < flowRateTable[i + 1].count)
        {
            lowerDataPoint = flowRateTable[i];
            upperDataPoint = flowRateTable[i + 1];
            break;
        }
    }

    float flowRate = map(
        flowCount,
        lowerDataPoint.count,
        upperDataPoint.count,
        lowerDataPoint.flow,
        upperDataPoint.flow);

    return flowRate;
}

TwoWire *FlowPlf2000Module::getI2cInstance(void)
{
    return I2C;
}

void FlowPlf2000Module::_setRawFlowRate(uint16_t newRawFlowRate)
{
    // Serial.println("_setRawFlowRate: " + String(newRawFlowRate));
    rawFlowRate = newRawFlowRate;
    rawFlowRateSmoother.add(newRawFlowRate);
}

uint16_t FlowPlf2000Module::getRawFlowRate(void)
{
    uint16_t result = rawFlowRateSmoother.get();
    Serial.println("rawFlowRateSmoother: " + String(result));
    return result;
    // return rawFlowRate;
}