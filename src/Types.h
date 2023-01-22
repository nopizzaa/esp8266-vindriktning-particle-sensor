#pragma once

struct particleSensorState_t
{
    uint16_t avgPM25 = 0;
    uint16_t measurements[5] = {0, 0, 0, 0, 0};
    uint8_t measurementIdx = 0;
    boolean valid = false;
};

struct aht10SensorState_t
{
    float temperature;
    float humidity;
};

struct bmp280SensorState_t
{
    float temperature;
    float altitude;
    float pressure;
    float waterBoilingPoint;
};

struct sgp30SensorState_t
{
    uint16_t tvoc;
    uint16_t eCo2;
};