#pragma once

struct particleSensorState_t {
    uint16_t avgPM25 = 0;
    uint16_t measurements = 0;
    uint8_t measurementIdx = 0;
    boolean valid = false;
};

struct vocSensorState_t {
    uint16_t avgTVOC = 0;
    uint16_t avgECO2 = 0;
    uint32_t sumTVOC = 0;
    uint32_t sumECO2 = 0;
    uint8_t measurementIdx = 0;
    boolean valid = false;
};
