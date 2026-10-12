#pragma once

#include <cstdint>

namespace SensorData{
    struct GPS{
        double px, py, pz;
    };
    struct IMU{
        int_fast16_t accRange = 16;
        int_fast16_t gyrRange = 2000;
        int_fast16_t accx, accy, accz;
        int_fast16_t gyrx, gyry, gyrz;
    };
    struct Barometer{
        double alt;
    };
}
