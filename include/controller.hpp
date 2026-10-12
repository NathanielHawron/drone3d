#pragma once

#include <array>

#include "droneSensors.hpp"

#include "glm/glm.hpp"

class Controller{
public:
    struct state{
        glm::dvec3 pos;
        glm::dvec3 vel;
        glm::dvec3 acc;
        double yaw;
    };
public:
    SensorData::GPS gps;
    SensorData::IMU imu;
    SensorData::Barometer barometer;

    state s;

    virtual void updateEKF();
    // Produces differential controls
    virtual std::array<double, 4> control() = 0;
};
class QuadController : public Controller{
public:
    QuadController();
    ~QuadController();
    std::array<double, 4> control() override;
};