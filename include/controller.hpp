#pragma once

#include <array>
#include <string>

#include "droneSensors.hpp"

#include "glm/glm.hpp"
#include "GLFW/glfw3.h"

class Controller{
public:
    const static int MODE_BIT_NONE        = 0;
    const static int MODE_BIT_DEBUG       = 1 << 0;
    const static int MODE_BIT_MANUAL      = 1 << 1;
    const static int MODE_BIT_ACRO        = 1 << 2;
    const static int MODE_BIT_HORIZON     = 1 << 3;
    const static int MODE_BIT_STABILIZE   = 1 << 4;
    const static int MODE_BIT_ALT_HOLD    = 1 << 5;
    const static int MODE_BIT_POS_HOLD    = 1 << 6;
    const static int MODE_BIT_LOITER      = 1 << 7;
    const static int MODE_BIT_WAYPOINT    = 1 << 8;
    const static int MODE_BIT_GUIDED      = 1 << 9;
    const static int MODE_BIT_CIRCLE      = 1 << 10;
    const static int MODE_BIT_LAND        = 1 << 11;
    const static int MODE_BIT_RTH         = 1 << 12;
    
    const static int MODE_DEBUG       = 0;
    const static int MODE_MANUAL      = 1;
    const static int MODE_ACRO        = 2;
    const static int MODE_HORIZON     = 3;
    const static int MODE_STABILIZE   = 4;
    const static int MODE_ALT_HOLD    = 5;
    const static int MODE_POS_HOLD    = 6;
    const static int MODE_LOITER      = 7;
    const static int MODE_WAYPOINT    = 8;
    const static int MODE_GUIDED      = 9;
    const static int MODE_CIRCLE      = 10;
    const static int MODE_LAND        = 11;
    const static int MODE_RTH         = 12;
    const static std::array<std::string, 13> MODE_STR;
    static int joysticks[16];
    static void joystickCallback(int jid, int event){
        if(event == GLFW_CONNECTED){
            Controller::joysticks[jid] = jid;;
        }else if(event == GLFW_DISCONNECTED){
            Controller::joysticks[jid] = -1;
        }
    }
    static void QueryControllers();
    struct state{
        glm::dvec3 pos;
        glm::dvec3 vel;
        glm::dvec3 acc;
        double yaw;
    };
public:
    int currentMode;
    int joystick = -1;
    SensorData::GPS gps;
    SensorData::IMU imu;
    SensorData::Barometer barometer;
    
    state s;
    std::array<float, 4> commands;
protected:
    Controller(int mode);
public:
    virtual void updateEKF();
    virtual bool isModeSupported(int mode) = 0;
    // Produces differential controls
    virtual std::array<double, 4> control(double maxAcc) = 0;
};


class QuadController : public Controller{
public:
    QuadController(int mode = MODE_MANUAL);
    ~QuadController();
    bool isModeSupported(int mode) override;
    std::array<double, 4> control(double maxAcc) override;
};