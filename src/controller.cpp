#include "controller.hpp"

const std::array<std::string, 13> Controller::MODE_STR = {
    "DEBUG",
    "MANUAL",
    "ACRO",
    "HORIZON",
    "STABILIZE",
    "ALT_HOLD",
    "POS_HOLD",
    "LOITER",
    "WAYPOINT",
    "GUIDED",
    "CIRCLE",
    "LAND",
    "RTH"
};
int Controller::joysticks[16] = {};
void Controller::QueryControllers(){
    for(int jid = GLFW_JOYSTICK_1; jid <= GLFW_JOYSTICK_16; ++jid){
        if(glfwJoystickPresent(jid)){
            Controller::joysticks[jid] = jid;
        }else{
            Controller::joysticks[jid] = -1;
        }
    }
}

Controller::Controller(int mode):
currentMode(mode){

}

void Controller::updateEKF(){
    double accFactor = (imu.accRange/INT16_MAX)*9.81;
    glm::vec3 acc = {
        this->imu.accx*accFactor,
        this->imu.accy*accFactor,
        this->imu.accz*accFactor
    };
}


QuadController::QuadController(int mode):
Controller(this->isModeSupported(mode) ? mode : MODE_MANUAL){

}
QuadController::~QuadController(){

}
bool QuadController::isModeSupported(int mode){
    const int SUPPORTED_MODES = MODE_BIT_DEBUG | MODE_BIT_MANUAL | MODE_BIT_ACRO;
    return SUPPORTED_MODES & (1 << mode);
}
std::array<double, 4> QuadController::control(double maxAcc){
    std::array<double, 4> res = {0,0,0,0};
    if(this->joystick != -1){
        if(Controller::joysticks[this->joystick] == -1){
            this->joystick = -1;
        }else{
            int count;
            const float deadZone = 0.005;
            const float *axes = glfwGetJoystickAxes(Controller::joysticks[this->joystick], &count);
            if(count >= 4){
                if(std::abs(axes[1]) < deadZone){
                    this->commands.at(0) = 0;
                }else{
                    this->commands.at(0) = -axes[1];
                }
                if(std::abs(axes[0]) < deadZone){
                    this->commands.at(1) = 0;
                }else{
                    this->commands.at(1) = -axes[0];
                }
                if(std::abs(axes[3]) < deadZone){
                    this->commands.at(2) = 0;
                }else{
                    this->commands.at(2) = -axes[3];
                }
                if(std::abs(axes[2]) < deadZone){
                    this->commands.at(3) = 0;
                }else{
                    this->commands.at(3) = axes[2];
                }
            }
        }
    }
    switch(this->currentMode){
        case Controller::MODE_MANUAL:{
            res[0] = commands.at(0) * maxAcc;
            res[1] = commands.at(1) * maxAcc;
            res[2] = commands.at(2) * maxAcc;
            res[3] = commands.at(3) * maxAcc;
        }break;
        case Controller::MODE_ACRO:{

        }break;
    }

    return res;
}