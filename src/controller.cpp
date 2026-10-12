#include "controller.hpp"

void Controller::updateEKF(){
    double accFactor = (imu.accRange/INT16_MAX)*9.81;
    glm::vec3 acc = {
        this->imu.accx*accFactor,
        this->imu.accy*accFactor,
        this->imu.accz*accFactor
    };
}

QuadController::QuadController(){

}
QuadController::~QuadController(){
    
}
std::array<double, 4> QuadController::control(){
    std::array<double, 4> res = {0,0,0,0};

    return res;
}