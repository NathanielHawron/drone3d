#pragma once

#include <string>
#include <vector>

#include "NRA_visionGL/shader.h"
#include "NRA_visionGL/renderable.h"
#include "NRA_visionGL/mesh.h"
#include "NRA_visionGL/spacial.h"
#include "NRA_visionGL/texture.h"
#include "NRA_visionGL/frameBufferObject.h"

#include "controller.hpp"

class Drone{
public:
    struct droneBodyState{
        glm::dvec3 bodySize = glm::dvec3{0.25, 0.1, 0.25};
        NRA::VGL::SpacialTree pos = {};
        glm::dvec3 vel = {}, acc = {};
        glm::dvec3 rotVel = {};
        glm::dvec3 localRotVel = {};
        double mass = 0.468;
        glm::dvec3 centerMassOffset = {};
        glm::dvec3 inertia = {4.856e-3, 8.801e-3, 4.856e-3};
        glm::dvec3 dragQuadratic = {0.02, 0.05, 0.02};
        glm::dvec3 dragAngularQuadratic = {0.01, 0.02, 0.01};
        droneBodyState(){};
    };
    struct thruster{
        double minThrust = 0.0, maxThrust = 0.5;
        double controlRef = 0.0;
        double torqueRatio = 0.022;
        double rpmScale = 10000.0;
        double dir = 1.0, spin = 0.0;
        NRA::VGL::SpacialTree rPos = {};
        void setControl(double val){this->controlRef = ((val < this->minThrust) ? this->minThrust : ((val > this->maxThrust) ? this->maxThrust : val));}
    };
    using controlDifferentialFunc = void(*)(Drone *self, double t, double dx, double dy, double dz);
    static void controlDifferentialQuad(Drone *self, double t, double dy, double dp, double dr);
private:
    std::string name;
    droneBodyState bodyState;
    NRA::VGL::Renderable bodyRenderable, propRenderable;
    std::vector<thruster*> thrusters;
    glm::dvec3 gravity = {0.0,-9.81,0.0};

    bool showDisabledControlModes = false;
    bool showDisabledJoysticks = false;
public:
    Controller *controller = nullptr;
public:
    static void DroneMenu(std::vector<Drone*> &drones, NRA::VGL::VertexBufferLayout &layout);
    Drone(std::string name, NRA::VGL::VertexBufferLayout vbLayout, droneBodyState state=droneBodyState{}, std::vector<thruster> thrusters={}, controlDifferentialFunc controlDifferential = Drone::controlDifferentialQuad);
    ~Drone();
    void update(double dt);
    void render(NRA::VGL::Shader &shader);
    void renderUI(std::size_t index, bool &del, bool &focus, bool isMain = false);
    void renderFocus(bool debug);
    void control(std::vector<double> &powers);
    void updateController();
    void readGPS(SensorData::GPS &gps);
    void readIMU(SensorData::IMU &imu);
    void readBarometer(SensorData::Barometer &barometer);
    controlDifferentialFunc controlDifferential;
    inline NRA::VGL::SpacialTree *getSpacial(){return &this->bodyState.pos;};
};