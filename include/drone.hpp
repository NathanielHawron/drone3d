#pragma once

#include <string>
#include <vector>

#include "NRA_visionGL/shader.h"
#include "NRA_visionGL/renderable.h"
#include "NRA_visionGL/mesh.h"
#include "NRA_visionGL/spacial.h"
#include "NRA_visionGL/texture.h"
#include "NRA_visionGL/frameBufferObject.h"

class Drone{
public:
    struct droneBodyState{
        glm::dvec3 bodySize = glm::dvec3{0.25, 0.25, 0.1};
        NRA::VGL::SpacialTree pos = {};
        glm::dvec3 vel = {}, acc = {};
        glm::dvec3 rotVel = {};
        double mass = 1.0;
        glm::dvec3 centerMassOffset = {};
        glm::dvec3 inertia = {0.03, 0.03, 0.05};
        droneBodyState(){};
    };
    struct thruster{
        double minThrust = 0.0, maxThrust = 0.5;
        double controlRef = 0.0;
        double torqueRatio = 0.022;
        double rpmScale = 3000.0;
        NRA::VGL::SpacialTree rPos = {};
        void setControl(double val){this->controlRef = ((val < this->minThrust) ? this->minThrust : ((val > this->maxThrust) ? this->maxThrust : val));}
    };
private:
    std::string name;
    droneBodyState bodyState;
    NRA::VGL::Renderable bodyRenderable, propRenderable;
    std::vector<thruster*> thrusters;
public:
    static void DroneMenu(std::vector<Drone*> &drones, NRA::VGL::VertexBufferLayout &layout);
    Drone(std::string name, NRA::VGL::VertexBufferLayout vbLayout, droneBodyState state=droneBodyState{}, std::vector<thruster> thrusters={});
    ~Drone();
    void update(double dt);
    void render(NRA::VGL::Shader &shader);
    void renderUI(std::size_t index, bool &del, bool &focus);
    void renderDebug();
    void control(std::vector<double> powers);
    inline NRA::VGL::SpacialTree *getSpacial(){return &this->bodyState.pos;};
};