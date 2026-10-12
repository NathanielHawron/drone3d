/*
    Drone parameters:
    Abdulkareem, Ademola, Oguntosin, Victoria, Popoola, Olawale M., Idowu, Ademola A., Modeling and Nonlinear Control of a Quadcopter for Stabilization and Trajectory Tracking, Journal of Engineering, 2022, 2449901, 19 pages, 2022. https://doi.org/10.1155/2022/2449901
*/

#include "drone.hpp"

#include "imgui/imgui.h"

#include "NRA_visionGL/meshbuilder.h"

#include "droneSensors.hpp"

void Drone::controlDifferentialQuad(Drone *self, double t, double dx, double dy, double dz){
    std::vector<double> powers = {
        t + dx + dy - dz,
        t - dx - dy - dz,
        t + dx - dy + dz,
        t - dx + dy + dz
    };
    self->control(powers);
}

void Drone::DroneMenu(std::vector<Drone*> &drones, NRA::VGL::VertexBufferLayout &layout){
    static const std::size_t newDroneNameSize = 10;
    static char newDroneName[newDroneNameSize] = {"Drone 0\0"};
    ImGui::Begin("Drones");
    ImGui::InputText("Drone name", newDroneName, newDroneNameSize);
    if(ImGui::Button("+##Add drone")){
        std::vector<Drone::thruster> thrusters{ // FL, FR, BL, BR
            {0.0f,5.0f,1.14777f,1.14e-7,10000.0, 1.0,0.0,{{-0.16f,0.1f,0.16f},glm::quat(glm::vec3(0,0,0))}},
            {0.0f,5.0f,1.14777f,1.14e-7,10000.0,-1.0,0.0,{{0.16f,0.1f,0.16f},glm::quat(glm::vec3(0,0,0))}},
            {0.0f,5.0f,1.14777f,1.14e-7,10000.0,-1.0,0.0,{{-0.16f,0.1f,-0.16f},glm::quat(glm::vec3(0,0,0))}},
            {0.0f,5.0f,1.14777f,1.14e-7,10000.0, 1.0,0.0,{{0.16f,0.1f,-0.16f},glm::quat(glm::vec3(0,0,0))}}
        };
        Drone *drone = new Drone(newDroneName, layout, {}, thrusters);
        drone->controller = new QuadController();
        drones.push_back(drone);

        std::size_t nameLen = strnlen(newDroneName, newDroneNameSize);
        if(0<nameLen && nameLen<newDroneNameSize){
            std::size_t i=nameLen-1;
            bool incremented = false;
            if('0' <= newDroneName[i] && newDroneName[i] <= '9'){
                std::size_t j = i;
                while(0<j && '0' <= newDroneName[j] && newDroneName[j] <= '9'){
                    if(newDroneName[j] < '9'){
                        ++newDroneName[j];
                        incremented = true;
                        break;
                    }else{
                        newDroneName[j] = '0';
                    }
                    --j;
                }
                if(!incremented){
                    if(nameLen < newDroneNameSize-1){
                        newDroneName[++j] = '1';
                        while(j<=i){
                            newDroneName[++j] = '0';
                        }
                    }else{
                        while(j<i){
                            newDroneName[++j] = '0';
                        }
                    }
                }
            }
        }
    }
}
Drone::Drone(std::string name, NRA::VGL::VertexBufferLayout vbLayout, droneBodyState state, std::vector<thruster> thrusters, controlDifferentialFunc controlDifferential):
name{name},
bodyState{state},
thrusters{},
controlDifferential{controlDifferential}{
    for(const thruster &t : thrusters){
        thruster *nt = new thruster();
        nt->minThrust = t.minThrust;
        nt->maxThrust = t.maxThrust;
        nt->controlRef = t.controlRef;
        nt->rPos = t.rPos;
        nt->torqueRatio = t.torqueRatio;
        nt->rpmScale = t.rpmScale;
        nt->dir = t.dir;
        nt->rPos.changeParent(&this->bodyState.pos);
        
        this->thrusters.push_back(nt);
    }

    this->bodyRenderable.init();
    this->bodyRenderable.setVBOLayout(vbLayout);
    this->propRenderable.init();
    this->propRenderable.setVBOLayout(vbLayout);

    NRA::VGL::Mesh bodyMesh = NRA::VGL::Mesh(8);
    NRA::VGL::MeshBuilder<GLuint>::createBox(bodyMesh, {}, this->bodyState.bodySize.x, this->bodyState.bodySize.y, this->bodyState.bodySize.z);
    this->bodyRenderable.loadMesh(bodyMesh);

    NRA::VGL::Mesh propMesh = NRA::VGL::Mesh(8);
    NRA::VGL::MeshBuilder<GLuint>::createBox(propMesh, {}, 0.25f, 0.025f, 0.05f);
    this->propRenderable.loadMesh(propMesh);
}
Drone::~Drone(){
    for(thruster *t : this->thrusters){
        delete t;
    }
}

void Drone::update(double dt){
    // These start off in body space, and are summed relative to the drone
    glm::dvec3 sumForce = glm::dvec3(0.0,0.0,0.0);
    glm::dvec3 sumTorque = glm::dvec3(0.0,0.0,0.0);
    glm::dquat posQuat = this->bodyState.pos.getQuat();
    glm::dquat posQuatConj = glm::conjugate(posQuat);

    for(thruster *t : this->thrusters){
        // @TODO: Add prop rotation inertia
        double rps = (t->rpmScale/60.0)*std::sqrt(t->controlRef);
        t->spin += dt*2*std::acos(-1.0)*rps*t->dir;
        glm::dvec3 dir = (glm::dquat)t->rPos.getQuat()*glm::dvec3(0.0f,1.0f,0.0f);
        sumForce += dir*t->controlRef;

        glm::dvec3 delta = (glm::dvec3)t->rPos.getPos() - this->bodyState.centerMassOffset;
        sumTorque += t->controlRef*glm::cross(delta, dir) + t->dir*rps*rps*dir*t->torqueRatio;
    }
    // Linear acceleration
    glm::dvec3 bodyVel = posQuatConj*this->bodyState.vel;
    glm::dvec3 dragForce = this->bodyState.dragQuadratic*(bodyVel*glm::abs(bodyVel));
    glm::dvec3 sumAcc = (sumForce-dragForce)/this->bodyState.mass;
    // Calculate drag
    // World space acceleration
    glm::dvec3 worldAcc = (posQuat*sumAcc) + this->gravity;
    // this->bodyState.acc is meant to be accelerometer reading.
    this->bodyState.acc = sumAcc + (posQuatConj*this->gravity);
    // Move body in world space by v*dt + 0.5a*dt^2
    this->bodyState.pos.moveA(dt*(this->bodyState.vel + 0.5*dt*worldAcc));
    // Update velocity
    this->bodyState.vel += worldAcc * dt;

    // Angular Acceleration
    // @TODO: Add cross-coupled inertia
    glm::dvec3 sumMoment = sumTorque / this->bodyState.inertia;
    // Convert to world coordinates
    glm::dvec3 worldRotAcc = posQuat*sumMoment;
    // v*dt + 0.5*a*dt^2
    glm::dvec3 rotVec = this->bodyState.rotVel + 0.5*dt*worldRotAcc;
    // Apply rotation to pos in absolute coordinates, using unit axis vector and angle
    double rotVecMag = glm::length(rotVec);
    if(rotVecMag > 0){
        this->bodyState.pos.rotateA(rotVec/rotVecMag, dt*rotVecMag);
    }
    // Update angular velocity in world coordinates
    this->bodyState.rotVel += dt*worldRotAcc;
    // @TODO: Add gyroscope (body space degrees/sec)
}
void Drone::render(NRA::VGL::Shader &shader){
    glm::mat4 modelMat = glm::mat4(1.0f);
    this->bodyState.pos.transform(modelMat);
    shader.setUniformMat<4>("U_mMat", &modelMat[0][0]);
    this->bodyRenderable.bindBuffers();
    this->bodyRenderable.render();

    for(thruster * t : this->thrusters){
        glm::mat4 thrusterMat = modelMat;
        t->rPos.transform(thrusterMat);
        glm::vec3 dir = t->rPos.getQuat()*glm::vec3(0.0f,1.0f,0.0f);
        thrusterMat = thrusterMat * glm::rotate(glm::mat4(1.0),(float)t->spin,dir);
        shader.setUniformMat<4>("U_mMat", &thrusterMat[0][0]);
        this->propRenderable.bindBuffers();
        this->propRenderable.render();
    }
}
void Drone::renderUI(std::size_t index, bool &del, bool &focus){
    ImGui::PushID((this->name + std::to_string(index)).c_str());
    ImGui::BeginChild(this->name.c_str(), {100,75});

    ImGui::Text(this->name.c_str());
    if(ImGui::Button("Delete")){
        del = true;
    }
    if(ImGui::Button("Focus")){
        focus = true;
    }

    ImGui::EndChild();
    ImGui::PopID();
}
void Drone::renderDebug(){
    static std::vector<double> powers = {0.0,0.0,0.0,0.0};
    static std::vector<double> differentials = {0.0,0.0,0.0,0.0};
    ImGui::PushID((this->name + "dev").c_str());
    ImGui::Begin((this->name + " dev menu").c_str());

    glm::vec3 pos = this->bodyState.pos.getPos();
    glm::vec3 vel = this->bodyState.vel;
    std::string posStr = "Pos: (" + std::to_string(pos.x) + ", " + std::to_string(pos.y) + ", " + std::to_string(pos.z) + ")";
    std::string velStr = "Vel: (" + std::to_string(vel.x) + ", " + std::to_string(vel.y) + ", " + std::to_string(vel.z) + ")";
    std::string ctrlStr = "Powers: ";
    for(thruster *t : this->thrusters){
        ctrlStr += std::to_string(t->controlRef) +" ";
    }
    ImGui::InputDouble("Mass", &this->bodyState.mass);
    this->bodyState.mass = std::max(this->bodyState.mass, 0.0);
    ImGui::Text(posStr.c_str());
    ImGui::Text(velStr.c_str());
    ImGui::Text(ctrlStr.c_str());

    ImGui::InputScalarN("Motor powers", ImGuiDataType_Double, &powers.at(0), 4, nullptr, nullptr, "%.3f");
    ImGui::InputScalarN("Motor differentials", ImGuiDataType_Double, &differentials.at(0), 4, nullptr, nullptr, "%.3f");

    if(ImGui::Button("Set Powers")){
        this->control(powers);
    }
    if(ImGui::Button("Set Differentials")){
        this->controlDifferential(this, differentials.at(0),differentials.at(1),differentials.at(2),differentials.at(3));
    }

    ImGui::End();
    ImGui::PopID();
}
void Drone::control(std::vector<double> &powers){
    for (std::size_t i = 0; i < std::min(this->thrusters.size(), powers.size()); i++){
        this->thrusters.at(i)->setControl(powers.at(i));
    }
}
void Drone::updateController(){
    if(this->controller != nullptr){
        this->readGPS(this->controller->gps);
        this->readIMU(this->controller->imu);
        this->readBarometer(this->controller->barometer);
        this->controller->updateEKF();
        std::array<double, 4> controls = this->controller->control();
        double cosangle = glm::length(glm::dot(this->gravity, (glm::dquat)this->bodyState.pos.getQuat()*glm::dvec3(0,1,0))) / glm::length(this->gravity);
        controls.at(0) += glm::length(this->gravity) / std::max((4*cosangle),0.2);
        for(int i=0;i<4;++i){
            controls.at(i) *= this->bodyState.mass;
        }

        this->controlDifferential(this, controls.at(0), controls.at(1), controls.at(2), controls.at(3));
    }
}
void Drone::readGPS(SensorData::GPS &gps){
    gps.px = this->bodyState.pos.pos.x;
    gps.py = this->bodyState.pos.pos.y;
    gps.pz = this->bodyState.pos.pos.z;
}
void Drone::readIMU(SensorData::IMU &imu){
    double accFactor = (INT16_MAX/imu.accRange)/9.81;
    glm::dvec3 gravity = this->bodyState.pos.getQuat()*glm::vec3{0.0,-9.81,0.0};
    imu.accx = accFactor*(this->bodyState.acc.x+gravity.x);
    imu.accy = accFactor*(this->bodyState.acc.y+gravity.y);
    imu.accz = accFactor*(this->bodyState.acc.z+gravity.z);

    double gyrFactor = (INT16_MAX/imu.gyrRange)*180.0/std::acos(-1);
    // imu.gyrx = gyrFactor*(this->bodyState.rotAcc.x);
    // imu.gyry = gyrFactor*(this->bodyState.rotAcc.y);
    // imu.gyrz = gyrFactor*(this->bodyState.rotAcc.z);
}
void Drone::readBarometer(SensorData::Barometer &barometer){
    barometer.alt = this->bodyState.pos.pos.y;
}