#include "drone.hpp"

#include "imgui/imgui.h"

#include "NRA_visionGL/meshbuilder.h"

void Drone::DroneMenu(std::vector<Drone*> &drones, NRA::VGL::VertexBufferLayout &layout){
    static char newDroneName[64] = {"Drone 0\n"};
    ImGui::Begin("Drones");
    ImGui::InputText("Drone name", newDroneName, 64);
    if(ImGui::Button("+##Add drone")){
        std::vector<Drone::thruster> thrusters{
            {0.0f,5.0f,0.0f,0.022,9000000.0, 1.0,{{-0.3f,0.1f,-0.3f},glm::quat(glm::vec3(0,0,0))}},
            {0.0f,5.0f,0.0f,0.022,9000000.0,-1.0,{{-0.3f,0.1f,0.3f},glm::quat(glm::vec3(0,0,0))}},
            {0.0f,5.0f,0.0f,0.022,9000000.0, 1.0,{{0.3f,0.1f,0.3f},glm::quat(glm::vec3(0,0,0))}},
            {0.0f,5.0f,0.0f,0.022,9000000.0,-1.0,{{0.3f,0.1f,-0.3f},glm::quat(glm::vec3(0,0,0))}}
        };
        drones.push_back(new Drone("Drone", layout, {}, thrusters));
    }
}
Drone::Drone(std::string name, NRA::VGL::VertexBufferLayout vbLayout, droneBodyState state, std::vector<thruster> thrusters):
name{name},
bodyState{state},
thrusters{}{
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
    glm::dvec3 sumForce = glm::dvec3(0.0,0.0,0.0);
    glm::dvec3 sumTorque = glm::dvec3(0.0,0.0,0.0);

    for(thruster *t : this->thrusters){
        double dpsiSq = t->controlRef*t->rpmScale;
        double dpsi = std::sqrt(dpsiSq);
        t->rPos.rotateA({0,1,0},dt*2*std::acos(-1)*dpsi*t->dir);
        glm::dvec3 dir = (glm::dquat)t->rPos.getQuat()*glm::dvec3(0.0f,1.0f,0.0f);
        glm::dvec3 delta = (glm::dvec3)t->rPos.getPos() - this->bodyState.centerMassOffset;
        sumForce += t->controlRef*dir;
        sumTorque += t->controlRef*glm::cross(delta, dir) + t->dir*dpsiSq*((glm::dquat)t->rPos.getQuat()*glm::dvec3(0.0,1.0,0.0));
    }
    // Linear force
    glm::dvec3 absForce = (glm::dquat)this->bodyState.pos.rot*sumForce;
    glm::dvec3 absAcc = absForce / this->bodyState.mass;
    absAcc += glm::dvec3(0.0,-9.8,0.0);
    
    this->bodyState.pos.moveA(dt*(this->bodyState.vel + 0.5*dt*absAcc));
    this->bodyState.vel += absAcc * dt;

    // Angular force
    glm::dvec3 rotAcc = sumTorque*dt / this->bodyState.inertia;
    glm::dvec3 rotVec = ((this->bodyState.rotVel / this->bodyState.inertia) + 0.5*rotAcc)*dt;
    double rotVecMag = glm::length(rotVec);
    if(rotVecMag > 0){
        this->bodyState.pos.rotateA(rotVec/rotVecMag, dt*rotVecMag);
    }
    this->bodyState.rotVel += rotAcc;
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

    if(ImGui::Button("Set")){
        this->control(powers);
    }

    ImGui::End();
    ImGui::PopID();
}
void Drone::control(std::vector<double> powers){
    for (std::size_t i = 0; i < std::min(this->thrusters.size(), powers.size()); i++){
        this->thrusters.at(i)->setControl(powers.at(i));
    }
}