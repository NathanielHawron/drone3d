#include <string>
#include <iostream>
#include <chrono>
#include <vector>

#include "imgui/imgui.h"
#include "imgui/backends/imgui_impl_glfw.h"
#include "imgui/backends/imgui_impl_opengl3.h"

#include "NRA_visionGL/config.h"
#include "NRA_visionGL/window.h"
#include "NRA_visionGL/shader.h"
#include "NRA_visionGL/renderable.h"
#include "NRA_visionGL/mesh.h"
#include "NRA_visionGL/meshbuilder.h"
#include "NRA_visionGL/controls.h"
#include "NRA_visionGL/camera.h"
#include "NRA_visionGL/spacial.h"
#include "NRA_visionGL/texture.h"
#include "NRA_visionGL/frameBufferObject.h"

#include "drone.hpp"
#include "ground.hpp"

#include "controlBindInit.hpp"

int main(){
    std::cout << "NRA_visionGL test v" << (std::string)NRA_visionGL_VERSION << std::endl;

    NRA::VGL::Window::init();

    std::vector<NRA::VGL::ControlsInit> controlsList;
    getControlList(controlsList);

    NRA::VGL::Controls controls{controlsList};
    NRA::VGL::Window window(800,800,"NRA vision GL test",controls);

    window.makeCurrent();
    window.swapInterval(1);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);
    glClearColor(0.0f, 0.5f, 0.8f, 1.0f);

    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();

    std::filesystem::path shaderPath = std::filesystem::current_path().append("submodule/NRA_visionGL/res/shaders/");
    std::filesystem::path vertexPath = shaderPath/"shaded_texture"/"shaded_texture.vertex";
    std::filesystem::path fragmentPath = shaderPath/"shaded_texture"/"shaded_texture.fragment";

    NRA::VGL::Shader shader(vertexPath,fragmentPath);

    NRA::VGL::VertexBufferLayout layout;

    layout.push(GL_FLOAT,3);
    layout.push(GL_FLOAT,2);
    layout.push(GL_FLOAT,3);

    
    NRA::VGL::ProjectionParams projectionParams = {window.getAspect(), NRA::VGL::ProjectionParams::horizontalFOV(90.0f,window.getAspect())};
    NRA::VGL::Camera_T<NRA::VGL::SpacialTree> droneCamera({0.0f,0.0f,10.0f},glm::quat(glm::vec3(0.0f,0.0f,0.0f)),projectionParams,(NRA::VGL::SpacialTree*)nullptr);
    //camera.rotLock = true;
    
    glm::mat4 modelMat = glm::mat4(1.0f);
    glm::mat4 vpMat;
    glm::vec3 lightPos{5.0f,5.0f,0.0f};
    
    controls.setSensitivity(0.01);
    
    std::filesystem::path imageDir = std::filesystem::current_path() / "submodule" / "NRA_visionGL" / "res" / "textures" / "test.png";
    NRA::VGL::Image worldImage(imageDir);
    NRA::VGL::Texture worldTexture(worldImage);

    Ground ground(layout, 100, 0, 100.0);
    std::vector<Drone*> drones;
    Drone *mainDrone = nullptr;

    double totalTime = 0.0;
    auto previousTime = std::chrono::high_resolution_clock::now();
    double yaw = 0.0, pitch = 0.0;
    double sensitivity = 1.0;
    double minPitch = -1.5, maxPitch = 1.5;
    double radius = 10.0;

    while(!(window.shouldClose())){
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto elapsed = currentTime - previousTime;
        previousTime = currentTime;
        double dt = elapsed.count() / 1000000000.0;
        totalTime += dt;

        window.reset();
        NRA::VGL::Window::update();
        
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        Drone::DroneMenu(drones, layout);

        if(mainDrone != nullptr){
            mainDrone->renderDebug();
        }
        for(std::size_t i=0;i<drones.size();){
            Drone *d = drones.at(i);
            bool del = false, focus = false;
            d->renderUI(i, del, focus);
            if(del){
                if(d == mainDrone){
                    NRA::VGL::SpacialTree &cameraSpacial = droneCamera.getSpacial();
                    cameraSpacial.changeParent(nullptr);
                    mainDrone = nullptr;
                }
                delete d;
                drones.erase(drones.begin() + i);
            }else{
                if(focus){
                    mainDrone = d;
                    NRA::VGL::SpacialTree &cameraSpacial = droneCamera.getSpacial();
                    cameraSpacial.changeParent(d->getSpacial());
                }
                d->update(dt);
                ++i;
            }
        }


        if(controls.queryControl(TestControls::pause)){
            if(window.getIsPointerLocked()){
                window.unlockPointer();
            }else{
                window.lockPointer();
            }
            controls.unsetControl(TestControls::pause);
        }
        if (window.getIsPointerLocked() || controls.queryControl(TestControls::rotate)){
            NRA::VGL::SpacialTree &camSpacial = droneCamera.getSpacial();
            auto deltaMousePos = controls.getDMousePos();
            pitch = std::min(std::max(minPitch,pitch + deltaMousePos.second*sensitivity),maxPitch);
            yaw -= deltaMousePos.first*sensitivity;
            if(yaw > 2.0*std::acos(-1)){
                yaw -= 2.0*std::acos(-1);
            }
            if(yaw < -2.0*std::acos(-1)){
                yaw += 2.0*std::acos(-1);
            }
            double sinp = std::sin(pitch);
            double cosp = std::cos(pitch);
            double siny = std::sin(yaw);
            double cosy = std::cos(yaw);
            camSpacial.pos = (float)radius*glm::vec3(siny*cosp,-sinp,cosy*cosp);
            camSpacial.rot = glm::angleAxis((float)yaw, glm::vec3(0.0f,1.0f,0.0f)) * glm::angleAxis((float)pitch, glm::vec3(1.0f,0.0f,0.0f));
            //camSpacial.rot = glm::quat(glm::vec3(pitch, -yaw, 0.0));
        }
        
        
        if(window.getAspectChanged()){
            projectionParams.aspect = window.getAspect();
            droneCamera.updateProjectionMatrix(projectionParams);
        }
        glViewport(0,0,window.getWidth(),window.getHeight());
        glClearColor(0.0f, 0.5f, 0.8f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        vpMat = glm::mat4(1.0f);
        droneCamera.transformP(vpMat);
        lightPos = {20.0f*std::sin(totalTime*0.5f), 50.0f, 20.0f*cos(totalTime*0.5f)};

        // Render world
        worldTexture.bind(0);
        shader.bind();
        shader.setUniformMat<4>("U_vpMat", &vpMat[0][0]);
        shader.setUniformMat<4>("U_mMat", &modelMat[0][0]);
        ground.render(shader);
        for(Drone *d : drones){
            d->render(shader);
        }
        

        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        window.swapBuffer();
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}