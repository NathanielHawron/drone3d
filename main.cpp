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
    Controller::QueryControllers();
    glfwSetJoystickCallback(Controller::joystickCallback);

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

    std::filesystem::path shaderPath = std::filesystem::current_path().append("res/shaders/");
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
    
    glm::mat4 vpMat;
    glm::vec3 lightPos[2]{
        {0.0f,0.0,0.0f},
        {0.0f,0.0,0.0f}
    };
    float lightCol[2][4]{
        {1.0f, 0.95f, 0.85f, 1.0e4f},
        {0.85f, 0.95f, 1.0f, 1.0e2f}
    };
    
    controls.setSensitivity(0.01);
    
    std::filesystem::path imageDir = std::filesystem::current_path() / "res" / "textures" / "dirt.png";
    NRA::VGL::Image worldImage(imageDir);
    NRA::VGL::Texture worldTexture(worldImage, {{GL_TEXTURE_MIN_FILTER, GL_NEAREST},{GL_TEXTURE_MAG_FILTER, GL_NEAREST},{GL_TEXTURE_WRAP_S, GL_REPEAT},{GL_TEXTURE_WRAP_T, GL_REPEAT}});

    Ground::Config groundConfig;
    // groundConfig.originY = -1.0;
    groundConfig.bpc = 100;
    groundConfig.chunkCount = 10;
    groundConfig.chunkSize = 10;
    groundConfig.textureScale = 10.0;
    groundConfig.textureOffsetX = 0.0;
    groundConfig.textureOffsetY = 0.0;

    groundConfig.noiseParams.amplitude = 1;
    groundConfig.noiseParams.scale = 0.1;
    groundConfig.noiseParams.levels = 5;

    groundConfig.noiseParams.dAmp = 0.5;
    groundConfig.noiseParams.dScale = 2.0;

    Ground ground(layout, groundConfig);
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

        double theta = 0.01*currentTime.time_since_epoch().count()/180.0e6*std::acos(-1);
        lightPos[0] = {5.0e1*std::sin(theta), 1.0e2, 5.0e1*std::cos(theta)};
        lightPos[1] = {-4.0e1*std::sin(theta), 1.0e1, -4.0e1*std::cos(theta)};

        window.reset();
        NRA::VGL::Window::update();
        
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        Drone::DroneMenu(drones, layout);

        if(mainDrone != nullptr){
            mainDrone->renderFocus(true);
        }
        for(std::size_t i=0;i<drones.size();){
            Drone *d = drones.at(i);
            bool del = false, focus = false;
            d->renderUI(i, del, focus, d == mainDrone);
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
                if(d->controller != nullptr){
                    d->updateController();
                }
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
        }
        {
            NRA::VGL::SpacialTree &camSpacial = droneCamera.getSpacial();
            auto deltaMouseScroll = controls.getDMouseScroll();;
            radius = std::max(1.0,radius-deltaMouseScroll.second*2.0);

            double sinp = std::sin(pitch);
            double cosp = std::cos(pitch);
            double siny = std::sin(yaw);
            double cosy = std::cos(yaw);
            camSpacial.pos = (float)radius*glm::vec3(siny*cosp,-sinp,cosy*cosp);
            camSpacial.rot = glm::angleAxis((float)yaw, glm::vec3(0.0f,1.0f,0.0f)) * glm::angleAxis((float)pitch, glm::vec3(1.0f,0.0f,0.0f));
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

        // Render world
        worldTexture.bind(0);
        shader.bind();
        shader.setUniformMat<4>("U_vpMat", &vpMat[0][0]);

        shader.setUniform3v<float>("u_lightPos",&lightPos[0][0],2);
        shader.setUniform4v<float>("u_lightColor",&lightCol[0][0],2);
        ground.render(shader);
        for(Drone *d : drones){
            d->updateController();
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