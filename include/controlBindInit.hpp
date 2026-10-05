#pragma once

#include <vector>
#include "NRA_visionGL/controls.h"

namespace TestControls{
    // First block (0 - 31):    Movement
    // WASD + up/down
    const NRA::VGL::ControlBind forwards =      {false, 0};
    const NRA::VGL::ControlBind backwards =     {false, 1};
    const NRA::VGL::ControlBind left =          {false, 2};
    const NRA::VGL::ControlBind right =         {false, 3};
    const NRA::VGL::ControlBind up =            {false, 4};
    const NRA::VGL::ControlBind down =          {false, 5};
    // Arrow keys + QE
    const NRA::VGL::ControlBind pitchUp =       {false, 6};
    const NRA::VGL::ControlBind pitchDown =     {false, 7};
    const NRA::VGL::ControlBind yawLeft =       {false, 8};
    const NRA::VGL::ControlBind yawRight =      {false, 9};
    const NRA::VGL::ControlBind rollLeft =      {};
    const NRA::VGL::ControlBind rollRight =     {};
    // Mouse controls
    const NRA::VGL::ControlBind rotate =        {false, 12};
    const NRA::VGL::ControlBind pan =           {false, 13};
    const NRA::VGL::ControlBind zoom =          {false, 14};
    // Zoom keys
    const NRA::VGL::ControlBind zoomIn =        {false, 15};
    const NRA::VGL::ControlBind zoomOut =       {false, 16};

    // Second block (32 - 63):  Admin
    const NRA::VGL::ControlBind pause =         {false, 32};
};

void getControlList(std::vector<NRA::VGL::ControlsInit> &controlsList){
    using namespace NRA;
    using namespace VGL;
    using namespace TestControls;
    controlsList.emplace_back(ControlsInit{ButtonType::KEY, GLFW_KEY_W,                 forwards});
    controlsList.emplace_back(ControlsInit{ButtonType::KEY, GLFW_KEY_S,                 backwards});
    controlsList.emplace_back(ControlsInit{ButtonType::KEY, GLFW_KEY_A,                 left});
    controlsList.emplace_back(ControlsInit{ButtonType::KEY, GLFW_KEY_D,                 right});
    controlsList.emplace_back(ControlsInit{ButtonType::KEY, GLFW_KEY_SPACE,             up});
    controlsList.emplace_back(ControlsInit{ButtonType::KEY, GLFW_KEY_LEFT_SHIFT,        down});
    
    controlsList.emplace_back(ControlsInit{ButtonType::KEY, GLFW_KEY_UP,                pitchUp});
    controlsList.emplace_back(ControlsInit{ButtonType::KEY, GLFW_KEY_DOWN,              pitchDown});
    controlsList.emplace_back(ControlsInit{ButtonType::KEY, GLFW_KEY_LEFT,              yawLeft});
    controlsList.emplace_back(ControlsInit{ButtonType::KEY, GLFW_KEY_RIGHT,             yawRight});
    controlsList.emplace_back(ControlsInit{ButtonType::KEY, GLFW_KEY_Q,                 rollLeft});
    controlsList.emplace_back(ControlsInit{ButtonType::KEY, GLFW_KEY_E,                 rollRight});
    
    controlsList.emplace_back(ControlsInit{ButtonType::MOUSE,GLFW_MOUSE_BUTTON_RIGHT,   rotate});
    controlsList.emplace_back(ControlsInit{ButtonType::MOUSE,GLFW_MOUSE_BUTTON_LEFT,    pan});
    controlsList.emplace_back(ControlsInit{ButtonType::MOUSE,GLFW_MOUSE_BUTTON_MIDDLE,  zoom});
    
    controlsList.emplace_back(ControlsInit{ButtonType::KEY, GLFW_KEY_Z,                 zoomIn});
    controlsList.emplace_back(ControlsInit{ButtonType::KEY, GLFW_KEY_X,                 zoomOut});
    
    controlsList.emplace_back(ControlsInit{ButtonType::KEY, GLFW_KEY_ESCAPE,            pause});
}