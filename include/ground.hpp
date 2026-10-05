#pragma once

#include "NRA_visionGL/shader.h"
#include "NRA_visionGL/renderable.h"
#include "NRA_visionGL/mesh.h"
#include "NRA_visionGL/spacial.h"
#include "NRA_visionGL/texture.h"
#include "NRA_visionGL/frameBufferObject.h"

class Ground{
private:
    NRA::VGL::Renderable renderable;
public:
    Ground(NRA::VGL::VertexBufferLayout vbLayout, std::size_t size, std::size_t seed, double scale, double offsetX = 0.0, double offsetY = 0.0);
    void render(NRA::VGL::Shader &shader);
};