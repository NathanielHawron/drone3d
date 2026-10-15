#pragma once

#include "NRA_visionGL/shader.h"
#include "NRA_visionGL/renderable.h"
#include "NRA_visionGL/mesh.h"
#include "NRA_visionGL/spacial.h"
#include "NRA_visionGL/texture.h"
#include "NRA_visionGL/frameBufferObject.h"

#include "noise.hpp"

class Ground{
public:
    struct Config{
        uint64_t seed = 0;
        std::size_t bpc = 100;
        std::size_t chunkCount = 3;
        double chunkSize = 100.0;
        double originX = 0.0, originY = 0.0, originZ = 0.0;
        double textureScale = 1.0;
        double textureOffsetX = 0.0;
        double textureOffsetY = 0.0;
        FBN_params noiseParams;
        Config(){}
    };
    struct Chunk{
        NRA::VGL::Renderable renderable;
        double x, y;
    };
    struct vertex{
        GLfloat pos[3];
        GLfloat tex[2];
        GLfloat norm[3];
    };
private:
    std::vector<Chunk*> chunks;
    NRA::VGL::VertexBufferLayout vbLayout;
    Config conf;
public:
    Ground(NRA::VGL::VertexBufferLayout vbLayout, Config conf = Config());
    ~Ground();
    void render(NRA::VGL::Shader &shader);
private:
    void generateChunk(double x, double y, double tx0, double ty0, double dtx0, double dty0);
};