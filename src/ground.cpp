#include "ground.hpp"


Ground::Ground(NRA::VGL::VertexBufferLayout vbLayout, Config conf):
vbLayout{vbLayout},
conf{conf}{
    double offset = -(this->conf.chunkCount-1.0) / 2.0;
    for(int i=0;i<this->conf.chunkCount;++i){
        double x0 = ((double)i+offset)*this->conf.chunkSize;
        double tx0 = (x0+conf.textureOffsetX) / conf.textureScale;
        double dtx0 = this->conf.chunkSize / conf.textureScale;
        for(int j=0;j<this->conf.chunkCount;++j){
            double y0 = ((double)j+offset)*this->conf.chunkSize;
            double ty0 = (y0+conf.textureOffsetY) / conf.textureScale;
            double dty0 = this->conf.chunkSize / conf.textureScale;
            this->generateChunk(x0, y0, tx0, ty0, dtx0, dty0);
        }
    }
}
void Ground::generateChunk(double x, double y, double tx0, double ty0, double dtx0, double dty0){
    int samples = this->conf.bpc + 1;
    vertex* positions = new vertex[samples*samples];
    // Step size
    double dx = this->conf.chunkSize / this->conf.bpc;
    double dy = this->conf.chunkSize / this->conf.bpc;
    // Texture step size
    double dtx = dtx0/this->conf.bpc;
    double dty = dty0/this->conf.bpc;

    double minH = std::numeric_limits<double>::max();
    double maxH = std::numeric_limits<double>::min(); 

    for(int xi=0;xi<samples;++xi){
        for(int yi=0;yi<samples;++yi){
            // Position index
            int i = xi*(samples) + yi;
            // Base relative position (first corner)
            double rx = -this->conf.chunkSize*0.5 + xi*dx;
            double ry = -this->conf.chunkSize*0.5 + yi*dy;
            // Base relative texture
            double tx = xi * dtx;
            double ty = yi * dty;

            double dx = 0.0, dy = 0.0;
            double h = fractalNoise2d(rx+x, ry+y, this->conf.seed, &dx, &dy, defaultNoiseHash<double>, this->conf.noiseParams);
            double slopeScale = 1.0/std::sqrt(dx*dx+1+dy*dy);

            positions[i] = {
                {(float)rx, (float)h, (float)ry},
                {(float)(tx0+tx), (float)(ty0+ty)},
                {-slopeScale*dx, slopeScale, -slopeScale*dy}
            };

            // Check min/max values
            // if(h > maxH){
            //     maxH = h;
            // }
            // if(h < minH){
            //     minH = h;
            // }
        }
    }
    //std::cout << minH << " : " << maxH << std::endl;
    
    GLuint *indices = new GLuint[3*2*this->conf.bpc*this->conf.bpc];
    for(int xi=0;xi<this->conf.bpc;++xi){
        for(int yi=0;yi<this->conf.bpc;++yi){
            // Position index
            int i = xi*this->conf.bpc + yi;
            int ip = xi*samples + yi;

            indices[i*6 + 0] = ip;
            indices[i*6 + 1] = ip+samples;
            indices[i*6 + 2] = ip+1;

            indices[i*6 + 3] = ip+samples;
            indices[i*6 + 4] = ip+samples+1;
            indices[i*6 + 5] = ip+1;
        }
    }

    Chunk *chunk = new Chunk();
    chunk->x = x;
    chunk->y = y;
    chunk->renderable.init();
    chunk->renderable.setVBOLayout(this->vbLayout);
    NRA::VGL::Mesh<GLuint> mesh = NRA::VGL::Mesh<GLuint>(sizeof(Ground::vertex)/4);
    mesh.add(positions,indices,samples*samples,3*2*this->conf.bpc*this->conf.bpc);

    chunk->renderable.loadMesh(mesh);
    delete[] positions;
    delete[] indices;

    this->chunks.push_back(chunk);
}
Ground::~Ground(){
    for(int i=0;i<this->chunks.size();++i){
        delete this->chunks.at(i);
    }
}

void Ground::render(NRA::VGL::Shader &shader){
    for(Chunk *c : this->chunks){
        glm::mat4 modelMat = glm::translate(glm::mat4(1.0f), glm::vec3(c->x+this->conf.originX, this->conf.originY, c->y+this->conf.originZ));
        shader.setUniformMat<4>("U_mMat", &modelMat[0][0]);
        c->renderable.bindBuffers();
        c->renderable.render();
    }
}