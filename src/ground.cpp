#include "ground.hpp"
#include "noise.hpp"


Ground::Ground(NRA::VGL::VertexBufferLayout vbLayout, std::size_t size, std::size_t seed, double scale, double offsetX, double offsetY){
    struct vertex{
        GLfloat pos[3];
        GLfloat tex[2];
        GLfloat norm[3];
    };

    double maxH = 0;
    double minH = 100.0;
    vertex* positions = new vertex[(size+1)*(size+1)];
    for(int x=0;x<=size;++x){
        // Step size
        float dx = scale / size;
        float dy = scale / size;
        // Texture step size
        float dtx = 1.0f/size;
        float dty = 1.0f/size;

        for(int y=0;y<=size;++y){
            // Position index
            int i = x*(size+1) + y;
            // Base relative position (first corner)
            float rx = -scale*0.5f + x*dx + offsetX;
            float ry = -scale*0.5f + y*dy + offsetY;
            // Base relative texture
            float tx = x * dtx;
            float ty = y * dty;

            double h = noise2d(x/10.0, y/10.0, seed);
            if(h > maxH){maxH = h;}
            if(h < minH){minH = h;}

            positions[i] = {{rx,h,ry}, {tx, ty}, {0.0, 1.0, 0.0}};
        }
    }
    //std::cout << minH << ":" << maxH << std::endl;
    
    GLuint *indices = new GLuint[3*2*size*size];
    for(int x=0;x<size;++x){
        for(int y=0;y<size;++y){
            // Position index
            int i = x*size + y;
            int ip = x*(size+1) + y;

            indices[i*6 + 0] = ip;
            indices[i*6 + 1] = ip+size+1;
            indices[i*6 + 2] = ip+1;

            indices[i*6 + 3] = ip+size+1;
            indices[i*6 + 4] = ip+size+2;
            indices[i*6 + 5] = ip+1;
        }
    }

    this->renderable.init();
    this->renderable.setVBOLayout(vbLayout);
    NRA::VGL::Mesh<GLuint> mesh = NRA::VGL::Mesh<GLuint>(sizeof(vertex)/4);
    mesh.add(positions,indices,(size+1)*(size+1),3*2*size*size);

    renderable.loadMesh(mesh);
    delete[] positions;
    delete[] indices;
}

void Ground::render(NRA::VGL::Shader &shader){
    this->renderable.bindBuffers();
    this->renderable.render();
}