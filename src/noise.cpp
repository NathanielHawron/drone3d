#include "noise.hpp"

#include <algorithm>
#include <cmath>

template <class T>
T falloff(T n){
    T t = std::max((2.0/3.0)-n,0.0);
    return t*t*t*t;
}
template <class T>
T dfalloff(T n){
    T t = std::max((2.0/3.0)-n,0.0);
    return -4.0*t*t*t;
}

uint64_t hash64shift(uint64_t key){
  key = (~key) + (key << 21);
  key = key ^ (key >> 24);
  key = (key + (key << 3)) + (key << 8); // key * 265
  key = key ^ (key >> 14);
  key = (key + (key << 2)) + (key << 4); // key * 21
  key = key ^ (key >> 28);
  key = key + (key << 31);
  return key;
}
template <class T>
std::pair<T, T> defaultNoiseHash(int x, int y, uint64_t seed){
    uint32_t *xr = (uint32_t*)&x;
    uint32_t *yr = (uint32_t*)&y;
    uint64_t h0 = hash64shift((((uint64_t)*xr)<<32) | (uint64_t)*yr);
    h0 ^= seed;
    h0 = hash64shift(h0);
    double theta = (((double)(h0%(1<<16))) / (double)(1<<15)) * std::acos(-1);
    double scale = ((double)h0 / (double)(UINT64_MAX));
    scale = std::powf(scale, 0.4);
    return {scale*std::sin(theta), scale*std::cos(theta)};
}

template <class T>
T noise2d(T x, T y, uint64_t seed, T *dx, T *dy, std::pair<T,T>(*hashFunction)(int x, int y, uint64_t seed)){
    const static T SKEW = (std::sqrt(3)-1.0)/2.0;
    const static T UNSKEW = (std::sqrt(3)-3.0)/6.0;

    T skewVec = (x+y)*SKEW;
    // Skew into simplex space
    T sw = x+skewVec, sy = y+skewVec;
    // Calculate lattice integer
    int cx = std::floor(sw), cy = std::floor(sy);
    // Get relative position in skew space
    T ix = sw-cx, iy = sy-cy;
    T unskewVec = (ix+iy)*UNSKEW;
    // Unskew into cartesian space
    T nx = ix+unskewVec, ny = iy+unskewVec;

    // Calculate distance from unskewed point to unskewed lattice points, and dot product of lattice vectors
    // a0: gets unskewed back to origin
    std::pair<T,T> g0 = defaultNoiseHash(cx, cy, seed);
    T a0 = falloff(nx*nx+ny*ny);
    T d0 = nx*g0.first + ny*g0.second;

    std::pair<T,T> g1;
    T L01 = UNSKEW;
    T L10 = UNSKEW;
    if(nx < ny){
        g1 = defaultNoiseHash(cx, cy+1, seed);
        L01 += 1.0;
    }else{
        g1 = defaultNoiseHash(cx+1, cy, seed);
        L10 += 1.0;
    }
    T a1 = falloff(((L10-nx)*(L10-nx)) + ((L01-ny)*(L01-ny)));
    T d1 = (nx-L10)*g1.first + (ny-L01)*g1.second;

    // Since lattice +1, +1 is along y=x line, use same number for x and y components
    std::pair<T,T> g2 = defaultNoiseHash(cx+1, cy+1, seed);
    T L11 = 1.0+2.0*UNSKEW;
    T a2 = falloff(((L11-nx)*(L11-nx)) + ((L11-ny)*(L11-ny)));
    T d2 = (nx-L11)*g2.first + (ny-L11)*g2.second;

    if(dx != nullptr){
        // d(an * dn) / dnx
        double dx0 = 2.0*nx*dfalloff(nx*nx+ny*ny)*d0 + a0*g0.first;
        double dx1 = 2.0*(nx-L10)*dfalloff(((L10-nx)*(L10-nx)) + ((L01-ny)*(L01-ny)))*d1 + a1*g1.first;
        double dx2 = 2.0*(nx-L11)*dfalloff(((L11-nx)*(L11-nx)) + ((L11-ny)*(L11-ny)))*d2 + a2*g2.first;
        *dx += (dx0+dx1+dx2) /(2.0*0.0271275);
    }
    if(dy != nullptr){
        // d(an * dn) / dny
        double dy0 = 2.0*ny*dfalloff(nx*nx+ny*ny)*d0 + a0*g0.second;
        double dy1 = 2.0*(ny-L01)*dfalloff(((L10-nx)*(L10-nx)) + ((L01-ny)*(L01-ny)))*d1 + a1*g1.second;
        double dy2 = 2.0*(ny-L11)*dfalloff(((L11-nx)*(L11-nx)) + ((L11-ny)*(L11-ny)))*d2 + a2*g2.second;
        *dy += (dy0+dy1+dy2) /(2.0*0.0271275);
    }

    // Sum contributions * dot with lattice's vector / range of function
    return (a0*d0 + a1*d1 + a2*d2) / (2.0*0.0271275);
}
#include <iostream>
template <class T>
T fractalNoise2d(T x, T y, uint64_t seed, T *dx, T *dy, std::pair<T,T>(*hashFunction)(int x, int y, uint64_t seed), FBN_params params){
    T res = 0.0;
    double scale = params.scale;
    double amplitude = params.amplitude;
    T dxt = 0.0, dyt = 0.0;
    if(dx == nullptr){
        dx = &dxt;
    }
    if(dy == nullptr){
        dy = &dyt;
    }
    for(std::size_t l=0;l<params.levels;++l){
        T dxl = 0.0;
        T dyl = 0.0;
        res += amplitude*noise2d(x*scale, y*scale, seed++, &dxl, &dyl, hashFunction);
        *dx += amplitude*scale*dxl;
        *dy += amplitude*scale*dyl;
        scale *= params.dScale;
        amplitude *= params.dAmp;
    }
    return res;
}

template double fractalNoise2d<double>(double x, double y, uint64_t seed, double *dx, double *dy, std::pair<double,double>(*hashFunction)(int x, int y, uint64_t seed), FBN_params params);