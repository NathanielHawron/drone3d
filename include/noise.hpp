#pragma once
#include <cstdint>

template <class T>
T falloff(T n){
    T t = std::max((2.0/3.0)-n,0.0);
    return t*t*t*t;
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
template <class T = double>
std::pair<T, T> defaultNoiseHash(int x, int y, uint64_t seed){
    uint64_t h0 = hash64shift(((uint64_t)x<<32) | y);
    h0 ^= seed;
    h0 = hash64shift(h0);
    double theta = ((h0%2048) / 1024.0) * std::acos(-1);
    return {std::sin(theta), std::cos(theta)};
}


template <class T = double>
T noise2d(T x, T y, uint64_t seed, std::pair<T,T>(*hashFunction)(int x, int y, uint64_t seed) = defaultNoiseHash<T>){
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

    // Sum contributions * dot with lattice's vector / range of function
    return (a0*d0 + a1*d1 + a2*d2) / (2.0*0.0271275);
}