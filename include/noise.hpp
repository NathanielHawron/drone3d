#pragma once
#include <cstdint>
#include <utility>

uint64_t hash64shift(uint64_t key);

template <class T = double>
std::pair<T, T> defaultNoiseHash(int x, int y, uint64_t seed);

template <class T = double>
T noise2d(T x, T y, uint64_t seed, T *dx = nullptr, T *dy = nullptr, std::pair<T,T>(*hashFunction)(int x, int y, uint64_t seed) = defaultNoiseHash<T>);

struct FBN_params{
    double scale = 0.1;
    double amplitude = 1.0/(1.0+0.5+0.25);
    std::size_t levels = 3;
    double dScale = 2.0;
    double dAmp = 0.5;
    FBN_params(){}
};
template <class T = double>
T fractalNoise2d(T x, T y, uint64_t seed, T *dx = nullptr, T *dy = nullptr, std::pair<T,T>(*hashFunction)(int x, int y, uint64_t seed) = defaultNoiseHash<T>, FBN_params params = {});