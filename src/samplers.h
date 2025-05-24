#ifndef SAMPLERS_H
#define SAMPLERS_H

#include <cmath>
#include <random>

#include "float3.h"
#include "random.h"
#include "common.h"

/// Sampler object, used at the level of the integrator to control how the random number generation is done.
class Sampler {
public:
    virtual ~Sampler() {}
    virtual float operator () () = 0;
};

/// Uniform sampler, the easiest and most simple sampling method.
class UniformSampler : public Sampler {
public:
    UniformSampler() : uniform01(0, 1) {}

    UniformSampler(uint32_t seed)
        : UniformSampler()
    {
        rnd_gen.seed(seed);
    }

    float operator () () override { return uniform01(rnd_gen); }

private:
    std::mt19937 rnd_gen;
    std::uniform_real_distribution<float> uniform01;
};

#endif // SAMPLERS_H
