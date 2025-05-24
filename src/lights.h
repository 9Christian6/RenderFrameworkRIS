#ifndef LIGHTS_H
#define LIGHTS_H

#include "color.h"
#include "float3.h"
#include "samplers.h"

/// Result from sampling a light source.
struct EmissionSample {
    float3 pos;         ///< Position on the light source
    float3 dir;         ///< Direction of the light going outwards
    rgb intensity;      ///< Intensity along the direction
    float pdf_area;     ///< Probability to sample the point on the light
    float pdf_dir;      ///< Probability to sample the direction on the light, conditioned on the point on the light source
    float cos;          ///< Cosine between the direction and the light source geometry

    EmissionSample() {}
    EmissionSample(const float3& p, const float3& d, const rgb& i, float pa, float pd, float c)
        : pos(p), dir(d), intensity(i), pdf_area(pa), pdf_dir(pd), cos(c)
    {}
};

/// Result from sampling direct lighting from a light source.
struct DirectLightingSample {
    float3 pos;         ///< Position on the light source
    rgb intensity;      ///< Intensity along the direction
    float pdf_area;     ///< Probability to sample the point on the light
    float pdf_dir;      ///< Probability to sample the direction using emission sampling
    float cos;          ///< Cosine between the direction and the light source geometry

    DirectLightingSample() {}
    DirectLightingSample(const float3& p, const rgb& i, float pa, float pd, float c)
        : pos(p), intensity(i), pdf_area(pa), pdf_dir(pd), cos(c)
    {}
};

/// Emission value at a given point on the light surface.
struct EmissionValue {
    rgb intensity;      ///< Intensity along the direction
    float pdf_area;     ///< Probability to sample the point on the light
    float pdf_dir;      ///< Probability to sample the direction using emission sampling

    EmissionValue() {}
    EmissionValue(const rgb& i, float pa, float pd)
        : intensity(i), pdf_area(pa), pdf_dir(pd)
    {}
};

/// Base class for all lights.
class Light {
public:
    virtual ~Light() {}

    /// Samples direct illumination from this light source at the given point on a surface.
    virtual DirectLightingSample sample_direct(const float3& from, Sampler& sampler) const = 0;

    /// Samples the emitting surface of the light.
    virtual EmissionSample sample_emission(Sampler& sampler) const = 0;

    /// Returns the emission of a light source (only for light sources with an area).
    virtual EmissionValue emission(const float3& dir, float u, float v) const = 0;

    /// Returns true if the light has an area (i.e. can be hit by a ray).
    virtual bool has_area() const = 0;

protected:
    EmissionSample make_emission_sample(const float3& pos, const float3& dir, const rgb& intensity, float pdf_area, float pdf_dir, float cos) const {
        return pdf_area > 0 && pdf_dir > 0 && cos > 0
               ? EmissionSample(pos, dir, intensity, pdf_area, pdf_dir, cos)
               : EmissionSample(pos, dir, rgb(0.0f), 1.0f, 1.0f, 1.0f);
    }

    DirectLightingSample make_direct_sample(const float3& pos, const rgb& intensity, float pdf_area, float pdf_dir, float cos) const {
        return pdf_area > 0 && pdf_dir > 0 && cos > 0
               ? DirectLightingSample(pos, intensity, pdf_area, pdf_dir, cos)
               : DirectLightingSample(pos, rgb(0.0f), 1.0f, 1.0f, 1.0f);
    }
};

/// Simple point light, with intensity decreasing quadratically.
class PointLight : public Light {
public:
    PointLight(const float3& p, const rgb& c) : pos(p), color(c * (1.0f / (4.0f * pi))) {}

    DirectLightingSample sample_direct(const float3&, Sampler&) const override final {
        return make_direct_sample(pos, color, 1.0f, uniform_sphere_pdf(), 1.0f);
    }

    EmissionSample sample_emission(Sampler& sampler) const override final {
        auto sample = sample_uniform_sphere(sampler(), sampler());
        return make_emission_sample(pos, sample.dir, color, 1.0f, sample.pdf, 1.0f);
    }

    EmissionValue emission(const float3& /*dir*/, float /*u*/, float /*v*/) const override final {
        return EmissionValue(rgb(0.0f), 1.0f, 1.0f);
    }

    bool has_area() const override final {
        return false;
    }

private:
    float3 pos;
    rgb color;
};

/// Triangle light source, useful to represent area lights made of meshes.
class TriangleLight : public Light {
public:
    TriangleLight(const float3& v0, const float3& v1, const float3& v2, const rgb& c)
        : v0(v0), v1(v1), v2(v2), color(c)
    {
        n = cross(v1 - v0, v2 - v0);
        auto len = length(n);
        auto area = len * 0.5f;
        inv_area = 1.0f / area;
        n *= inv_area * 0.5f;
    }

    DirectLightingSample sample_direct(const float3& from, Sampler& sampler) const override final {
        auto pos = sample(sampler);
        auto dir = from - pos;
        float cos = dot(dir, n) / length(dir);
        return make_direct_sample(pos, color, inv_area, cosine_hemisphere_pdf(cos), cos);
    }

    EmissionSample sample_emission(Sampler& sampler) const override final {
        auto pos = sample(sampler);
        auto sample = sample_cosine_hemisphere(gen_local_coords(n), sampler(), sampler());
        return make_emission_sample(pos, sample.dir, color, inv_area, sample.pdf, dot(sample.dir, n));
    }

    EmissionValue emission(const float3& dir, float /*u*/, float /*v*/) const override final {
        auto cos = cosine_hemisphere_pdf(dot(dir, n));
        return cos > 0
            ? EmissionValue(color, inv_area, cosine_hemisphere_pdf(dot(dir, n)))
            : EmissionValue(rgb(0.0f), 1.0f, 1.0f);
    }

    bool has_area() const override final {
        return true;
    }

private:
    float3 sample(Sampler& sampler) const {
        float u = sampler();
        float v = sampler();
        if (u + v > 1.0f) {
            u = 1.0f - u;
            v = 1.0f - v;
        }
        return lerp(v0, v1, v2, u, v);
    }

    float3 v0, v1, v2;
    float3 n;
    float inv_area;
    rgb color;
};

#endif // LIGHTS_H
