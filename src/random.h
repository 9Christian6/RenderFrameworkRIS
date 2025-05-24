#ifndef RANDOM_H
#define RANDOM_H

#include "common.h"
#include "float3.h"
#include "color.h"

#include <algorithm>

/// Local coordinates for shading.
struct LocalCoords {
    float3 n;           ///< Normal
    float3 t;           ///< Tangent
    float3 bt;          ///< Bitangent
    LocalCoords() {}
    LocalCoords(const float3& n, const float3& t, const float3& bt) : n(n), t(t), bt(bt) {}
};

/// Generates local coordinates given a normal vector.
inline LocalCoords gen_local_coords(const float3& n) {
    // From "Building an Orthonormal Basis, Revisited", Duff et al.
    const float sign = copysignf(1.0f, n.z);
    const float a = -1.0f / (sign + n.z);
    const float b = n.x * n.y * a;
    float3 t (1.0f + sign * n.x* n.x * a, sign * b, -sign * n.x);
    float3 bt(b, sign + n.y * n.y * a, -n.y);
    return LocalCoords(n, t, bt);
}

/// Direction sample, from sampling a set of directions.
struct DirSample {
    float3 dir;
    float pdf;
    DirSample() {}
    DirSample(const float3& d, float p) : dir(d), pdf(p) {}
};

/// Evaluates the probability to sample a direction on a uniform sphere.
inline float uniform_sphere_pdf() {
    return 1.0f / (4.0f * pi);
}

/// Samples a sphere uniformly.
inline DirSample sample_uniform_sphere(float u, float v) {
    const float c = 2.0f * v - 1.0f;
    const float s = std::sqrt(1.0f - c * c);
    const float phi = 2.0f * pi * u;
    const float x = s * std::cos(phi);
    const float y = s * std::sin(phi);
    const float z = c;
    return DirSample(float3(x, y, z), uniform_sphere_pdf());
}

/// Evaluates the probability to sample a direction on a cosine-weighted hemisphere.
inline float cosine_hemisphere_pdf(float c) {
    // TODO: "c" is the cosine of the direction.
    // You should return the corresponding pdf.
    return 1.0f; // <--- This is probably incorrect
    return c / pi;
    return 1.0f; // <--- This is probably incorrect
}

/// Samples a hemisphere proportionally to the cosine with the normal.
inline DirSample sample_cosine_hemisphere(const LocalCoords& coords, float u, float v) {
    // TODO: Sample a direction on the hemisphere using a pdf proportional to cos(theta).
    // The hemisphere is defined by the coordinate system "coords".
    // "u" and "v" are random numbers between [0, 1].

    float r = sqrtf(u);       // radial component
    float theta = 2.0f * M_PI * v;  // angular component

    float x = r * cosf(theta);
    float y = r * sinf(theta);

    // Step 2: Compute z so that the vector is on the hemisphere
    float z = sqrtf(1.0f - x*x - y*y); // This ensures normalization

    // Step 3: Construct direction in local coordinates
    float3 dir(x, y, z);

    // PDF is cos(theta) / PI, since it's cosine-weighted hemisphere sampling
    float pdf = z / pi;

    return DirSample(dir, pdf);
    //return DirSample(float3(1, 0, 0), 1.0f); // <--- This is probably incorrect
}

/// Evaluates the probability to sample a direction on a power-cosine-weighted hemisphere.
inline float cosine_power_hemisphere_pdf(float c, float k) {
    // TODO: "c" is the cosine of the direction, and k is the power.
    // You should return the corresponding pdf.
    return (k + 1.0f) / (2.0f * M_PI) * powf(c, k);
    return 1.0f; // <--- This is probably incorrect
}

/// Samples a hemisphere proportionally to the cosine lobe spanned by the normal.
inline DirSample sample_cosine_power_hemisphere(const LocalCoords& coords, float k, float u, float v) {
    // TODO: Sample a direction on the hemisphere using a pdf proportional to cos(theta)^k.
    // The hemisphere is defined by the coordinate system "coords".
    // "u" and "v" are random numbers between [0, 1].

    // Handle degenerate case: if k is infinity (perfect specular), return the normal
    if (std::isinf(k)) {
        float3 dir_local(0, 0, 1);
        return DirSample(dir_local, 1.0f); // PDF is infinite but treated specially in practice
    }

    // Compute cos(theta) using inverse transform sampling
    float cos_theta = powf(u, 1.0f / (k + 1.0f));
    float sin_theta = sqrtf(1.0f - cos_theta * cos_theta);

    // Sample phi uniformly in [0, 2*pi]
    float phi = 2.0f * M_PI * v;

    // Convert to Cartesian coordinates in local frame (z-up)
    float x = cosf(phi) * sin_theta;
    float y = sinf(phi) * sin_theta;
    float z = cos_theta;

    float3 dir_local(x, y, z);

    // Transform direction to world space

    // Compute PDF: (k + 1)/(2*M_PI) * pow(cos(theta), k)
    float pdf = ((k + 1.0f) / (2.0f * M_PI)) * powf(z, k);

    return DirSample(dir_local, pdf);

    return DirSample(float3(1, 0, 0), 1.0f); // <--- This is probably incorrect
}

/// Returns the survival probability of a path, given its contribution.
/// \param c   the contribution of the path
/// \param max the maximum survival probability allowed
inline float russian_roulette(const rgb& c, float max = 0.75f) {
    return std::min(max, dot(c, luminance) * 2.0f);
}

#endif // RANDOM_H
