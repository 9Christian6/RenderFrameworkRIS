#ifndef INTERSECT_H
#define INTERSECT_H

#include <cfloat>
#include <cstdint>

#include "float4.h"
#include "float3.h"

/// Ray defined as org + t * dir, with t in [tmin, tmax].
struct Ray {
    float3 org;     ///< Origin of the ray
    float tmin;     ///< Minimum t parameter
    float3 dir;     ///< Direction of the ray
    float tmax;     ///< Maximum t parameter

    Ray()
        : org(0.0f), tmin(0.0f), dir(0.0f), tmax(0.0f)
    {}

    Ray(const float3& o, const float3& d,
        float t0 = 0.0f, float t1 = FLT_MAX)
        : org(o), tmin(t0), dir(d), tmax(t1)
    {}
};

/// Ray-triangle hit information.
struct Hit {
    int32_t tri;    ///< Triangle index, or -1 if no intersection was found
    float t;        ///< Time of intersection
    float u;        ///< First barycentric coordinate
    float v;        ///< Second barycentric coordinate

    Hit()
        : tri(-1), t(0.0f), u(0.0f), v(0.0f)
    {}

    Hit(int32_t tri, float t, float u, float v)
        : tri(tri), t(t), u(u), v(v)
    {}
};

/// Precomputed triangle structure to accelerate ray-scene intersections.
struct PrecomputedTri {
    float3 v0; float nx;
    float3 e1; float ny;
    float3 e2; float nz;

    PrecomputedTri() {}
    PrecomputedTri(const float3& v0, const float3& v1, const float3& v2)
        : v0(v0), e1(v0 - v1), e2(v2 - v0)
    {
        auto n = cross(e1, e2);
        nx = n.x;
        ny = n.y;
        nz = n.z;
    }
};

/// Intersects a ray with a precomputed triangle, using a Moeller-Trumbore test.
inline bool intersect_ray_tri(const Ray& ray, const PrecomputedTri& tri, float& t, float& u, float& v) {
    const float eps = 1e-9f;
    float3 n(tri.nx, tri.ny, tri.nz);

    auto c = tri.v0 - ray.org;
    auto r = cross(ray.dir, c);
    auto det = dot(n, ray.dir);
    auto abs_det = std::fabs(det);

    auto u_ = prodsign(dot(r, tri.e2), det);
    auto v_ = prodsign(dot(r, tri.e1), det);
    auto w_ = abs_det - u_ - v_;

    if (u_ >= -eps && v_ >= -eps && w_ >= -eps) {
        auto t_ = prodsign(dot(n, c), det);
        if (t_ >= abs_det * ray.tmin && abs_det * t > t_) {
            auto inv_det = 1.0f / abs_det;
            t = t_ * inv_det;
            u = u_ * inv_det;
            v = v_ * inv_det;
            return true;
        }
    }

    return false;
}

#endif // INTERSECT_H
