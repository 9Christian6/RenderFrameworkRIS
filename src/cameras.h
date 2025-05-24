#ifndef CAMERA_H
#define CAMERA_H

#include "float3.h"
#include "float2.h"
#include "intersect.h"

/// Structure that holds the local geometry information on a camera lens.
struct CameraGeometry {
    float cos;      ///< Cosine between the local camera direction and the image plane normal
    float dist;     ///< Distance between the camera and the point on the image plane
    float area;     ///< Local pixel area divided by total area
    CameraGeometry() {}
    CameraGeometry(float c, float d, float a)
        : cos(c), dist(d), area(a)
    {}
};

/// Base class for cameras.
class Camera {
public:
    virtual ~Camera() {}
    /// Generates a ray for a point on the image plane, represented by (u, v) in [-1,1]^2.
    virtual Ray gen_ray(float u, float v) const = 0;
    /// Projects a point onto the image plane and returns the corresponding (u, v, z) coordinates.
    virtual float3 project(const float3& p) const = 0;
    /// Unprojects a point on the image plane, represented by (u, v, z) with (u, v) in [-z, z]^2 and z in [0, inf[.
    virtual float3 unproject(const float3& p) const = 0;
    /// Returns the geometry at a given point on the image plane.
    virtual CameraGeometry geometry(float u, float v) const = 0;
    /// Updates the camera after mouse input.
    virtual void mouse_motion(float x, float y) = 0;
    /// Updates the camera after keyboard input.
    virtual void keyboard_motion(float x, float y, float z) = 0;
};

/// A perspective camera, defined by the position of the eye, the point to look at,
/// an up vector, a field of view, and a width/height ratio.
class PerspectiveCamera : public Camera {
public:
    PerspectiveCamera(const float3& e, const float3& c, const float3& u, float fov, float ratio) {
        eye = e;
        dir = normalize(c - e);
        right = normalize(cross(dir, u));
        up = normalize(cross(right, dir));
        
        w = std::tan(fov * pi / 360.0f);
        h = w / ratio;
        right *= w;
        up *= h;
    }

    Ray gen_ray(float u, float v) const override final {
        return Ray(eye, normalize(dir + u * right + v * up));
    }

    float3 project(const float3& p) const override final {
        auto d = normalize(p - eye);
        return float3(dot(d, right) / (w * w), dot(d, up) / (h * h), dot(d, dir));
    }

    float3 unproject(const float3&) const override final {
        return eye;
    }

    CameraGeometry geometry(float u, float v) const override final {
        float d = std::sqrt(1.0f + u * u * w * w + v * v * h * h);
        return CameraGeometry(1.0f / d, d, 1.0f / (4.0f * w * h));
    }

    void mouse_motion(float x, float y) override final {
        dir = rotate(dir, right, -y);
        dir = rotate(dir, up,    -x);
        dir = normalize(dir);
        right = normalize(cross(dir, up));
        up = normalize(cross(right, dir));

        right *= w;
        up *= h;
    }

    void keyboard_motion(float x, float y, float z) override final {
        eye += dir * z + right * x + up * y;
    }

private:
    float3 eye;
    float3 dir;
    float3 up;
    float3 right;
    float w, h;
    float area;
};

#endif // CAMERA_H
