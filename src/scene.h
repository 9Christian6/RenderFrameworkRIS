#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include <memory>
#include <string>

#include "lights.h"
#include "materials.h"
#include "cameras.h"
#include "textures.h"
#include "float3.h"
#include "float2.h"
#include "bvh.h"

struct Scene {
    template <typename T>
    using unique_vector = std::vector<std::unique_ptr<T>>;

    // Camera and viewport
    std::unique_ptr<Camera>     camera;
    size_t                      width, height;

    // Shading data
    unique_vector<Bsdf>         bsdfs;
    unique_vector<Light>        lights;
    unique_vector<Texture>      textures;
    std::vector<Material>       materials;

    // Traversal data
    Bvh                         bvh;

    // Mesh data
    std::vector<float3>         vertices;
    std::vector<float2>         texcoords;
    std::vector<float3>         normals;
    std::vector<uint32_t>       indices;

    std::vector<float3>         face_normals;

    /// Returns the intersection point between a ray and the scene.
    /// If not intersection is found, hit.tri == -1.
    Hit intersect(const Ray& ray) const {
        Hit hit;
        bvh.traverse(ray, hit);
        return hit;
    }

    /// Returns true if the given ray hits the scene.
    bool occluded(const Ray& ray) const {
        Hit hit;
        bvh.traverse<true>(ray, hit);
        return hit.tri >= 0;
    }

    /// Returns the material associated with a hit point.
    const Material& material(const Hit& hit) const {
        assert(hit.tri >= 0);
        return materials[indices[hit.tri * 4 + 3]];
    }

    /// Returns the surface parameters for a hit point.
    SurfaceParams surface_params(const Ray& ray, const Hit& hit) const {
        assert(hit.tri >= 0);
        auto i0 = indices[hit.tri * 4 + 0];
        auto i1 = indices[hit.tri * 4 + 1];
        auto i2 = indices[hit.tri * 4 + 2];

        auto& fn = face_normals[hit.tri];
        auto n = normalize(lerp(normals[i0], normals[i1], normals[i2], hit.u, hit.v));
        auto uv = lerp(texcoords[i0], texcoords[i1], texcoords[i2], hit.u, hit.v);

        // Compute the surface parameters, and make sure the face and per-vertex normal agree
        SurfaceParams surf;
        surf.entering = dot(ray.dir, fn) <= 0;
        surf.face_normal = surf.entering ? fn : -fn;
        surf.point = ray.org + ray.dir * hit.t;
        surf.coords = gen_local_coords(dot(ray.dir, n) <= 0 ? n : -n);
        surf.uv = uv;
        return surf;
    }
};

/// Load a scene from the given YAML configuration file.
bool load_scene(const std::string& config, Scene& scene);

#endif // SCENE_H
