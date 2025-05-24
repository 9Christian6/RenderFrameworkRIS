#include <iostream>
#include "../scene.h"
#include "../color.h"
#include "../samplers.h"
#include "../cameras.h"
#include "../hash.h"
#include "../debug.h"
#include "../renderer.h"

class DebugRenderer : public Renderer {
public:
    DebugRenderer(const Scene& scene)
        : Renderer(scene)
    {}

    std::string name() const override { return "debug"; }

    void reset() override { iter = 1; }

    void render(Image& img) {
        auto kx = 2.0f / (img.width - 1);
        auto ky = 2.0f / (img.height - 1);
        process_tiles(0, 0, img.width, img.height,
            default_tile_width, default_tile_height,
            [&] (size_t xmin, size_t ymin, size_t xmax, size_t ymax) {
            UniformSampler sampler(sampler_seed(xmin ^ ymin, iter));
            for (size_t y = ymin; y < ymax; ++y) {
                for (size_t x = xmin; x < xmax; ++x) {
                    auto ray = scene.camera->gen_ray(
                        (x + sampler()) * kx - 1.0f,
                        1.0f - (y + sampler()) * ky);
                    Hit hit = scene.intersect(ray);

                    rgba color(0.0f);
                    if (hit.tri >= 0) {
                        auto n0 = scene.normals[scene.indices[hit.tri * 4 + 0]];
                        auto n1 = scene.normals[scene.indices[hit.tri * 4 + 1]];
                        auto n2 = scene.normals[scene.indices[hit.tri * 4 + 2]];
                        auto n = normalize(lerp(n0, n1, n2, hit.u, hit.v));
                        auto k = fabsf(dot(n, ray.dir));
                        color = rgba(k, k, k, 1.0f);
                    }

                    img(x, y) += color;
                }
            }
        });
        iter++;
    }

private:
    size_t iter;
};

std::unique_ptr<Renderer> create_debug_renderer(const Scene& scene) {
    return std::unique_ptr<Renderer>(new DebugRenderer(scene));
}
