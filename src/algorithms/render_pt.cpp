#include "../scene.h"
#include "../color.h"
#include "../samplers.h"
#include "../cameras.h"
#include "../hash.h"
#include "../debug.h"
#include "../renderer.h"

/// Path Tracing with MIS and Russian Roulette.
class PathTracingRenderer : public Renderer {
public:
    PathTracingRenderer(const Scene& scene, size_t max_path_len)
        : Renderer(scene), max_path_len(max_path_len)
    {}

    std::string name() const override { return "pt"; }

    void reset() override { iter = 1; }

    void render(Image& img) override {
        auto kx = 2.0f / (img.width - 1);
        auto ky = 2.0f / (img.height - 1);

        process_tiles(0, 0, img.width, img.height,
            default_tile_width, default_tile_height,
            [&] (size_t xmin, size_t ymin, size_t xmax, size_t ymax) {
            UniformSampler sampler(sampler_seed(xmin ^ ymin, iter));
            for (size_t y = ymin; y < ymax; y++) {
                for (size_t x = xmin; x < xmax; x++) {
                    auto ray = scene.camera->gen_ray(
                        (x + sampler()) * kx - 1.0f,
                        1.0f - (y + sampler()) * ky);

                    debug_raster(x, y);
                    img(x, y) += rgba(path_trace(ray, sampler), 1.0f);
                }
            }
        });
        iter++;
    }

    inline rgb path_trace(Ray ray, Sampler& sampler);

private:
    size_t max_path_len;
    size_t iter;
};

rgb PathTracingRenderer::path_trace(Ray ray, Sampler& sampler) {
    rgb color(0.0f);

    ray.tmin = offset;
    for (size_t path_len = 0; path_len < max_path_len; path_len++) {
        Hit hit = scene.intersect(ray);
        if (hit.tri < 0) break;

        auto surf = scene.surface_params(ray, hit);
        auto mat = scene.material(hit);
        auto out = -ray.dir;

        if (auto light = mat.emitter) {
            // Direct hits on a light source
            if (surf.entering) {
                // DONE: compute the incoming radiance from the emitter.
		color = mat.emitter->emission(ray.dir, hit.u, hit.v).intensity;
		mat.bsdf->sample(sampler, surf, out);
            }
        }
        // Materials without BSDFs act like black bodies
        if (!mat.bsdf) break;

        bool specular = mat.bsdf->type() == Bsdf::Type::Specular;

        // TODO: Evaluate direct lighting
        // When using Next Event Estimation, you should select a light in the scene
        // (uniformly, for now) and compute direct illumination from it.
        // Important notes:
        //   - Weight the contribution of one light with the probability of choosing that light,
        //   - Be careful when combining Next Event Estimation with BRDF sampling (see assignment),
        //   - Do not take lights that face away the surface at the hit point into account.

        // TODO: Add Russian Roulette to prevent infinite recursion

        // TODO:
        // If the path is not terminated, sample a direction to continue the path with from the material.
        // You can do this using the sample() function of the Bsdf class. Update the path weight, and do not
        // forget to take the Russian Roulette probability into account!

        break; // TODO: Remove this

        // General remarks:
        //   - The algorithm is currently expressed in an iterative form.
        //     If you do not feel confident with it, feel free to use a recursive form instead.
        //   - Because this function is called in parallel, make sure you avoid data races.
        //     Disable OpenMP if you are encountering any issue.
        //   - The emission() function in the Light class is required to evaluate the radiance when hitting a light source.
        //     Of all the fields in the EmissionValue structure, you will only need two: pdf_area (only for MIS) and intensity.
        //   - The sample_direct() function in the Light class is required to evaluate direct illumination at a given point.
        //     You will only need the pdf_area (only for MIS), cos, pos, and intensity fields.
    }
    return color;
}

std::unique_ptr<Renderer> create_pt_renderer(const Scene& scene, size_t max_path_len) {
    return std::unique_ptr<Renderer>(new PathTracingRenderer(scene, max_path_len));
}
