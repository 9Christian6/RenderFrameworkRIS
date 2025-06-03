#include "../scene.h"
#include "../color.h"
#include "../samplers.h"
#include "../cameras.h"
#include "../hash.h"
#include "../debug.h"
#include "../renderer.h"

/// Path Tracing with MIS and Russian Roulette.
class PathTracingRenderer : public Renderer
{
public:
    PathTracingRenderer(const Scene &scene, size_t max_path_len)
        : Renderer(scene), max_path_len(max_path_len)
    {
    }

    std::string name() const override { return "pt"; }

    void reset() override { iter = 1; }

    void render(Image &img) override
    {
        auto kx = 2.0f / (img.width - 1);
        auto ky = 2.0f / (img.height - 1);

        process_tiles(0, 0, img.width, img.height,
                      default_tile_width, default_tile_height,
                      [&](size_t xmin, size_t ymin, size_t xmax, size_t ymax)
                      {
                          UniformSampler sampler(sampler_seed(xmin ^ ymin, iter));
                          for (size_t y = ymin; y < ymax; y++)
                          {
                              for (size_t x = xmin; x < xmax; x++)
                              {
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

    inline rgb path_trace(Ray ray, Sampler &sampler);

private:
    size_t max_path_len;
    size_t iter;
};

rgb PathTracingRenderer::path_trace(Ray ray, Sampler &sampler)
{
    rgb color(0.0f);
    rgb throughput(1.0f);

    ray.tmin = offset;
    for (size_t path_len = 0; path_len < max_path_len; path_len++)
    {
        Hit hit = scene.intersect(ray);
        if (hit.tri < 0)
            break;

        auto surf = scene.surface_params(ray, hit);
        auto mat = scene.material(hit);
        auto out = -ray.dir;
        if (auto light = mat.emitter)
        {
            // Direct hits on a light source
            if (surf.entering)
            {
                auto light_sample = light->emission(out, hit.u, hit.v);
                color += throughput * light_sample.intensity;
            }
        }

        // Materials without BSDFs act like black bodies
        if (!mat.bsdf)
            break;

        bool specular = mat.bsdf->type() == Bsdf::Type::Specular;

        // Evaluate direct lighting using Next Event Estimation (NEE)
        if (!specular && !scene.lights.empty())
        {
            // Randomly select a light source
            size_t light_idx = size_t(sampler() * scene.lights.size());
            float light_select_prob = 1.0f / scene.lights.size();

            // Sample direct illumination from the selected light
            auto light_sample = scene.lights[light_idx]->sample_direct(surf.point, sampler);
            auto light_dir = normalize(light_sample.pos - surf.point);
            float dist = length(light_sample.pos - surf.point);

            // Check visibility
            Ray shadow_ray(surf.point, light_dir, offset, dist - offset);
            if (!scene.occluded(shadow_ray))
            {
                // Evaluate BSDF for the light direction
                auto bsdf_val = mat.bsdf->eval(light_dir, surf, out);
                float bsdf_pdf = mat.bsdf->pdf(light_dir, surf, out);

                // Multiple Importance Sampling weight
                float light_pdf = light_sample.pdf_area * dist * dist / std::abs(dot(light_dir, light_sample.pos - surf.point));
                float mis_weight = light_pdf / (light_pdf + bsdf_pdf);

                // Add contribution
                float cos_theta = std::abs(dot(light_dir, surf.coords.n));
                color += throughput * bsdf_val * light_sample.intensity * mis_weight / (light_pdf * light_select_prob);
            }
        }

        // Russian Roulette for path termination
        if (path_len > 3)
        {
            float rr_prob = std::min(0.95f, std::max(throughput.x, std::max(throughput.y, throughput.z)));
            if (sampler() > rr_prob)
                break;
            throughput = rgb(throughput.x / rr_prob, throughput.y / rr_prob, throughput.z / rr_prob);
        }

        // Sample new direction from BSDF
        auto bsdf_sample = mat.bsdf->sample(sampler, surf, out);
        if (bsdf_sample.pdf <= 0.0f)
            break;

        // Update throughput and ray
        float cos_theta = std::abs(dot(bsdf_sample.in, surf.coords.n));
        throughput *= bsdf_sample.color * cos_theta / bsdf_sample.pdf;
        ray = Ray(surf.point, bsdf_sample.in, offset);
    }
    return color;
}

std::unique_ptr<Renderer> create_pt_renderer(const Scene &scene, size_t max_path_len)
{
    return std::unique_ptr<Renderer>(new PathTracingRenderer(scene, max_path_len));
}
