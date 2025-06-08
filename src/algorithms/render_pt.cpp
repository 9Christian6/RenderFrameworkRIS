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
                auto light_emission = light->emission(out, hit.u, hit.v);
                color += throughput * light_emission.intensity;
            }
        }

        // Materials without BSDFs act like black bodies
        if (!mat.bsdf)
            break;

        bool specular = mat.bsdf->type() == Bsdf::Type::Specular;

        float cos_theta = -3;
        // Evaluate direct lighting using Next Event Estimation (NEE)
        if (!specular && !scene.lights.empty())
        {
            // Randomly select a light source
            size_t light_idx = size_t(sampler() * scene.lights.size());
            float light_select_prob = 1.0f / scene.lights.size();

            // Sample direct illumination from the selected light
            auto light = scene.lights[light_idx].get();
            auto light_sample = light->sample_direct(surf.point, sampler);
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
                float light_pdf;
                if (light->has_area())
                {
                    // For area lights, convert area PDF to solid angle PDF
                    light_pdf = light_sample.pdf_area * dist * dist / light_sample.cos;
                }
                else
                { // PointLight
                    // For point lights, use directional PDF as solid angle PDF
                    light_pdf = light_sample.pdf_dir;
                }

                //**float mis_weight = light_pdf / (light_pdf + bsdf_pdf);** replacing this with bellow

                float sum_pdf = light_pdf + bsdf_pdf;
                float w_ne, w_brdf;
                if (sum_pdf > 0.0f)
                {
                    w_ne = light_pdf / sum_pdf;
                    w_brdf = bsdf_pdf / sum_pdf;
                }

                else
                {
                    w_ne = 0.0f;
                    w_brdf = 0.0f;
                }

                // Add contribution
                cos_theta = std::abs(dot(light_dir, surf.coords.n));

                rgb light_contribution = light_sample.intensity;
                if (!light->has_area())
                {
                    // For point lights, radiance is intensity / (dist^2)
                    light_contribution = light_contribution * (1.0f / (dist * dist));
                }

                // color += throughput * bsdf_val * light_contribution * mis_weight / (light_pdf * light_select_prob);
                //**color += throughput * bsdf_val * light_contribution * cos_theta * mis_weight / (light_pdf * light_select_prob);**

                color += throughput * bsdf_val * light_contribution * cos_theta * w_ne / (light_pdf * light_select_prob);
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

        cos_theta = std::abs(dot(bsdf_sample.in, surf.coords.n));

        // Update throughput and ray
        throughput *= bsdf_sample.color * cos_theta / bsdf_sample.pdf;
        ray = Ray(surf.point, bsdf_sample.in, offset);
    }
    return color;
}

std::unique_ptr<Renderer> create_pt_renderer(const Scene &scene, size_t max_path_len)
{
    return std::unique_ptr<Renderer>(new PathTracingRenderer(scene, max_path_len));
}
