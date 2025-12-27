#include "../scene.h"
#include "../color.h"
#include "../samplers.h"
#include "../cameras.h"
#include "../hash.h"
#include "../hash_grid.h"
#include "../debug.h"
#include "../renderer.h"

#ifdef USE_STD_THREAD
#include <mutex>
#endif

#ifdef _OPENMP
#include <omp.h>
#define OMP_THREAD_NUM() omp_get_thread_num()
#else
#define OMP_THREAD_NUM() 0
#endif

struct Photon
{
  rgb contrib;        ///< Path contribution
  SurfaceParams surf; ///< Surface parameters at the vertex
  float3 in_dir;      ///< Incoming direction

  Photon() {}
  Photon(const rgb &c, const SurfaceParams &s, const float3 &i)
    : contrib(c), surf(s), in_dir(i)
  {
  }
};

class PhotonMappingRenderer : public Renderer
{
  public:
    PhotonMappingRenderer(const Scene &scene, size_t max_path_len)
      : Renderer(scene), max_path_len(max_path_len)
    {
    }

    std::string name() const override { return "ppm"; }

    void reset() override { iter = 1; }

    void render(Image &img) override
    {
      constexpr float alpha = 0.75f;

      static float base_radius = 1.0f;
      if (iter == 1)
	base_radius = 2.0f * estimate_pixel_size(img.width, img.height);

      auto kx = 2.0f / (img.width - 1);
      auto ky = 2.0f / (img.height - 1);

      auto light_path_count = img.width * img.height;
      photons.clear();

      // Trace a light path for every pixel
#ifdef USE_STD_THREAD
      std::mutex photon_mutex;
      size_t batch_size = 32;
      parallel_for(0, light_path_count / batch_size + (light_path_count % batch_size ? 1 : 0),
	  [&](size_t batch_id)
	  {
	  std::vector<Photon> photon_buffer;
	  UniformSampler sampler(sampler_seed(batch_id, iter));

	  int num = std::min((batch_id + 1) * batch_size, light_path_count) - batch_id * batch_size;
	  for (int i = 0; i < num; ++i)
	  trace_photons(photon_buffer, sampler);

	  {
	  std::lock_guard guard(photon_mutex);
	  photons.insert(photons.end(), photon_buffer.begin(), photon_buffer.end());
	  }
	  });
#else
#pragma omp parallel
      {
	std::vector<Photon> photon_buffer;
	UniformSampler sampler(sampler_seed(OMP_THREAD_NUM(), iter));

#pragma omp for schedule(dynamic) nowait
	for (size_t i = 0; i < light_path_count; i++)
	  trace_photons(photon_buffer, sampler);

#pragma omp critical
	{
	  photons.insert(photons.end(), photon_buffer.begin(), photon_buffer.end());
	}
      }
#endif

      // Build the photon map
      radius = base_radius / std::pow(float(iter), 0.5f * (1.0f - alpha));
      photon_map.build(
	  [&](size_t i)
	  { return photons[i].surf.point; },
	  photons.size(),
	  radius);

      // Trace the eye paths
      process_tiles(0, 0, img.width, img.height,
	  default_tile_width, default_tile_height,
	  [&](size_t xmin, size_t ymin, size_t xmax, size_t ymax)
	  {
	  UniformSampler sampler(sampler_seed(xmin ^ ymin, iter));
	  for (size_t y = ymin; y < ymax; y++)
	  {
	  for (size_t x = xmin; x < xmax; x++)
	  {
	  auto ray = scene.camera->gen_ray((x + sampler()) * kx - 1.0f, 1.0f - (y + sampler()) * ky);
	  debug_raster(x, y);
	  img(x, y) += atomically(rgba(trace_eye_path(ray, sampler, light_path_count), 1.0f));
	  }
	  }
	  });
      iter++;
    }

    void trace_photons(std::vector<Photon> &photons, Sampler &sampler);
    rgb trace_eye_path(Ray ray, Sampler &sampler, size_t light_path_count);
    float estimate_pixel_size(size_t w, size_t h);

  private:
    std::vector<Photon> photons;
    HashGrid photon_map;
    size_t max_path_len;
    size_t iter;
    float radius;
};

void PhotonMappingRenderer::trace_photons(std::vector<Photon> &photons, Sampler &sampler)
{
  // Choose a light to sample from (uniformly)
  size_t light_idx = size_t(sampler() * scene.lights.size()) % scene.lights.size();
  auto &light = scene.lights[light_idx];
  float light_select_prob = 1.0f / scene.lights.size();
  EmissionSample emission = light->sample_emission(sampler);
  Ray ray(emission.pos, emission.dir, offset);
  rgb throughput = rgb(1.0f);

  // Compute total PDF for importance weighting:
  // - Area PDF (position on light)
  // - Direction PDF (direction from light)
  // - Uniform probability over lights
  float total_light_pdf = emission.pdf_area * emission.pdf_dir * light_select_prob;
  // Multiply throughput by emitted radiance (already includes any color/intensity)
  throughput *= emission.intensity / total_light_pdf;

  for (size_t path_len = 0; path_len < max_path_len; path_len++)
  {
    Hit hit = scene.intersect(ray);
    if (hit.tri < 0)
      break;
    auto mat = scene.material(hit);
    auto surf = scene.surface_params(ray, hit);
    auto out = -ray.dir;

    if (!mat.bsdf)
      break;

    if (mat.bsdf->type() == Bsdf::Type::Glossy)
    {
      auto ls = light->sample_direct(surf.point, sampler);
      auto wi = normalize(ls.pos - surf.point);
      float dist = length(ls.pos - surf.point);
      //if (dot(wi, surf.coords.n) > 0 && !scene.occluded(Ray(surf.point, wi, offset, dist - offset)))
      if (ls.cos > 0 && !scene.occluded(Ray(surf.point, wi, offset, dist - offset)))
      {
	// auto bsdf_val = mat.bsdf->eval(wi, surf, out);
	// convert area to solid‐angle or use pdf_dir for point lights
	// The light pdf is wrong here
	// float light_pdf = ls.pdf_dir;
	// if (light->has_area()){
	//   light_pdf = ls.pdf_area * dist * dist / ls.cos;
	// }
	// rgb Li = ls.intensity;
	// if (!light->has_area())
	//   Li *= (1.0f / (dist * dist));

	// add the NEE contribution (no MIS here for simplicity)
	//throughput += throughput * bsdf_val * Li * ls.cos / (light_pdf * light_select_prob);
	photons.push_back(Photon(throughput, surf, out));
      }
      // Now sample the glossy lobe and continue the eye path
      auto bs = mat.bsdf->sample(sampler, surf, out);
      if (bs.pdf <= 0.0f)
	break;
      throughput *= bs.color / bs.pdf; // bs.color includes cosine
      ray = Ray(surf.point, bs.in, offset);
      continue;
    }

    // Sample BSDF for next direction
    BsdfSample bsdf_sample = mat.bsdf->sample(sampler, surf, out);
    if (bsdf_sample.pdf <= 0.0f)
      break;

    // Store photon if surface is non-specular
    if (mat.bsdf->type() != Bsdf::Type::Specular)
    {
      photons.push_back(Photon(throughput, surf, out));
    }

    // Update throughput: multiply by BSDF/pdf
    throughput *= bsdf_sample.color / bsdf_sample.pdf;

    // Continue path
    ray = Ray(surf.point, bsdf_sample.in, offset);

    // Apply Russian Roulette after a few bounces
    if (path_len > 3)
    {
      float rr_prob = russian_roulette(throughput, 0.75f); // e.g., based on max component
      if (sampler() > rr_prob)
	break;
      throughput = throughput / rr_prob;
    }
  }
}

rgb PhotonMappingRenderer::trace_eye_path(Ray ray, Sampler &sampler, size_t light_path_count)
{
  static size_t max_path_len = 10;
  rgb color(0.0f);
  float pdf = 1.0f;
  bool lastBounceGlossy = false;

  ray.tmin = offset;
  for (size_t path_len = 0; path_len < max_path_len; path_len++)
  {
    Hit hit = scene.intersect(ray);
    if (hit.tri < 0)
      break;

    auto surf = scene.surface_params(ray, hit);
    auto mat = scene.material(hit);
    auto out = -ray.dir;
    auto cosTheta = std::abs(dot(out, surf.face_normal));

    if (auto light = mat.emitter)
    {
      // Direct hits on a light source - add the emission
      if (surf.entering && !lastBounceGlossy)
      {
	auto light_emission = light->emission(out, hit.u, hit.v);
	color += light_emission.intensity;
      }
    }

    if (!mat.bsdf)
      break;

    if (mat.bsdf->type() == Bsdf::Type::Glossy)
    {
      // (a) Compute direct illumination: sample a light
      size_t light_idx = size_t(sampler() * scene.lights.size());
      float light_select_prob = 1.0f / scene.lights.size();
      auto light = scene.lights[light_idx].get();
      auto ls = light->sample_direct(surf.point, sampler);
      auto wi = normalize(ls.pos - surf.point);
      float dist = length(ls.pos - surf.point);

      if (dot(wi, surf.coords.n) > 0 && !scene.occluded(Ray(surf.point, wi, offset, dist - offset)))
      {
	auto bsdf_val = mat.bsdf->eval(wi, surf, out);
	float light_pdf = light->has_area()
	  ? ls.pdf_area * dist * dist / ls.cos
	  : ls.pdf_dir;
	float cos_theta = std::abs(dot(wi, surf.coords.n));
	rgb Li = ls.intensity;
	if (!light->has_area())
	  Li *= (1.0f / (dist * dist));

	// add direct lighting (no MIS here, as per instruction)
	color += bsdf_val * Li * cos_theta / (light_pdf * light_select_prob);
      }

      // (b) Continue via glossy lobe
      auto bs = mat.bsdf->sample(sampler, surf, out);
      if (bs.pdf <= 0.0f)
	break;
      ray = Ray(surf.point, bs.in, offset);
      lastBounceGlossy = true;
    }

    if (mat.bsdf->type() != Bsdf::Type::Specular)
    {
      rgb accumulated = rgb(0.0f);

      photon_map.query(surf.point, [&](size_t i)
	  { return photons[i].surf.point; }, [&](size_t i, float d)
	  {
	  const Photon& p = photons[i];
	  auto hitDiff = p.surf.point - surf.point;
	  hitDiff = hitDiff / radius;
	  auto square = dot(hitDiff, hitDiff);
	  square = 1 - square;
	  float r2 = radius * radius;
	  float d2 = d * d;
	  float cosTheta_p = std::abs(dot(p.in_dir, surf.face_normal));

	  if (d2 > r2) return;
	  float r = d / radius;
	  float k = (r <= 1.0f) ? (3.0f / 4.0f) * (1.0f - r * r) : 0.0f;

	  rgb bsdf = mat.bsdf->eval(p.in_dir, surf, out);

	  float norm = (3.0f / (4.0f * M_PI * r2)) * (1.0f / light_path_count);
	  accumulated += bsdf * p.contrib * k * cosTheta_p * norm;
	  });

      lastBounceGlossy = false;
      return color + accumulated;
    }

    // Sample BSDF for next direction
    BsdfSample bsdf = mat.bsdf->sample(sampler, surf, out);
    if (bsdf.pdf <= 0.0f)
      break;

    // Update contribution and ray
    pdf *= bsdf.pdf;
    ray = Ray(surf.point, bsdf.in, offset);

    // Russian Roulette
    if (path_len > 1)
    {
      float q = std::min(0.95f, dot(bsdf.color, luminance));
      if (sampler() > q)
	break;
      pdf *= q;
    }
  }

  return color;
}

float PhotonMappingRenderer::estimate_pixel_size(size_t w, size_t h)
{
  float total_dist = 0.0f;
  int total_count = 0;

  auto kx = 2.0f / (w - 1);
  auto ky = 2.0f / (h - 1);

  // Compute distance between neighboring pixels in world space,
  // in order to get a good estimate for the initial photon size.
#ifdef USE_STD_THREAD
  parallel_for(0, h / 8 + (h % 8 > 0 ? 1 : 0), [&](size_t ybin)
      {
      size_t y = ybin * 8;
      float d = 0; int c = 0;
      for (size_t x = 0; x < w; x += 8) {
      Ray rays[4]; Hit hits[4];
      for (int i = 0; i < 4; i++) {
      rays[i] = scene.camera->gen_ray(
	  (x + (i % 2 ? 4 : 0)) * kx - 1.0f,
	  1.0f - (y + (i / 2 ? 4 : 0)) * ky);
      hits[i] = scene.intersect(rays[i]);
      }
      auto eval_distance = [&] (int i, int j) {
      if (hits[i].tri >= 0 && hits[i].tri == hits[j].tri) {
      d += length((rays[i].org + hits[i].t * rays[i].dir) -
	  (rays[j].org + hits[j].t * rays[j].dir));
      c++;
      }
      };
      eval_distance(0, 1);
      eval_distance(2, 3);
      eval_distance(0, 2);
      eval_distance(1, 3);
      }

      std::atomic_ref<float>(total_dist) += d;
      std::atomic_ref<int>(total_count) += c; });
#else
#pragma omp parallel for
  for (size_t y = 0; y < h; y += 8)
  {
    float d = 0;
    int c = 0;
    for (size_t x = 0; x < w; x += 8)
    {
      Ray rays[4];
      Hit hits[4];
      for (int i = 0; i < 4; i++)
      {
	rays[i] = scene.camera->gen_ray(
	    (x + (i % 2 ? 4 : 0)) * kx - 1.0f,
	    1.0f - (y + (i / 2 ? 4 : 0)) * ky);
	hits[i] = scene.intersect(rays[i]);
      }
      auto eval_distance = [&](int i, int j)
      {
	if (hits[i].tri >= 0 && hits[i].tri == hits[j].tri)
	{
	  d += length((rays[i].org + hits[i].t * rays[i].dir) -
	      (rays[j].org + hits[j].t * rays[j].dir));
	  c++;
	}
      };
      eval_distance(0, 1);
      eval_distance(2, 3);
      eval_distance(0, 2);
      eval_distance(1, 3);
    }

#pragma omp atomic
    total_dist += d;
#pragma omp atomic
    total_count += c;
  }
#endif

  return total_count > 0 ? total_dist / (4 * total_count) : 1.0f;
}

std::unique_ptr<Renderer> create_ppm_renderer(const Scene &scene, size_t max_path_len)
{
  return std::unique_ptr<Renderer>(new PhotonMappingRenderer(scene, max_path_len));
}
