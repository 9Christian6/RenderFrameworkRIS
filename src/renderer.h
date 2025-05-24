#ifndef RENDERER_H
#define RENDERER_H

#include <memory>
#include <string>

struct Scene;
struct Image;

class Renderer {
public:
    Renderer(const Scene& scene)
        : scene(scene)
    {}

    virtual std::string name() const = 0;
    virtual void reset() = 0;
    virtual void render(Image& img) = 0;

protected:
    static constexpr float offset = 1e-3f;
    const Scene& scene;
};

static constexpr size_t default_tile_width  = 32;
static constexpr size_t default_tile_height = 32;

#ifdef USE_STD_THREAD

#include "parallel.h"

template <typename F>
void process_tiles(size_t x, size_t y, size_t w, size_t h, size_t tile_w, size_t tile_h, F f) {
    size_t cols = (w - x) / tile_w + ((w - x) % tile_w ? 1 : 0);
    size_t rows = (h - y) / tile_h + ((h - y) % tile_h ? 1 : 0);
    parallel_for(0, cols * rows,
        [&](size_t pos){
            size_t ystart = pos / cols * tile_h + y;
            size_t xstart = pos % cols * tile_w + x;
            f(xstart, ystart, std::min(xstart + tile_w, w), std::min(ystart + tile_h, h));
        });
}

#else

template <typename F>
void process_tiles(size_t x, size_t y, size_t w, size_t h, size_t tile_w, size_t tile_h, F f) {
    #pragma omp parallel for collapse(2) schedule(dynamic)
    for (size_t tile_x = x; tile_x < w; tile_x += tile_w) {
        for (size_t tile_y = y; tile_y < h; tile_y += tile_h) {
            f(tile_x, tile_y, std::min(tile_x + tile_w, w), std::min(tile_y + tile_h, h));
        }
    }
}

#endif

std::unique_ptr<Renderer> create_debug_renderer(const Scene& scene);
std::unique_ptr<Renderer> create_pt_renderer(const Scene& scene, size_t max_path_len = 64);
std::unique_ptr<Renderer> create_bpt_renderer(const Scene& scene, bool connect = true, bool light_tracing = true, size_t max_path_len = 64);
std::unique_ptr<Renderer> create_ppm_renderer(const Scene& scene, size_t max_path_len = 64);

#endif // RENDERER_H
