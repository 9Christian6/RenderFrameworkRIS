#include <iostream>
#include <chrono>
#include <climits>
#include <fstream>
#include <sstream>

#ifndef DISABLE_GUI
#include <SDL2/SDL.h>
#endif

#include "common.h"
#include "scene.h"
#include "options.h"
#include "renderer.h"
#include "cameras.h"
#include "debug.h"

#ifndef NDEBUG
static bool debug = false;
#endif

static std::vector<std::unique_ptr<Renderer>> renderers;

#ifndef DISABLE_GUI
bool handle_events(SDL_Window* window, Scene& scene, size_t& render_fn, size_t& accum) {
    SDL_Event event;

    static bool arrows[4], speed[2];
    const float rspeed = 0.005f;
    static float tspeed = 0.1f;

    static bool camera_on = false;
#ifndef NDEBUG
    static bool select_on = false;
#endif

    while (SDL_PollEvent(&event)) {
        bool key_down = event.type == SDL_KEYDOWN;
        switch (event.type) {
            case SDL_QUIT: return true;
            case SDL_MOUSEBUTTONDOWN:
                if (event.button.button == SDL_BUTTON_LEFT) {
                    SDL_SetRelativeMouseMode(SDL_TRUE);
                    camera_on = true;
                }
#ifndef NDEBUG
                if (!camera_on && event.button.button == SDL_BUTTON_RIGHT) {
                    select_on = true;
                    debug_xmin = event.button.x;
                    debug_xmax = INT_MIN;
                    debug_ymin = event.button.y;
                    debug_ymax = INT_MIN;
                }
#endif
                break;

            case SDL_MOUSEBUTTONUP:
                if (event.button.button == SDL_BUTTON_LEFT) {
                    SDL_SetRelativeMouseMode(SDL_FALSE);
                    camera_on = false;
                }
#ifndef NDEBUG
                if (event.button.button == SDL_BUTTON_RIGHT) select_on = false;
#endif
                break;
            case SDL_MOUSEMOTION:
                {
                    if (camera_on) {
                        scene.camera->mouse_motion(event.motion.xrel * rspeed, event.motion.yrel * rspeed);
                        accum = 0;
                    }
#ifndef NDEBUG
                    if (select_on) {
                        debug_xmax = std::max(debug_xmax, event.motion.x);
                        debug_ymax = std::max(debug_ymax, event.motion.y);
                    }
#endif
                }
                break;
            case SDL_KEYUP:
            case SDL_KEYDOWN:
                switch (event.key.keysym.sym) {
#ifndef NDEBUG
                    case SDLK_d:        debug = key_down; break;
#endif
                    case SDLK_UP:       arrows[0] = key_down; break;
                    case SDLK_DOWN:     arrows[1] = key_down; break;
                    case SDLK_LEFT:     arrows[2] = key_down; break;
                    case SDLK_RIGHT:    arrows[3] = key_down; break;
                    case SDLK_KP_PLUS:  speed[0] = key_down; break;
                    case SDLK_KP_MINUS: speed[1] = key_down; break;
                    case SDLK_r:
                        if (key_down) {
                            std::ostringstream title;
                            render_fn = (render_fn + 1) % renderers.size();
                            title << "arty (" << renderers[render_fn]->name() << ")";
                            SDL_SetWindowTitle(window, title.str().c_str());
                            accum = 0;
                        }
                        break;
                    case SDLK_ESCAPE:
                        return true;
                    default:
                        break;
                }
                break;
            default:
                break;
        }
    }

    if (arrows[0]) { scene.camera->keyboard_motion(0, 0,  tspeed); accum = 0; }
    if (arrows[1]) { scene.camera->keyboard_motion(0, 0, -tspeed); accum = 0; }
    if (arrows[2]) { scene.camera->keyboard_motion(-tspeed, 0, 0); accum = 0; }
    if (arrows[3]) { scene.camera->keyboard_motion( tspeed, 0, 0); accum = 0; }
    if (speed[0]) tspeed *= 1.1f;
    if (speed[1]) tspeed *= 0.9f;

    return false;
}
#endif // DISABLE_GUI

int main(int argc, char** argv) {
    ArgParser parser(argc, argv);

    bool help;
    size_t width, height;
    std::string output_image, renderer_name;
    double max_time;
    size_t max_samples;

    parser.add_option("help",      "h",    "Prints this message",               help,   false);
    parser.add_option("width",     "sx",   "Sets the window width, in pixels",  width,  size_t(1080), "px");
    parser.add_option("height",    "sy",   "Sets the window height, in pixels", height, size_t(720), "px");

    parser.add_option("output",    "o",    "Sets the output file name", output_image, std::string("render.exr"), "file.exr");

    parser.add_option("samples",   "s",    "Sets the desired number of samples", max_samples, size_t(0));
    parser.add_option("time",      "t",    "Sets the desired render time in seconds", max_time, 0.0);

    parser.add_option("algo",      "a",    "Sets the algorithm used for rendering: debug, pt, bpt, ppm", renderer_name, std::string("debug"));

    parser.parse();
    if (help) {
        parser.usage();
        return 0;
    }

    auto args = parser.arguments();
    if (!args.size()) {
        parser.usage();
        error("No configuration file specified. Exiting.");
        return 1;
    } else if (args.size() > 1) {
        warn("Too many configuration files specified, all but the first will be ignored.");
    }

    Scene scene;
    scene.width = width;
    scene.height = height;
    if (!load_scene(args[0], scene))
        return 1;
#ifdef DISABLE_GUI
    info("Compiled with GUI disabled (DISABLE_GUI = ON).");
    if (max_samples == 0) {
        info("Defaulting to 4 samples per pixel (use --samples or -s to change this value).");
        max_samples = 4;
    }
#else
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        error("Cannot initialize SDL.");
        return 1;
    }
#endif

    renderers.emplace_back(create_debug_renderer(scene));
    renderers.emplace_back(create_pt_renderer(scene));
    renderers.emplace_back(create_ppm_renderer(scene));

    auto renderer_it = std::find_if(renderers.begin(), renderers.end(), [&] (const std::unique_ptr<Renderer>& renderer) {
        return renderer_name == renderer->name();
    });
    if (renderer_it == renderers.end()) {
        error("No renderer with name '", renderer_name, "'.");
        return 1;
    }
    size_t render_fn = renderer_it - renderers.begin();

#ifndef DISABLE_GUI
    SDL_Window* window = SDL_CreateWindow("arty", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, 0);
    SDL_Surface* screen = SDL_GetWindowSurface(window);
    SDL_FlushEvents(SDL_FIRSTEVENT, SDL_LASTEVENT);
#endif

    Image img(width, height);
    img.clear();

#ifndef NDEBUG
    debug_xmin = INT_MAX;
    debug_xmax = INT_MIN;
    debug_ymin = INT_MAX;
    debug_ymax = INT_MIN;
#endif

    bool done = false;
    size_t frames = 0, accum = 0;
    uint64_t frame_time = 0;
    double total_time = 0;
    size_t total_frames = 0;

    while (!done) {
        using namespace std::chrono;

#ifndef NDEBUG
        if (debug || (debug_xmin >= debug_xmax && debug_ymin >= debug_ymax)) {
#endif
            if (accum++ == 0) {
                renderers[render_fn]->reset();
                total_time = 0;
                total_frames = 0;
                img.clear();
            }

            auto start_render = high_resolution_clock::now();
            renderers[render_fn]->render(img);
            auto end_render = high_resolution_clock::now();
            auto render_time = duration_cast<milliseconds>(end_render - start_render).count();
            frame_time += render_time;
            total_time += render_time * 0.001;
            frames++;
            total_frames++;

#ifndef NDEBUG
            if (debug) info("Debug information dumped.");
            debug = false;
        }
#endif

        if (frames > 20 || (frames > 0 && frame_time > 5000)) {
            info("Average frame time: ", frame_time / frames, " ms.");
            frames = 0;
            frame_time = 0;
        }

#ifndef DISABLE_GUI
        if (SDL_MUSTLOCK(screen)) SDL_LockSurface(screen);
        #pragma omp parallel for
        for (size_t y = 0; y < img.height; y++) {
            uint32_t* row = (uint32_t*)((uint8_t*)screen->pixels + screen->pitch * y);
            for (size_t x = 0; x < img.width; x++) {
                auto pix = gamma(img(x, y) / accum);
                const uint8_t r = clamp(pix.x, 0.0f, 1.0f) * 255.0f;
                const uint8_t g = clamp(pix.y, 0.0f, 1.0f) * 255.0f;
                const uint8_t b = clamp(pix.z, 0.0f, 1.0f) * 255.0f;
                const uint8_t a = clamp(pix.w, 0.0f, 1.0f) * 255.0f;
                row[x] = ((r << screen->format->Rshift) & screen->format->Rmask) |
                         ((g << screen->format->Gshift) & screen->format->Gmask) |
                         ((b << screen->format->Bshift) & screen->format->Bmask) |
                         ((a << screen->format->Ashift) & screen->format->Amask);
            }
        }

#ifndef NDEBUG
        if (debug_xmin < debug_xmax && debug_ymin < debug_ymax) {
            for (size_t y = std::max(0, debug_ymin), h = std::min(img.height, size_t(debug_ymax)); y < h; y++) {
                uint32_t* row = (uint32_t*)((uint8_t*)screen->pixels + screen->pitch * y);
                for (size_t x = std::max(0, debug_xmin), w = std::min(img.width, size_t(debug_xmax)); x < w; x++) {
                const uint8_t r = row[x] & screen->format->Rmask;
                const uint8_t g = row[x] & screen->format->Gmask;
                const uint8_t b = row[x] & screen->format->Bmask;
                const uint8_t a = row[x] & screen->format->Amask;
                row[x] = (((r + 64) << screen->format->Rshift) & screen->format->Rmask) |
                         (((g + 64) << screen->format->Gshift) & screen->format->Gmask) |
                         (((b + 64) << screen->format->Bshift) & screen->format->Bmask) |
                         (((a) << screen->format->Ashift) & screen->format->Amask);
                }
            }
        }
#endif
        if (SDL_MUSTLOCK(screen)) SDL_UnlockSurface(screen);

        SDL_UpdateWindowSurface(window);
        done = handle_events(window, scene, render_fn, accum);
#endif
        done |= max_samples != 0 && total_frames >= max_samples;
        done |= max_time != 0.0  && total_time   >= max_time;
    }

    bool save_as_png = output_image.rfind(".png") == output_image.length() - 4;
    bool save_as_exr = output_image.rfind(".exr") == output_image.length() - 4;
    if (output_image != "" && !save_as_png && !save_as_exr) {
        warn("Could not determine output file type from extension, using PNG");
        save_as_png = true;
    }
    if (save_as_png) {
        // Perform gamma correction before saving to disk
        for (auto& pix : img.pixels) pix = gamma(pix / accum);
        if (!save_png(output_image, img)) {
            error("Failed to save image to '", output_image, "'.");
            return 1;
        }
    }
    if (save_as_exr) {
        for (auto& pix : img.pixels) pix = pix / accum;
        if (!save_exr(output_image, img)) {
            error("Failed to save image to '", output_image, "'.");
            return 1;
        }
    }
    info("Image saved to '", output_image, "' (", accum, " samples, ", total_time, " s).");

#ifndef DISABLE_GUI
    SDL_DestroyWindow(window);
    SDL_Quit();
#endif

    return 0;
}
