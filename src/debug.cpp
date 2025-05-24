#include <fstream>
#include "debug.h"

#ifndef NDEBUG

static std::ofstream debug_file("debug.obj");
static thread_local int cur_x, cur_y;

int debug_xmin, debug_xmax;
int debug_ymin, debug_ymax;

void debug_raster(int x, int y) {
    cur_x = x;
    cur_y = y;
}

bool debug_flag() {
    return cur_x >= debug_xmin && cur_x < debug_xmax &&
           cur_y >= debug_ymin && cur_y < debug_ymax;
}

void debug_path(const std::vector<float3>& path) {
    if (debug_flag()) {
        #pragma omp critical
        {
            for (auto& v : path)
                debug_file << "v " << v.x << " " << v.y << " " << v.z << std::endl;

            const int n = path.size();
            debug_file << "l";
            for (int i = 0; i < n; i++)
                debug_file << " " << i - n;
            debug_file << std::endl;
        }
    }
}

#endif // NDEBUG
