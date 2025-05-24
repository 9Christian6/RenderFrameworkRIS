#ifndef DEBUG_H
#define DEBUG_H

#include <vector>
#include <functional>

#include "intersect.h"
#include "color.h"
#include "common.h"

#ifdef NDEBUG
inline void debug_raster(int, int) {}
inline bool debug_flag() { return false; }
inline void debug_path(const std::vector<float3>&) {}
template <typename... Args>
void debug_print(Args... args) {}
#else
extern int debug_xmin, debug_xmax;
extern int debug_ymin, debug_ymax;

/// Tells the debugger that a new path has started at pixel (x, y).
void debug_raster(int x, int y);
/// Returns true if the current pixel is in the debug window.
bool debug_flag();
/// Records a path into the debugger output, if the current pixel is in the debug window.
void debug_path(const std::vector<float3>& path);
/// Prints the given arguments if the current pixel is in the debug window.
template <typename... Args>
void debug_print(Args... args) {
    if (debug_flag()) {
        #pragma omp critical
        info(args...);
    }
}
#endif // NDEBUG

#endif // DEBUG_H
