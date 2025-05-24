#ifndef BBOX_H
#define BBOX_H

#include <cfloat>
#include <algorithm>
#include "float3.h"

/// Bounding box represented by its two extreme points.
struct BBox {
    float3 min, max;

    BBox() {}
    BBox(const float3& f) : min(f), max(f) {}
    BBox(const float3& min, const float3& max) : min(min), max(max) {}

    static BBox empty() { return BBox(float3(FLT_MAX), float3(-FLT_MAX)); }
    static BBox full() { return BBox(float3(-FLT_MAX), float3(FLT_MAX)); }
};

inline BBox extend(const BBox& bb, const float3& f) {
    return BBox(min(bb.min, f), max(bb.max, f));
}

inline BBox extend(const BBox& bb1, const BBox& bb2) {
    return BBox(min(bb1.min, bb2.min), max(bb1.max, bb2.max));
}

inline BBox overlap(const BBox& bb1, const BBox& bb2) {
    return BBox(max(bb1.min, bb2.min), min(bb1.max, bb2.max));
}

inline float half_area(const float3& min, const float3& max) {
    const float3 len = max - min;
    const float kx = std::max(len.x, 0.0f);
    const float ky = std::max(len.y, 0.0f);
    const float kz = std::max(len.z, 0.0f);
    return kx * (ky + kz) + ky * kz;
}

inline float half_area(const BBox& bb) {
    return half_area(bb.min, bb.max);
}

inline float volume(const float3& min, const float3& max) {
    const float3 len = max - min;
    return len.x * len.y * len.z;
}

inline float volume(const BBox& bb) {
    return volume(bb.min, bb.max);
}

inline bool is_empty(const BBox& bb) {
    return bb.min.x > bb.max.x ||
           bb.min.y > bb.max.y ||
           bb.min.z > bb.max.z;
}

inline bool is_inside(const BBox& bb, const float3& f) {
    return f.x >= bb.min.x && f.y >= bb.min.y && f.z >= bb.min.z &&
           f.x <= bb.max.x && f.y <= bb.max.y && f.z <= bb.max.z;
}

inline bool is_overlapping(const BBox& bb1, const BBox& bb2) {
    return bb1.min.x <= bb2.max.x && bb1.max.x >= bb2.min.x &&
           bb1.min.y <= bb2.max.y && bb1.max.y >= bb2.min.y &&
           bb1.min.z <= bb2.max.z && bb1.max.z >= bb2.min.z;
}

inline bool is_included(const BBox& bb1, const BBox& bb2) {
    return bb1.min.x >= bb2.min.x && bb1.max.x <= bb2.max.x &&
           bb1.min.y >= bb2.min.y && bb1.max.y <= bb2.max.y &&
           bb1.min.z >= bb2.min.z && bb1.max.z <= bb2.max.z;
}

inline bool is_strictly_included(const BBox& bb1, const BBox& bb2) {
    return is_included(bb1, bb2) &&
           (bb1.min.x > bb2.min.x || bb1.max.x < bb2.max.x ||
            bb1.min.y > bb2.min.y || bb1.max.y < bb2.max.y ||
            bb1.min.z > bb2.min.z || bb1.max.z < bb2.max.z);
}

// currently not supported by MSVC
#ifndef _MSC_VER
#pragma omp declare reduction \
    (bbox_extend:BBox:omp_out = extend(omp_out, omp_in)) \
    initializer(omp_priv = BBox::empty())
#endif

#endif // BBOX_H
