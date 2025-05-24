#ifndef HASH_GRID_H
#define HASH_GRID_H

#include <vector>
#include <algorithm>
#include <numeric>
#include <cstdint>
#include <atomic>

#include "hash.h"
#include "parallel.h"

class HashGrid {
public:
    HashGrid() {}

    template <typename PositionFn>
    void build(PositionFn positions, size_t num_photons, float radius) {
        radius_sqr = radius * radius;
        inv_size  = 0.5f / radius;

        // Compute the global bounding box encompassing all the photons
        bbox = BBox::empty();

        #pragma omp parallel for reduction(bbox_extend: bbox)
        for (size_t i = 0; i < num_photons; ++i)
            bbox = extend(bbox, positions(i));

        // Slightly enlarge the bounding box of the photons to avoid numerical problems
        auto extents = bbox.max - bbox.min;
        bbox.max += extents * 0.001f;
        bbox.min -= extents * 0.001f;

        photons.resize(num_photons);
        cell_counts.resize(1 << (closest_log2(num_photons) + 1));
        std::fill(cell_counts.begin(), cell_counts.end(), 0);

        // Count the number of photons per cell
#ifdef USE_STD_THREAD
        // C++20 version with atomic_ref
        parallel_for(0, num_photons, [&](size_t i) {
            auto h = hash_photon(positions(i));
            std::atomic_ref<uint32_t>(cell_counts[h])++;
        });
#else
        #pragma omp parallel for
        for (size_t i = 0; i < num_photons; i++) {
            auto h = hash_photon(positions(i));
            #pragma omp atomic
            cell_counts[h]++;
        }
#endif

        // Compute the insertion position for each cell
        std::partial_sum(cell_counts.begin(), cell_counts.end(), cell_counts.begin());
        assert(cell_counts.back() == photons.size());

        // Put the photons in their respective cells
#ifdef USE_STD_THREAD
        // C++20 version with atomic_ref
        parallel_for(0, num_photons, [&](size_t i) {
            auto h = hash_photon(positions(i));
            int old_count;
            old_count = --std::atomic_ref<uint32_t>(cell_counts[h]);
            photons[old_count] = uint32_t(i);
        });
#else
        #pragma omp parallel for
        for (size_t i = 0; i < num_photons; i++) {
            auto h = hash_photon(positions(i));
            int old_count;
            #pragma omp atomic capture
            old_count = --cell_counts[h];
            photons[old_count] = uint32_t(i);
        }
#endif
    }

    template <typename PositionFn, typename InsertFn>
    void query(const float3& pos, PositionFn positions, InsertFn insert) const {
        if (!is_inside(bbox, pos)) return;

        auto p = (pos - bbox.min) * inv_size;
        int px1 = p.x;
        int py1 = p.y;
        int pz1 = p.z;
        int px2 = px1 + (p.x - px1 > 0.5f ? 1 : -1);
        int py2 = py1 + (p.y - py1 > 0.5f ? 1 : -1);
        int pz2 = pz1 + (p.z - pz1 > 0.5f ? 1 : -1);

        for (int i = 0; i < 8; i++) {
            auto range = cell_range(i & 1 ? px2 : px1,
                                    i & 2 ? py2 : py1,
                                    i & 4 ? pz2 : pz1);

            for (auto j = range.first; j < range.second; j++) {
                int photon_id = photons[j];
                auto photon_pos = positions(photon_id);
                auto d = lensqr(pos - photon_pos);
                if (d < radius_sqr)
                    insert(photon_id, d);
            }
        }
    }

private:
    std::pair<size_t, size_t> cell_range(uint32_t x, uint32_t y, uint32_t z) const {
        auto h = hash_cell(x, y, z);
        return std::make_pair(cell_counts[h], h == cell_counts.size() - 1 ? photons.size() : cell_counts[h + 1]);
    }

    uint32_t hash_cell(uint32_t x, uint32_t y, uint32_t z) const {
        uint32_t h = bernstein_hash(bernstein_init(), x);
        h = bernstein_hash(h, y);
        h = bernstein_hash(h, z);
        return h & (cell_counts.size() - 1);
    }

    uint32_t hash_photon(const float3& pos) const {
        auto p = (pos - bbox.min) * inv_size;
        return hash_cell(p.x, p.y, p.z);
    }

    std::vector<uint32_t> photons;
    std::vector<uint32_t> cell_counts;
    BBox bbox;
    float inv_size;
    float radius_sqr;
};

#endif // HASH_GRID_H
