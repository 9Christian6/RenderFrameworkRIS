#ifndef BVH_H
#define BVH_H

#include <memory>
#include <cstdint>

#include "float3.h"
#include "intersect.h"
#include "bbox.h"

#ifdef EMBREE
#include <embree3/rtcore.h>
#endif

/// Bounding Volume Hierarchy.
class Bvh {
public:
#ifdef EMBREE
    ~Bvh();
#endif

    /// Builds a BVH given a list of vertices and a list of indices.
    void build(const float3* verts, const uint32_t* indices, size_t num_tris);

    /// Traverses the BVH in order to find the closest intersection, or any intersection if 'any' is set.
    template <bool any = false>
    void traverse(const Ray& ray, Hit& hit) const;

    /// Returns the number of nodes in the BVH.
    size_t node_count() const { return num_nodes; }

private:
#ifdef EMBREE
    RTCDevice device;
    RTCGeometry mesh;
    RTCScene scene;
#else
    friend struct BvhBuilder;
    struct Node {
        float3 min;             ///< Min. BB corners
        union {
            int32_t child;      ///< Index of the first child, children are located next to each other in memory
            int32_t first_prim; ///< Index of the first primitive in the leaf
        };
        float3 max;             ///< Max. BB corners
        union {
            int32_t num_prims;  ///< Number of primitives for a leaf
            int32_t axis;       ///< Axis on which the inner node was split (negative value to distinguish leaves from inner nodes)
        };

        BBox bbox() const { return BBox(min, max); }
        bool is_leaf() const { return num_prims > 0; };
        inline std::pair<float, float> intersect(const float3&, const float3&, float, float, const int*) const;
    };

    void try_split(size_t, const float3*, BBox*, float3*, uint32_t*, float, size_t&, size_t);
    size_t pre_split(const float3*, const uint32_t*, BBox*, float3*, uint32_t*, float, size_t, size_t);
    void fix_refs(const uint32_t*);
    void build(const BBox&, const BBox*, const float3*, size_t);
    void compute_inefficiencies(float*);
    void compute_parents(size_t*);
    size_t remove_node(size_t, size_t*);
    size_t find_reinsertion(const Node&);
    void refit_parents(size_t, const size_t*);
    void reinsert_node(const Node&, size_t, size_t, size_t*);
    void reorder_nodes(std::unique_ptr<Node[]>&, size_t*);
    void optimize(size_t);

    std::unique_ptr<Node[]>           nodes;
    std::unique_ptr<uint32_t[]>       prim_ids;
    std::unique_ptr<PrecomputedTri[]> tris;
#endif
    size_t                            num_nodes;
};

#endif // BVH_H
