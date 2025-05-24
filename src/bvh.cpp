#include <cassert>
#include <cmath>
#include <cfloat>
#include <algorithm>
#include <memory>
#include <queue>
#include <tuple>
#include <numeric>

#include "bvh.h"
#include "bbox.h"

template void Bvh::traverse<true>(const Ray&, Hit&) const;
template void Bvh::traverse<false>(const Ray&, Hit&) const;

#ifdef EMBREE
Bvh::~Bvh() {
    rtcReleaseGeometry(mesh);
    rtcReleaseScene(scene);
    rtcReleaseDevice(device);
}

void Bvh::build(const float3* verts, const uint32_t* indices, size_t num_tris) {
    device = rtcNewDevice(nullptr);

    scene = rtcNewScene(device);
    mesh = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

    // Infer the number of vertices from the mesh data set
    size_t num_verts = 0;
    for (size_t i = 0; i < num_tris; ++i) {
        num_verts = std::max(uint32_t(num_verts), indices[i * 4 + 0]);
        num_verts = std::max(uint32_t(num_verts), indices[i * 4 + 1]);
        num_verts = std::max(uint32_t(num_verts), indices[i * 4 + 2]);
    }
    num_verts++;

    auto vertex_buf = (float*)rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(float) * 4, num_verts);
    for (size_t i = 0; i < num_verts; i++) {
        auto& v = verts[i];
        vertex_buf[4 * i +  0] = v.x;
        vertex_buf[4 * i +  1] = v.y;
        vertex_buf[4 * i +  2] = v.z;
        vertex_buf[4 * i +  3] = 1.0f;
    }

    auto index_buf = (int*)rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(int) * 3, num_tris);
    for (size_t i = 0, j = 0; i < num_tris; i++, j += 3) {
        index_buf[j + 0] = indices[4 * i + 0];
        index_buf[j + 1] = indices[4 * i + 1];
        index_buf[j + 2] = indices[4 * i + 2];
    }

    rtcCommitGeometry(mesh);
    rtcAttachGeometry(scene, mesh);
    rtcCommitScene(scene);
    num_nodes = 0;
}

template <bool any>
void Bvh::traverse(const Ray& ray, Hit& hit) const {
    RTCIntersectContext context;
    context.flags = RTC_INTERSECT_CONTEXT_FLAG_INCOHERENT;
    context.filter = NULL;
    context.instID[0] = RTC_INVALID_GEOMETRY_ID;

    RTCRay embree_ray;
    embree_ray.org_x = ray.org.x;
    embree_ray.org_y = ray.org.y;
    embree_ray.org_z = ray.org.z;
    embree_ray.dir_x = ray.dir.x;
    embree_ray.dir_y = ray.dir.y;
    embree_ray.dir_z = ray.dir.z;
    embree_ray.tnear = ray.tmin;
    embree_ray.tfar = ray.tmax;
    embree_ray.mask = -1;
    embree_ray.id = 0;
    embree_ray.flags = 0;

    if (any) {
        rtcOccluded1(scene, &context, &embree_ray);
        hit.tri = embree_ray.tfar < 0.0f ? 0 : -1;
    } else {
        RTCRayHit ray_hit;
        ray_hit.hit.primID = RTC_INVALID_GEOMETRY_ID;
        ray_hit.ray = embree_ray;
        rtcIntersect1(scene, &context, &ray_hit);
        hit.tri = ray_hit.hit.primID != RTC_INVALID_GEOMETRY_ID ? ray_hit.hit.primID : -1;
        hit.t = ray_hit.ray.tfar;
        hit.u = ray_hit.hit.u;
        hit.v = ray_hit.hit.v;
    }
}
#else
static inline std::tuple<size_t, float, BBox> find_split(const uint32_t* prims, float* costs, size_t begin, size_t end, const BBox* bboxes) {
    BBox cur_bb = BBox::empty();

    // Sweep from the left and compute costs
    for (size_t i = begin; i < end - 1; i++) {
        cur_bb = extend(cur_bb, bboxes[prims[i]]);
        costs[i] = (i - begin + 1) * half_area(cur_bb);
    }

    size_t min_split = -1;
    float min_cost = FLT_MAX;
    BBox min_bb = BBox::empty();

    cur_bb = BBox::empty();

    // Sweep from the right and find the minimum cost
    for (size_t i = end - 1; i > begin; i--) {
        cur_bb = extend(cur_bb, bboxes[prims[i]]);

        const float c = costs[i - 1] + (end - i) * half_area(cur_bb);
        if (c < min_cost) {
            min_split = i;
            min_cost  = c;
            min_bb    = cur_bb;
        }
    }

    return std::make_tuple(min_split, min_cost, min_bb);
}

struct BvhBuilder {
    BvhBuilder(const BBox* bboxes,
               const float3* centers,
               float* costs,
               uint32_t** prims,
               Bvh::Node* nodes,
               size_t& node_count)
        : bboxes(bboxes)
        , centers(centers)
        , costs(costs)
        , prims(prims)
        , nodes(nodes)
        , node_count(node_count)
    {}

    void build(size_t node_id) {
        const float traversal_cost = 1.0f;

        Bvh::Node& node = nodes[node_id];
        const size_t begin = node.first_prim;
        const size_t end   = node.first_prim + node.num_prims;

        if (end - begin <= 1)
            return;

        // On all three axes, try to split this node
        BBox min_right   = BBox::empty();
        float min_cost   = FLT_MAX;
        size_t min_split = -1;
        size_t min_axis  = -1;

        for (size_t i = 0; i < 3; i++) {
            auto split = find_split(prims[i], costs, begin, end, bboxes);
            if (std::get<1>(split) < min_cost) {
                min_right = std::get<2>(split);
                min_cost  = std::get<1>(split);
                min_split = std::get<0>(split);
                min_axis  = i;
            }
        }

        assert(min_split > begin && min_split < end);

        // Compare the minimum split cost with the SAH of this node
        if (min_cost < ((end - begin) - traversal_cost) * half_area(node.min, node.max)) {
            const size_t axis1 = (min_axis + 1) % 3;
            const size_t axis2 = (min_axis + 2) % 3;

            uint32_t split_ref = prims[min_axis][min_split - 1];
            float split_pos = centers[split_ref][min_axis];
            auto is_on_left_side = [&] (uint32_t ref) {
                auto pos = centers[ref][min_axis];
                return pos < split_pos || unlikely(pos == split_pos && ref <= split_ref);
            };
            size_t n1 = std::stable_partition(prims[axis1] + begin, prims[axis1] + end, is_on_left_side) - prims[axis1];
            size_t n2 = std::stable_partition(prims[axis2] + begin, prims[axis2] + end, is_on_left_side) - prims[axis2];
            assert(n1 == min_split); (void)n1;
            assert(n2 == min_split); (void)n2;

            // Recompute the bounding box of the left child
            BBox min_left = BBox::empty();
            for (size_t i = begin; i < min_split; i++)
                min_left = extend(min_left, bboxes[prims[min_axis][i]]);

            size_t num_nodes;

            #pragma omp atomic capture
            { num_nodes = node_count; node_count += 2; }

            // Mark the node as an inner node
            node.child = num_nodes;
            node.axis = -min_axis;

            // Setup the child nodes
            Bvh::Node& left = nodes[num_nodes];
            left.first_prim = begin;
            left.num_prims  = min_split - begin;
            left.min = min_left.min;
            left.max = min_left.max;

            Bvh::Node& right = nodes[num_nodes + 1];
            right.first_prim = min_split;
            right.num_prims  = end - min_split;
            right.min = min_right.min;
            right.max = min_right.max;

            const auto smallest_node = right.num_prims <  left.num_prims ? num_nodes + 1 : num_nodes;
            const auto biggest_node  = right.num_prims >= left.num_prims ? num_nodes + 1 : num_nodes;

            bool spawn_task = nodes[smallest_node].num_prims > parallel_threshold();
            if (spawn_task) {
                BvhBuilder* builder = new BvhBuilder(bboxes, centers, costs, prims, nodes, node_count);
                #pragma omp task firstprivate(builder, smallest_node)
                {
                    builder->build_and_delete(smallest_node);
                }
            }

            build(biggest_node);
            if (!spawn_task) build(smallest_node);
        }
    }

    void build_and_delete(size_t node_id) {
        build(node_id);
        delete this;
    }

    static constexpr int parallel_threshold() { return 1000; }

    const BBox*   bboxes;
    const float3* centers;
    float*        costs;

    uint32_t** prims;

    Bvh::Node* nodes;
    size_t& node_count;
};

void Bvh::build(const BBox& global_bbox, const BBox* bboxes, const float3* centers, size_t num_refs) {
    prim_ids.reset(new uint32_t[num_refs]);
    nodes.reset(new Node[num_refs * 2 + 1]);

    std::unique_ptr<uint32_t[]> all_prims(new uint32_t[2 * num_refs]);
    std::unique_ptr<float[]> costs(new float[num_refs]);
    uint32_t* prims[3] = { prim_ids.get(), all_prims.get(), all_prims.get() + num_refs };

    BvhBuilder* builder = new BvhBuilder(bboxes, centers, costs.get(), prims, nodes.get(), num_nodes);

    #pragma omp parallel
    {
        // Sort according to projection of barycenter on each axis
        #pragma omp sections
        {
            #pragma omp section
            {
                for (size_t i = 0; i < num_refs; i++) prims[0][i] = i;
                std::stable_sort(prims[0], prims[0] + num_refs, [&] (int p0, int p1) { return centers[p0].x < centers[p1].x; });
            }

            #pragma omp section
            {
                for (size_t i = 0; i < num_refs; i++) prims[1][i] = i;
                std::stable_sort(prims[1], prims[1] + num_refs, [&] (int p0, int p1) { return centers[p0].y < centers[p1].y; });
            }

            #pragma omp section
            {
                for (size_t i = 0; i < num_refs; i++) prims[2][i] = i;
                std::stable_sort(prims[2], prims[2] + num_refs, [&] (int p0, int p1) { return centers[p0].z < centers[p1].z; });
            }
        }

        // Start first builder task
        #pragma omp single
        {
            Node& root = nodes[0];
            root.first_prim = 0;
            root.num_prims  = num_refs;
            root.min = global_bbox.min;
            root.max = global_bbox.max;
            num_nodes = 1;
            builder->build_and_delete(0);
        }
    }

    // Resize the array of nodes
    Node* tmp_nodes = new Node[num_nodes];
    std::copy(nodes.get(), nodes.get() + num_nodes, tmp_nodes);
    nodes.reset(tmp_nodes);
}

void Bvh::try_split(size_t ref, const float3* tri, BBox* bboxes, float3* centers, uint32_t* refs, float threshold, size_t& num_refs, size_t max_refs) {
    // Triangle splitting according to the Edge Volume Heuristic
    static constexpr size_t stack_size = 32;
    struct {
        float3 tri[3];
        size_t i;
    } stack[stack_size];
    int stack_ptr = 0;

    std::copy(tri, tri + 3, stack[0].tri);
    stack[0].i = ref;
    while (stack_ptr >= 0) {
        auto& top = stack[stack_ptr];

        float vol[] = {
            volume(extend(BBox(top.tri[0]), top.tri[1])),
            volume(extend(BBox(top.tri[1]), top.tri[2])),
            volume(extend(BBox(top.tri[2]), top.tri[0]))
        };
        auto max_vol = std::max(vol[0], std::max(vol[1], vol[2]));

        if (max_vol > threshold && stack_ptr + 1 < int(stack_size)) {
            size_t j;
            #pragma omp atomic capture
            j = num_refs++;

            if (j < max_refs) {
                for (int k = 0; k < 3; ++k) {
                    if (max_vol == vol[k]) {
                        auto l = (k + 1) % 3;
                        auto m = (top.tri[k] + top.tri[l]) * 0.5f;
                        auto& other = stack[++stack_ptr];
                        std::copy(top.tri, top.tri + 3, other.tri);
                        top.tri[k] = m;
                        other.tri[l] = m;
                        other.i = j;
                        break;
                    }
                }
                continue;
            }
        }

        stack_ptr--;
        centers[top.i] = (1.0f / 3.0f) * (top.tri[0] + top.tri[1] + top.tri[2]);
        bboxes[top.i].min = min(top.tri[0], min(top.tri[1], top.tri[2]));
        bboxes[top.i].max = max(top.tri[0], max(top.tri[1], top.tri[2]));
        refs[top.i] = ref;
    }
}

size_t Bvh::pre_split(const float3* verts, const uint32_t* indices, BBox* bboxes, float3* centers, uint32_t* refs, float threshold, size_t num_tris, size_t max_refs) {
    size_t num_refs = num_tris;

    #pragma omp parallel for
    for (size_t i = 0; i < num_tris; i++) {
        float3 tri[] = {
            verts[indices[i * 4 + 0]],
            verts[indices[i * 4 + 1]],
            verts[indices[i * 4 + 2]]
        };
        try_split(i, tri, bboxes, centers, refs, threshold, num_refs, max_refs);
    }

    return std::min(num_refs, max_refs);
}

void Bvh::fix_refs(const uint32_t* refs) {
    #pragma omp parallel for
    for (size_t i = 0; i < num_nodes; ++i) {
        if (!nodes[i].is_leaf())
            continue;
        size_t begin = nodes[i].first_prim;
        size_t end   = begin + nodes[i].num_prims;
        for (size_t j = begin; j < end; ++j)
            prim_ids[j] = refs[prim_ids[j]];
        std::sort(prim_ids.get() + begin, prim_ids.get() + end);
        nodes[i].num_prims = std::unique(prim_ids.get() + begin, prim_ids.get() + end) - (prim_ids.get() + begin);
    }
}

void Bvh::build(const float3* verts, const uint32_t* indices, size_t num_tris) {
    auto max_refs = num_tris * 3 / 2;
    std::unique_ptr<BBox[]>     bboxes(new BBox[max_refs]);
    std::unique_ptr<float3[]>   centers(new float3[max_refs]);
    std::unique_ptr<uint32_t[]> refs(new uint32_t[max_refs]);

    // Compute global bounding box
    auto global_bbox = BBox::empty();
    #pragma omp parallel for reduction(bbox_extend: global_bbox)
    for (size_t i = 0; i < num_tris; ++i) {
        auto& v0 = verts[indices[i * 4 + 0]];
        auto& v1 = verts[indices[i * 4 + 1]];
        auto& v2 = verts[indices[i * 4 + 2]];
        global_bbox = extend(global_bbox, v0);
        global_bbox = extend(global_bbox, v1);
        global_bbox = extend(global_bbox, v2);
    }

    auto threshold = volume(global_bbox) / float(1 << 14);
    auto num_refs = pre_split(verts, indices, bboxes.get(), centers.get(), refs.get(), threshold, num_tris, max_refs);
    build(global_bbox, bboxes.get(), centers.get(), num_refs);
    fix_refs(refs.get());
    optimize(3);

    tris.reset(new PrecomputedTri[num_refs]);

    #pragma omp parallel for
    for (size_t i = 0; i < num_refs; i++) {
        auto tri_id = prim_ids[i];
        auto i0 = indices[tri_id * 4 + 0];
        auto i1 = indices[tri_id * 4 + 1];
        auto i2 = indices[tri_id * 4 + 2];
        tris[i] = PrecomputedTri(verts[i0], verts[i1], verts[i2]);
    }
}

void Bvh::compute_inefficiencies(float* inefficiencies) {
    std::unique_ptr<float[]> min_area(new float[num_nodes]);
    std::unique_ptr<float[]> sum_area(new float[num_nodes]);
    std::unique_ptr<size_t[]> num_children(new size_t[num_nodes]);
    float area_epsilon = 1e-10f;
    for (ptrdiff_t i = num_nodes - 1; i >= 0; --i) {
        auto& node = nodes[i];
        auto area = half_area(BBox(node.min, node.max));
        if (node.is_leaf()) {
            inefficiencies[i] = 0.0f;
            min_area[i] = area;
            sum_area[i] = area;
            num_children[i] = 1;
        } else {
            auto next_num_children = num_children[node.child + 0] + num_children[node.child + 1];
            auto next_sum_area = sum_area[node.child + 0] + sum_area[node.child + 1];
            auto next_min_area = std::max(area_epsilon, std::min(min_area[node.child + 0], min_area[node.child + 1]));
            float m_sum = area / (next_sum_area / next_num_children);
            float m_min = area / next_min_area;
            float m_area = area;
            float m_comb = m_sum * m_min * m_area;
            inefficiencies[i] = m_comb;
            min_area[i] = std::min(next_min_area, area);
            sum_area[i] = next_sum_area + area;
            num_children[i] = next_num_children + 1;
        }
    }
}

void Bvh::compute_parents(size_t* parents) {
    parents[0] = 0;
    for (size_t i = 0; i < num_nodes; ++i) {
        auto& node = nodes[i];
        if (node.is_leaf())
            continue;
        parents[node.child + 0] = i;
        parents[node.child + 1] = i;
    }
}

size_t Bvh::remove_node(size_t node_id, size_t* parents) {
    size_t parent = parents[node_id];
    size_t other_child = nodes[parent].child;
    size_t free = other_child;
    assert(parent != node_id);
    assert(node_id == other_child || node_id == other_child + 1);
    if (other_child == node_id)
        other_child++;
    nodes[parent] = nodes[other_child];
    if (!nodes[other_child].is_leaf()) {
        auto child = nodes[other_child].child;
        assert(parents[child + 0] == other_child);
        assert(parents[child + 1] == other_child);
        parents[child + 0] = parent;
        parents[child + 1] = parent;
    }
    refit_parents(parent, parents);
    return free;
}

size_t Bvh::find_reinsertion(const Node& node) {
    struct CandidateNode {
        size_t node_id;
        float induced_cost;
        float priority;

        CandidateNode(size_t node_id, float induced_cost, float priority)
            : node_id(node_id), induced_cost(induced_cost), priority(priority)
        {}

        bool operator < (const CandidateNode& other) const {
            return priority < other.priority;
        }
    };

    std::priority_queue<CandidateNode> candidates;
    auto node_area = half_area(node.bbox());
    float best_cost = FLT_MAX;
    float epsilon = 1e-20f;

    auto best_candidate = CandidateNode(0, 0, 1.0f / epsilon);
    candidates.emplace(best_candidate);
    while (!candidates.empty()) {
        auto candidate = candidates.top();
        candidates.pop();
        if (candidate.induced_cost + node_area >= best_cost)
            break;
        auto direct_cost = half_area(extend(node.bbox(), nodes[candidate.node_id].bbox()));
        auto total_cost = candidate.induced_cost + direct_cost;
        if (total_cost < best_cost) {
            best_cost = total_cost;
            best_candidate = candidate;
        }
        auto child_cost = total_cost - half_area(nodes[candidate.node_id].bbox());
        if (child_cost + node_area < best_cost) {
            if (!nodes[candidate.node_id].is_leaf()) {
                auto child_id = nodes[candidate.node_id].child;
                candidates.emplace(child_id + 0, child_cost, 1 / (child_cost + epsilon));
                candidates.emplace(child_id + 1, child_cost, 1 / (child_cost + epsilon));
            }
        }
    }
    return best_candidate.node_id;
}

void Bvh::refit_parents(size_t node_id, const size_t* parents) {
    auto cur = node_id;
    while (cur != 0) {
        cur = parents[cur];
        assert(!nodes[cur].is_leaf());
        auto child = nodes[cur].child;
        nodes[cur].min = min(nodes[child + 0].min, nodes[child + 1].min);
        nodes[cur].max = max(nodes[child + 0].max, nodes[child + 1].max);
    }
}

void Bvh::reinsert_node(const Node& node, size_t pos, size_t free, size_t* parents) {
    assert(pos != free && pos != free + 1);
    auto other = nodes[pos];
    nodes[free + 0] = node;
    nodes[free + 1] = other;
    if (!node.is_leaf()) {
        auto child = node.child;
        parents[child + 0] = free + 0;
        parents[child + 1] = free + 0;
    }
    if (!other.is_leaf()) {
        auto child = other.child;
        parents[child + 0] = free + 1;
        parents[child + 1] = free + 1;
    }
    parents[free + 0] = pos;
    parents[free + 1] = pos;
    nodes[pos].min = min(node.min, other.min);
    nodes[pos].max = max(node.max, other.max);
    nodes[pos].num_prims = 0;
    nodes[pos].child = free;
    refit_parents(pos, parents);
}

void Bvh::reorder_nodes(std::unique_ptr<Node[]>& tmp_nodes, size_t* parents) {
    static constexpr size_t stack_size = 64;
    int32_t stack[stack_size];
    int32_t stack_ptr = 0;

    parents[0] = 0;
    tmp_nodes[0] = nodes[0];
    if (nodes[0].num_prims <= 0) {
        stack[0] = 0;
        size_t cur = 1;
        while (stack_ptr >= 0) {
            auto parent_id = stack[stack_ptr--];
            auto& parent = tmp_nodes[parent_id];
            parents[cur + 0] = parent_id;
            parents[cur + 1] = parent_id;
            tmp_nodes[cur + 0] = nodes[parent.child + 0];
            tmp_nodes[cur + 1] = nodes[parent.child + 1];
            parent.child = cur;
            if (tmp_nodes[cur + 0].num_prims <= 0)
                stack[++stack_ptr] = cur + 0;
            if (tmp_nodes[cur + 1].num_prims <= 0)
                stack[++stack_ptr] = cur + 1;
            cur += 2;
        }
    }
    std::swap(nodes, tmp_nodes);
}

void Bvh::optimize(size_t num_iters) {
    size_t num_ranks = num_nodes - 1;

    std::unique_ptr<float[]> inefficiencies(new float[num_nodes]);
    std::unique_ptr<size_t[]> parents(new size_t[num_nodes]);
    std::unique_ptr<size_t[]> ranks(new size_t[num_ranks]);
    std::unique_ptr<Node[]> tmp_nodes(new Node[num_nodes]);
    compute_parents(parents.get());
    for (size_t iter = 0; iter < num_iters; ++iter) {
        auto batch_size = num_ranks / 10;
        if (batch_size == 0)
            return;

        compute_inefficiencies(inefficiencies.get());
        std::iota(ranks.get(), ranks.get() + num_ranks, 1);
        std::partial_sort(ranks.get(), ranks.get() + batch_size, ranks.get() + num_ranks, [&] (size_t a, size_t b) {
            return inefficiencies[a] > inefficiencies[b];
        });

        for (size_t i = 0; i < batch_size; ++i) {
            auto node = nodes[ranks[i]];
            auto free = remove_node(ranks[i], parents.get());
            auto pos  = find_reinsertion(node);
            reinsert_node(node, pos, free, parents.get());
        }
        reorder_nodes(tmp_nodes, parents.get());
    }
}

inline float fast_multiply_add(float a, float b, float c) {
#ifdef FP_FAST_FMAF
    return std::fmaf(a, b, c);
#else
    return a * b + c;
#endif
}

std::pair<float, float> Bvh::Node::intersect(const float3& inv_dir, const float3& org_div_dir, float tmin, float tmax, const int* octant) const {
    const float* bounds = reinterpret_cast<const float*>(this);
    float t0x = fast_multiply_add(bounds[    octant[0]], inv_dir.x, -org_div_dir.x);
    float t1x = fast_multiply_add(bounds[4 - octant[0]], inv_dir.x, -org_div_dir.x);
    float t0y = fast_multiply_add(bounds[    octant[1]], inv_dir.y, -org_div_dir.y);
    float t1y = fast_multiply_add(bounds[6 - octant[1]], inv_dir.y, -org_div_dir.y);
    float t0z = fast_multiply_add(bounds[    octant[2]], inv_dir.z, -org_div_dir.z);
    float t1z = fast_multiply_add(bounds[8 - octant[2]], inv_dir.z, -org_div_dir.z);
    auto t0 = std::max(std::max(t0x, t0y), std::max(tmin, t0z));
    auto t1 = std::min(std::min(t1x, t1y), std::min(tmax, t1z));
    return std::make_pair(t0, t1);
}

template <bool any>
void Bvh::traverse(const Ray& ray, Hit& hit) const {
    constexpr int stack_size = 64;
    int32_t stack[stack_size];
    int32_t top = nodes[0].child;
    int32_t stack_ptr = 0;

    hit.tri = -1;
    hit.t = ray.tmax;
    hit.u = 0;
    hit.v = 0;

    int octant[] = {
        ray.dir.x > 0 ? 0 : 4,
        ray.dir.y > 0 ? 1 : 5,
        ray.dir.z > 0 ? 2 : 6
    };
    auto inv_dir = float3(1.0f) / ray.dir;
    auto org_div_dir = ray.org * inv_dir;

    stack[0] = -1;
    while (true) {
        auto& left  = nodes[top + 0];
        auto& right = nodes[top + 1];

        // Intersect the two children of this node
        float t0[2], t1[2];
        std::tie(t0[0], t1[0]) = left .intersect(inv_dir, org_div_dir, ray.tmin, hit.t, octant);
        std::tie(t0[1], t1[1]) = right.intersect(inv_dir, org_div_dir, ray.tmin, hit.t, octant);

#define INTERSECT_LEAF(leaf) \
    { \
        for (auto j = leaf.first_prim; likely(j < leaf.first_prim + leaf.num_prims); j++) { \
            if (intersect_ray_tri(ray, tris[j], hit.t, hit.u, hit.v)) { \
                hit.tri = j; \
                if (any) return; \
            } \
        } \
    }

        int32_t child[2] = { -1, -1 };
        if (t0[0] <= t1[0]) {
            if unlikely(left.is_leaf()) INTERSECT_LEAF(left)
            else child[0] = left.child;
        }
        if (t0[1] <= t1[1]) {
            if unlikely(right.is_leaf()) INTERSECT_LEAF(right)
            else child[1] = right.child;
        }

        // Push the children on the stack
        if (child[0] >= 0 && child[1] >= 0) {
            if (t0[0] < t0[1])
                std::swap(child[0], child[1]);
            stack[++stack_ptr] = child[0];
            top = child[1];
        } else if (child[1] >= 0) {
            top = child[1];
        } else if (child[0] >= 0) {
            top = child[0];
        } else {
            top = stack[stack_ptr--];
            if (top < 0)
                break;
        }
    }

    if (hit.tri >= 0) hit.tri = prim_ids[hit.tri];
}
#endif // EMBREE
