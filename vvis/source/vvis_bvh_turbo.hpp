#pragma once
// ============================================================================
// VVIS++ ARGON EDITION (x64) v4 — Challenge B: Procedural Map Gen Acceleration
// (`-turbovis`, Conservative SAH-BVH Portal Culling, Topological Graph Visibility)
//
// CRITICAL OPTIMIZATIONS (Argon v4):
//   - Replaced heap-allocated `std::vector<uint64_t> portalmemory` and `flood_stack`
//     in `base_portal_vis_bvh_worker` with stack-allocated arrays (`MAX_PORTAL_LONGS`),
//     eliminating heap contention across worker threads during `BasePortalVis`.
//   - Conservative AABB padding (`BVH_CONSERVATIVE_EPSILON = 1.0f`) in `PortalBVH`
//     so grazing portals are never falsely culled.
// ============================================================================

#include "vvis_argon.hpp"
#include "vvis_simd_avx2.hpp"
#include <numeric>

namespace vvis_argon {

constexpr float BVH_CONSERVATIVE_EPSILON = 1.0f;

// ----------------------------------------------------------------------------
// 1. Flat Cache-Aligned SAH Bounding Volume Hierarchy (BVH) over Portals
// ----------------------------------------------------------------------------
struct AABB {
    Vector3 min_pt{ 1e30f,  1e30f,  1e30f};
    Vector3 max_pt{-1e30f, -1e30f, -1e30f};

    VVIS_FORCEINLINE void expand(const Vector3& p) {
        min_pt.x = std::min(min_pt.x, p.x);
        min_pt.y = std::min(min_pt.y, p.y);
        min_pt.z = std::min(min_pt.z, p.z);
        max_pt.x = std::max(max_pt.x, p.x);
        max_pt.y = std::max(max_pt.y, p.y);
        max_pt.z = std::max(max_pt.z, p.z);
    }

    VVIS_FORCEINLINE void expand(const AABB& b) {
        expand(b.min_pt);
        expand(b.max_pt);
    }

    VVIS_FORCEINLINE Vector3 centroid() const {
        return (min_pt + max_pt) * 0.5f;
    }

    // Signed distance of the positive-most AABB corner ("p-vertex") along plane normal
    VVIS_FORCEINLINE float max_signed_distance(const Plane& pl) const {
        float px = (pl.normal.x >= 0.0f) ? max_pt.x : min_pt.x;
        float py = (pl.normal.y >= 0.0f) ? max_pt.y : min_pt.y;
        float pz = (pl.normal.z >= 0.0f) ? max_pt.z : min_pt.z;
        return px * pl.normal.x + py * pl.normal.y + pz * pl.normal.z - pl.dist;
    }

    // Minimum squared Euclidean distance from point `p` to AABB
    VVIS_FORCEINLINE float min_dist_sq_to_point(const Vector3& p) const {
        float dx = std::max({min_pt.x - p.x, 0.0f, p.x - max_pt.x});
        float dy = std::max({min_pt.y - p.y, 0.0f, p.y - max_pt.y});
        float dz = std::max({min_pt.z - p.z, 0.0f, p.z - max_pt.z});
        return dx * dx + dy * dy + dz * dz;
    }
};

struct VVIS_ALIGN(32) PortalBVHNode {
    AABB    bounds;
    int32_t left_or_first = 0;
    int32_t count = 0;
};

class PortalBVH {
public:
    std::vector<PortalBVHNode> nodes;
    std::vector<int32_t>       portal_indices;
    std::vector<AABB>          portal_aabbs;

    void build(const std::vector<VPortal>& portals) {
        const int32_t n = static_cast<int32_t>(portals.size());
        portal_indices.resize(n);
        std::iota(portal_indices.begin(), portal_indices.end(), 0);
        portal_aabbs.resize(n);

        for (int32_t i = 0; i < n; ++i) {
            AABB box;
            if (portals[i].winding) {
                for (int k = 0; k < portals[i].winding->numpoints; ++k) {
                    box.expand(portals[i].winding->points[k]);
                }
            }
            box.min_pt = box.min_pt - Vector3(ON_EPSILON, ON_EPSILON, ON_EPSILON);
            box.max_pt = box.max_pt + Vector3(ON_EPSILON, ON_EPSILON, ON_EPSILON);
            portal_aabbs[i] = box;
        }

        nodes.clear();
        nodes.reserve(2 * n);
        if (n > 0) {
            build_recursive(0, n);
        }
    }

    template <typename VisitorFunc>
    void query_potentially_visible(
        const VPortal& src_portal,
        float max_dist_sq,
        bool use_radial_cull,
        VisitorFunc&& visitor) const
    {
        if (nodes.empty() || !src_portal.winding) return;
        const Plane& pl = src_portal.plane;
        const Vector3& orig = src_portal.winding->origin;
        const float src_r = src_portal.winding->radius;
        const float max_center_to_box = use_radial_cull ? (std::sqrt(max_dist_sq) + src_r + BVH_CONSERVATIVE_EPSILON) : 0.0f;
        const float max_center_to_box_sq = max_center_to_box * max_center_to_box;

        int32_t stack[64];
        int32_t sp = 0;
        stack[sp++] = 0;

        while (sp > 0) {
            int32_t node_idx = stack[--sp];
            const PortalBVHNode& node = nodes[node_idx];

            if (node.bounds.max_signed_distance(pl) < -BVH_CONSERVATIVE_EPSILON) {
                continue;
            }

            if (use_radial_cull) {
                if (node.bounds.min_dist_sq_to_point(orig) > max_center_to_box_sq) {
                    continue;
                }
            }

            if (node.count > 0) {
                for (int32_t i = 0; i < node.count; ++i) {
                    visitor(portal_indices[node.left_or_first + i]);
                }
            } else {
                int32_t left  = node.left_or_first;
                int32_t right = left + 1;
                if (sp + 2 <= 64) {
                    stack[sp++] = right;
                    stack[sp++] = left;
                }
            }
        }
    }

private:
    int32_t build_recursive(int32_t start, int32_t end) {
        int32_t node_idx = static_cast<int32_t>(nodes.size());
        nodes.emplace_back();

        AABB bounds;
        AABB centroid_bounds;
        for (int32_t i = start; i < end; ++i) {
            const AABB& b = portal_aabbs[portal_indices[i]];
            bounds.expand(b);
            centroid_bounds.expand(b.centroid());
        }
        nodes[node_idx].bounds = bounds;

        int32_t count = end - start;
        if (count <= 16) {
            nodes[node_idx].left_or_first = start;
            nodes[node_idx].count = count;
            return node_idx;
        }

        Vector3 ext = centroid_bounds.max_pt - centroid_bounds.min_pt;
        int axis = 0;
        if (ext.y > ext.x && ext.y >= ext.z) axis = 1;
        else if (ext.z > ext.x && ext.z >= ext.y) axis = 2;

        int32_t mid = (start + end) / 2;
        std::nth_element(
            portal_indices.begin() + start,
            portal_indices.begin() + mid,
            portal_indices.begin() + end,
            [&](int32_t a, int32_t b) {
                Vector3 ca = portal_aabbs[a].centroid();
                Vector3 cb = portal_aabbs[b].centroid();
                return (&ca.x)[axis] < (&cb.x)[axis];
            });

        int32_t left_child = static_cast<int32_t>(nodes.size());
        nodes.emplace_back();
        nodes.emplace_back();

        nodes[node_idx].left_or_first = left_child;
        nodes[node_idx].count = 0;

        build_subnode(left_child, start, mid);
        build_subnode(left_child + 1, mid, end);
        return node_idx;
    }

    void build_subnode(int32_t node_idx, int32_t start, int32_t end) {
        AABB bounds;
        AABB centroid_bounds;
        for (int32_t i = start; i < end; ++i) {
            const AABB& b = portal_aabbs[portal_indices[i]];
            bounds.expand(b);
            centroid_bounds.expand(b.centroid());
        }
        nodes[node_idx].bounds = bounds;

        int32_t count = end - start;
        if (count <= 16) {
            nodes[node_idx].left_or_first = start;
            nodes[node_idx].count = count;
            return;
        }

        Vector3 ext = centroid_bounds.max_pt - centroid_bounds.min_pt;
        int axis = 0;
        if (ext.y > ext.x && ext.y >= ext.z) axis = 1;
        else if (ext.z > ext.x && ext.z >= ext.y) axis = 2;

        int32_t mid = (start + end) / 2;
        std::nth_element(
            portal_indices.begin() + start,
            portal_indices.begin() + mid,
            portal_indices.begin() + end,
            [&](int32_t a, int32_t b) {
                Vector3 ca = portal_aabbs[a].centroid();
                Vector3 cb = portal_aabbs[b].centroid();
                return (&ca.x)[axis] < (&cb.x)[axis];
            });

        int32_t left_child = static_cast<int32_t>(nodes.size());
        nodes.emplace_back();
        nodes.emplace_back();

        nodes[node_idx].left_or_first = left_child;
        nodes[node_idx].count = 0;

        build_subnode(left_child, start, mid);
        build_subnode(left_child + 1, mid, end);
    }
};

// ----------------------------------------------------------------------------
// 2. BVH-Accelerated `BasePortalVis` + Iterative Flood Fill
// Uses stack-allocated `portalmemory` and `flood_stack` (zero heap contention!)
// ----------------------------------------------------------------------------
inline void base_portal_vis_bvh_worker(
    int32_t portal_idx,
    std::vector<VPortal>& portals,
    const std::vector<ClusterLeaf>& leafs,
    const PortalBVH* bvh,
    int32_t c_portallongs,
    int32_t c_leafs_longs,
    float max_dist_sq,
    bool use_radial_vis)
{
    VPortal& p = portals[portal_idx];
    if (!p.winding) return;

    p.portalmightsee = new uint64_t[c_leafs_longs];
    std::memset(p.portalmightsee, 0, c_leafs_longs * sizeof(uint64_t));

    VVIS_ALIGN(32) uint64_t portalmemory[MAX_PORTAL_LONGS];
    std::memset(portalmemory, 0, c_portallongs * sizeof(uint64_t));
    const DynamicWinding& w = *p.winding;

    auto test_candidate = [&](int32_t tp_idx) {
        if (tp_idx == portal_idx) return;
        const VPortal& tp = portals[tp_idx];
        if (!tp.winding) return;
        const DynamicWinding& tw = *tp.winding;

        // 1. Test if `tw` has any vertex in front of (or on) `p.plane`
        float d_center_tp = tw.origin.dot(p.plane.normal) - p.plane.dist;
        if (d_center_tp < -(tw.radius + ON_EPSILON)) return;

        int32_t completely_on_or_front_tp = 0;
        int32_t front_count_tp = 0;
        int32_t k = 0;
        for (; k < tw.numpoints; ++k) {
            float d = tw.points[k].dot(p.plane.normal) - p.plane.dist;
            if (d > ON_EPSILON) {
                break;
            }
            if (d >= MINUS_ON_EPSILON) {
                front_count_tp++;
            }
        }
        if (k == tw.numpoints) {
            if (front_count_tp != tw.numpoints) return;
            completely_on_or_front_tp = tw.numpoints;
        }

        // 2. Reverse plane test: is `w` completely in front of `tp.plane`?
        float d_center_p = w.origin.dot(tp.plane.normal) - tp.plane.dist;
        if (d_center_p > (w.radius + ON_EPSILON)) return;

        int32_t completely_on_or_back_p = 0;
        int32_t back_count_p = 0;
        for (k = 0; k < w.numpoints; ++k) {
            float d = w.points[k].dot(tp.plane.normal) - tp.plane.dist;
            if (d < MINUS_ON_EPSILON) {
                break;
            }
            if (d <= ON_EPSILON) {
                back_count_p++;
            }
        }
        if (k == w.numpoints) {
            if (back_count_p != w.numpoints) return;
            completely_on_or_back_p = w.numpoints;
        }

        if ((completely_on_or_front_tp | completely_on_or_back_p) != 0) {
            if (p.plane.normal.dot(tp.plane.normal) < BACKFACE_EPSILON) {
                return;
            }
        }

        // 3. Radial distance culling (`env_fog_controller` >= 8192 or `-radius_override`)
        if (use_radial_vis) {
            if (!avx2_portals_within_radius(w, tw, max_dist_sq)) {
                return;
            }
        }

        portalmemory[tp_idx >> 6] |= (1ULL << (tp_idx & 63));
    };

    if (bvh) {
        bvh->query_potentially_visible(p, max_dist_sq, use_radial_vis, test_candidate);
    } else {
        const int32_t total_p = static_cast<int32_t>(portals.size());
        for (int32_t j = 0; j < total_p; ++j) {
            test_candidate(j);
        }
    }

    // Flood-fill cluster visibility through `portalmemory` using fixed stack array
    p.nummightsee = 0;
    int32_t flood_stack[MAX_MAP_CLUSTERS];
    int32_t sp = 0;
    flood_stack[sp++] = p.leaf;

    while (sp > 0) {
        int32_t leafnum = flood_stack[--sp];

        uint64_t mask = 1ULL << (leafnum & 63);
        if (p.portalmightsee[leafnum >> 6] & mask) {
            continue;
        }
        p.portalmightsee[leafnum >> 6] |= mask;
        p.nummightsee++;

        const ClusterLeaf& cl = leafs[leafnum];
        for (const VPortal* next_p : cl.portals) {
            int32_t pnum = next_p->portal_id;
            if (portalmemory[pnum >> 6] & (1ULL << (pnum & 63))) {
                int32_t dst_leaf = next_p->leaf;
                if (!(p.portalmightsee[dst_leaf >> 6] & (1ULL << (dst_leaf & 63)))) {
                    if (sp < MAX_MAP_CLUSTERS) {
                        flood_stack[sp++] = dst_leaf;
                    }
                }
            }
        }
    }

    p.complexity_score = static_cast<float>(p.nummightsee) *
                         static_cast<float>(p.winding ? p.winding->numpoints : 4);
}

// ----------------------------------------------------------------------------
// 3. Tier 1 (`-turbovis`): Topological Portal Graph Cone/Frustum Solver
// ----------------------------------------------------------------------------
inline void turbo_procedural_portal_vis_worker(
    int32_t portal_idx,
    std::vector<VPortal>& portals,
    const std::vector<ClusterLeaf>& leafs,
    int32_t c_leafs_longs,
    int32_t max_depth,
    float max_dist_sq,
    bool use_radial_vis)
{
    VPortal& p = portals[portal_idx];
    p.status.store(PortalStatus::StatWorking, std::memory_order_relaxed);
    p.portalvis = new uint64_t[c_leafs_longs];
    std::memset(p.portalvis, 0, c_leafs_longs * sizeof(uint64_t));

    if (!p.winding || !p.portalmightsee) {
        p.status.store(PortalStatus::StatDone, std::memory_order_release);
        return;
    }

    const int32_t effective_max_depth = (max_depth > 0) ? max_depth : 8;
    const Vector3 src_origin = p.winding->origin;
    const float   src_radius = p.winding->radius;
    const Plane   src_plane  = p.plane;

    struct GraphStep {
        int32_t leaf_idx;
        int32_t depth;
        Vector3 cone_axis;
        float   cos_half_angle;
        Vector3 prev_portal_origin;
        float   prev_portal_radius;
    };

    std::vector<GraphStep> stack;
    stack.reserve(128);
    stack.push_back({
        p.leaf,
        1,
        src_plane.normal,
        -0.25f,
        src_origin,
        src_radius
    });

    while (!stack.empty()) {
        GraphStep cur = stack.back();
        stack.pop_back();

        int32_t lnum = cur.leaf_idx;
        uint64_t bit = 1ULL << (lnum & 63);
        if (!(p.portalmightsee[lnum >> 6] & bit)) {
            continue;
        }

        if (!(p.portalvis[lnum >> 6] & bit)) {
            p.portalvis[lnum >> 6] |= bit;
            p.numcansee++;
        }

        if (cur.depth >= effective_max_depth) {
            continue;
        }

        const ClusterLeaf& cl = leafs[lnum];
        for (const VPortal* next_p : cl.portals) {
            int32_t target_leaf = next_p->leaf;
            if (target_leaf == p.src_leaf) continue;

            uint64_t tbit = 1ULL << (target_leaf & 63);
            if (!(p.portalmightsee[target_leaf >> 6] & tbit)) {
                continue;
            }
            if (!next_p->winding) continue;

            const Vector3& next_orig = next_p->winding->origin;
            const float    next_rad  = next_p->winding->radius;

            Vector3 delta = next_orig - src_origin;
            float dist_sq = delta.length_sq();
            if (use_radial_vis && dist_sq > max_dist_sq) {
                continue;
            }

            if (next_orig.dot(src_plane.normal) - src_plane.dist + next_rad < MINUS_ON_EPSILON) {
                continue;
            }

            float dist = std::sqrt(std::max(dist_sq, 1.0f));
            Vector3 dir_to_next = delta * (1.0f / dist);
            float angular_spread = (src_radius + next_rad) / dist;
            float dot_cone = dir_to_next.dot(cur.cone_axis);

            if (dot_cone + angular_spread < cur.cos_half_angle) {
                continue;
            }

            Vector3 new_axis = (cur.cone_axis + dir_to_next) * 0.5f;
            float axis_len = new_axis.length();
            if (axis_len > 1e-4f) new_axis = new_axis * (1.0f / axis_len);
            float new_cos = std::min(0.90f, std::max(cur.cos_half_angle, 1.0f - angular_spread));

            stack.push_back({
                target_leaf,
                cur.depth + 1,
                new_axis,
                new_cos,
                next_orig,
                next_rad
            });
        }
    }

    p.status.store(PortalStatus::StatDone, std::memory_order_release);
}

} // namespace vvis_argon
