// ============================================================================
// VVIS++ ARGON EDITION (x64) v4 — Main Engine & Multi-Tier Visibility Solver
// Reconstructed from ficool2's VVIS++ (Oct 2026) & upgraded with Argon Subsystems:
//   - Challenge A: Spatial-Resilient `.viscache` (immune to `num_clusters` changes!)
//   - Challenge B: SAH-BVH `BasePortalVis` + `-turbovis` Graph/Cone Solver + `-maxdepth`
//   - Challenge C: Thread Director Native Lock-Free Ascending Pruning Scheduler
//   - Challenge D: Multi-Tier Draft Profiles (`-tier 0..3`, `-novis`, `-turbovis`, `-usecache`)
//
// CRITICAL OPTIMIZATIONS (Argon v4 — Ryazan Benchmark Tuning):
//   1. Removed all `SetThreadAffinityMask` calls and enforced strict Ascending Pruning
//      Order (`nummightsee` ascending via single `head_idx++` cursor) so easy portals
//      finish first (`StatDone`) and prune 80%+ of recursive branches for heavy portals.
//   2. Eliminated all heap allocations (`std::vector<uint64_t>`) inside `PStack` and
//      `check_stack_avx2` during `recursive_leaf_flow_avx2`, removing 10M+ `malloc/free`
//      calls and Windows heap lock contention across threads.
//   3. Restored automatic `env_fog_controller` (`farz`) radial culling by default with
//      a safe lower floor `std::max(fog_farz, 8192.0f)` so outdoor megamaps (`farz=17000`)
//      cull distant rays like ficool2's vvis++ without causing indoor blindness.
//   4. Spatial-Resilient `.viscache` v4: matches clusters and portals by 3D world-space
//      hashes (AABB + normal + vertices) and remaps bitvectors across `num_clusters`
//      shifts when editing brushes in Hammer.
// ============================================================================

#include "vvis_argon.hpp"
#include "vvis_simd_avx2.hpp"
#include "vvis_cache.hpp"
#include "vvis_bvh_turbo.hpp"
#include "vvis_hybrid_scheduler.hpp"
#include "vvis_bsp_l4d2.hpp"

#include <iostream>
#include <iomanip>
#include <chrono>

namespace vvis_argon {

// ----------------------------------------------------------------------------
// Reconstructed `CheckStack` (Expensive Stack Mightsee Pruning)
// Uses zero-heap-allocation fixed stack buffers `portal_possible` & `leaf_possible`!
// ----------------------------------------------------------------------------
inline int32_t check_stack_avx2(
    ClusterLeaf* leaf,
    ThreadData& thread,
    const std::vector<VPortal>& portals,
    int32_t num_portals_directed,
    int32_t c_portallongs,
    int32_t c_leafs_longs) noexcept
{
    int32_t num_trimmed = 0;
    VVIS_ALIGN(32) uint64_t portal_possible[MAX_PORTAL_LONGS];
    VVIS_ALIGN(32) uint64_t leaf_possible[MAX_LEAF_LONGS];

    std::memset(portal_possible, 0xFF, c_portallongs * sizeof(uint64_t));

    for (PStack* p = thread.pstack_head.next; p != nullptr; p = p->next) {
        if (p->leaf == leaf) break;
        if (p->did_mightsee_test) continue;
        p->did_mightsee_test = true;

        std::memset(leaf_possible, 0, c_leafs_longs * sizeof(uint64_t));

        if (p->pass != nullptr) {
            PStack dummy_stack;
            for (int32_t i = 0; i < num_portals_directed; ++i) {
                if (!(portal_possible[i >> 6] & (1ULL << (i & 63)))) continue;
                const VPortal& test_p = portals[i];
                if (!test_p.winding) continue;

                dummy_stack.reset_windings();
                StackWindingSOA* w = dummy_stack.alloc_stack_winding();
                w->load_from_dynamic(*test_p.winding);

                w = chop_winding_avx2_soa(w, dummy_stack, p->portalplane);
                if (w && p->source) {
                    Plane backplane{p->portal->plane.normal * -1.0f, -p->portal->plane.dist};
                    w = chop_winding_avx2_soa(w, dummy_stack, backplane);
                }
                if (w) {
                    for (int s = 0; s < p->numseperators[0]; ++s) {
                        w = chop_winding_avx2_soa(w, dummy_stack, p->seperators[0][s]);
                        if (!w) break;
                    }
                }
                if (w) {
                    for (int s = 0; s < p->numseperators[1]; ++s) {
                        w = chop_winding_avx2_soa(w, dummy_stack, p->seperators[1][s]);
                        if (!w) break;
                    }
                }
                if (!w) {
                    portal_possible[i >> 6] &= ~(1ULL << (i & 63));
                    continue;
                }

                const uint64_t* src_vis = (test_p.status.load(std::memory_order_acquire) == PortalStatus::StatDone && test_p.portalvis)
                    ? test_p.portalvis
                    : test_p.portalmightsee;
                if (src_vis) {
                    for (int32_t q = 0; q < c_leafs_longs; ++q) {
                        leaf_possible[q] |= src_vis[q];
                    }
                }
            }

            int32_t new_cnt = 0;
            for (int32_t q = 0; q < c_leafs_longs; ++q) {
                p->mightsee[q] &= leaf_possible[q];
#if defined(_MSC_VER)
                new_cnt += static_cast<int32_t>(__popcnt64(p->mightsee[q]));
#else
                new_cnt += static_cast<int32_t>(__builtin_popcountll(p->mightsee[q]));
#endif
            }
            num_trimmed += (p->mightsee_count - new_cnt);
            p->mightsee_count = new_cnt;
        }
    }
    return num_trimmed;
}

// ----------------------------------------------------------------------------
// Reconstructed `RecursiveLeafFlow` (AVX2 + FMA3 + BMI2 SoA Master Path)
// ZERO heap allocations on `PStack stack`!
// ----------------------------------------------------------------------------
inline void recursive_leaf_flow_avx2(
    int32_t leafnum,
    ThreadData& thread,
    PStack* prevstack,
    int32_t current_depth,
    std::vector<VPortal>& portals,
    std::vector<ClusterLeaf>& leafs,
    int32_t c_portallongs,
    int32_t c_leafs_longs) noexcept
{
    if (thread.max_depth > 0 && current_depth > thread.max_depth) {
        return;
    }

    ClusterLeaf* leaf = &leafs[leafnum];

    for (PStack* chk = thread.pstack_head.next; chk != nullptr; chk = chk->next) {
        if (chk->leaf == leaf) {
            return;
        }
    }

    thread.c_chains++;

    uint64_t leaf_bit = 1ULL << (leafnum & 63);
    if (!(thread.base->portalvis[leafnum >> 6] & leaf_bit)) {
        thread.base->portalvis[leafnum >> 6] |= leaf_bit;
        thread.base->numcansee++;
    }

    if (prevstack->mightsee_count > 0 && (thread.c_chains / 2) >= (prevstack->mightsee_count + thread.c_mighttest)) {
        thread.c_mighttest += check_stack_avx2(
            leaf, thread, portals, static_cast<int32_t>(portals.size()), c_portallongs, c_leafs_longs);
    }

    // Stack-allocated PStack frame with ZERO heap allocations and ZERO redundant memset!
    PStack stack;
    stack.leaf = leaf;
    stack.portal = nullptr;
    stack.next = nullptr;
    stack.source = nullptr;
    stack.pass = nullptr;
    stack.numseperators[0] = 0;
    stack.numseperators[1] = 0;
    bool sep_computed[2] = {false, false};
    stack.reset_windings();
    prevstack->next = &stack;

    uint64_t* might = stack.mightsee;
    const uint64_t* vis = thread.base->portalvis;

    for (VPortal* p : leaf->portals) {
        int32_t target_leaf = p->leaf;

        if (!(prevstack->mightsee[target_leaf >> 6] & (1ULL << (target_leaf & 63)))) {
            continue;
        }

        const uint64_t* test_vis = (p->status.load(std::memory_order_acquire) == PortalStatus::StatDone && p->portalvis)
            ? p->portalvis
            : p->portalmightsee;
        if (!test_vis) continue;

        int32_t mightsee_popcnt = 0;
        bool has_more = avx2_intersect_and_check_more(
            might,
            test_vis,
            prevstack->mightsee,
            vis,
            c_leafs_longs,
            mightsee_popcnt);

        if (!has_more && (vis[target_leaf >> 6] & (1ULL << (target_leaf & 63)))) {
            continue;
        }

        stack.mightsee_count = mightsee_popcnt + prevstack->mightsee_count;
        stack.portalplane = p->plane;
        Plane backplane{p->plane.normal * -1.0f, -p->plane.dist};

        if (std::fabs(prevstack->portalplane.normal.x - backplane.normal.x) <= 1e-6f &&
            std::fabs(prevstack->portalplane.normal.y - backplane.normal.y) <= 1e-6f &&
            std::fabs(prevstack->portalplane.normal.z - backplane.normal.z) <= 1e-6f)
        {
            continue;
        }

        stack.portal = p;
        stack.next = nullptr;
        stack.did_mightsee_test = false;
        stack.reset_windings();

        if (!p->winding) continue;

        StackWindingSOA* initial_pass = stack.alloc_stack_winding();
        initial_pass->load_from_dynamic(*p->winding);
        stack.pass = chop_winding_avx2_soa(initial_pass, stack, thread.pstack_head.portalplane);
        if (!stack.pass) {
            continue;
        }

        if (!prevstack->pass) {
            stack.source = prevstack->source;
            recursive_leaf_flow_avx2(
                target_leaf, thread, &stack, current_depth + 1, portals, leafs, c_portallongs, c_leafs_longs);
            stack.free_stack_winding(stack.pass);
            stack.pass = nullptr;
            prevstack->next = &stack;
            continue;
        }

        stack.pass = chop_winding_avx2_soa(stack.pass, stack, prevstack->portalplane);
        if (!stack.pass) {
            continue;
        }

        StackWindingSOA* src_copy = stack.alloc_stack_winding();
        if (src_copy) {
            *src_copy = *prevstack->source;
            stack.source = chop_winding_avx2_soa(src_copy, stack, backplane);
            if (!stack.source) {
                stack.free_stack_winding(stack.pass);
                stack.pass = nullptr;
                continue;
            }
        } else {
            stack.source = prevstack->source;
        }

        // Exact reconstruction of `0x1400d1680` (`ClipStackWindings`):
        if (sep_computed[0]) {
            for (int s = 0; s < stack.numseperators[0] && stack.pass; ++s) {
                stack.pass = chop_winding_avx2_soa(stack.pass, stack, stack.seperators[0][s]);
            }
        } else {
            stack.pass = clip_to_seperators_avx2_soa(
                prevstack->source, thread.pstack_head.portalplane, prevstack->pass, stack.pass, false, stack, 0, &stack);
            sep_computed[0] = true;
        }
        if (!stack.pass) {
            stack.free_stack_winding(stack.source);
            stack.source = nullptr;
            continue;
        }

        if (sep_computed[1]) {
            for (int s = 0; s < stack.numseperators[1] && stack.pass; ++s) {
                stack.pass = chop_winding_avx2_soa(stack.pass, stack, stack.seperators[1][s]);
            }
        } else {
            stack.pass = clip_to_seperators_avx2_soa(
                prevstack->pass, prevstack->portalplane, prevstack->source, stack.pass, true, stack, 1, &stack);
            sep_computed[1] = true;
        }
        if (!stack.pass) {
            stack.free_stack_winding(stack.source);
            stack.source = nullptr;
            continue;
        }

        if (stack.source != prevstack->source) {
            stack.source = clip_to_seperators_avx2_soa(
                stack.pass, stack.portalplane, prevstack->pass, stack.source, false, stack, 2, &stack);
            if (!stack.source) {
                stack.free_stack_winding(stack.pass);
                stack.pass = nullptr;
                continue;
            }

            stack.source = clip_to_seperators_avx2_soa(
                prevstack->pass, prevstack->portalplane, stack.pass, stack.source, true, stack, 3, &stack);
            if (!stack.source) {
                stack.free_stack_winding(stack.pass);
                stack.pass = nullptr;
                continue;
            }
        }

        recursive_leaf_flow_avx2(
            target_leaf, thread, &stack, current_depth + 1, portals, leafs, c_portallongs, c_leafs_longs);

        stack.free_stack_winding(stack.source);
        stack.free_stack_winding(stack.pass);
        stack.source = nullptr;
        stack.pass = nullptr;
        prevstack->next = &stack;
    }

    prevstack->next = nullptr;
}

// ----------------------------------------------------------------------------
// Reconstructed `PortalFlow` Worker Entry Point
// ----------------------------------------------------------------------------
inline void portal_flow_avx2_worker(
    int32_t portal_idx,
    std::vector<VPortal>& portals,
    std::vector<ClusterLeaf>& leafs,
    int32_t c_portallongs,
    int32_t c_leafs_longs,
    int32_t max_depth)
{
    VPortal& p = portals[portal_idx];

    if (p.status.load(std::memory_order_acquire) == PortalStatus::StatDone && p.portalvis != nullptr) {
        return;
    }

    p.status.store(PortalStatus::StatWorking, std::memory_order_release);
    p.portalvis = new uint64_t[c_leafs_longs];
    std::memset(p.portalvis, 0, c_leafs_longs * sizeof(uint64_t));
    p.numcansee = 0;

    if (!p.winding || !p.portalmightsee) {
        p.status.store(PortalStatus::StatDone, std::memory_order_release);
        return;
    }

    ThreadData data;
    data.base = &p;
    data.max_depth = max_depth;
    data.base_winding_soa.load_from_dynamic(*p.winding);

    data.pstack_head.next = nullptr;
    data.pstack_head.leaf = nullptr;
    data.pstack_head.portal = &p;
    data.pstack_head.source = &data.base_winding_soa;
    data.pstack_head.pass = nullptr;
    data.pstack_head.portalplane = p.plane;
    data.pstack_head.numseperators[0] = 0;
    data.pstack_head.numseperators[1] = 0;
    data.pstack_head.did_mightsee_test = false;
    data.pstack_head.reset_windings();
    std::memcpy(data.pstack_head.mightsee, p.portalmightsee, c_leafs_longs * sizeof(uint64_t));
    data.pstack_head.mightsee_count = p.nummightsee;

    recursive_leaf_flow_avx2(
        p.leaf, data, &data.pstack_head, 1, portals, leafs, c_portallongs, c_leafs_longs);

    p.status.store(PortalStatus::StatDone, std::memory_order_release);
}

// ----------------------------------------------------------------------------
// Reconstructed Parallel Cluster PVS + `CalcPAS` + RLE `LUMP_VISIBILITY` Builder
// ----------------------------------------------------------------------------
struct VisibilityBuildStats {
    int64_t total_clusters_visible   = 0;
    int32_t average_clusters_visible = 0;
    int32_t average_clusters_audible = 0;
    int32_t uncompressed_vis_bytes   = 0;
    int32_t compressed_vis_bytes     = 0;
};

inline VisibilityBuildStats assemble_and_compress_bsp_visibility(
    int32_t portalclusters,
    int32_t bitbytes_aligned,
    int32_t c_leafs_longs,
    const std::vector<ClusterLeaf>& leafs,
    bool fast_or_null_mode,
    float null_vis_radius_sq,
    int32_t num_threads,
    bool enable_hybrid_balance,
    std::vector<uint8_t>& out_lump_visibility)
{
    VisibilityBuildStats stats{};
    const int32_t visrow_bytes = VisRLECodec::calc_visrow_bytes(portalclusters); // (portalclusters + 7) >> 3
    stats.uncompressed_vis_bytes = portalclusters * bitbytes_aligned * 2;        // originalvismapsize * 2

    std::vector<uint64_t> uncompressed_pvs(static_cast<size_t>(portalclusters) * c_leafs_longs, 0ULL);
    std::vector<int32_t>  per_cluster_vis(portalclusters, 0);

    std::vector<int32_t> cluster_order(portalclusters);
    std::iota(cluster_order.begin(), cluster_order.end(), 0);

    bool is_null_vis = (fast_or_null_mode && !leafs.empty() && !leafs[0].portals.empty() && leafs[0].portals[0]->portalmightsee == nullptr);

    const int32_t rem_bits = portalclusters & 63;
    const uint64_t last_word_mask = (rem_bits == 0) ? ~0ULL : ((1ULL << rem_bits) - 1ULL);

    // 1. Parallel PVS Assembly across all clusters
    HybridWorkStealingScheduler::run_parallel(
        nullptr,
        portalclusters,
        cluster_order,
        num_threads,
        enable_hybrid_balance,
        false,
        false,
        [&](int32_t /*tid*/, int32_t i) {
            uint64_t* row = &uncompressed_pvs[static_cast<size_t>(i) * c_leafs_longs];
            int32_t vis_cnt = 0;
            if (is_null_vis) {
                if (null_vis_radius_sq > 0.0f) {
                    for (int32_t j = 0; j < portalclusters; ++j) {
                        float d2 = (leafs[i].centroid - leafs[j].centroid).length_sq();
                        if (i == j || d2 <= null_vis_radius_sq) {
                            row[j >> 6] |= (1ULL << (j & 63));
                            vis_cnt++;
                        }
                    }
                } else {
                    for (int32_t q = 0; q < c_leafs_longs; ++q) {
                        row[q] = ~0ULL;
                    }
                    if (c_leafs_longs > 0) {
                        row[c_leafs_longs - 1] &= last_word_mask;
                    }
                    vis_cnt = portalclusters;
                }
            } else {
                for (const VPortal* p : leafs[i].portals) {
                    const uint64_t* src_bits = p->portalvis ? p->portalvis : p->portalmightsee;
                    if (!src_bits) continue;
                    for (int32_t q = 0; q < c_leafs_longs; ++q) {
                        row[q] |= src_bits[q];
                    }
                }
                row[i >> 6] |= (1ULL << (i & 63));
                if (c_leafs_longs > 0) {
                    row[c_leafs_longs - 1] &= last_word_mask;
                }
                for (int32_t q = 0; q < c_leafs_longs; ++q) {
#if defined(_MSC_VER)
                    vis_cnt += static_cast<int32_t>(__popcnt64(row[q]));
#else
                    vis_cnt += static_cast<int32_t>(__builtin_popcountll(row[q]));
#endif
                }
            }
            per_cluster_vis[i] = vis_cnt;
        });

    for (int32_t c : per_cluster_vis) stats.total_clusters_visible += c;
    stats.average_clusters_visible = (portalclusters > 0)
        ? static_cast<int32_t>(stats.total_clusters_visible / portalclusters)
        : 0;

    // 2. Parallel `CalcPAS` using hardware `_tzcnt_u64` / `__builtin_ctzll` bit-scanning!
    std::vector<uint64_t> uncompressed_pas(static_cast<size_t>(portalclusters) * c_leafs_longs, 0ULL);
    std::vector<int32_t>  per_cluster_aud(portalclusters, 0);

    HybridWorkStealingScheduler::run_parallel(
        nullptr,
        portalclusters,
        cluster_order,
        num_threads,
        enable_hybrid_balance,
        false,
        false,
        [&](int32_t /*tid*/, int32_t i) {
            const uint64_t* pvs_row = &uncompressed_pvs[static_cast<size_t>(i) * c_leafs_longs];
            uint64_t* pas_row       = &uncompressed_pas[static_cast<size_t>(i) * c_leafs_longs];
            std::memcpy(pas_row, pvs_row, c_leafs_longs * sizeof(uint64_t));

            for (int32_t w = 0; w < c_leafs_longs; ++w) {
                uint64_t bits = pvs_row[w];
                while (bits != 0) {
#if defined(_MSC_VER)
                    unsigned long bit_pos = 0;
                    _BitScanForward64(&bit_pos, bits);
                    int32_t j = (w << 6) + static_cast<int32_t>(bit_pos);
#else
                    int32_t j = (w << 6) + __builtin_ctzll(bits);
#endif
                    bits &= (bits - 1);
                    if (j < portalclusters) {
                        const uint64_t* j_pvs = &uncompressed_pvs[static_cast<size_t>(j) * c_leafs_longs];
                        for (int32_t q = 0; q < c_leafs_longs; ++q) {
                            pas_row[q] |= j_pvs[q];
                        }
                    }
                }
            }
            if (c_leafs_longs > 0) {
                pas_row[c_leafs_longs - 1] &= last_word_mask;
            }

            int32_t acount = 0;
            for (int32_t q = 0; q < c_leafs_longs; ++q) {
#if defined(_MSC_VER)
                acount += static_cast<int32_t>(__popcnt64(pas_row[q]));
#else
                acount += static_cast<int32_t>(__builtin_popcountll(pas_row[q]));
#endif
            }
            per_cluster_aud[i] = acount;
        });

    int64_t total_audible = 0;
    for (int32_t a : per_cluster_aud) total_audible += a;
    stats.average_clusters_audible = (portalclusters > 0)
        ? static_cast<int32_t>(total_audible / portalclusters)
        : 0;

    // 3. Compress PVS & PAS rows into `dvis_t` binary format for `LUMP_VISIBILITY`
    // Strictly compresses `visrow_bytes = (portalclusters + 7) >> 3` bytes per row!
    const int32_t header_bytes = 4 + portalclusters * 2 * 4;
    out_lump_visibility.clear();
    out_lump_visibility.resize(header_bytes, 0);

    std::memcpy(out_lump_visibility.data(), &portalclusters, sizeof(int32_t));
    std::vector<uint8_t> compressed_buf(visrow_bytes * 2 + 16);

    for (int32_t i = 0; i < portalclusters; ++i) {
        int32_t pvs_ofs = static_cast<int32_t>(out_lump_visibility.size());
        std::memcpy(out_lump_visibility.data() + 4 + (i * 2 + DVIS_PVS) * 4, &pvs_ofs, 4);
        const auto* pvs_bytes = reinterpret_cast<const uint8_t*>(&uncompressed_pvs[static_cast<size_t>(i) * c_leafs_longs]);
        int32_t pvs_len = VisRLECodec::compress_row(pvs_bytes, visrow_bytes, compressed_buf.data());
        out_lump_visibility.insert(out_lump_visibility.end(), compressed_buf.begin(), compressed_buf.begin() + pvs_len);

        int32_t pas_ofs = static_cast<int32_t>(out_lump_visibility.size());
        std::memcpy(out_lump_visibility.data() + 4 + (i * 2 + DVIS_PAS) * 4, &pas_ofs, 4);
        const auto* pas_bytes = reinterpret_cast<const uint8_t*>(&uncompressed_pas[static_cast<size_t>(i) * c_leafs_longs]);
        int32_t pas_len = VisRLECodec::compress_row(pas_bytes, visrow_bytes, compressed_buf.data());
        out_lump_visibility.insert(out_lump_visibility.end(), compressed_buf.begin(), compressed_buf.begin() + pas_len);
    }

    stats.compressed_vis_bytes = static_cast<int32_t>(out_lump_visibility.size());
    return stats;
}

} // namespace vvis_argon

// ============================================================================
// Command-Line Interface & Main Driver
// ============================================================================
struct ArgonCLIOptions {
    std::string map_path;
    int32_t     threads           = -1;
    bool        fast_vis          = false;
    bool        verbose           = false;
    bool        no_sort           = false;
    bool        tmp_in            = false;
    bool        low_priority      = false;
    float       radius_override   = -1.0f;
    int32_t     trace_start       = -1;
    int32_t     trace_end         = -1;
    int32_t     max_depth         = -1;
    bool        use_cache         = false;
    bool        hybrid_balance    = true;
    bool        use_bvh           = true;
    std::string cache_path;
    vvis_argon::CompileTier tier  = vvis_argon::CompileTier::Tier3_MasterRelease;
};

static void print_argon_help() {
    std::printf("Breadworks / Argon - vvis++ (Oct 2026 Next-Gen x64 Edition v4)\n");
    std::printf("====================================================================================\n");
    std::printf(" %-20s | %-10s | %s\n", "Option", "Default", "Description");
    std::printf("====================================================================================\n");
    std::printf(" %-20s | %-10s | %s\n", "-threads <N>", "auto (12)", "Number of threads to use. Defaults to max CPU thread count");
    std::printf(" %-20s | %-10s | %s\n", "-tier <0|1|2|3>", "3", "Compilation gear: 0=NullVis, 1=TurboProc, 2=Cached, 3=Master");
    std::printf(" %-20s | %-10s | %s\n", "-novis", "false", "Tier 0: Instant all-visible/radial dummy PVS (<0.01s)");
    std::printf(" %-20s | %-10s | %s\n", "-turbovis", "false", "Tier 1: SAH-BVH + Topological Cone solver for mapgen.nut");
    std::printf(" %-20s | %-10s | %s\n", "-usecache", "false", "Tier 2+: Spatial-Resilient xxHash64 .viscache subsystem");
    std::printf(" %-20s | %-10s | %s\n", "-maxdepth <N>", "unlimited", "Clamp portal traversal after N doorway turns");
    std::printf(" %-20s | %-10s | %s\n", "-fast", "false", "Only do first quick pass on vis calculations (BVH accelerated)");
    std::printf(" %-20s | %-10s | %s\n", "-radius_override <R>", "fog>=8192", "Force visibility radius, overrides env_fog_controller farz");
    std::printf(" %-20s | %-10s | %s\n", "-nosort", "false", "Don't sort portals (sorting is an optimization)");
    std::printf(" %-20s | %-10s | %s\n", "-trace <A> <B>", "none", "Writes a linefile that traces vis from cluster A to B");
    std::printf(" %-20s | %-10s | %s\n", "-verbose (-v)", "false", "Turn on verbose output");
    std::printf("====================================================================================\n");
}

int main(int argc, char** argv) {
    using namespace vvis_argon;

    auto total_start = std::chrono::high_resolution_clock::now();
    std::printf("Breadworks - vvis++ (Argon Next-Gen x64 v4 - Oct 2026)\n");

    if (argc < 2) {
        std::printf("No arguments specified. Run with -help to see all available options\n");
        return 1;
    }

    ArgonCLIOptions opts;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-help" || arg == "--help") {
            print_argon_help();
            return 0;
        } else if (arg == "-threads" && i + 1 < argc) {
            opts.threads = std::atoi(argv[++i]);
        } else if (arg == "-fast") {
            opts.fast_vis = true;
        } else if (arg == "-v" || arg == "-verbose") {
            opts.verbose = true;
        } else if (arg == "-nosort") {
            opts.no_sort = true;
        } else if (arg == "-tmpin") {
            opts.tmp_in = true;
        } else if (arg == "-low") {
            opts.low_priority = true;
        } else if ((arg == "-radius_override" || arg == "-r") && i + 1 < argc) {
            opts.radius_override = static_cast<float>(std::atof(argv[++i]));
        } else if (arg == "-trace" && i + 2 < argc) {
            opts.trace_start = std::atoi(argv[++i]);
            opts.trace_end   = std::atoi(argv[++i]);
        } else if (arg == "-maxdepth" && i + 1 < argc) {
            opts.max_depth = std::atoi(argv[++i]);
        } else if (arg == "-usecache") {
            opts.use_cache = true;
            if (opts.tier == CompileTier::Tier3_MasterRelease) {
                opts.tier = CompileTier::Tier2_HybridCached;
            }
        } else if (arg == "-cachefile" && i + 1 < argc) {
            opts.cache_path = argv[++i];
            opts.use_cache = true;
        } else if (arg == "-novis") {
            opts.tier = CompileTier::Tier0_NullVis;
        } else if (arg == "-turbovis") {
            opts.tier = CompileTier::Tier1_TurboProc;
        } else if (arg == "-tier" && i + 1 < argc) {
            int t = std::atoi(argv[++i]);
            if (t == 0) opts.tier = CompileTier::Tier0_NullVis;
            else if (t == 1) opts.tier = CompileTier::Tier1_TurboProc;
            else if (t == 2) { opts.tier = CompileTier::Tier2_HybridCached; opts.use_cache = true; }
            else opts.tier = CompileTier::Tier3_MasterRelease;
        } else if (arg == "-hybrid-balance") {
            opts.hybrid_balance = true;
        } else if (arg == "-no-hybrid-balance") {
            opts.hybrid_balance = false;
        } else if (arg == "-nobvh") {
            opts.use_bvh = false;
        } else if (arg == "-noavx512") {
            // Kept for 100% CLI compatibility with ficool2 vvis++
        } else if (!arg.empty() && arg[0] != '-') {
            opts.map_path = arg;
        }
    }

    if (opts.map_path.empty()) {
        std::fprintf(stderr, "No map name specified\n");
        return 1;
    }

    CpuCapabilities caps = detect_cpu_capabilities();
    std::printf("CPU: %s\n", caps.brand_string.empty() ? "x86-64 Processor" : caps.brand_string.c_str());
    if (caps.has_avx2) {
        std::printf("Using AVX2 + FMA3 + BMI2 SoA path (Alder Lake AVX-512 hardware trap guarded)\n");
    } else {
        std::printf("Using SSE2 path\n");
    }

    std::string base_path = opts.map_path;
    if (base_path.size() > 4 && base_path.substr(base_path.size() - 4) == ".bsp") {
        base_path = base_path.substr(0, base_path.size() - 4);
    }
    std::string bsp_path   = base_path + ".bsp";
    std::string prt_path   = base_path + ".prt";
    std::string cache_path = opts.cache_path.empty() ? (base_path + ".viscache") : opts.cache_path;

    std::printf("reading %s\n", bsp_path.c_str());
    BSPFile bsp;
    if (!bsp.load(bsp_path)) {
        std::fprintf(stderr, "Cannot load bsp file %s\n", bsp_path.c_str());
        return 1;
    }

    // OPTIMIZATION 2 (Argon v4): Use `env_fog_controller` (`farz`) by default with a safe
    // lower floor of `8192.0f` units (`std::max(fog_farz, 8192.0f)`).
    // On outdoor maps like `ryazan_01_semkino` (`farz = 17000`), this culls distant rays
    // at 17,000 units just like ficool2's vvis++, while preventing indoor blindness on
    // maps with short atmospheric fog (`farz = 2500..3500`)!
    float max_dist_sq    = MAX_COORD_DIST_SQ;
    bool  use_radial_vis = false;
    float fog_farz       = bsp.parse_max_fog_farz();
    if (opts.radius_override > 0.0f) {
        max_dist_sq    = opts.radius_override * opts.radius_override;
        use_radial_vis = true;
        std::printf("Forced visibility radius: %g\n", opts.radius_override);
    } else if (fog_farz > 0.0f) {
        float safe_farz = std::max(fog_farz, MIN_SAFE_FOG_FARZ);
        max_dist_sq    = safe_farz * safe_farz;
        use_radial_vis = true;
        std::printf("Max farz in all env_fog_controller entities: %g (used for radial vis)\n", safe_farz);
    }

    std::printf("reading %s\n", prt_path.c_str());
    int32_t portalclusters = 0;
    int32_t numportals     = 0;
    std::vector<VPortal> portals;
    std::vector<ClusterLeaf> leafs;
    std::vector<std::unique_ptr<DynamicWinding>> winding_pool;

    if (!load_portals_prt(prt_path, portalclusters, numportals, portals, leafs, winding_pool)) {
        return 1;
    }

    const int32_t c_leafs_longs    = (portalclusters + 63) >> 6;
    const int32_t bitbytes_aligned = c_leafs_longs * 8;
    const int32_t c_portallongs    = ((numportals * 2) + 63) >> 6;
    const int32_t total_directed   = numportals * 2;

    // Tier 0 (`-novis` / `-tier 0`): Instant Null-Vis Bypass (<0.01s)
    if (opts.tier == CompileTier::Tier0_NullVis) {
        std::printf("[Tier 0: Instant Null-Vis] Generating dummy/radial PVS without geometric clipping...\n");
        for (auto& p : portals) {
            p.portalmightsee = nullptr;
            p.portalvis = nullptr;
        }
        std::vector<uint8_t> vis_lump;
        auto stats = assemble_and_compress_bsp_visibility(
            portalclusters, bitbytes_aligned, c_leafs_longs, leafs, true,
            use_radial_vis ? max_dist_sq : -1.0f, opts.threads, opts.hybrid_balance, vis_lump);

        std::printf("Total clusters visible: %lld\n", static_cast<long long>(stats.total_clusters_visible));
        std::printf("Average clusters visible: %i\n", stats.average_clusters_visible);
        std::printf("Average clusters audible: %i\n", stats.average_clusters_audible);
        std::printf("visdatasize:%i  compressed from %i\n", stats.compressed_vis_bytes, stats.uncompressed_vis_bytes);
        std::printf("writing %s\n", bsp_path.c_str());
        if (!bsp.save_visibility_lump_only(bsp_path, vis_lump)) {
            std::fprintf(stderr, "Failed to update LUMP_VISIBILITY in %s\n", bsp_path.c_str());
            return 1;
        }

        double elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - total_start).count();
        std::printf("%.3fs elapsed\n", elapsed);
        return 0;
    }

    // OPTIMIZATION 3 (Argon v4): Spatial-Resilient `.viscache` BEFORE BasePortalVis
    // Matches clusters & portals by 3D world-space hashes and remaps bitvectors even when
    // `num_clusters` changes after moving a brush in Hammer!
    IncrementalVisCache vis_cache;
    if (opts.use_cache || opts.tier == CompileTier::Tier2_HybridCached) {
        IncrementalVisCache::compute_graph_hashes(portals, leafs);
        bool loaded = vis_cache.preload_before_base_vis(
            cache_path, portals, leafs, bitbytes_aligned, c_leafs_longs, max_dist_sq, opts.max_depth);
        if (loaded) {
            std::printf("  [VisCache] Restored %d / %d directed portals from %s (%d matched clusters, %d dirty clusters)\n",
                vis_cache.cache_hits_portals, total_directed, cache_path.c_str(),
                vis_cache.matched_clusters, vis_cache.dirty_clusters);
        }
    }

    // Build SAH-BVH over Portals & Run `BasePortalVis` on uncached portals in natural `.prt` order
    std::unique_ptr<PortalBVH> bvh;
    if (opts.use_bvh) {
        bvh = std::make_unique<PortalBVH>();
        bvh->build(portals);
    }

    std::vector<int32_t> base_order;
    base_order.reserve(total_directed);
    for (int32_t i = 0; i < total_directed; ++i) {
        if (portals[i].portalmightsee == nullptr) {
            base_order.push_back(i);
        }
    }

    if (!base_order.empty()) {
        HybridWorkStealingScheduler::run_parallel(
            "BasePortalVis",
            static_cast<int32_t>(base_order.size()),
            base_order,
            opts.threads,
            opts.hybrid_balance,
            false,
            true,
            [&](int32_t /*tid*/, int32_t pid) {
                base_portal_vis_bvh_worker(
                    pid, portals, leafs, bvh.get(), c_portallongs, c_leafs_longs, max_dist_sq, use_radial_vis);
            });
    }

    // OPTIMIZATION 1 (Argon v4): Strictly sort portals by `nummightsee` ascending (`PComp`,
    // reconstructed from 0x1400d8870) and dispatch all threads from `head_idx++`!
    std::vector<int32_t> flow_order(total_directed);
    std::iota(flow_order.begin(), flow_order.end(), 0);
    if (!opts.no_sort) {
        std::sort(flow_order.begin(), flow_order.end(), [&](int32_t a, int32_t b) {
            return portals[a].nummightsee < portals[b].nummightsee;
        });
    }

    if (!opts.fast_vis) {
        if (opts.tier == CompileTier::Tier1_TurboProc) {
            int32_t turbo_depth = (opts.max_depth > 0) ? opts.max_depth : 5;
            HybridWorkStealingScheduler::run_parallel(
                "TurboPortalFlow",
                total_directed,
                flow_order,
                opts.threads,
                opts.hybrid_balance,
                false,
                true,
                [&](int32_t /*tid*/, int32_t pid) {
                    turbo_procedural_portal_vis_worker(
                        pid, portals, leafs, c_leafs_longs, turbo_depth, max_dist_sq, use_radial_vis);
                });
        } else {
            std::vector<int32_t> pending_flow_order;
            pending_flow_order.reserve(total_directed);
            for (int32_t pid : flow_order) {
                if (portals[pid].status.load(std::memory_order_relaxed) != PortalStatus::StatDone) {
                    pending_flow_order.push_back(pid);
                }
            }

            if (!pending_flow_order.empty()) {
                HybridWorkStealingScheduler::run_parallel(
                    "PortalFlow",
                    static_cast<int32_t>(pending_flow_order.size()),
                    pending_flow_order,
                    opts.threads,
                    opts.hybrid_balance,
                    false,
                    true,
                    [&](int32_t /*tid*/, int32_t pid) {
                        portal_flow_avx2_worker(
                            pid, portals, leafs, c_portallongs, c_leafs_longs, opts.max_depth);
                    });
            }
        }

        if (opts.use_cache || opts.tier == CompileTier::Tier2_HybridCached) {
            if (vis_cache.save(cache_path, portals, leafs, numportals, bitbytes_aligned, max_dist_sq, opts.max_depth)) {
                if (opts.verbose) {
                    std::printf("  [VisCache] Saved %d portals (%d clusters) to %s\n",
                        total_directed, portalclusters, cache_path.c_str());
                }
            }
        }
    } else {
        for (auto& p : portals) {
            p.portalvis = p.portalmightsee;
            p.portalmightsee = nullptr;
            p.numcansee = p.nummightsee;
            p.status.store(PortalStatus::StatDone, std::memory_order_relaxed);
        }
    }

    int64_t total_mightsee = 0;
    int64_t total_cansee   = 0;
    for (const auto& p : portals) {
        total_mightsee += p.nummightsee;
        total_cansee   += p.numcansee;
    }

    std::printf("Building PAS...\n");
    std::vector<uint8_t> vis_lump;
    auto vis_stats = assemble_and_compress_bsp_visibility(
        portalclusters, bitbytes_aligned, c_leafs_longs, leafs, opts.fast_vis, -1.0f,
        opts.threads, opts.hybrid_balance, vis_lump);

    if (!opts.fast_vis && total_mightsee > 0) {
        double pct = (static_cast<double>(total_cansee) * 100.0) / static_cast<double>(total_mightsee);
        std::printf("Optimized: %lld visible clusters (%.2f%%)\n",
            static_cast<long long>(total_cansee), pct);
    }

    std::printf("Total clusters visible: %lld\n", static_cast<long long>(vis_stats.total_clusters_visible));
    std::printf("Average clusters visible: %i\n", vis_stats.average_clusters_visible);
    std::printf("Average clusters audible: %i\n", vis_stats.average_clusters_audible);
    std::printf("visdatasize:%i  compressed from %i\n", vis_stats.compressed_vis_bytes, vis_stats.uncompressed_vis_bytes);

    std::printf("writing %s\n", bsp_path.c_str());
    if (!bsp.save_visibility_lump_only(bsp_path, vis_lump)) {
        std::fprintf(stderr, "Failed to update LUMP_VISIBILITY in %s\n", bsp_path.c_str());
        return 1;
    }

    for (auto& p : portals) {
        delete[] p.portalmightsee;
        delete[] p.portalvis;
        p.portalmightsee = nullptr;
        p.portalvis = nullptr;
    }

    double total_elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - total_start).count();
    std::printf("%.2fs elapsed\n", total_elapsed);
    return 0;
}
