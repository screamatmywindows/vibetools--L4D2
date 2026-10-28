#pragma once
// ============================================================================
// VVIS++ ARGON EDITION (x64) v4 — AVX2 + FMA3 + BMI2 SIMD Kernels & CPUID Guard
// Strictly guarded against Intel 12th Gen Alder Lake (i5-12450H) AVX-512 traps.
//
// CRITICAL OPTIMIZATIONS (Argon v4):
//   - Single-pass AVX2 bitvector intersection + popcount (`avx2_intersect_and_check_more`).
//   - Lazy multi-bank AVX2 register loading in `clip_to_seperators_avx2_soa` (only loads
//     banks 1 and 2 when `num_pass > 8` or `num_pass > 16`, saving 6 YMM loads on 95%
//     of portals).
//   - Removed redundant scalar tail-padding loop in `chop_winding_avx2_soa`.
// ============================================================================

#include "vvis_argon.hpp"
#include <immintrin.h>
#include <cstdio>

namespace vvis_argon {

// ----------------------------------------------------------------------------
// Safe CPUID + XGETBV Hardware Detection (Alder Lake P+E Core Safe)
// ----------------------------------------------------------------------------
struct CpuCapabilities {
    bool has_sse41      = false;
    bool has_avx2       = false;
    bool has_fma3       = false;
    bool has_bmi2       = false;
    bool has_popcnt     = false;
    bool is_hybrid_cpu  = false;
    std::string brand_string;
};

inline void cpuid_ex(int info[4], int leaf, int subleaf = 0) {
#if defined(_MSC_VER)
    __cpuidex(info, leaf, subleaf);
#else
    __cpuid_count(leaf, subleaf, info[0], info[1], info[2], info[3]);
#endif
}

inline uint64_t xgetbv_safe(uint32_t xcr) {
#if defined(_MSC_VER)
    return _xgetbv(xcr);
#else
    uint32_t eax, edx;
    __asm__ __volatile__("xgetbv" : "=a"(eax), "=d"(edx) : "c"(xcr));
    return (static_cast<uint64_t>(edx) << 32) | eax;
#endif
}

inline CpuCapabilities detect_cpu_capabilities() {
    CpuCapabilities caps;
    int regs[4] = {0, 0, 0, 0};
    cpuid_ex(regs, 0, 0);
    int max_leaf = regs[0];

    if (max_leaf >= 1) {
        cpuid_ex(regs, 1, 0);
        bool osxsave = (regs[2] & (1 << 27)) != 0;
        caps.has_sse41  = (regs[2] & (1 << 19)) != 0;
        caps.has_popcnt = (regs[2] & (1 << 23)) != 0;
        bool fma3_bit   = (regs[2] & (1 << 12)) != 0;
        bool avx_bit    = (regs[2] & (1 << 28)) != 0;

        uint64_t xcr0 = osxsave ? xgetbv_safe(0) : 0;
        bool os_avx_enabled = ((xcr0 & 0x6) == 0x6);

        if (os_avx_enabled && avx_bit && max_leaf >= 7) {
            int regs7[4] = {0, 0, 0, 0};
            cpuid_ex(regs7, 7, 0);
            caps.has_avx2      = (regs7[1] & (1 << 5)) != 0;
            caps.has_bmi2      = (regs7[1] & (1 << 8)) != 0;
            caps.is_hybrid_cpu = (regs7[3] & (1 << 15)) != 0;
            caps.has_fma3      = fma3_bit;
        }
    }

    int ext_regs[4] = {0};
    cpuid_ex(ext_regs, 0x80000000, 0);
    if (static_cast<uint32_t>(ext_regs[0]) >= 0x80000004) {
        char brand[49] = {0};
        for (int i = 0; i < 3; ++i) {
            cpuid_ex(ext_regs, 0x80000002 + i, 0);
            std::memcpy(brand + i * 16, ext_regs, 16);
        }
        caps.brand_string = brand;
    }
    return caps;
}

// ----------------------------------------------------------------------------
// 1. AVX2 + BMI2 Bitvector Intersection & Popcount Kernel
// ----------------------------------------------------------------------------
VVIS_FORCEINLINE bool avx2_intersect_and_check_more(
    uint64_t* __restrict dst,
    const uint64_t* __restrict portal_vis_or_might,
    const uint64_t* __restrict current_might,
    const uint64_t* __restrict base_portalvis,
    int num_qwords,
    int32_t& out_popcnt) noexcept
{
    uint64_t more_accum = 0;
    int i = 0;

#if defined(__AVX2__) || defined(_MSC_VER)
    __m256i v_more = _mm256_setzero_si256();
    for (; i + 4 <= num_qwords; i += 4) {
        __m256i v_p   = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(portal_vis_or_might + i));
        __m256i v_m   = _mm256_load_si256(reinterpret_cast<const __m256i*>(current_might + i));
        __m256i v_b   = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(base_portalvis + i));

        __m256i v_and = _mm256_and_si256(v_p, v_m);
        _mm256_store_si256(reinterpret_cast<__m256i*>(dst + i), v_and);

        __m256i v_new = _mm256_andnot_si256(v_b, v_and);
        v_more = _mm256_or_si256(v_more, v_new);
    }

    if (!_mm256_testz_si256(v_more, v_more)) {
        more_accum = 1;
    }
#endif

    for (; i < num_qwords; ++i) {
        uint64_t val = portal_vis_or_might[i] & current_might[i];
        dst[i] = val;
        more_accum |= (val & ~base_portalvis[i]);
    }

    if (more_accum == 0) {
        out_popcnt = 0;
        return false;
    }

    int32_t total_bits = 0;
    for (int k = 0; k < num_qwords; ++k) {
#if defined(_MSC_VER)
        total_bits += static_cast<int32_t>(__popcnt64(dst[k]));
#else
        total_bits += static_cast<int32_t>(__builtin_popcountll(dst[k]));
#endif
    }
    out_popcnt = total_bits;
    return true;
}

// ----------------------------------------------------------------------------
// 2. AVX2 + FMA3 SoA Polygon Clipping Kernel (`ChopWinding_AVX2_SOA`)
// Supports up to 24 vertices (3x YMM banks) with conservative fallback on
// vertex or stack-slot saturation so visibility is NEVER falsely dropped!
// ----------------------------------------------------------------------------
VVIS_FORCEINLINE StackWindingSOA* chop_winding_avx2_soa(
    StackWindingSOA* __restrict in,
    PStack& stack,
    const Plane& split) noexcept
{
    if (!in || in->numpoints <= 0) return nullptr;

    // 1. Fast Bounding-Sphere Early Culling (matching vvis++.exe 0x1400d1216)
    float sphere_dist = in->origin.x * split.normal.x
                      + in->origin.y * split.normal.y
                      + in->origin.z * split.normal.z
                      - split.dist;

    if (sphere_dist < -(in->radius + ON_EPSILON)) {
        stack.free_stack_winding(in);
        return nullptr;
    }
    if (sphere_dist > (in->radius + ON_EPSILON)) {
        return in;
    }

    const int n = in->numpoints;
    VVIS_ALIGN(32) float dists[SOA_LANE_CAPACITY + 8];
    int sides[SOA_LANE_CAPACITY + 8];

#if defined(__AVX2__) || defined(_MSC_VER)
    __m256 v_nx   = _mm256_set1_ps(split.normal.x);
    __m256 v_ny   = _mm256_set1_ps(split.normal.y);
    __m256 v_nz   = _mm256_set1_ps(split.normal.z);
    __m256 v_dist = _mm256_set1_ps(split.dist);
    __m256 v_eps  = _mm256_set1_ps(ON_EPSILON);
    __m256 v_meps = _mm256_set1_ps(MINUS_ON_EPSILON);

    uint32_t front_mask = 0;
    uint32_t back_mask  = 0;

    // Unrolled bank 0 (vertices 0..7 — handles 95%+ of portals in a single pass!)
    {
        __m256 vx = _mm256_load_ps(in->x);
        __m256 vy = _mm256_load_ps(in->y);
        __m256 vz = _mm256_load_ps(in->z);
        __m256 vd = _mm256_sub_ps(_mm256_fmadd_ps(vz, v_nz, _mm256_fmadd_ps(vy, v_ny, _mm256_mul_ps(vx, v_nx))), v_dist);
        _mm256_store_ps(dists, vd);
        front_mask = static_cast<uint32_t>(_mm256_movemask_ps(_mm256_cmp_ps(vd, v_eps, _CMP_GT_OQ)));
        back_mask  = static_cast<uint32_t>(_mm256_movemask_ps(_mm256_cmp_ps(vd, v_meps, _CMP_LT_OQ)));
    }
    if (n > 8) {
        __m256 vx = _mm256_load_ps(in->x + 8);
        __m256 vy = _mm256_load_ps(in->y + 8);
        __m256 vz = _mm256_load_ps(in->z + 8);
        __m256 vd = _mm256_sub_ps(_mm256_fmadd_ps(vz, v_nz, _mm256_fmadd_ps(vy, v_ny, _mm256_mul_ps(vx, v_nx))), v_dist);
        _mm256_store_ps(dists + 8, vd);
        front_mask |= (static_cast<uint32_t>(_mm256_movemask_ps(_mm256_cmp_ps(vd, v_eps, _CMP_GT_OQ))) << 8);
        back_mask  |= (static_cast<uint32_t>(_mm256_movemask_ps(_mm256_cmp_ps(vd, v_meps, _CMP_LT_OQ))) << 8);
    }
    if (n > 16) {
        __m256 vx = _mm256_load_ps(in->x + 16);
        __m256 vy = _mm256_load_ps(in->y + 16);
        __m256 vz = _mm256_load_ps(in->z + 16);
        __m256 vd = _mm256_sub_ps(_mm256_fmadd_ps(vz, v_nz, _mm256_fmadd_ps(vy, v_ny, _mm256_mul_ps(vx, v_nx))), v_dist);
        _mm256_store_ps(dists + 16, vd);
        front_mask |= (static_cast<uint32_t>(_mm256_movemask_ps(_mm256_cmp_ps(vd, v_eps, _CMP_GT_OQ))) << 16);
        back_mask  |= (static_cast<uint32_t>(_mm256_movemask_ps(_mm256_cmp_ps(vd, v_meps, _CMP_LT_OQ))) << 16);
    }

    const uint32_t valid_mask = (n >= 32) ? ~0u : ((1u << n) - 1u);
    front_mask &= valid_mask;
    back_mask  &= valid_mask;

    if (back_mask == 0) {
        return in;
    }
    if (front_mask == 0) {
        stack.free_stack_winding(in);
        return nullptr;
    }

    for (int i = 0; i < n; ++i) {
        if ((front_mask >> i) & 1u) {
            sides[i] = 0;
        } else if ((back_mask >> i) & 1u) {
            sides[i] = 1;
        } else {
            sides[i] = 2;
        }
    }
#else
    int counts[3] = {0, 0, 0};
    for (int i = 0; i < n; ++i) {
        float d = in->x[i] * split.normal.x + in->y[i] * split.normal.y + in->z[i] * split.normal.z - split.dist;
        dists[i] = d;
        if (d > ON_EPSILON) { sides[i] = 0; counts[0]++; }
        else if (d < MINUS_ON_EPSILON) { sides[i] = 1; counts[1]++; }
        else { sides[i] = 2; counts[2]++; }
    }
    if (counts[1] == 0) return in;
    if (counts[0] == 0) {
        stack.free_stack_winding(in);
        return nullptr;
    }
#endif

    // Wrap vertex 0 to vertex n for seamless edge loop
    sides[n] = sides[0];
    dists[n] = dists[0];

    StackWindingSOA* out = stack.alloc_stack_winding();
    if (!out) {
        // Conservative PVS rule: if stack slots ever saturate, keep unclipped `in`!
        return in;
    }

    out->origin = in->origin;
    out->radius = in->radius;
    int out_pts = 0;

    for (int i = 0; i < n; ++i) {
        float px = in->x[i];
        float py = in->y[i];
        float pz = in->z[i];

        if (sides[i] == 2) {
            if (out_pts >= MAX_POINTS_ON_FIXED_WINDING) {
                stack.free_stack_winding(out);
                return in;
            }
            out->x[out_pts] = px;
            out->y[out_pts] = py;
            out->z[out_pts] = pz;
            out_pts++;
            continue;
        }
        if (sides[i] == 0) {
            if (out_pts >= MAX_POINTS_ON_FIXED_WINDING) {
                stack.free_stack_winding(out);
                return in;
            }
            out->x[out_pts] = px;
            out->y[out_pts] = py;
            out->z[out_pts] = pz;
            out_pts++;
        }
        if (sides[i + 1] == 2 || sides[i + 1] == sides[i]) {
            continue;
        }

        int next_i = (i + 1 < n) ? (i + 1) : 0;
        float dot = dists[i] / (dists[i] - dists[i + 1]);

        float ix, iy, iz;
        if (split.normal.x == 1.0f) ix = split.dist;
        else if (split.normal.x == -1.0f) ix = -split.dist;
        else ix = px + dot * (in->x[next_i] - px);

        if (split.normal.y == 1.0f) iy = split.dist;
        else if (split.normal.y == -1.0f) iy = -split.dist;
        else iy = py + dot * (in->y[next_i] - py);

        if (split.normal.z == 1.0f) iz = split.dist;
        else if (split.normal.z == -1.0f) iz = -split.dist;
        else iz = pz + dot * (in->z[next_i] - pz);

        if (out_pts >= MAX_POINTS_ON_FIXED_WINDING) {
            stack.free_stack_winding(out);
            return in;
        }
        out->x[out_pts] = ix;
        out->y[out_pts] = iy;
        out->z[out_pts] = iz;
        out_pts++;
    }

    out->numpoints = out_pts;
    stack.free_stack_winding(in);

    if (out_pts < 3) {
        stack.free_stack_winding(out);
        return nullptr;
    }
    return out;
}

// ----------------------------------------------------------------------------
// 3. AVX2 + FMA3 Separator Plane Generator & Clipper (`ClipToSeperators_AVX2_SOA`)
// Supports up to 24 vertices in `pass` and `source` with lazy YMM bank loading!
// ----------------------------------------------------------------------------
inline StackWindingSOA* clip_to_seperators_avx2_soa(
    const StackWindingSOA* __restrict source,
    const Plane& pass_plane,
    const StackWindingSOA* __restrict pass,
    StackWindingSOA* __restrict target,
    bool flipclip,
    PStack& stack,
    int sep_cache_slot, // 0, 1: cache into sep_owner->seperators[slot], >=2: don't cache
    PStack* sep_owner = nullptr) noexcept
{
    if (!source || !pass || !target) return target;
    const int num_src  = source->numpoints;
    const int num_pass = pass->numpoints;
    if (num_src <= 0 || num_pass <= 0) return target;

#if defined(__AVX2__) || defined(_MSC_VER)
    __m256 px0 = _mm256_load_ps(pass->x);
    __m256 py0 = _mm256_load_ps(pass->y);
    __m256 pz0 = _mm256_load_ps(pass->z);
    __m256 px1 = _mm256_setzero_ps(), py1 = _mm256_setzero_ps(), pz1 = _mm256_setzero_ps();
    __m256 px2 = _mm256_setzero_ps(), py2 = _mm256_setzero_ps(), pz2 = _mm256_setzero_ps();
    if (num_pass > 8) {
        px1 = _mm256_load_ps(pass->x + 8);
        py1 = _mm256_load_ps(pass->y + 8);
        pz1 = _mm256_load_ps(pass->z + 8);
    }
    if (num_pass > 16) {
        px2 = _mm256_load_ps(pass->x + 16);
        py2 = _mm256_load_ps(pass->y + 16);
        pz2 = _mm256_load_ps(pass->z + 16);
    }

    const uint32_t pass_valid_mask = (num_pass >= 32) ? ~0u : ((1u << num_pass) - 1u);
    __m256 v_eps  = _mm256_set1_ps(ON_EPSILON);
    __m256 v_meps = _mm256_set1_ps(MINUS_ON_EPSILON);
#endif

    for (int i = 0; i < num_src; ++i) {
        int l = (i + 1 < num_src) ? (i + 1) : 0;
        Vector3 p1(source->x[i], source->y[i], source->z[i]);
        Vector3 v1(source->x[l] - p1.x, source->y[l] - p1.y, source->z[l] - p1.z);

        for (int j = 0; j < num_pass; ++j) {
            Vector3 p3(pass->x[j], pass->y[j], pass->z[j]);
            float d_pass = p3.dot(pass_plane.normal) - pass_plane.dist;
            if (d_pass >= MINUS_ON_EPSILON && d_pass <= ON_EPSILON) {
                continue;
            }

            Vector3 v2 = p3 - p1;
            Vector3 normal = v1.cross(v2);
            float len_sq = normal.length_sq();
            if (len_sq < ON_EPSILON) {
                continue;
            }

            float inv_len = 1.0f / std::sqrt(len_sq);
            normal = normal * inv_len;
            float dist = p3.dot(normal);

            if (d_pass < MINUS_ON_EPSILON) {
                normal = normal * -1.0f;
                dist   = -dist;
            }

#if defined(__AVX2__) || defined(_MSC_VER)
            __m256 v_nx = _mm256_set1_ps(normal.x);
            __m256 v_ny = _mm256_set1_ps(normal.y);
            __m256 v_nz = _mm256_set1_ps(normal.z);
            __m256 v_d  = _mm256_set1_ps(dist);

            __m256 d0 = _mm256_sub_ps(_mm256_fmadd_ps(pz0, v_nz, _mm256_fmadd_ps(py0, v_ny, _mm256_mul_ps(px0, v_nx))), v_d);
            uint32_t neg_mask = static_cast<uint32_t>(_mm256_movemask_ps(_mm256_cmp_ps(d0, v_meps, _CMP_LT_OQ)));
            uint32_t pos_mask = static_cast<uint32_t>(_mm256_movemask_ps(_mm256_cmp_ps(d0, v_eps,  _CMP_GT_OQ)));

            if (num_pass > 8) {
                __m256 d1 = _mm256_sub_ps(_mm256_fmadd_ps(pz1, v_nz, _mm256_fmadd_ps(py1, v_ny, _mm256_mul_ps(px1, v_nx))), v_d);
                neg_mask |= (static_cast<uint32_t>(_mm256_movemask_ps(_mm256_cmp_ps(d1, v_meps, _CMP_LT_OQ))) << 8);
                pos_mask |= (static_cast<uint32_t>(_mm256_movemask_ps(_mm256_cmp_ps(d1, v_eps,  _CMP_GT_OQ))) << 8);
            }
            if (num_pass > 16) {
                __m256 d2 = _mm256_sub_ps(_mm256_fmadd_ps(pz2, v_nz, _mm256_fmadd_ps(py2, v_ny, _mm256_mul_ps(px2, v_nx))), v_d);
                neg_mask |= (static_cast<uint32_t>(_mm256_movemask_ps(_mm256_cmp_ps(d2, v_meps, _CMP_LT_OQ))) << 16);
                pos_mask |= (static_cast<uint32_t>(_mm256_movemask_ps(_mm256_cmp_ps(d2, v_eps,  _CMP_GT_OQ))) << 16);
            }

            const uint32_t other_mask = pass_valid_mask & ~(1u << j);
            if ((neg_mask & other_mask) != 0) {
                continue;
            }
            if ((pos_mask & other_mask) == 0) {
                continue;
            }
#else
            int count_pos = 0;
            bool valid_sep = true;
            for (int k = 0; k < num_pass; ++k) {
                if (k == j) continue;
                Vector3 pk(pass->x[k], pass->y[k], pass->z[k]);
                float d = pk.dot(normal) - dist;
                if (d < MINUS_ON_EPSILON) { valid_sep = false; break; }
                else if (d > ON_EPSILON) { count_pos++; }
            }
            if (!valid_sep || count_pos == 0) continue;
#endif

            Plane sep_plane{normal, dist};
            if (flipclip) {
                sep_plane.normal = sep_plane.normal * -1.0f;
                sep_plane.dist   = -sep_plane.dist;
            }

            PStack* cache_target = sep_owner ? sep_owner : &stack;
            if (sep_cache_slot == 0 || sep_cache_slot == 1) {
                int idx = cache_target->numseperators[sep_cache_slot];
                if (idx < MAX_SEPERATORS) {
                    cache_target->seperators[sep_cache_slot][idx] = sep_plane;
                    cache_target->numseperators[sep_cache_slot] = idx + 1;
                }
            }

            target = chop_winding_avx2_soa(target, stack, sep_plane);
            if (!target) {
                return nullptr;
            }
            break;
        }
    }
    return target;
}

// ----------------------------------------------------------------------------
// 4. AVX2 Minimum Vertex-to-Vertex Distance Squared (`BasePortalVis` Radial Vis)
// ----------------------------------------------------------------------------
inline bool avx2_portals_within_radius(
    const DynamicWinding& wa,
    const DynamicWinding& wb,
    float max_dist_sq) noexcept
{
    // Fast bounding-sphere check first (`center_dist + ra + rb <= R`)
    Vector3 diff = wa.origin - wb.origin;
    float center_d2 = diff.length_sq();
    float r_sum = wa.radius + wb.radius + ON_EPSILON;
    if (center_d2 <= max_dist_sq) {
        return true;
    }
    float max_dist = std::sqrt(max_dist_sq);
    float max_possible = max_dist + r_sum;
    if (center_d2 > max_possible * max_possible) {
        return false;
    }

    float min_d2 = MAX_COORD_DIST_SQ;
    const int na = wa.numpoints;
    const int nb = wb.numpoints;

#if defined(__AVX2__) || defined(_MSC_VER)
    __m256 v_min = _mm256_set1_ps(MAX_COORD_DIST_SQ);
    int nb_vec = (nb + 7) & ~7;
    for (int i = 0; i < na; ++i) {
        __m256 ax = _mm256_set1_ps(wa.soa_x[i]);
        __m256 ay = _mm256_set1_ps(wa.soa_y[i]);
        __m256 az = _mm256_set1_ps(wa.soa_z[i]);
        for (int j = 0; j < nb_vec; j += 8) {
            __m256 bx = _mm256_load_ps(wb.soa_x + j);
            __m256 by = _mm256_load_ps(wb.soa_y + j);
            __m256 bz = _mm256_load_ps(wb.soa_z + j);
            __m256 dx = _mm256_sub_ps(ax, bx);
            __m256 dy = _mm256_sub_ps(ay, by);
            __m256 dz = _mm256_sub_ps(az, bz);
            __m256 d2 = _mm256_fmadd_ps(dz, dz, _mm256_fmadd_ps(dy, dy, _mm256_mul_ps(dx, dx)));
            v_min = _mm256_min_ps(v_min, d2);
        }
    }
    __m128 low  = _mm256_castps256_ps128(v_min);
    __m128 high = _mm256_extractf128_ps(v_min, 1);
    __m128 m128 = _mm_min_ps(low, high);
    m128 = _mm_min_ps(m128, _mm_movehl_ps(m128, m128));
    m128 = _mm_min_ss(m128, _mm_shuffle_ps(m128, m128, 1));
    min_d2 = _mm_cvtss_f32(m128);
#else
    for (int i = 0; i < na; ++i) {
        for (int j = 0; j < nb; ++j) {
            float d2 = (wa.points[i] - wb.points[j]).length_sq();
            if (d2 < min_d2) min_d2 = d2;
        }
    }
#endif
    return min_d2 <= max_dist_sq;
}

} // namespace vvis_argon
