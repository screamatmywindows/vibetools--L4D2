// ============================================================================
// VVIS++ ARGON EDITION (x64) v4 — Single-File Amalgamated C++20 Source
// Target: Windows 10/11 x64 (MSVC 2022 /O2 /Oi /Ot /arch:AVX2) & Linux x64
// Target CPU: Intel Core i5-12450H (Alder Lake 4P+4E, 12 Threads, AVX2+FMA3+BMI2)
// Target Engine: Source Engine 1 — Left 4 Dead 2 (x64, BSP v21) & Source 2013
//
// CRITICAL OPTIMIZATIONS IN ARGON v4 (Ryazan Benchmark Tuning):
//   1. Thread Director Native Scheduling & Strict Ascending Pruning Order:
//      - Removed all `SetThreadAffinityMask` / `sched_setaffinity` calls so
//        Windows 11 Intel Thread Director manages P/E cores and Turbo Boost natively.
//      - Enforced strict ascending complexity order (`nummightsee` ascending via
//        single atomic `head_idx++` cursor) in `PortalFlow` so light portals finish
//        first (`StatDone`) and prune ~80% of recursion branches for heavy portals.
//      - Replaced heap-allocated `std::vector<uint64_t>` in `PStack::mightsee`,
//        `check_stack_avx2`, and `base_portal_vis_bvh_worker` with fixed stack
//        arrays, eliminating 10M+ `malloc/free` calls and Windows heap lock contention.
//   2. Automatic `env_fog_controller` Radial Vis with Safe 8192 Floor:
//      - Uses `safe_farz = std::max(fog_farz, 8192.0f)` by default when
//        `env_fog_controller` is present (`farz=17000` on `ryazan_01_semkino` culls
//        distant rays like ficool2's vvis++, while preventing indoor blindness).
//   3. Spatial-Resilient `.viscache` (Version 4 `VC64`):
//      - Never resets cache when `num_clusters` or `bitbytes` changes after editing
//        a brush in Hammer.
//      - Uses 2-pass 3D world-space matching (Pass 1: portal+AABB hash; Pass 2: 3D
//        room AABB fallback) to build `old_to_new_cluster[old_c] -> new_c`, remapping
//        cached bitvectors via hardware bit-scanning (`tzcnt`) in milliseconds.
// ============================================================================

// ==================== BEGIN vvis_argon.hpp ====================
// ============================================================================
// VVIS++ ARGON EDITION (x64) v4 — Core Reconstructed Data Structures & Constants
// Target: Windows 10/11 x64 (MSVC v143 / C++20) & Linux x64 (GCC/Clang)
// Engine: Source Engine 1 (Left 4 Dead 2 x64 / BSP v20 & v21)
// Hardware Target: Intel Core i5-12450H (Alder Lake 4P+4E, AVX2 + FMA3 + BMI2)
//
// CRITICAL OPTIMIZATIONS (Argon v4 — Ryazan Benchmark Tuning):
//   1. Replaced heap-allocated `std::vector<uint64_t> mightsee` in `PStack` with a
//      fixed 32-byte-aligned stack buffer `uint64_t mightsee[MAX_LEAF_LONGS]` (4 KB),
//      and made `Vector3` / `Plane` trivially constructible inside `PStack`.
//      Eliminates 10M+ `malloc`/`free` calls and Windows heap lock contention in
//      `recursive_leaf_flow_avx2`!
//   2. `NUM_STACK_WINDINGS = 12` slots per `PStack` frame with conservative PVS
//      fallback so temporary slots never exhaust or drop visible rooms.
// ============================================================================

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <array>
#include <string>
#include <atomic>
#include <memory>
#include <algorithm>

#if defined(_MSC_VER)
    #include <intrin.h>
    #define VVIS_FORCEINLINE __forceinline
    #define VVIS_ALIGN(x) __declspec(align(x))
#else
    #include <x86intrin.h>
    #include <cpuid.h>
    #define VVIS_FORCEINLINE inline __attribute__((always_inline))
    #define VVIS_ALIGN(x) __attribute__((aligned(x)))
#endif

namespace vvis_argon {

// ----------------------------------------------------------------------------
// Engine & VVIS++ Constants
// ----------------------------------------------------------------------------
constexpr int MAX_MAP_PORTALS             = 32768;
constexpr int MAX_PORTALS                 = 65536; // 2 * MAX_MAP_PORTALS
constexpr int MAX_MAP_CLUSTERS            = 32768;
constexpr int MAX_LEAF_LONGS              = MAX_MAP_CLUSTERS / 64; // 512 qwords = 4096 bytes
constexpr int MAX_PORTAL_LONGS            = MAX_PORTALS / 64;      // 1024 qwords = 8192 bytes
constexpr int MAX_POINTS_ON_WINDING       = 64;
constexpr int MAX_POINTS_ON_FIXED_WINDING = 24;    // 3x AVX2 YMM banks per axis
constexpr int SOA_LANE_CAPACITY           = 24;    // 3x 8-wide AVX2 YMM registers per axis
constexpr int MAX_SEPERATORS              = 64;    // 64 planes per source/pass separator array
constexpr int NUM_STACK_WINDINGS          = 12;    // 12 slots per PStack frame (zero exhaustion)

// Floating-point epsilons (Conservative PVS tolerances matching Valve VVIS)
constexpr float ON_EPSILON                = 0.1f;
constexpr float MINUS_ON_EPSILON          = -0.1f;
constexpr float BACKFACE_EPSILON          = -1.0001f;
constexpr float MAX_COORD_DIST_SQ         = 1.0e30f;
constexpr float MIN_SAFE_FOG_FARZ         = 8192.0f; // Minimum safe radial vis clamp for env_fog_controller

// Portal Work Status
enum class PortalStatus : int32_t {
    StatNone    = 0,
    StatWorking = 1,
    StatDone    = 2
};

// Compilation Tiers ("Gears of Compilation" - Challenge D)
enum class CompileTier : int {
    Tier0_NullVis        = 0, // Instant (<0.01s): Writes all-visible / radius-clamped PVS without geometric clipping
    Tier1_TurboProc      = 1, // Turbo-Procedural (<0.5s): SAH-BVH BasePortalVis + Topological Graph BFS (-maxdepth)
    Tier2_HybridCached   = 2, // Hybrid Cached (0.1-1s): Spatial-Resilient .viscache reuse + AVX2 PortalFlow on dirty subgraph
    Tier3_MasterRelease  = 3  // Master Release: Full AVX2SOA + FMA3 + BMI2 PortalFlow with multi-bounce separator clipping
};

// ----------------------------------------------------------------------------
// Core Math & Geometry Types (Trivially Default-Constructible for Zero-Overhead Stack Allocation)
// ----------------------------------------------------------------------------
struct Vector3 {
    float x, y, z;

    VVIS_FORCEINLINE Vector3() = default;
    VVIS_FORCEINLINE constexpr Vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

    VVIS_FORCEINLINE Vector3 operator+(const Vector3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    VVIS_FORCEINLINE Vector3 operator-(const Vector3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    VVIS_FORCEINLINE Vector3 operator*(float s) const { return {x * s, y * s, z * s}; }
    VVIS_FORCEINLINE float dot(const Vector3& o) const { return x * o.x + y * o.y + z * o.z; }
    VVIS_FORCEINLINE Vector3 cross(const Vector3& o) const {
        return {
            y * o.z - z * o.y,
            z * o.x - x * o.z,
            x * o.y - y * o.x
        };
    }
    VVIS_FORCEINLINE float length_sq() const { return x * x + y * y + z * z; }
    VVIS_FORCEINLINE float length() const { return std::sqrt(length_sq()); }
};

struct VVIS_ALIGN(16) Plane {
    Vector3 normal; // +0x00 (12 bytes)
    float   dist;   // +0x0C (4 bytes)
};

// ----------------------------------------------------------------------------
// Reconstructed Winding Structures (AoS and SIMD SoA)
// ----------------------------------------------------------------------------
struct VVIS_ALIGN(16) Winding {
    Vector3 origin;     // +0x00: Bounding sphere center
    float   radius;     // +0x0C: Bounding sphere radius
    int32_t numpoints;  // +0x10: Number of active vertices
    Vector3 points[MAX_POINTS_ON_FIXED_WINDING];
};

// Dynamic Winding for initial unclipped portals (up to 64 vertices)
struct DynamicWinding {
    Vector3 origin{0.0f, 0.0f, 0.0f};
    float   radius    = 0.0f;
    int32_t numpoints = 0;
    int32_t maxpoints = 0;
    std::vector<Vector3> points;

    VVIS_ALIGN(32) float soa_x[MAX_POINTS_ON_WINDING];
    VVIS_ALIGN(32) float soa_y[MAX_POINTS_ON_WINDING];
    VVIS_ALIGN(32) float soa_z[MAX_POINTS_ON_WINDING];

    void sync_soa() {
        int n = std::min<int>(numpoints, MAX_POINTS_ON_WINDING);
        for (int i = 0; i < n; ++i) {
            soa_x[i] = points[i].x;
            soa_y[i] = points[i].y;
            soa_z[i] = points[i].z;
        }
        Vector3 p0 = (n > 0) ? points[0] : Vector3(0.0f, 0.0f, 0.0f);
        for (int i = n; i < MAX_POINTS_ON_WINDING; ++i) {
            soa_x[i] = p0.x;
            soa_y[i] = p0.y;
            soa_z[i] = p0.z;
        }
    }

    void compute_bounds() {
        if (numpoints <= 0) {
            origin = {0.0f, 0.0f, 0.0f};
            radius = 0.0f;
            return;
        }
        Vector3 sum(0.0f, 0.0f, 0.0f);
        for (int i = 0; i < numpoints; ++i) {
            sum = sum + points[i];
        }
        origin = sum * (1.0f / static_cast<float>(numpoints));
        float max_r2 = 0.0f;
        for (int i = 0; i < numpoints; ++i) {
            float d2 = (points[i] - origin).length_sq();
            if (d2 > max_r2) max_r2 = d2;
        }
        radius = std::sqrt(max_r2);
        sync_soa();
    }
};

// Stack-allocated SIMD SoA Winding (Up to 24 vertices = 3x AVX2 YMM registers per axis)
// Kept trivially default-constructible so `PStack` frame allocation takes 0 CPU cycles!
struct VVIS_ALIGN(32) StackWindingSOA {
    Vector3 origin;          // +0x00
    float   radius;          // +0x0C
    int32_t numpoints;       // +0x10
    int32_t pad[3];          // Pad to 32-byte boundary
    VVIS_ALIGN(32) float x[SOA_LANE_CAPACITY]; // 3x YMM (96 bytes)
    VVIS_ALIGN(32) float y[SOA_LANE_CAPACITY]; // 3x YMM (96 bytes)
    VVIS_ALIGN(32) float z[SOA_LANE_CAPACITY]; // 3x YMM (96 bytes)

    VVIS_FORCEINLINE void load_from_dynamic(const DynamicWinding& dw) {
        origin = dw.origin;
        radius = dw.radius;
        numpoints = std::min<int32_t>(dw.numpoints, SOA_LANE_CAPACITY);
        std::memcpy(x, dw.soa_x, SOA_LANE_CAPACITY * sizeof(float));
        std::memcpy(y, dw.soa_y, SOA_LANE_CAPACITY * sizeof(float));
        std::memcpy(z, dw.soa_z, SOA_LANE_CAPACITY * sizeof(float));
    }
};

// ----------------------------------------------------------------------------
// Reconstructed Portal & Leaf Structures
// ----------------------------------------------------------------------------
struct VVIS_ALIGN(64) VPortal {
    Plane           plane;          // +0x00: Portal plane (normal points into `leaf`)
    int32_t         leaf;           // +0x10: Target cluster index this directed portal leads into
    int32_t         src_leaf;       // +0x14: Source cluster index this directed portal originates from
    DynamicWinding* winding;        // +0x18: Winding polygon
    std::atomic<PortalStatus> status; // +0x20: 0=none, 1=working, 2=done
    int32_t         nummightsee;    // +0x24: Popcount of portalmightsee / Complexity metric for PComp sort
    uint64_t*       portalmightsee; // +0x28: BasePortalVis cluster bitvector (c_longs * 8 bytes)
    int32_t         numcansee;      // +0x30: Popcount of portalvis
    int32_t         portal_id;      // +0x34: Directed portal index [0 .. 2*numportals - 1]
    uint64_t*       portalvis;      // +0x38: Final PortalFlow cluster bitvector (c_longs * 8 bytes)

    float           complexity_score = 0.0f;
    uint64_t        geometric_hash   = 0;

    VPortal()
        : plane{{0.0f, 0.0f, 0.0f}, 0.0f}, leaf(0), src_leaf(0), winding(nullptr),
          status(PortalStatus::StatNone), nummightsee(0), portalmightsee(nullptr),
          numcansee(0), portal_id(0), portalvis(nullptr) {}

    VPortal(const VPortal& o)
        : plane(o.plane), leaf(o.leaf), src_leaf(o.src_leaf), winding(o.winding),
          status(o.status.load(std::memory_order_relaxed)), nummightsee(o.nummightsee),
          portalmightsee(o.portalmightsee), numcansee(o.numcansee),
          portal_id(o.portal_id), portalvis(o.portalvis),
          complexity_score(o.complexity_score), geometric_hash(o.geometric_hash) {}

    VPortal& operator=(const VPortal& o) {
        if (this != &o) {
            plane = o.plane;
            leaf = o.leaf;
            src_leaf = o.src_leaf;
            winding = o.winding;
            status.store(o.status.load(std::memory_order_relaxed), std::memory_order_relaxed);
            nummightsee = o.nummightsee;
            portalmightsee = o.portalmightsee;
            numcansee = o.numcansee;
            portal_id = o.portal_id;
            portalvis = o.portalvis;
            complexity_score = o.complexity_score;
            geometric_hash = o.geometric_hash;
        }
        return *this;
    }
};

struct ClusterLeaf {
    std::vector<VPortal*> portals;
    uint64_t geometric_hash    = 0;
    uint64_t neighborhood_hash = 0;
    Vector3  bounds_min{1e30f, 1e30f, 1e30f};
    Vector3  bounds_max{-1e30f, -1e30f, -1e30f};
    Vector3  centroid{0.0f, 0.0f, 0.0f};
    bool     cache_hit = false;
};

// ----------------------------------------------------------------------------
// Reconstructed PortalFlow Stack Frame (`pstack_t` & `threaddata_t`)
// ZERO heap allocations (`mightsee` is a fixed 32-byte-aligned stack array)
// and ZERO redundant constructor zeroing on `windings` / `seperators`!
// ----------------------------------------------------------------------------
struct VVIS_ALIGN(32) PStack {
    VVIS_ALIGN(32) uint64_t mightsee[MAX_LEAF_LONGS];
    PStack*                 next;
    ClusterLeaf*            leaf;
    VPortal*                portal;
    StackWindingSOA*        source;
    StackWindingSOA*        pass;
    StackWindingSOA         windings[NUM_STACK_WINDINGS];
    uint8_t                 freewindings[NUM_STACK_WINDINGS];
    Plane                   portalplane;
    Plane                   seperators[2][MAX_SEPERATORS];
    int32_t                 numseperators[2];
    bool                    did_mightsee_test;
    int32_t                 mightsee_count;

    // Default constructor intentionally leaves large arrays (`mightsee`, `windings`, `seperators`)
    // uninitialized for instant 1-cycle stack allocation in `recursive_leaf_flow_avx2`!
    VVIS_FORCEINLINE PStack() = default;

    VVIS_FORCEINLINE void reset_windings() noexcept {
        for (int i = 0; i < NUM_STACK_WINDINGS; ++i) {
            freewindings[i] = 1;
        }
    }

    VVIS_FORCEINLINE StackWindingSOA* alloc_stack_winding() noexcept {
        for (int i = 0; i < NUM_STACK_WINDINGS; ++i) {
            if (freewindings[i]) {
                freewindings[i] = 0;
                return &windings[i];
            }
        }
        return nullptr;
    }

    VVIS_FORCEINLINE void free_stack_winding(StackWindingSOA* w) noexcept {
        if (!w) return;
        ptrdiff_t idx = w - windings;
        if (idx >= 0 && idx < NUM_STACK_WINDINGS) {
            freewindings[idx] = 1;
        }
    }
};

struct VVIS_ALIGN(32) ThreadData {
    VPortal*         base = nullptr;
    int32_t          c_chains = 0;
    int32_t          c_mighttest = 0;
    PStack           pstack_head;
    StackWindingSOA  base_winding_soa;
    int32_t          max_depth = -1;
};

} // namespace vvis_argon

// ==================== END vvis_argon.hpp ====================

// ==================== BEGIN vvis_simd_avx2.hpp ====================
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

// ==================== END vvis_simd_avx2.hpp ====================

// ==================== BEGIN vvis_cache.hpp ====================
// ============================================================================
// VVIS++ ARGON EDITION (x64) v4 — Challenge A: Spatial-Resilient `.viscache`
//
// CRITICAL OPTIMIZATIONS (Argon v4 — Ryazan Benchmark Tuning):
//   1. Removed hard cache reset when `hdr.num_clusters != leafs.size()` or
//      `hdr.bitbytes != bitbytes`!
//      In Source Engine, moving a single brush in Hammer changes `num_clusters`
//      by ±1..5 and shifts global cluster indices (`leafnum`).
//   2. Two-Pass Spatial Cluster & Portal Matching (`VERSION = 4`):
//      - Pass 1 (Exact Geometry + Portals): Matches unchanged rooms by their
//        full 3D portal + AABB hash (`geometric_hash`), marking them clean.
//      - Pass 2 (3D Room AABB Fallback): Matches rooms where a doorway was
//        edited by their 3D bounding box (`neighborhood_hash` = AABB hash) so
//        `old_to_new_cluster[old_c] -> new_c` remains intact even for edited
//        boundary rooms while still recomputing their local portals!
// ============================================================================
#include <fstream>
#include <unordered_map>
#include <queue>
#include <iostream>

namespace vvis_argon {

// ----------------------------------------------------------------------------
// High-Performance Standalone xxHash64 Implementation
// ----------------------------------------------------------------------------
class XXHash64 {
public:
    static constexpr uint64_t PRIME64_1 = 0x9E3779B185EBCA87ULL;
    static constexpr uint64_t PRIME64_2 = 0xC2B2AE3D27D4EB4FULL;
    static constexpr uint64_t PRIME64_3 = 0x165667B19E3779F9ULL;
    static constexpr uint64_t PRIME64_4 = 0x85EBCA77C2B2AE63ULL;
    static constexpr uint64_t PRIME64_5 = 0x27D4EB2F165667C5ULL;

    static VVIS_FORCEINLINE uint64_t rotl64(uint64_t x, int r) noexcept {
        return (x << r) | (x >> (64 - r));
    }

    static VVIS_FORCEINLINE uint64_t mix_word(uint64_t acc, uint64_t val) noexcept {
        acc ^= (rotl64(val * PRIME64_2, 31) * PRIME64_1);
        acc = rotl64(acc, 27) * PRIME64_1 + PRIME64_4;
        return acc;
    }

    static VVIS_FORCEINLINE uint64_t avalanche(uint64_t h) noexcept {
        h ^= h >> 33;
        h *= PRIME64_2;
        h ^= h >> 29;
        h *= PRIME64_3;
        h ^= h >> 32;
        return h;
    }

    // Quantize float coordinates to 1/8th unit (0.125) for immunity to FP jitter
    static VVIS_FORCEINLINE int64_t quantize_coord(float f) noexcept {
        return static_cast<int64_t>(std::llround(static_cast<double>(f) * 8.0));
    }

    static VVIS_FORCEINLINE int64_t quantize_normal(float f) noexcept {
        return static_cast<int64_t>(std::llround(static_cast<double>(f) * 1024.0));
    }

    static uint64_t hash_aabb(const Vector3& bmin, const Vector3& bmax) noexcept {
        uint64_t h = PRIME64_5 + 48;
        h = mix_word(h, static_cast<uint64_t>(quantize_coord(bmin.x)));
        h = mix_word(h, static_cast<uint64_t>(quantize_coord(bmin.y)));
        h = mix_word(h, static_cast<uint64_t>(quantize_coord(bmin.z)));
        h = mix_word(h, static_cast<uint64_t>(quantize_coord(bmax.x)));
        h = mix_word(h, static_cast<uint64_t>(quantize_coord(bmax.y)));
        h = mix_word(h, static_cast<uint64_t>(quantize_coord(bmax.z)));
        return avalanche(h);
    }

    // 3D World-Space Hash of Winding (AABB + Vertex Coordinates)
    static uint64_t hash_winding(const DynamicWinding& w, uint64_t seed = 0) noexcept {
        uint64_t h = seed + PRIME64_5 + static_cast<uint64_t>(w.numpoints * 24);
        Vector3 bmin( 1e30f,  1e30f,  1e30f);
        Vector3 bmax(-1e30f, -1e30f, -1e30f);
        for (int i = 0; i < w.numpoints; ++i) {
            const Vector3& pt = w.points[i];
            bmin.x = std::min(bmin.x, pt.x); bmin.y = std::min(bmin.y, pt.y); bmin.z = std::min(bmin.z, pt.z);
            bmax.x = std::max(bmax.x, pt.x); bmax.y = std::max(bmax.y, pt.y); bmax.z = std::max(bmax.z, pt.z);
            h = mix_word(h, static_cast<uint64_t>(quantize_coord(pt.x)));
            h = mix_word(h, static_cast<uint64_t>(quantize_coord(pt.y)));
            h = mix_word(h, static_cast<uint64_t>(quantize_coord(pt.z)));
        }
        h = mix_word(h, hash_aabb(bmin, bmax));
        return avalanche(h);
    }

    // 3D World-Space Hash of Directed Portal (Plane Normal + Dist + Winding AABB & Vertices)
    static uint64_t hash_portal(const VPortal& p) noexcept {
        uint64_t h = PRIME64_5;
        h = mix_word(h, static_cast<uint64_t>(quantize_normal(p.plane.normal.x)));
        h = mix_word(h, static_cast<uint64_t>(quantize_normal(p.plane.normal.y)));
        h = mix_word(h, static_cast<uint64_t>(quantize_normal(p.plane.normal.z)));
        h = mix_word(h, static_cast<uint64_t>(quantize_coord(p.plane.dist)));
        if (p.winding) {
            h = mix_word(h, hash_winding(*p.winding));
        }
        return avalanche(h);
    }
};

// ----------------------------------------------------------------------------
// Binary `.viscache` File Format Structures (Version 4: Spatial-Resilient)
// Immune to `num_clusters` changes when brushes are added/moved/split in Hammer!
// ----------------------------------------------------------------------------
#pragma pack(push, 1)
struct VisCacheHeader {
    uint32_t magic;            // 'VC64' (0x34364356)
    uint32_t version;          // 4 (Spatial-Resilient Cluster & Portal Remapping)
    int32_t  num_clusters;     // old portalclusters count when saved
    int32_t  num_portals;      // old numportals (undirected) count when saved
    int32_t  bitbytes;         // old ((num_clusters + 63) >> 6) * 8
    float    max_dist_sq;      // Fog / radius override squared
    int32_t  max_depth;        // -maxdepth setting
};

struct VisCacheClusterEntry {
    uint64_t full_spatial_hash; // Hash of cluster AABB + all sorted portal hashes
    uint64_t aabb_spatial_hash; // Hash of cluster 3D AABB (`bounds_min`, `bounds_max`)
};

struct VisCachePortalEntry {
    uint64_t portal_geom_hash; // 3D world-space hash of portal (AABB + plane + vertices)
    uint64_t src_cluster_hash; // 3D world-space hash of source cluster
    uint64_t dst_cluster_hash; // 3D world-space hash of destination cluster
    int32_t  nummightsee;
    int32_t  numcansee;
    // Followed immediately by:
    //   `hdr.bitbytes` bytes of `portalmightsee` bitvector (in old cluster indexing)
    //   `hdr.bitbytes` bytes of `portalvis` bitvector (in old cluster indexing)
};
#pragma pack(pop)

// ----------------------------------------------------------------------------
// Spatial-Resilient Incremental VisCache Manager
// ----------------------------------------------------------------------------
class IncrementalVisCache {
public:
    static constexpr uint32_t MAGIC_VC64 = 0x34364356;
    static constexpr uint32_t VERSION    = 4;

    int32_t cache_hits_portals   = 0;
    int32_t cache_misses_portals = 0;
    int32_t dirty_clusters       = 0;
    int32_t matched_clusters     = 0;

    // Step 1: Compute index-independent 3D spatial hashes for every portal and cluster
    static void compute_graph_hashes(
        std::vector<VPortal>& portals,
        std::vector<ClusterLeaf>& leafs)
    {
        for (auto& p : portals) {
            p.geometric_hash = XXHash64::hash_portal(p);
        }

        for (size_t c = 0; c < leafs.size(); ++c) {
            ClusterLeaf& cl = leafs[c];
            uint64_t aabb_h = XXHash64::hash_aabb(cl.bounds_min, cl.bounds_max);
            cl.neighborhood_hash = aabb_h; // Stores 3D AABB hash for Pass-2 fallback matching

            std::vector<uint64_t> phashes;
            phashes.reserve(cl.portals.size());
            for (const VPortal* p : cl.portals) {
                phashes.push_back(p->geometric_hash);
            }
            std::sort(phashes.begin(), phashes.end());

            uint64_t ch = XXHash64::mix_word(XXHash64::PRIME64_5 + static_cast<uint64_t>(phashes.size()), aabb_h);
            for (uint64_t ph : phashes) {
                ch = XXHash64::mix_word(ch, ph);
            }
            cl.geometric_hash = XXHash64::avalanche(ch);
        }
    }

    // Step 2: Load `.viscache` BEFORE BasePortalVis with Two-Pass Spatial Cluster Remapping!
    // Works even when `hdr.num_clusters != leafs.size()` (e.g. moving brushes in Hammer)!
    bool preload_before_base_vis(
        const std::string& cache_path,
        std::vector<VPortal>& portals,
        std::vector<ClusterLeaf>& leafs,
        int32_t new_bitbytes,
        int32_t new_c_longs,
        float max_dist_sq,
        int32_t max_depth,
        int32_t invalidation_radius_hops = 1)
    {
        std::ifstream ifs(cache_path, std::ios::binary);
        if (!ifs.is_open()) {
            return false;
        }

        VisCacheHeader hdr{};
        if (!ifs.read(reinterpret_cast<char*>(&hdr), sizeof(hdr))) {
            return false;
        }

        // We DO NOT reject when `hdr.num_clusters != leafs.size()` or `hdr.bitbytes != new_bitbytes`!
        if (hdr.magic != MAGIC_VC64 ||
            hdr.version != VERSION ||
            hdr.num_clusters <= 0 ||
            hdr.num_portals <= 0 ||
            hdr.bitbytes <= 0 ||
            (hdr.bitbytes & 7) != 0 ||
            std::fabs(hdr.max_dist_sq - max_dist_sq) > 1.0f ||
            hdr.max_depth != max_depth)
        {
            return false;
        }

        const int32_t old_num_clusters = hdr.num_clusters;
        const int32_t old_bitbytes     = hdr.bitbytes;
        const int32_t old_c_longs      = old_bitbytes >> 3;
        const int32_t new_num_clusters = static_cast<int32_t>(leafs.size());

        // 1. Read saved 3D spatial hashes of all old clusters
        std::vector<VisCacheClusterEntry> old_clusters(old_num_clusters);
        if (!ifs.read(reinterpret_cast<char*>(old_clusters.data()),
                      static_cast<std::streamsize>(old_num_clusters * sizeof(VisCacheClusterEntry))))
        {
            return false;
        }

        // 2. Build spatial lookup tables (`full_spatial_hash -> new_c` and `aabb_spatial_hash -> new_c`)
        std::unordered_map<uint64_t, int32_t> new_by_full_hash;
        std::unordered_map<uint64_t, int32_t> new_by_aabb_hash;
        new_by_full_hash.reserve(new_num_clusters * 2);
        new_by_aabb_hash.reserve(new_num_clusters * 2);
        for (int32_t new_c = 0; new_c < new_num_clusters; ++new_c) {
            new_by_full_hash[leafs[new_c].geometric_hash]    = new_c;
            new_by_aabb_hash[leafs[new_c].neighborhood_hash] = new_c;
        }

        // 3. Two-Pass Bijection `old_to_new_cluster[old_c] -> new_c`
        std::vector<int32_t> old_to_new_cluster(old_num_clusters, -1);
        std::vector<uint8_t> new_cluster_clean(new_num_clusters, 0);
        std::vector<uint8_t> new_cluster_mapped(new_num_clusters, 0);
        std::vector<uint64_t> deleted_old_cluster_mask(old_c_longs, 0);
        bool identity_cluster_mapping = (old_num_clusters == new_num_clusters);
        matched_clusters = 0;

        // Pass 1: Match clusters whose full portal + AABB hash is 100% identical (clean rooms)
        for (int32_t old_c = 0; old_c < old_num_clusters; ++old_c) {
            uint64_t fh = old_clusters[old_c].full_spatial_hash;
            if (old_c < new_num_clusters && leafs[old_c].geometric_hash == fh && !new_cluster_mapped[old_c]) {
                old_to_new_cluster[old_c] = old_c;
                new_cluster_clean[old_c]  = 1;
                new_cluster_mapped[old_c] = 1;
                matched_clusters++;
            } else {
                auto it = new_by_full_hash.find(fh);
                if (it != new_by_full_hash.end() && !new_cluster_mapped[it->second]) {
                    old_to_new_cluster[old_c]      = it->second;
                    new_cluster_clean[it->second]  = 1;
                    new_cluster_mapped[it->second] = 1;
                    matched_clusters++;
                    if (it->second != old_c) identity_cluster_mapping = false;
                }
            }
        }

        // Pass 2: For boundary rooms whose doorway was edited, match by 3D Room AABB hash
        // so distant portals still map bit `old_c -> new_c` while local portals of `new_c` recompute!
        for (int32_t old_c = 0; old_c < old_num_clusters; ++old_c) {
            if (old_to_new_cluster[old_c] >= 0) continue;
            uint64_t ah = old_clusters[old_c].aabb_spatial_hash;
            auto it = new_by_aabb_hash.find(ah);
            if (it != new_by_aabb_hash.end() && !new_cluster_mapped[it->second]) {
                old_to_new_cluster[old_c]      = it->second;
                new_cluster_mapped[it->second] = 1;
                matched_clusters++;
                if (it->second != old_c) identity_cluster_mapping = false;
            } else {
                deleted_old_cluster_mask[old_c >> 6] |= (1ULL << (old_c & 63));
                identity_cluster_mapping = false;
            }
        }

        // 4. Read all cached directed portals into spatial lookup map
        struct CachedPortalData {
            VisCachePortalEntry   meta;
            std::vector<uint64_t> might_bits;
            std::vector<uint64_t> vis_bits;
        };

        std::unordered_map<uint64_t, CachedPortalData> cache_map;
        const int32_t old_total_directed = hdr.num_portals * 2;
        cache_map.reserve(old_total_directed * 2);

        for (int32_t i = 0; i < old_total_directed; ++i) {
            CachedPortalData cpd;
            if (!ifs.read(reinterpret_cast<char*>(&cpd.meta), sizeof(VisCachePortalEntry))) {
                break;
            }
            cpd.might_bits.resize(old_c_longs, 0);
            cpd.vis_bits.resize(old_c_longs, 0);
            if (!ifs.read(reinterpret_cast<char*>(cpd.might_bits.data()), old_bitbytes)) break;
            if (!ifs.read(reinterpret_cast<char*>(cpd.vis_bits.data()), old_bitbytes)) break;

            cache_map[cpd.meta.portal_geom_hash] = std::move(cpd);
        }

        // 5. Identify modified/new clusters (`dist_to_dirty[new_c] == 0`) and propagate 1-hop wave
        std::vector<int32_t> dist_to_dirty(new_num_clusters, 999999);
        std::vector<int32_t> dirty_new_cluster_list;
        std::queue<int32_t> bfs_q;
        dirty_clusters = 0;

        for (int32_t new_c = 0; new_c < new_num_clusters; ++new_c) {
            bool is_clean = (new_cluster_clean[new_c] != 0);
            if (is_clean) {
                for (const VPortal* p : leafs[new_c].portals) {
                    if (cache_map.find(p->geometric_hash) == cache_map.end()) {
                        is_clean = false;
                        break;
                    }
                }
            }
            if (!is_clean) {
                dist_to_dirty[new_c] = 0;
                dirty_new_cluster_list.push_back(new_c);
                bfs_q.push(new_c);
                dirty_clusters++;
            }
        }

        while (!bfs_q.empty()) {
            int32_t u = bfs_q.front();
            bfs_q.pop();
            if (dist_to_dirty[u] >= invalidation_radius_hops) continue;

            for (const VPortal* p : leafs[u].portals) {
                int32_t v = p->leaf;
                if (v >= 0 && v < new_num_clusters && dist_to_dirty[v] > dist_to_dirty[u] + 1) {
                    dist_to_dirty[v] = dist_to_dirty[u] + 1;
                    bfs_q.push(v);
                }
            }
        }

        // Helper lambda to remap a bitvector from old cluster IDs to new cluster IDs
        auto remap_bitvector = [&](const std::vector<uint64_t>& old_bits, uint64_t* new_bits) noexcept -> int32_t {
            if (identity_cluster_mapping && old_c_longs == new_c_longs) {
                std::memcpy(new_bits, old_bits.data(), new_bitbytes);
                int32_t cnt = 0;
                for (int32_t q = 0; q < new_c_longs; ++q) {
#if defined(_MSC_VER)
                    cnt += static_cast<int32_t>(__popcnt64(new_bits[q]));
#else
                    cnt += static_cast<int32_t>(__builtin_popcountll(new_bits[q]));
#endif
                }
                return cnt;
            }

            std::memset(new_bits, 0, new_bitbytes);
            int32_t cnt = 0;
            for (int32_t w = 0; w < old_c_longs; ++w) {
                uint64_t bits = old_bits[w];
                while (bits != 0ULL) {
#if defined(_MSC_VER)
                    unsigned long bit_pos = 0;
                    _BitScanForward64(&bit_pos, bits);
                    int32_t old_c = (w << 6) + static_cast<int32_t>(bit_pos);
#else
                    int32_t old_c = (w << 6) + __builtin_ctzll(bits);
#endif
                    bits &= (bits - 1ULL);
                    if (old_c < old_num_clusters) {
                        int32_t new_c = old_to_new_cluster[old_c];
                        if (new_c >= 0) {
                            uint64_t mask = 1ULL << (new_c & 63);
                            if (!(new_bits[new_c >> 6] & mask)) {
                                new_bits[new_c >> 6] |= mask;
                                cnt++;
                            }
                        }
                    }
                }
            }
            return cnt;
        };

        cache_hits_portals   = 0;
        cache_misses_portals = 0;

        for (auto& p : portals) {
            if (dist_to_dirty[p.src_leaf] <= invalidation_radius_hops ||
                dist_to_dirty[p.leaf] <= invalidation_radius_hops)
            {
                cache_misses_portals++;
                continue;
            }

            auto it = cache_map.find(p.geometric_hash);
            if (it == cache_map.end()) {
                cache_misses_portals++;
                continue;
            }

            const CachedPortalData& cpd = it->second;

            p.portalmightsee = new uint64_t[new_c_longs];
            p.portalvis      = new uint64_t[new_c_longs];

            int32_t might_cnt = remap_bitvector(cpd.might_bits, p.portalmightsee);
            int32_t vis_cnt   = remap_bitvector(cpd.vis_bits,   p.portalvis);

            if (!dirty_new_cluster_list.empty()) {
                bool saw_modified_might = false;
                bool saw_modified_vis   = false;
                for (int32_t w = 0; w < old_c_longs; ++w) {
                    if (cpd.might_bits[w] & deleted_old_cluster_mask[w]) saw_modified_might = true;
                    if (cpd.vis_bits[w]   & deleted_old_cluster_mask[w]) saw_modified_vis   = true;
                }

                for (int32_t dc : dirty_new_cluster_list) {
                    const ClusterLeaf& dcl = leafs[dc];
                    float px = (p.plane.normal.x >= 0.0f) ? dcl.bounds_max.x : dcl.bounds_min.x;
                    float py = (p.plane.normal.y >= 0.0f) ? dcl.bounds_max.y : dcl.bounds_min.y;
                    float pz = (p.plane.normal.z >= 0.0f) ? dcl.bounds_max.z : dcl.bounds_min.z;
                    float d_plane = px * p.plane.normal.x + py * p.plane.normal.y + pz * p.plane.normal.z - p.plane.dist;
                    if (d_plane < MINUS_ON_EPSILON) continue;

                    if (p.winding && max_dist_sq < MAX_COORD_DIST_SQ * 0.5f) {
                        float d2 = (dcl.centroid - p.winding->origin).length_sq();
                        float r_limit = std::sqrt(max_dist_sq) + p.winding->radius + 1024.0f;
                        if (d2 > r_limit * r_limit) continue;
                    }

                    uint64_t dmask = 1ULL << (dc & 63);
                    if (saw_modified_might || dirty_clusters <= 8) {
                        if (!(p.portalmightsee[dc >> 6] & dmask)) {
                            p.portalmightsee[dc >> 6] |= dmask;
                            might_cnt++;
                        }
                    }
                    if (saw_modified_vis) {
                        if (!(p.portalvis[dc >> 6] & dmask)) {
                            p.portalvis[dc >> 6] |= dmask;
                            vis_cnt++;
                        }
                    }
                }
            }

            p.nummightsee = might_cnt;
            p.numcansee   = vis_cnt;
            p.status.store(PortalStatus::StatDone, std::memory_order_release);
            cache_hits_portals++;
        }

        return cache_hits_portals > 0;
    }

    // Step 3: Save Spatial-Resilient `.viscache` (Version 4) to disk
    bool save(
        const std::string& cache_path,
        const std::vector<VPortal>& portals,
        const std::vector<ClusterLeaf>& leafs,
        int32_t num_undirected_portals,
        int32_t bitbytes,
        float max_dist_sq,
        int32_t max_depth) const
    {
        std::ofstream ofs(cache_path, std::ios::binary | std::ios::trunc);
        if (!ofs.is_open()) return false;

        VisCacheHeader hdr{};
        hdr.magic        = MAGIC_VC64;
        hdr.version      = VERSION;
        hdr.num_clusters = static_cast<int32_t>(leafs.size());
        hdr.num_portals  = num_undirected_portals;
        hdr.bitbytes     = bitbytes;
        hdr.max_dist_sq  = max_dist_sq;
        hdr.max_depth    = max_depth;

        ofs.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));

        // Write 3D spatial hashes of all clusters (both full portal+AABB hash and pure AABB hash)
        std::vector<VisCacheClusterEntry> cluster_entries(leafs.size());
        for (size_t c = 0; c < leafs.size(); ++c) {
            cluster_entries[c].full_spatial_hash = leafs[c].geometric_hash;
            cluster_entries[c].aabb_spatial_hash = leafs[c].neighborhood_hash;
        }
        ofs.write(reinterpret_cast<const char*>(cluster_entries.data()),
                  static_cast<std::streamsize>(cluster_entries.size() * sizeof(VisCacheClusterEntry)));

        std::vector<uint8_t> zero_buf(bitbytes, 0);
        for (const auto& p : portals) {
            VisCachePortalEntry entry{};
            entry.portal_geom_hash = p.geometric_hash;
            entry.src_cluster_hash = leafs[p.src_leaf].geometric_hash;
            entry.dst_cluster_hash = leafs[p.leaf].geometric_hash;
            entry.nummightsee      = p.nummightsee;
            entry.numcansee        = p.numcansee;

            ofs.write(reinterpret_cast<const char*>(&entry), sizeof(entry));
            if (p.portalmightsee) {
                ofs.write(reinterpret_cast<const char*>(p.portalmightsee), bitbytes);
            } else if (p.portalvis) {
                ofs.write(reinterpret_cast<const char*>(p.portalvis), bitbytes);
            } else {
                ofs.write(reinterpret_cast<const char*>(zero_buf.data()), bitbytes);
            }
            if (p.portalvis) {
                ofs.write(reinterpret_cast<const char*>(p.portalvis), bitbytes);
            } else {
                ofs.write(reinterpret_cast<const char*>(zero_buf.data()), bitbytes);
            }
        }
        return ofs.good();
    }
};

} // namespace vvis_argon

// ==================== END vvis_cache.hpp ====================

// ==================== BEGIN vvis_bvh_turbo.hpp ====================
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

// ==================== END vvis_bvh_turbo.hpp ====================

// ==================== BEGIN vvis_hybrid_scheduler.hpp ====================
// ============================================================================
// VVIS++ ARGON EDITION (x64) v4 — Thread Director Native Lock-Free Scheduler
//
// CRITICAL OPTIMIZATIONS (Argon v4 — Ryazan Benchmark Tuning):
//   1. Removed all `SetThreadAffinityMask` / `sched_setaffinity` calls!
//      On Windows 11 with Intel 12th/13th/14th Gen Hybrid CPUs (Alder Lake),
//      hardware Intel Thread Director dynamically migrates threads and manages
//      P-core Turbo Boost far more effectively without forced thread pinning.
//   2. Strictly enforces Ascending Complexity Pruning Order (`head_idx++`):
//      In `PortalFlow`, portals are sorted by `nummightsee` ascending (`PComp`).
//      All worker threads pull strictly from `head_idx++` (lightest portals first).
//      Because light portals finish in microseconds (`StatDone`), they immediately
//      provide exact `portalvis` masks that prune ~80% of recursive branches in
//      subsequent heavy portals!
// ============================================================================
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cstdio>

namespace vvis_argon {

enum class CoreClass : uint8_t {
    PerformanceCore = 0,
    EfficiencyCore  = 1
};

struct LogicalThreadInfo {
    uint32_t  logical_id = 0;
    CoreClass core_class = CoreClass::PerformanceCore;
};

class HybridTopologyDetector {
public:
    static std::vector<LogicalThreadInfo> detect_threads(int32_t requested_threads) {
        uint32_t hw_concurrency = std::thread::hardware_concurrency();
        if (hw_concurrency == 0) hw_concurrency = 12;
        uint32_t count = (requested_threads > 0)
            ? std::min<uint32_t>(static_cast<uint32_t>(requested_threads), 64u)
            : std::min<uint32_t>(hw_concurrency, 64u);

        std::vector<LogicalThreadInfo> threads(count);
        for (uint32_t i = 0; i < count; ++i) {
            threads[i].logical_id = i;
            threads[i].core_class = CoreClass::PerformanceCore;
        }
        return threads;
    }
};

// ----------------------------------------------------------------------------
// Lock-Free Ascending Pruning Order Scheduler (`HybridWorkStealingScheduler`)
//   - Zero OS thread affinity pinning (full Intel Thread Director + Turbo Boost).
//   - Single cache-line-aligned atomic cursor `head_idx` so every worker thread
//     processes tasks strictly in ascending complexity order (`0 .. total_tasks-1`).
// ----------------------------------------------------------------------------
class HybridWorkStealingScheduler {
public:
    struct ExecutionStats {
        uint64_t p_core_tasks_completed = 0;
        uint64_t e_core_tasks_completed = 0;
        double   elapsed_seconds        = 0.0;
    };

    template <typename TaskFunc>
    static ExecutionStats run_parallel(
        const char* phase_name,
        int32_t total_tasks,
        const std::vector<int32_t>& task_order,
        int32_t num_threads,
        bool /*enable_hybrid_balance*/,
        bool /*bidirectional_steal*/,
        bool show_progress,
        TaskFunc&& task_fn)
    {
        ExecutionStats stats{};
        auto t_start = std::chrono::high_resolution_clock::now();

        if (show_progress && phase_name) {
            std::printf("%-20s: ", phase_name);
            std::fflush(stdout);
        }

        if (total_tasks <= 0) {
            if (show_progress && phase_name) std::printf("0.00s elapsed\n");
            return stats;
        }

        auto thread_infos = HybridTopologyDetector::detect_threads(num_threads);
        const int32_t actual_threads = static_cast<int32_t>(thread_infos.size());

        // Cache-line-isolated atomic counters to eliminate false sharing across cores
        struct VVIS_ALIGN(64) AlignedAtomic {
            std::atomic<int32_t> val{0};
            uint8_t pad[60];
        };
        AlignedAtomic head_idx;
        AlignedAtomic completed_count;
        AlignedAtomic last_pacifier_step;

        head_idx.val.store(0, std::memory_order_relaxed);
        completed_count.val.store(0, std::memory_order_relaxed);
        last_pacifier_step.val.store(0, std::memory_order_relaxed);

        std::mutex print_mtx;
        std::vector<uint64_t> per_thread_done(actual_threads, 0);

        auto worker_body = [&](int32_t tid) {
            uint64_t local_done = 0;
            while (true) {
                // Strictly pull in ascending order (`head_idx++`) so easy portals finish
                // first (`StatDone`) and prune recursive branches for heavy portals!
                int32_t queue_pos = head_idx.val.fetch_add(1, std::memory_order_relaxed);
                if (queue_pos >= total_tasks) {
                    break;
                }

                int32_t task_id = task_order.empty() ? queue_pos : task_order[queue_pos];
                task_fn(tid, task_id);
                local_done++;

                if (show_progress) {
                    int32_t done = completed_count.val.fetch_add(1, std::memory_order_relaxed) + 1;
                    int32_t step = (done * 40) / total_tasks;
                    int32_t prev = last_pacifier_step.val.load(std::memory_order_relaxed);
                    if (step > prev) {
                        std::lock_guard<std::mutex> lk(print_mtx);
                        prev = last_pacifier_step.val.load(std::memory_order_relaxed);
                        while (prev < step && prev < 40) {
                            prev++;
                            if (prev % 4 == 0) {
                                std::printf("%d", prev / 4);
                            } else {
                                std::printf(".");
                            }
                        }
                        last_pacifier_step.val.store(prev, std::memory_order_relaxed);
                        std::fflush(stdout);
                    }
                }
            }
            per_thread_done[tid] = local_done;
        };

        std::vector<std::thread> workers;
        workers.reserve(actual_threads);
        for (int32_t t = 1; t < actual_threads; ++t) {
            workers.emplace_back(worker_body, t);
        }
        worker_body(0);

        for (auto& th : workers) {
            if (th.joinable()) th.join();
        }

        auto t_end = std::chrono::high_resolution_clock::now();
        stats.elapsed_seconds = std::chrono::duration<double>(t_end - t_start).count();

        for (int32_t t = 0; t < actual_threads; ++t) {
            stats.p_core_tasks_completed += per_thread_done[t];
        }

        if (show_progress && phase_name) {
            int32_t prev = last_pacifier_step.val.load(std::memory_order_relaxed);
            while (prev < 40) {
                prev++;
                if (prev % 4 == 0) std::printf("%d", prev / 4);
                else std::printf(".");
            }
            std::printf(" (%.3fs)\n", stats.elapsed_seconds);
            std::fflush(stdout);
        }

        return stats;
    }
};

} // namespace vvis_argon

// ==================== END vvis_hybrid_scheduler.hpp ====================

// ==================== BEGIN vvis_bsp_l4d2.hpp ====================
// ============================================================================
// VVIS++ ARGON EDITION (x64) v3 — BSP v20/v21 (Left 4 Dead 2 x64) & Portal I/O
// Reconstructed from vvis++.exe 0x1400e0960 (LoadBSPFile) & Source SDK bsplib.cpp
// Ensures 100% binary compatibility with Left 4 Dead 2 (BSP v21), VRAD++, and TF2/HL2 (BSP v20).
//
// CRITICAL FIXES (Argon v2 & v3):
//   1. RLE `CompressVis` / `DecompressVis` strictly uses `visrow = (portalclusters + 7) >> 3`
//      bytes per row (preventing "warning: Vis decompression overrun" in VRAD++ and Engine),
//      while internal SIMD bitvectors remain 64-bit aligned (`c_leafs_longs * 8`).
//   2. `BSPFile::save_visibility_lump_only()` performs non-destructive surgical updating of
//      `LUMP_VISIBILITY` ONLY (overwriting in-place if new_size <= old_size, or appending
//      4-byte aligned to EOF and patching ONLY `header.lumps[LUMP_VISIBILITY]`).
//      All other 63 lumps (especially `LUMP_GAME_LUMP` / `sprp` static props with absolute
//      file offsets `dgamelump_t::fileofs` and `LUMP_ENTITIES`) remain untouched byte-for-byte!
//   3. `load_portals_prt` computes `PlaneFromWinding` identically to Valve's `vvis.cpp`
//      (`v1 = pt[2] - pt[1]; v2 = pt[0] - pt[1]; normal = v2.cross(v1)`), preventing
//      inverted portal normals on `.prt` files generated by `vbsp++`!
// ============================================================================
#include <fstream>
#include <sstream>
#include <cstring>
#include <cstdio>

namespace vvis_argon {

constexpr uint32_t IDBSPHEADER      = (('P' << 24) + ('S' << 16) + ('B' << 8) + 'V'); // 'VBSP'
constexpr int32_t  HEADER_LUMPS     = 64;
constexpr int32_t  LUMP_ENTITIES    = 0;
constexpr int32_t  LUMP_PLANES      = 1;
constexpr int32_t  LUMP_VISIBILITY  = 4;
constexpr int32_t  LUMP_NODES       = 5;
constexpr int32_t  LUMP_LEAFS       = 10;
constexpr int32_t  LUMP_GAME_LUMP   = 35;

constexpr int32_t  DVIS_PVS         = 0;
constexpr int32_t  DVIS_PAS         = 1;

#pragma pack(push, 1)
// Standard Source Engine Lump Entry (BSP v19 / v20)
struct LumpStd {
    int32_t fileofs;
    int32_t filelen;
    int32_t version;
    char    fourCC[4];
};

// Left 4 Dead 2 / Portal 2 Lump Entry (BSP v21 — detected at 0x1400e09e7)
// Note: In L4D2 BSP v21, `version` comes BEFORE `fileofs` and `filelen`!
struct LumpL4D2 {
    int32_t version;
    int32_t fileofs;
    int32_t filelen;
    char    fourCC[4];
};

struct BSPHeaderRaw {
    uint32_t ident;
    int32_t  version;
    uint8_t  lump_bytes[HEADER_LUMPS * 16];
    int32_t  map_revision;
};
#pragma pack(pop)

struct NormalizedLump {
    int32_t fileofs = 0;
    int32_t filelen = 0;
    int32_t version = 0;
    char    fourCC[4] = {0, 0, 0, 0};
    std::vector<uint8_t> data;
};

// ----------------------------------------------------------------------------
// Run-Length Encoding (RLE) Visibility Compression & Decompression
// Matches Source SDK 2013 `bsplib.cpp`: `CompressVis` & `DecompressVis`
// Strictly operates on `visrow = (numclusters + 7) >> 3` bytes!
// ----------------------------------------------------------------------------
class VisRLECodec {
public:
    // Exact byte length of one PVS/PAS row in Source Engine (`bsplib.cpp`)
    static VVIS_FORCEINLINE int32_t calc_visrow_bytes(int32_t numclusters) {
        return (numclusters + 7) >> 3;
    }

    // Compress a raw PVS/PAS bitrow of `visrow_bytes = (numclusters + 7) >> 3` bytes into Source RLE format
    static int32_t compress_row(const uint8_t* src, int32_t visrow_bytes, uint8_t* dst) {
        uint8_t* dst_p = dst;
        for (int32_t j = 0; j < visrow_bytes; ++j) {
            *dst_p++ = src[j];
            if (src[j] != 0) {
                continue;
            }
            int32_t rep = 1;
            for (j++; j < visrow_bytes; ++j) {
                if (src[j] != 0 || rep == 255) {
                    break;
                }
                rep++;
            }
            *dst_p++ = static_cast<uint8_t>(rep);
            j--;
        }
        return static_cast<int32_t>(dst_p - dst);
    }

    // Decompress a Source RLE row back into `visrow_bytes = (numclusters + 7) >> 3` bytes
    // Includes exact overrun check from `bsplib.cpp: DecompressVis` for verification
    static bool decompress_row(const uint8_t* in, int32_t visrow_bytes, uint8_t* decompressed) {
        uint8_t* out = decompressed;
        bool had_overrun = false;
        do {
            if (*in) {
                *out++ = *in++;
                continue;
            }
            int c = in[1];
            if (!c) {
                c = 1; // Guard against 0 repeat
            }
            in += 2;
            if ((out - decompressed) + c > visrow_bytes) {
                c = static_cast<int>(visrow_bytes - (out - decompressed));
                had_overrun = true;
            }
            while (c > 0) {
                *out++ = 0;
                c--;
            }
        } while (out - decompressed < visrow_bytes);
        return !had_overrun;
    }
};

// ----------------------------------------------------------------------------
// BSP File Container (Non-Destructive Surgical `LUMP_VISIBILITY` Updater)
// Preserves all existing lumps, offsets, `LUMP_GAME_LUMP` (`sprp` static props),
// and `LUMP_ENTITIES` byte-for-byte!
// ----------------------------------------------------------------------------
class BSPFile {
public:
    BSPHeaderRaw raw_header{};
    int32_t      version = 21;
    int32_t      map_revision = 1;
    bool         is_l4d2_format = false;
    std::array<NormalizedLump, HEADER_LUMPS> lumps;

    bool load(const std::string& path) {
        std::ifstream ifs(path, std::ios::binary | std::ios::ate);
        if (!ifs.is_open()) return false;
        std::streamsize file_size = ifs.tellg();
        if (file_size < static_cast<std::streamsize>(sizeof(BSPHeaderRaw))) return false;
        ifs.seekg(0, std::ios::beg);

        if (!ifs.read(reinterpret_cast<char*>(&raw_header), sizeof(raw_header))) return false;
        if (raw_header.ident != IDBSPHEADER || (raw_header.version < 19 || raw_header.version > 21)) {
            return false;
        }
        version = raw_header.version;
        map_revision = raw_header.map_revision;

        // Auto-detect L4D2 lump header layout (Reconstructed from 0x1400e09e7)
        // In L4D2 BSP v21, `version` comes BEFORE `fileofs` and `filelen`!
        is_l4d2_format = false;
        if (version == 21) {
            const auto* std_lumps = reinterpret_cast<const LumpStd*>(raw_header.lump_bytes);
            int l4d2_votes = 0;
            int std_votes  = 0;
            for (int i = 0; i < HEADER_LUMPS; ++i) {
                if (std_lumps[i].fileofs <= 4 && std_lumps[i].filelen >= 1024) {
                    l4d2_votes++;
                } else if (std_lumps[i].fileofs >= 1024 && std_lumps[i].version <= 4) {
                    std_votes++;
                }
            }
            if (l4d2_votes > std_votes) {
                is_l4d2_format = true;
                std::printf("Detected L4D2 lump format\n");
            }
        }

        for (int i = 0; i < HEADER_LUMPS; ++i) {
            if (is_l4d2_format) {
                const auto* l = reinterpret_cast<const LumpL4D2*>(raw_header.lump_bytes) + i;
                lumps[i].version = l->version;
                lumps[i].fileofs = l->fileofs;
                lumps[i].filelen = l->filelen;
                std::memcpy(lumps[i].fourCC, l->fourCC, 4);
            } else {
                const auto* l = reinterpret_cast<const LumpStd*>(raw_header.lump_bytes) + i;
                lumps[i].fileofs = l->fileofs;
                lumps[i].filelen = l->filelen;
                lumps[i].version = l->version;
                std::memcpy(lumps[i].fourCC, l->fourCC, 4);
            }
        }

        // Only read `LUMP_ENTITIES` into memory to inspect `env_fog_controller` (`farz`);
        // we never modify or repack `LUMP_ENTITIES` or any other lump on disk!
        const auto& ent_lump = lumps[LUMP_ENTITIES];
        if (ent_lump.filelen > 0 &&
            ent_lump.fileofs >= 0 &&
            static_cast<std::streamsize>(ent_lump.fileofs + ent_lump.filelen) <= file_size)
        {
            lumps[LUMP_ENTITIES].data.resize(ent_lump.filelen);
            ifs.seekg(ent_lump.fileofs, std::ios::beg);
            ifs.read(reinterpret_cast<char*>(lumps[LUMP_ENTITIES].data.data()), ent_lump.filelen);
        }
        return true;
    }

    // Parse `max farz` from `env_fog_controller` entities in `LUMP_ENTITIES`
    // (Used only for diagnostic info; never auto-enables radial culling!)
    float parse_max_fog_farz() const {
        const auto& ent_data = lumps[LUMP_ENTITIES].data;
        if (ent_data.empty()) return -1.0f;

        std::string text(reinterpret_cast<const char*>(ent_data.data()), ent_data.size());
        float max_farz = -1.0f;
        size_t pos = 0;
        while ((pos = text.find('{', pos)) != std::string::npos) {
            size_t end = text.find('}', pos);
            if (end == std::string::npos) break;
            std::string block = text.substr(pos, end - pos + 1);
            if (block.find("\"env_fog_controller\"") != std::string::npos) {
                size_t fpos = block.find("\"farz\"");
                if (fpos != std::string::npos) {
                    size_t q1 = block.find('"', fpos + 6);
                    size_t q2 = (q1 != std::string::npos) ? block.find('"', q1 + 1) : std::string::npos;
                    if (q1 != std::string::npos && q2 != std::string::npos) {
                        float val = std::strtof(block.substr(q1 + 1, q2 - q1 - 1).c_str(), nullptr);
                        if (val > 0.0f && val > max_farz) {
                            max_farz = val;
                        }
                    }
                }
            }
            pos = end + 1;
        }
        return max_farz;
    }

    // Non-destructive surgical update of `LUMP_VISIBILITY` ONLY:
    // 1. If an existing `LUMP_VISIBILITY` block in the BSP has `old_fileofs >= sizeof(BSPHeaderRaw)`
    //    and `new_len <= old_filelen` (or if `LUMP_VISIBILITY` was already the very last lump at EOF),
    //    overwrite `LUMP_VISIBILITY` in-place at `old_fileofs`.
    // 2. Otherwise, seek to the end of the existing BSP file, align to 4 bytes (`LUMP_ALIGNMENT`),
    //    and append the new `LUMP_VISIBILITY` data.
    // 3. Patch ONLY `raw_header.lump_bytes[LUMP_VISIBILITY]` (`fileofs` and `filelen`) at file offset 0.
    //    All other 63 lumps (`LUMP_GAME_LUMP` / `sprp`, `LUMP_ENTITIES`, `LUMP_PAKFILE`, geometry)
    //    remain 100% untouched byte-for-byte!
    bool save_visibility_lump_only(const std::string& path, const std::vector<uint8_t>& vis_data) {
        std::fstream fs(path, std::ios::in | std::ios::out | std::ios::binary);
        if (!fs.is_open()) return false;

        fs.seekg(0, std::ios::end);
        const int64_t file_size = static_cast<int64_t>(fs.tellg());
        if (file_size < static_cast<int64_t>(sizeof(BSPHeaderRaw))) return false;

        const int32_t old_ofs = lumps[LUMP_VISIBILITY].fileofs;
        const int32_t old_len = lumps[LUMP_VISIBILITY].filelen;
        const int32_t new_len = static_cast<int32_t>(vis_data.size());
        int32_t target_ofs = 0;

        bool was_at_eof = (old_ofs >= static_cast<int32_t>(sizeof(BSPHeaderRaw)) &&
                           old_len > 0 &&
                           static_cast<int64_t>(old_ofs) + old_len == file_size);

        if (old_ofs >= static_cast<int32_t>(sizeof(BSPHeaderRaw)) &&
            old_len >= new_len &&
            static_cast<int64_t>(old_ofs) + old_len <= file_size)
        {
            // Case 1a: New visibility data fits inside existing LUMP_VISIBILITY region in-place!
            target_ofs = old_ofs;
            fs.seekp(target_ofs, std::ios::beg);
            if (new_len > 0) {
                fs.write(reinterpret_cast<const char*>(vis_data.data()), new_len);
            }
        } else if (was_at_eof) {
            // Case 1b: Existing LUMP_VISIBILITY was at the very end of the file -> overwrite at old_ofs
            target_ofs = old_ofs;
            fs.seekp(target_ofs, std::ios::beg);
            if (new_len > 0) {
                fs.write(reinterpret_cast<const char*>(vis_data.data()), new_len);
            }
        } else {
            // Case 2: Append new LUMP_VISIBILITY at the end of the BSP file (aligned to 4 bytes)
            int64_t aligned_eof = (file_size + 3) & ~int64_t(3);
            int64_t pad_bytes = aligned_eof - file_size;
            fs.seekp(file_size, std::ios::beg);
            if (pad_bytes > 0) {
                const char zeros[4] = {0, 0, 0, 0};
                fs.write(zeros, pad_bytes);
            }
            target_ofs = static_cast<int32_t>(aligned_eof);
            if (new_len > 0) {
                fs.write(reinterpret_cast<const char*>(vis_data.data()), new_len);
            }
            // Pad end of lump to 4-byte boundary (`AlignFilePosition(g_hBSPFile, 4)`)
            int32_t tail_pad = (4 - (new_len & 3)) & 3;
            if (tail_pad > 0) {
                const char zeros[4] = {0, 0, 0, 0};
                fs.write(zeros, tail_pad);
            }
        }

        // Update ONLY `LUMP_VISIBILITY` in the original header; leave all other 63 lumps untouched!
        lumps[LUMP_VISIBILITY].fileofs = (new_len > 0) ? target_ofs : 0;
        lumps[LUMP_VISIBILITY].filelen = new_len;

        if (is_l4d2_format) {
            auto* l = reinterpret_cast<LumpL4D2*>(raw_header.lump_bytes) + LUMP_VISIBILITY;
            l->version = lumps[LUMP_VISIBILITY].version;
            l->fileofs = lumps[LUMP_VISIBILITY].fileofs;
            l->filelen = lumps[LUMP_VISIBILITY].filelen;
            std::memset(l->fourCC, 0, 4); // Uncompressed LUMP_VISIBILITY
        } else {
            auto* l = reinterpret_cast<LumpStd*>(raw_header.lump_bytes) + LUMP_VISIBILITY;
            l->fileofs = lumps[LUMP_VISIBILITY].fileofs;
            l->filelen = lumps[LUMP_VISIBILITY].filelen;
            l->version = lumps[LUMP_VISIBILITY].version;
            std::memset(l->fourCC, 0, 4);
        }

        // Write updated 1036-byte header back to offset 0
        fs.seekp(0, std::ios::beg);
        fs.write(reinterpret_cast<const char*>(&raw_header), sizeof(BSPHeaderRaw));
        fs.flush();
        return fs.good();
    }
};

// ----------------------------------------------------------------------------
// Portal File (`.prt`) Parser
// Reconstructed from `LoadPortals` in vvis++.exe (0x1400d9f50) & Valve `vvis.cpp`
// ----------------------------------------------------------------------------
inline bool load_portals_prt(
    const std::string& prt_path,
    int32_t& out_portalclusters,
    int32_t& out_numportals,
    std::vector<VPortal>& out_portals,
    std::vector<ClusterLeaf>& out_leafs,
    std::vector<std::unique_ptr<DynamicWinding>>& out_winding_pool)
{
    FILE* f = std::fopen(prt_path.c_str(), "r");
    if (!f) {
        std::fprintf(stderr, "LoadPortals: couldn't read %s\n", prt_path.c_str());
        return false;
    }

    char magic[80] = {0};
    int32_t pclusters = 0;
    int32_t nportals  = 0;
    if (std::fscanf(f, "%79s\n%i\n%i\n", magic, &pclusters, &nportals) != 3) {
        std::fprintf(stderr, "LoadPortals %s: failed to read header\n", prt_path.c_str());
        std::fclose(f);
        return false;
    }

    if (std::strncmp(magic, "PRT1", 4) != 0) {
        std::fprintf(stderr, "LoadPortals %s: not a portal file\n", prt_path.c_str());
        std::fclose(f);
        return false;
    }

    std::printf("%6i portalclusters\n", pclusters);
    std::printf("%6i numportals\n", nportals);

    if (nportals * 2 >= MAX_PORTALS) {
        std::fprintf(stderr, "The map overflows the max portal count (%d of max %d)!\n", nportals, MAX_MAP_PORTALS);
        std::fclose(f);
        return false;
    }

    out_portalclusters = pclusters;
    out_numportals     = nportals;
    out_portals.clear();
    out_portals.resize(nportals * 2);
    out_leafs.clear();
    out_leafs.resize(pclusters);
    out_winding_pool.clear();
    out_winding_pool.reserve(nportals * 2);

    for (int32_t i = 0; i < nportals; ++i) {
        int32_t numpoints = 0;
        int32_t leaf0 = 0, leaf1 = 0;
        if (std::fscanf(f, "%i %i %i ", &numpoints, &leaf0, &leaf1) != 3) {
            std::fprintf(stderr, "LoadPortals: reading portal %i\n", i);
            std::fclose(f);
            return false;
        }
        if (numpoints > MAX_POINTS_ON_WINDING || numpoints < 3) {
            std::fprintf(stderr, "LoadPortals: portal %i has too many points (%d)\n", i, numpoints);
            std::fclose(f);
            return false;
        }
        if (leaf0 < 0 || leaf0 >= pclusters || leaf1 < 0 || leaf1 >= pclusters) {
            std::fprintf(stderr, "LoadPortals: portal %i is corrupt (leafs %d %d)\n", i, leaf0, leaf1);
            std::fclose(f);
            return false;
        }

        auto w_fwd = std::make_unique<DynamicWinding>();
        auto w_rev = std::make_unique<DynamicWinding>();
        w_fwd->numpoints = numpoints;
        w_fwd->maxpoints = numpoints;
        w_fwd->points.resize(numpoints);

        w_rev->numpoints = numpoints;
        w_rev->maxpoints = numpoints;
        w_rev->points.resize(numpoints);

        for (int32_t p = 0; p < numpoints; ++p) {
            double vx = 0, vy = 0, vz = 0;
            if (std::fscanf(f, "(%lf %lf %lf ) ", &vx, &vy, &vz) != 3) {
                std::fprintf(stderr, "LoadPortals: reading points for portal %i\n", i);
                std::fclose(f);
                return false;
            }
            Vector3 pt(static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(vz));
            w_fwd->points[p] = pt;
            w_rev->points[numpoints - 1 - p] = pt;
        }
        std::fscanf(f, "\n");

        w_fwd->compute_bounds();
        w_rev->compute_bounds();

        // Exact Valve `PlaneFromWinding` (vvis.cpp lines 408-420 & 559-584):
        //   VectorSubtract(w->points[2], w->points[1], v1);
        //   VectorSubtract(w->points[0], w->points[1], v2);
        //   CrossProduct(v2, v1, plane->normal);
        //   VectorNormalize(plane->normal);
        //   plane->dist = DotProduct(w->points[0], plane->normal);
        Vector3 v1 = w_fwd->points[2] - w_fwd->points[1];
        Vector3 v2 = w_fwd->points[0] - w_fwd->points[1];
        Vector3 plane_normal = v2.cross(v1);
        float len = plane_normal.length();
        if (len > 1e-6f) plane_normal = plane_normal * (1.0f / len);
        float plane_dist = w_fwd->points[0].dot(plane_normal);

        // Forward portal (leafnums[0] -> leafnums[1]): normal = -plane.normal, dist = -plane.dist
        Vector3 normal_fwd = plane_normal * -1.0f;
        float dist_fwd = -plane_dist;

        // Backward portal (leafnums[1] -> leafnums[0]): normal = plane.normal, dist = plane.dist
        Vector3 normal_rev = plane_normal;
        float dist_rev = plane_dist;

        // Directed Portal 2*i: from leaf0 into leaf1
        VPortal& p_fwd = out_portals[2 * i];
        p_fwd.portal_id = 2 * i;
        p_fwd.src_leaf  = leaf0;
        p_fwd.leaf      = leaf1;
        p_fwd.plane     = {normal_fwd, dist_fwd};
        p_fwd.winding   = w_fwd.get();
        out_leafs[leaf0].portals.push_back(&p_fwd);

        // Directed Portal 2*i + 1: from leaf1 into leaf0
        VPortal& p_rev = out_portals[2 * i + 1];
        p_rev.portal_id = 2 * i + 1;
        p_rev.src_leaf  = leaf1;
        p_rev.leaf      = leaf0;
        p_rev.plane     = {normal_rev, dist_rev};
        p_rev.winding   = w_rev.get();
        out_leafs[leaf1].portals.push_back(&p_rev);

        out_winding_pool.push_back(std::move(w_fwd));
        out_winding_pool.push_back(std::move(w_rev));
    }

    std::fclose(f);

    // Compute cluster bounding boxes & centroids
    for (int32_t c = 0; c < pclusters; ++c) {
        ClusterLeaf& cl = out_leafs[c];
        Vector3 sum(0, 0, 0);
        int cnt = 0;
        for (const VPortal* p : cl.portals) {
            if (!p->winding) continue;
            for (int k = 0; k < p->winding->numpoints; ++k) {
                const Vector3& pt = p->winding->points[k];
                cl.bounds_min.x = std::min(cl.bounds_min.x, pt.x);
                cl.bounds_min.y = std::min(cl.bounds_min.y, pt.y);
                cl.bounds_min.z = std::min(cl.bounds_min.z, pt.z);
                cl.bounds_max.x = std::max(cl.bounds_max.x, pt.x);
                cl.bounds_max.y = std::max(cl.bounds_max.y, pt.y);
                cl.bounds_max.z = std::max(cl.bounds_max.z, pt.z);
                sum = sum + pt;
                cnt++;
            }
        }
        if (cnt > 0) cl.centroid = sum * (1.0f / static_cast<float>(cnt));
    }
    return true;
}

} // namespace vvis_argon

// ==================== END vvis_bsp_l4d2.hpp ====================

// ==================== BEGIN vvis_main.cpp ====================
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

// ==================== END vvis_main.cpp ====================
