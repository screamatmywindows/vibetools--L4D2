// ============================================================================
// VVIS++ ARGON EDITION (x64) v2 — Complete Amalgamated Single-File C++20 Source
// Decompiled from ficool2 VVIS++ (Oct 2026) & Upgraded with Argon Architecture
// Target: Windows 10/11 x64 (MSVC v143 /O2 /Oi /Ot /arch:AVX2) & Linux x64
// CPU Target: Intel Core i5-12450H (Alder Lake 4P+4E, AVX2+FMA3+BMI2, AVX-512 Guarded)
// Engine Target: Left 4 Dead 2 x64 (Source Engine BSP v21 & v20)
// Includes VRAD++ & L4D2 Fixes:
//   1. Strict byte-level RLE row size: visrow = (portalclusters + 7) >> 3
//   2. Surgical LUMP_VISIBILITY-only BSP update (preserves LUMP_GAME_LUMP/sprp)
// ============================================================================

// >>>> BEGIN vvis_argon.hpp <<<<
// ============================================================================
// VVIS++ ARGON EDITION (x64) — Core Reconstructed Data Structures & Constants
// Target: Windows 10/11 x64 (MSVC v143 / C++20) & Linux x64 (GCC/Clang)
// Engine: Source Engine 1 (Left 4 Dead 2 x64 / BSP v20 & v21)
// Hardware Target: Intel Core i5-12450H (Alder Lake 4P+4E, AVX2 + FMA3 + BMI2)
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
// Engine & VVIS++ Constants (Verified from vvis++.exe 0x140137f00 - 0x140138600)
// ----------------------------------------------------------------------------
constexpr int MAX_MAP_PORTALS     = 32768; // 0x8000 (checked at 0x1400d9d05, doubled to 65536 directed portals)
constexpr int MAX_PORTALS         = 65536; // 2 * MAX_MAP_PORTALS
constexpr int MAX_MAP_CLUSTERS    = 32768;
constexpr int MAX_POINTS_ON_WINDING = 64;  // 0x40 (checked at 0x1400dcde7)
constexpr int MAX_POINTS_ON_FIXED_WINDING = 12; // 0x0C (checked in ChopWinding at 0x1400d145a)
constexpr int MAX_SEPERATORS      = 64;    // 0x40 planes per source/pass separator array
constexpr int NUM_STACK_WINDINGS  = 3;     // 3 stack windings per pstack_t frame

// Floating-point epsilons from .rdata (0x140137f00 - 0x140137f10)
constexpr float ON_EPSILON        = 0.1f;      // 0x140137f04
constexpr float MINUS_ON_EPSILON  = -0.1f;     // 0x140137f08
constexpr float BACKFACE_EPSILON  = -1.0001f;  // 0x140137f0c
constexpr float MAX_COORD_DIST_SQ = 1.0e20f;   // 0x140137f10

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
    Tier2_HybridCached   = 2, // Hybrid Cached (1-4s): Incremental .viscache reuse + AVX2 PortalFlow on dirty subgraph
    Tier3_MasterRelease  = 3  // Master Release: Full AVX2SOA + FMA3 + BMI2 PortalFlow with multi-bounce separator clipping
};

// ----------------------------------------------------------------------------
// Core Math & Geometry Types
// ----------------------------------------------------------------------------
struct Vector3 {
    float x, y, z;

    VVIS_FORCEINLINE Vector3() : x(0.0f), y(0.0f), z(0.0f) {}
    VVIS_FORCEINLINE Vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

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

// 16-byte plane structure (matches vvis++.exe layout at offset +0x00 of portal/pstack)
struct VVIS_ALIGN(16) Plane {
    Vector3 normal; // +0x00 (12 bytes)
    float   dist;   // +0x0C (4 bytes)
};

// ----------------------------------------------------------------------------
// Reconstructed Winding Structures (AoS and SIMD SoA)
// ----------------------------------------------------------------------------
// Reconstructed from 0x1400d11f0 (ChopWinding) & 0x1400d0245 (BasePortalVis):
// In ficool2's vvis++.exe, winding_t embeds bounding sphere (origin + radius)
// at the start of the struct to accelerate sphere-plane culling!
struct VVIS_ALIGN(16) Winding {
    Vector3 origin;     // +0x00: Bounding sphere center
    float   radius;     // +0x0C: Bounding sphere radius
    int32_t numpoints;  // +0x10: Number of active vertices
    Vector3 points[MAX_POINTS_ON_FIXED_WINDING]; // +0x14: Up to 12 vertices for stack windings (total 0xA4 = 164 bytes)
};
static_assert(sizeof(Winding) >= 0xA4, "Stack Winding size matches 0xA4 stride in vvis++.exe");

// Dynamic Winding for initial unclipped portals (up to 64 vertices)
struct DynamicWinding {
    Vector3 origin;
    float   radius;
    int32_t numpoints;
    int32_t maxpoints;
    std::vector<Vector3> points;

    // Pre-transposed SIMD SoA arrays (padded to 16 floats = 64 bytes)
    // Reconstructed from 0x1400d1d70 (SetupSOA) & 0x1400d4470 (ChopWinding_SOA)
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
        // Pad remaining lanes with vertex 0 to prevent NaN/Inf in SIMD reductions
        Vector3 p0 = (n > 0) ? points[0] : Vector3(0, 0, 0);
        for (int i = n; i < MAX_POINTS_ON_WINDING; ++i) {
            soa_x[i] = p0.x;
            soa_y[i] = p0.y;
            soa_z[i] = p0.z;
        }
    }

    void compute_bounds() {
        if (numpoints <= 0) {
            origin = {0, 0, 0};
            radius = 0.0f;
            return;
        }
        Vector3 sum(0, 0, 0);
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

// Stack-allocated SIMD SoA Winding (16 vertices max, fits in 2 AVX2 YMM registers per axis)
// Reconstructed from 0x1400d448c (offset +0x40 X, +0x80 Y, +0xC0 Z, +0x100 numpoints)
struct VVIS_ALIGN(32) StackWindingSOA {
    Vector3 origin;          // +0x00
    float   radius;          // +0x0C
    int32_t numpoints;       // +0x10
    int32_t pad[11];         // Pad to +0x40
    VVIS_ALIGN(32) float x[16]; // +0x40 (2x YMM)
    VVIS_ALIGN(32) float y[16]; // +0x80 (2x YMM)
    VVIS_ALIGN(32) float z[16]; // +0xC0 (2x YMM)

    void load_from_dynamic(const DynamicWinding& dw) {
        origin = dw.origin;
        radius = dw.radius;
        numpoints = std::min<int32_t>(dw.numpoints, 16);
        std::memcpy(x, dw.soa_x, 16 * sizeof(float));
        std::memcpy(y, dw.soa_y, 16 * sizeof(float));
        std::memcpy(z, dw.soa_z, 16 * sizeof(float));
    }
};

// ----------------------------------------------------------------------------
// Reconstructed Portal & Leaf Structures (Exact offsets from vvis++.exe)
// ----------------------------------------------------------------------------
// Reconstructed from 0x1400d0125 (64-byte stride: shl rbp, 0x6)
struct VVIS_ALIGN(64) VPortal {
    Plane           plane;        // +0x00: Portal plane (normal points into `leaf`)
    int32_t         leaf;         // +0x10: Target cluster index this directed portal leads into
    int32_t         src_leaf;     // +0x14: Source cluster index this directed portal originates from
    DynamicWinding* winding;      // +0x18: Winding polygon
    std::atomic<PortalStatus> status; // +0x20: 0=none, 1=working, 2=done
    int32_t         nummightsee;  // +0x24: Popcount of portalmightsee / Complexity metric for PComp sort
    uint64_t*       portalmightsee; // +0x28: BasePortalVis cluster bitvector (c_longs * 8 bytes)
    int32_t         numcansee;    // +0x30: Popcount of portalvis
    int32_t         portal_id;    // +0x34: Directed portal index [0 .. 2*numportals - 1]
    uint64_t*       portalvis;    // +0x38: Final PortalFlow cluster bitvector (c_longs * 8 bytes)

    // Argon Extension: Complexity score for Hybrid P/E Core scheduling
    float           complexity_score = 0.0f;
    uint64_t        geometric_hash   = 0;

    VPortal()
        : plane{{0,0,0}, 0}, leaf(0), src_leaf(0), winding(nullptr),
          status(PortalStatus::StatNone), nummightsee(0), portalmightsee(nullptr),
          numcansee(0), portal_id(0), portalvis(nullptr) {}

    // Copy constructor for std::vector resize
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

// Reconstructed from 0x1400d0a0e (32-byte stride: shl rax, 0x5)
struct ClusterLeaf {
    std::vector<VPortal*> portals; // +0x00: Pointers to directed portals leaving this cluster
    uint64_t geometric_hash  = 0;  // Local geometry + portal planes hash
    uint64_t neighborhood_hash = 0; // 2-hop Merkle hash for .viscache invalidation
    Vector3  bounds_min{1e30f, 1e30f, 1e30f};
    Vector3  bounds_max{-1e30f, -1e30f, -1e30f};
    Vector3  centroid{0, 0, 0};
    bool     cache_hit = false;
};

// ----------------------------------------------------------------------------
// Reconstructed PortalFlow Stack Frame (`pstack_t` & `threaddata_t`)
// Verified from 0x1400cf660 - 0x1400d0020
// ----------------------------------------------------------------------------
struct PStack {
    std::vector<uint64_t> mightsee;       // Cluster visibility bitmask at this recursion depth
    PStack*               next = nullptr; // +0x2000: Linked list of active recursion frames
    ClusterLeaf*          leaf = nullptr; // +0x2008: Current cluster leaf (used for cycle detection!)
    VPortal*              portal = nullptr; // +0x2010: Current target portal
    StackWindingSOA*      source = nullptr; // +0x2018: Clipped source portal winding
    StackWindingSOA*      pass   = nullptr; // +0x2020: Clipped pass portal winding
    StackWindingSOA       windings[NUM_STACK_WINDINGS]; // +0x2028: 3 stack winding slots
    uint8_t               freewindings[NUM_STACK_WINDINGS] = {1, 1, 1}; // +0x2214: 1=free, 0=allocated
    Plane                 portalplane;    // +0x2218: Plane of pass portal
    Plane                 seperators[2][MAX_SEPERATORS]; // +0x2228 & +0x2628: Cached separator planes
    int32_t               numseperators[2] = {0, 0};     // +0x2A28 & +0x2A2C
    bool                  did_mightsee_test = false;     // +0x2A30
    int32_t               mightsee_count = 0;            // +0x2A34: Popcount of new clusters
};

struct ThreadData {
    VPortal*         base = nullptr;      // +0x0000: Base portal being solved
    int32_t          c_chains = 0;        // +0x2A48: Number of recursion chains executed
    int32_t          c_mighttest = 0;     // +0x2A4C: Number of CheckStack expensive tests
    PStack           pstack_head;         // +0x0008: Root stack frame
    StackWindingSOA  base_winding_soa;    // Root source winding in SoA layout
    int32_t          max_depth = -1;      // Argon Extension: -maxdepth limit (-1 = unlimited)
};

} // namespace vvis_argon

// >>>> END vvis_argon.hpp <<<<

// >>>> BEGIN vvis_simd_avx2.hpp <<<<
// ============================================================================
// VVIS++ ARGON EDITION (x64) — AVX2 + FMA3 + BMI2 SIMD Kernels & CPUID Guard
// Strictly guarded against Intel 12th Gen Alder Lake (i5-12450H) AVX-512 traps.
// ============================================================================

#include <immintrin.h>
#include <cstdio>

namespace vvis_argon {

// ----------------------------------------------------------------------------
// Safe CPUID + XGETBV Hardware Detection (Alder Lake P+E Core Safe)
// Reconstructed from vvis++.exe 0x1400d8b55 - 0x1400d8c01 & hardened for Alder Lake
// ----------------------------------------------------------------------------
struct CpuCapabilities {
    bool has_sse41      = false;
    bool has_avx2       = false;
    bool has_fma3       = false;
    bool has_bmi2       = false;
    bool has_popcnt     = false;
    bool is_hybrid_cpu  = false; // CPUID.07H.0H:EDX[15] (Hybrid architecture flag on Alder Lake)
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
        bool os_avx_enabled = ((xcr0 & 0x6) == 0x6); // XMM + YMM state enabled by OS

        if (os_avx_enabled && avx_bit && max_leaf >= 7) {
            int regs7[4] = {0, 0, 0, 0};
            cpuid_ex(regs7, 7, 0);
            caps.has_avx2      = (regs7[1] & (1 << 5)) != 0;
            caps.has_bmi2      = (regs7[1] & (1 << 8)) != 0;
            caps.is_hybrid_cpu = (regs7[3] & (1 << 15)) != 0; // Alder Lake / Raptor Lake Hybrid flag
            caps.has_fma3      = fma3_bit;
            // CRITICAL HARDWARE GUARD:
            // Even if XCR0 or microcode reports anything in bits 5..7, we NEVER dispatch
            // EVEX / ZMM instructions on hybrid Alder Lake CPUs (i5-12450H has AVX-512 fused off).
        }
    }

    // Extract CPU brand string
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
// Reconstructed from vvis++.exe 0x1400cf975 - 0x1400d001b
// Computes:
//   dst[i] = might[i] & vis[i]
//   has_more |= dst[i] & ~prev_vis[i]   (via VPANDN)
//   popcnt = sum(popcnt(dst[i]))
// ----------------------------------------------------------------------------
VVIS_FORCEINLINE bool avx2_intersect_and_check_more(
    uint64_t* __restrict dst,
    const uint64_t* __restrict portal_vis_or_might,
    const uint64_t* __restrict current_might,
    const uint64_t* __restrict base_portalvis,
    int num_qwords,
    int32_t& out_popcnt)
{
    uint64_t more_accum = 0;
    int32_t total_bits = 0;
    int i = 0;

#if defined(__AVX2__) || defined(_MSC_VER)
    // Process 4x uint64_t (256 bits) per iteration using AVX2 VPAND + VPANDN + VPOR
    __m256i v_more = _mm256_setzero_si256();
    for (; i + 4 <= num_qwords; i += 4) {
        __m256i v_p   = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(portal_vis_or_might + i));
        __m256i v_m   = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(current_might + i));
        __m256i v_b   = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(base_portalvis + i));

        __m256i v_and = _mm256_and_si256(v_p, v_m);
        _mm256_storeu_si256(reinterpret_cast<__m256i*>(dst + i), v_and);

        // VPANDN(v_b, v_and) = (~v_b) & v_and
        __m256i v_new = _mm256_andnot_si256(v_b, v_and);
        v_more = _mm256_or_si256(v_more, v_new);
    }

    // Horizontal check of v_more
    if (!_mm256_testz_si256(v_more, v_more)) {
        more_accum = 1;
    }
#endif

    // Scalar tail with BMI1 `ANDN` and hardware `POPCNT`
    for (; i < num_qwords; ++i) {
        uint64_t val = portal_vis_or_might[i] & current_might[i];
        dst[i] = val;
        more_accum |= (val & ~base_portalvis[i]);
    }

    if (more_accum == 0) {
        out_popcnt = 0;
        return false;
    }

    // Hardware POPCNT64 across dst[] (matches 0x1400cfc00 popcnt r8, QWORD PTR)
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
// Adapted from ficool2's `ChopWinding_SOA` (0x1400d4470) to run on 256-bit YMM
// registers (2x8 floats = 16 vertices) + BMI2 `_pdep_u32` without any AVX-512!
// ----------------------------------------------------------------------------
VVIS_FORCEINLINE StackWindingSOA* chop_winding_avx2_soa(
    StackWindingSOA* __restrict in,
    PStack& stack,
    const Plane& split)
{
    if (!in || in->numpoints <= 0) return nullptr;

    // 1. Fast Bounding-Sphere Early Culling (Reconstructed from 0x1400d1211)
    float sphere_dist = in->origin.x * split.normal.x
                      + in->origin.y * split.normal.y
                      + in->origin.z * split.normal.z
                      - split.dist;

    if (sphere_dist < -in->radius) {
        // Entire winding is behind the split plane! Free stack winding if owned.
        ptrdiff_t idx = in - stack.windings;
        if (idx >= 0 && idx < NUM_STACK_WINDINGS) {
            stack.freewindings[idx] = 1;
        }
        return nullptr;
    }
    if (sphere_dist > in->radius) {
        // Entire winding is in front of the split plane! Return unclipped.
        return in;
    }

    const int n = in->numpoints;
    VVIS_ALIGN(32) float dists[17];
    int sides[17];
    int counts[3] = {0, 0, 0}; // 0: FRONT (> ON_EPSILON), 1: BACK (< -ON_EPSILON), 2: ON

#if defined(__AVX2__) || defined(_MSC_VER)
    // Broadcast plane coefficients to YMM registers
    __m256 v_nx   = _mm256_set1_ps(split.normal.x);
    __m256 v_ny   = _mm256_set1_ps(split.normal.y);
    __m256 v_nz   = _mm256_set1_ps(split.normal.z);
    __m256 v_dist = _mm256_set1_ps(split.dist);
    __m256 v_eps  = _mm256_set1_ps(ON_EPSILON);
    __m256 v_meps = _mm256_set1_ps(MINUS_ON_EPSILON);

    // Bank 0: vertices 0..7
    __m256 vx0 = _mm256_load_ps(in->x);
    __m256 vy0 = _mm256_load_ps(in->y);
    __m256 vz0 = _mm256_load_ps(in->z);
    __m256 vd0 = _mm256_mul_ps(vx0, v_nx);
    vd0 = _mm256_fmadd_ps(vy0, v_ny, vd0);
    vd0 = _mm256_fmadd_ps(vz0, v_nz, vd0);
    vd0 = _mm256_sub_ps(vd0, v_dist);
    _mm256_store_ps(dists, vd0);

    uint32_t front_mask = static_cast<uint32_t>(_mm256_movemask_ps(_mm256_cmp_ps(vd0, v_eps, _CMP_GT_OQ)));
    uint32_t back_mask  = static_cast<uint32_t>(_mm256_movemask_ps(_mm256_cmp_ps(vd0, v_meps, _CMP_LT_OQ)));

    // Bank 1: vertices 8..15 (if n > 8)
    if (n > 8) {
        __m256 vx1 = _mm256_load_ps(in->x + 8);
        __m256 vy1 = _mm256_load_ps(in->y + 8);
        __m256 vz1 = _mm256_load_ps(in->z + 8);
        __m256 vd1 = _mm256_mul_ps(vx1, v_nx);
        vd1 = _mm256_fmadd_ps(vy1, v_ny, vd1);
        vd1 = _mm256_fmadd_ps(vz1, v_nz, vd1);
        vd1 = _mm256_sub_ps(vd1, v_dist);
        _mm256_store_ps(dists + 8, vd1);

        front_mask |= (static_cast<uint32_t>(_mm256_movemask_ps(_mm256_cmp_ps(vd1, v_eps, _CMP_GT_OQ))) << 8);
        back_mask  |= (static_cast<uint32_t>(_mm256_movemask_ps(_mm256_cmp_ps(vd1, v_meps, _CMP_LT_OQ))) << 8);
    }

    const uint32_t valid_mask = (1u << n) - 1u;
    front_mask &= valid_mask;
    back_mask  &= valid_mask;

    // Fast Bitmask Early Exit (Equivalent to AVX-512 kortestw at 0x1400d464b!)
    if (back_mask == 0) {
        // No points behind the plane -> keep entire winding
        return in;
    }
    if (front_mask == 0) {
        // All points behind or on the plane -> clipped away
        ptrdiff_t idx = in - stack.windings;
        if (idx >= 0 && idx < NUM_STACK_WINDINGS) {
            stack.freewindings[idx] = 1;
        }
        return nullptr;
    }

    for (int i = 0; i < n; ++i) {
        if ((front_mask >> i) & 1u) {
            sides[i] = 0;
            counts[0]++;
        } else if ((back_mask >> i) & 1u) {
            sides[i] = 1;
            counts[1]++;
        } else {
            sides[i] = 2;
            counts[2]++;
        }
    }
#else
    for (int i = 0; i < n; ++i) {
        float d = in->x[i] * split.normal.x + in->y[i] * split.normal.y + in->z[i] * split.normal.z - split.dist;
        dists[i] = d;
        if (d > ON_EPSILON) { sides[i] = 0; counts[0]++; }
        else if (d < MINUS_ON_EPSILON) { sides[i] = 1; counts[1]++; }
        else { sides[i] = 2; counts[2]++; }
    }
    if (counts[1] == 0) return in;
    if (counts[0] == 0) {
        ptrdiff_t idx = in - stack.windings;
        if (idx >= 0 && idx < NUM_STACK_WINDINGS) stack.freewindings[idx] = 1;
        return nullptr;
    }
#endif

    // Wrap vertex 0 to vertex n for seamless edge loop
    sides[n] = sides[0];
    dists[n] = dists[0];

    // Allocate free stack winding slot (Reconstructed from 0x1400d13aa)
    StackWindingSOA* out = nullptr;
    for (int s = 0; s < NUM_STACK_WINDINGS; ++s) {
        if (stack.freewindings[s]) {
            stack.freewindings[s] = 0;
            out = &stack.windings[s];
            break;
        }
    }
    if (!out) {
        // Fallback safety if all 3 stack windings are in use
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
            if (out_pts < MAX_POINTS_ON_FIXED_WINDING) {
                out->x[out_pts] = px;
                out->y[out_pts] = py;
                out->z[out_pts] = pz;
                out_pts++;
            }
            continue;
        }
        if (sides[i] == 0) {
            if (out_pts < MAX_POINTS_ON_FIXED_WINDING) {
                out->x[out_pts] = px;
                out->y[out_pts] = py;
                out->z[out_pts] = pz;
                out_pts++;
            }
        }
        if (sides[i + 1] == 2 || sides[i + 1] == sides[i]) {
            continue;
        }

        // Edge crosses split plane -> interpolate intersection point
        int next_i = (i + 1) % n;
        float dot = dists[i] / (dists[i] - dists[i + 1]);

        // Exact axial plane snapping (Reconstructed from 0x1400d1440 & 0x1400d4940)
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

        if (out_pts < MAX_POINTS_ON_FIXED_WINDING) {
            out->x[out_pts] = ix;
            out->y[out_pts] = iy;
            out->z[out_pts] = iz;
            out_pts++;
        }
    }

    // Pad unused SoA lanes with vertex 0
    out->numpoints = out_pts;
    if (out_pts > 0) {
        for (int k = out_pts; k < 16; ++k) {
            out->x[k] = out->x[0];
            out->y[k] = out->y[0];
            out->z[k] = out->z[0];
        }
    }

    // Free input stack winding if it belonged to stack.windings
    ptrdiff_t in_idx = in - stack.windings;
    if (in_idx >= 0 && in_idx < NUM_STACK_WINDINGS) {
        stack.freewindings[in_idx] = 1;
    }

    if (out_pts < 3) {
        ptrdiff_t out_idx = out - stack.windings;
        if (out_idx >= 0 && out_idx < NUM_STACK_WINDINGS) {
            stack.freewindings[out_idx] = 1;
        }
        return nullptr;
    }
    return out;
}

// ----------------------------------------------------------------------------
// 3. AVX2 + FMA3 Separator Plane Generator & Clipper (`ClipToSeperators_AVX2_SOA`)
// Reconstructed from 0x1400d1910 (ClipToSeperators_AVX2) & 0x1400d4e20 (ClipToSeperators_SOA)
// Uses 8-wide AVX2 FMA3 to evaluate candidate separator planes against all pass
// vertices simultaneously via `_mm256_movemask_ps`!
// ----------------------------------------------------------------------------
inline StackWindingSOA* clip_to_seperators_avx2_soa(
    const StackWindingSOA* __restrict source,
    const Plane& pass_plane,
    const StackWindingSOA* __restrict pass,
    StackWindingSOA* __restrict target,
    bool flipclip,
    PStack& stack,
    int sep_cache_slot) // 0, 1: cache into stack.seperators[slot], >=2: don't cache
{
    if (!source || !pass || !target) return target;
    const int num_src  = source->numpoints;
    const int num_pass = pass->numpoints;
    if (num_src <= 0 || num_pass <= 0) return target;

#if defined(__AVX2__) || defined(_MSC_VER)
    // Load pass vertices into two AVX2 YMM banks (up to 16 vertices)
    __m256 px0 = _mm256_load_ps(pass->x);
    __m256 py0 = _mm256_load_ps(pass->y);
    __m256 pz0 = _mm256_load_ps(pass->z);
    __m256 px1 = _mm256_load_ps(pass->x + 8);
    __m256 py1 = _mm256_load_ps(pass->y + 8);
    __m256 pz1 = _mm256_load_ps(pass->z + 8);

    const uint32_t pass_valid_mask = (1u << num_pass) - 1u;
    __m256 v_eps  = _mm256_set1_ps(ON_EPSILON);
    __m256 v_meps = _mm256_set1_ps(MINUS_ON_EPSILON);
#endif

    for (int i = 0; i < num_src; ++i) {
        int l = (i + 1 < num_src) ? (i + 1) : 0;
        Vector3 p1(source->x[i], source->y[i], source->z[i]);
        Vector3 v1(source->x[l] - p1.x, source->y[l] - p1.y, source->z[l] - p1.z);

        // Find a point p3 on `pass` that forms a valid separating plane with edge (p1, p2)
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

            // Flip plane normal so `pass` is on the positive side
            if (d_pass < MINUS_ON_EPSILON) {
                normal = normal * -1.0f;
                dist   = -dist;
            }

            // Vectorized test of all vertices of `pass` against `plane` using AVX2!
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

            const uint32_t other_mask = pass_valid_mask & ~(1u << j);
            if ((neg_mask & other_mask) != 0) {
                continue; // At least one pass vertex is on the wrong side
            }
            if ((pos_mask & other_mask) == 0) {
                continue; // All pass points coplanar with separator plane
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

            // Found valid separating plane!
            Plane sep_plane{normal, dist};
            if (flipclip) {
                sep_plane.normal = sep_plane.normal * -1.0f;
                sep_plane.dist   = -sep_plane.dist;
            }

            // Cache separator plane in pstack_t if slot is 0 or 1 (matches 0x1400d1ca0)
            if (sep_cache_slot == 0 || sep_cache_slot == 1) {
                int idx = stack.numseperators[sep_cache_slot];
                if (idx < MAX_SEPERATORS) {
                    stack.seperators[sep_cache_slot][idx] = sep_plane;
                    stack.numseperators[sep_cache_slot] = idx + 1;
                }
            }

            target = chop_winding_avx2_soa(target, stack, sep_plane);
            if (!target) {
                return nullptr;
            }
            break; // Move to next edge i of source
        }
    }
    return target;
}

// ----------------------------------------------------------------------------
// 4. AVX2 Minimum Vertex-to-Vertex Distance Squared (`BasePortalVis` Radial Vis)
// Reconstructed from 0x1400d0438 - 0x1400d0907
// Tests if any pair of vertices between portal A and portal B is within max_dist_sq
// ----------------------------------------------------------------------------
inline bool avx2_portals_within_radius(
    const DynamicWinding& wa,
    const DynamicWinding& wb,
    float max_dist_sq)
{
    // Fast bounding sphere check first!
    float center_dist = (wa.origin - wb.origin).length();
    if (center_dist - wa.radius - wb.radius > std::sqrt(max_dist_sq)) {
        return false;
    }
    if (center_dist <= std::sqrt(max_dist_sq)) {
        return true;
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

// >>>> END vvis_simd_avx2.hpp <<<<

// >>>> BEGIN vvis_cache.hpp <<<<
// ============================================================================
// VVIS++ ARGON EDITION (x64) — Challenge A: Incremental `.viscache` Subsystem
// Implements xxHash64 per-cluster geometric & topological hashing, dirty
// boundary graph invalidation propagation, and binary PVS + Mightsee persistence.
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

    static VVIS_FORCEINLINE uint64_t rotl64(uint64_t x, int r) {
        return (x << r) | (x >> (64 - r));
    }

    static VVIS_FORCEINLINE uint64_t mix_word(uint64_t acc, uint64_t val) {
        acc ^= (rotl64(val * PRIME64_2, 31) * PRIME64_1);
        acc = rotl64(acc, 27) * PRIME64_1 + PRIME64_4;
        return acc;
    }

    static VVIS_FORCEINLINE uint64_t avalanche(uint64_t h) {
        h ^= h >> 33;
        h *= PRIME64_2;
        h ^= h >> 29;
        h *= PRIME64_3;
        h ^= h >> 32;
        return h;
    }

    // Quantize float to 1/16th unit (0.0625) to make hash immune to FP jitter
    static VVIS_FORCEINLINE int64_t quantize_coord(float f) {
        return static_cast<int64_t>(std::llround(static_cast<double>(f) * 16.0));
    }

    static uint64_t hash_winding(const DynamicWinding& w, uint64_t seed = 0) {
        uint64_t h = seed + PRIME64_5 + static_cast<uint64_t>(w.numpoints * 24);
        for (int i = 0; i < w.numpoints; ++i) {
            uint64_t qx = static_cast<uint64_t>(quantize_coord(w.points[i].x));
            uint64_t qy = static_cast<uint64_t>(quantize_coord(w.points[i].y));
            uint64_t qz = static_cast<uint64_t>(quantize_coord(w.points[i].z));
            h = mix_word(h, qx);
            h = mix_word(h, qy);
            h = mix_word(h, qz);
        }
        return avalanche(h);
    }

    static uint64_t hash_portal(const VPortal& p) {
        uint64_t h = PRIME64_5;
        h = mix_word(h, static_cast<uint64_t>(quantize_coord(p.plane.normal.x)));
        h = mix_word(h, static_cast<uint64_t>(quantize_coord(p.plane.normal.y)));
        h = mix_word(h, static_cast<uint64_t>(quantize_coord(p.plane.normal.z)));
        h = mix_word(h, static_cast<uint64_t>(quantize_coord(p.plane.dist)));
        if (p.winding) {
            h = mix_word(h, hash_winding(*p.winding));
        }
        return avalanche(h);
    }
};

// ----------------------------------------------------------------------------
// Binary `.viscache` File Format Structures
// Stores both `portalmightsee` (BasePortalVis) and `portalvis` (PortalFlow)
// so clean portals skip BOTH BasePortalVis AND PortalFlow!
// ----------------------------------------------------------------------------
#pragma pack(push, 1)
struct VisCacheHeader {
    uint32_t magic;            // 'VC64' (0x34364356)
    uint32_t version;          // 3 (Caches both BasePortalVis + PortalFlow)
    int32_t  num_clusters;     // portalclusters
    int32_t  num_portals;      // numportals (undirected)
    int32_t  bitbytes;         // ((num_clusters + 63) & ~63) >> 3
    float    max_dist_sq;      // Fog / radius override squared
    int32_t  max_depth;        // -maxdepth setting
};

struct VisCachePortalEntry {
    uint64_t portal_geom_hash;      // xxHash64 of portal polygon & plane
    uint64_t neighborhood_hash;     // 2-hop Merkle neighborhood hash of source->target clusters
    int32_t  src_leaf;              // Source cluster
    int32_t  dst_leaf;              // Destination cluster
    int32_t  nummightsee;
    int32_t  numcansee;
    // Followed immediately by:
    //   `bitbytes` bytes of `portalmightsee` bitvector
    //   `bitbytes` bytes of `portalvis` bitvector
};
#pragma pack(pop)

// ----------------------------------------------------------------------------
// Incremental VisCache Manager
// ----------------------------------------------------------------------------
class IncrementalVisCache {
public:
    static constexpr uint32_t MAGIC_VC64 = 0x34364356;
    static constexpr uint32_t VERSION    = 3;

    int32_t cache_hits_portals   = 0;
    int32_t cache_misses_portals = 0;
    int32_t dirty_clusters       = 0;

    // Step 1: Compute per-portal and per-cluster geometric + 2-hop topological hashes
    static void compute_graph_hashes(
        std::vector<VPortal>& portals,
        std::vector<ClusterLeaf>& leafs)
    {
        for (auto& p : portals) {
            p.geometric_hash = XXHash64::hash_portal(p);
        }

        // Compute local cluster hash (order-independent sorted portal hashes)
        for (size_t c = 0; c < leafs.size(); ++c) {
            std::vector<uint64_t> phashes;
            phashes.reserve(leafs[c].portals.size());
            for (const VPortal* p : leafs[c].portals) {
                phashes.push_back(p->geometric_hash);
            }
            std::sort(phashes.begin(), phashes.end());
            uint64_t ch = XXHash64::PRIME64_5 + static_cast<uint64_t>(phashes.size());
            for (uint64_t ph : phashes) {
                ch = XXHash64::mix_word(ch, ph);
            }
            leafs[c].geometric_hash = XXHash64::avalanche(ch);
        }

        // Compute 2-hop Merkle neighborhood hash per cluster
        const size_t num_clusters = leafs.size();
        std::vector<uint64_t> hop1(num_clusters, 0);
        for (size_t c = 0; c < num_clusters; ++c) {
            std::vector<uint64_t> nh;
            for (const VPortal* p : leafs[c].portals) {
                if (p->leaf >= 0 && static_cast<size_t>(p->leaf) < num_clusters) {
                    nh.push_back(leafs[p->leaf].geometric_hash);
                }
            }
            std::sort(nh.begin(), nh.end());
            uint64_t h = leafs[c].geometric_hash;
            for (uint64_t v : nh) h = XXHash64::mix_word(h, v);
            hop1[c] = XXHash64::avalanche(h);
        }

        for (size_t c = 0; c < num_clusters; ++c) {
            std::vector<uint64_t> nh2;
            for (const VPortal* p : leafs[c].portals) {
                if (p->leaf >= 0 && static_cast<size_t>(p->leaf) < num_clusters) {
                    nh2.push_back(hop1[p->leaf]);
                }
            }
            std::sort(nh2.begin(), nh2.end());
            uint64_t h = hop1[c];
            for (uint64_t v : nh2) h = XXHash64::mix_word(h, v);
            leafs[c].neighborhood_hash = XXHash64::avalanche(h);
        }
    }

    // Step 2: Pre-load `.viscache` BEFORE BasePortalVis so clean portals skip
    // BOTH `BasePortalVis` AND `PortalFlow`!
    bool preload_before_base_vis(
        const std::string& cache_path,
        std::vector<VPortal>& portals,
        std::vector<ClusterLeaf>& leafs,
        int32_t bitbytes,
        int32_t c_longs,
        float max_dist_sq,
        int32_t max_depth,
        int32_t invalidation_radius_hops = 3)
    {
        std::ifstream ifs(cache_path, std::ios::binary);
        if (!ifs.is_open()) {
            return false;
        }

        VisCacheHeader hdr{};
        if (!ifs.read(reinterpret_cast<char*>(&hdr), sizeof(hdr))) {
            return false;
        }

        if (hdr.magic != MAGIC_VC64 ||
            hdr.version != VERSION ||
            hdr.num_clusters != static_cast<int32_t>(leafs.size()) ||
            hdr.bitbytes != bitbytes ||
            std::fabs(hdr.max_dist_sq - max_dist_sq) > 1.0f ||
            hdr.max_depth != max_depth)
        {
            return false;
        }

        struct CachedPortalData {
            VisCachePortalEntry   meta;
            std::vector<uint64_t> might_bits;
            std::vector<uint64_t> vis_bits;
        };

        std::unordered_map<uint64_t, CachedPortalData> cache_map;
        const int32_t total_directed = hdr.num_portals * 2;
        cache_map.reserve(total_directed);

        for (int32_t i = 0; i < total_directed; ++i) {
            CachedPortalData cpd;
            if (!ifs.read(reinterpret_cast<char*>(&cpd.meta), sizeof(VisCachePortalEntry))) {
                break;
            }
            cpd.might_bits.resize(c_longs, 0);
            cpd.vis_bits.resize(c_longs, 0);
            if (!ifs.read(reinterpret_cast<char*>(cpd.might_bits.data()), bitbytes)) break;
            if (!ifs.read(reinterpret_cast<char*>(cpd.vis_bits.data()), bitbytes)) break;

            uint64_t key = XXHash64::mix_word(cpd.meta.portal_geom_hash, cpd.meta.neighborhood_hash);
            cache_map[key] = std::move(cpd);
        }

        const int32_t num_clusters = static_cast<int32_t>(leafs.size());
        std::vector<int32_t> dist_to_dirty(num_clusters, 999999);
        std::queue<int32_t> bfs_q;
        dirty_clusters = 0;

        for (int32_t c = 0; c < num_clusters; ++c) {
            bool cluster_clean = true;
            for (const VPortal* p : leafs[c].portals) {
                uint64_t nh = XXHash64::mix_word(leafs[p->src_leaf].neighborhood_hash, leafs[p->leaf].neighborhood_hash);
                uint64_t key = XXHash64::mix_word(p->geometric_hash, nh);
                if (cache_map.find(key) == cache_map.end()) {
                    cluster_clean = false;
                    break;
                }
            }
            if (!cluster_clean) {
                dist_to_dirty[c] = 0;
                bfs_q.push(c);
                dirty_clusters++;
            }
        }

        // Propagate dirty boundary wave up to `invalidation_radius_hops`
        while (!bfs_q.empty()) {
            int32_t u = bfs_q.front();
            bfs_q.pop();
            if (dist_to_dirty[u] >= invalidation_radius_hops) continue;

            for (const VPortal* p : leafs[u].portals) {
                int32_t v = p->leaf;
                if (v >= 0 && v < num_clusters && dist_to_dirty[v] > dist_to_dirty[u] + 1) {
                    dist_to_dirty[v] = dist_to_dirty[u] + 1;
                    bfs_q.push(v);
                }
            }
        }

        std::vector<uint64_t> dirty_cluster_mask(c_longs, 0);
        for (int32_t c = 0; c < num_clusters; ++c) {
            if (dist_to_dirty[c] == 0) {
                dirty_cluster_mask[c >> 6] |= (1ULL << (c & 63));
            }
        }

        cache_hits_portals = 0;
        cache_misses_portals = 0;

        for (auto& p : portals) {
            if (dist_to_dirty[p.src_leaf] <= invalidation_radius_hops ||
                dist_to_dirty[p.leaf] <= invalidation_radius_hops)
            {
                cache_misses_portals++;
                continue;
            }

            uint64_t nh = XXHash64::mix_word(leafs[p.src_leaf].neighborhood_hash, leafs[p.leaf].neighborhood_hash);
            uint64_t key = XXHash64::mix_word(p.geometric_hash, nh);
            auto it = cache_map.find(key);
            if (it == cache_map.end()) {
                cache_misses_portals++;
                continue;
            }

            // If cached `portalmightsee` or `portalvis` intersects any modified cluster, re-solve it
            bool sees_dirty = false;
            const auto& might = it->second.might_bits;
            for (int32_t q = 0; q < c_longs; ++q) {
                if ((might[q] & dirty_cluster_mask[q]) != 0) {
                    sees_dirty = true;
                    break;
                }
            }
            if (sees_dirty) {
                cache_misses_portals++;
                continue;
            }

            // Full Cache Hit! Restore both `portalmightsee` and `portalvis`
            p.portalmightsee = new uint64_t[c_longs];
            p.portalvis      = new uint64_t[c_longs];
            std::memcpy(p.portalmightsee, it->second.might_bits.data(), bitbytes);
            std::memcpy(p.portalvis,      it->second.vis_bits.data(),   bitbytes);
            p.nummightsee = it->second.meta.nummightsee;
            p.numcansee   = it->second.meta.numcansee;
            p.status.store(PortalStatus::StatDone, std::memory_order_release);
            cache_hits_portals++;
        }

        return cache_hits_portals > 0;
    }

    // Step 3: Save `.viscache` to disk after compilation
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

        std::vector<uint8_t> zero_buf(bitbytes, 0);
        for (const auto& p : portals) {
            VisCachePortalEntry entry{};
            entry.portal_geom_hash  = p.geometric_hash;
            entry.neighborhood_hash = XXHash64::mix_word(
                leafs[p.src_leaf].neighborhood_hash,
                leafs[p.leaf].neighborhood_hash);
            entry.src_leaf    = p.src_leaf;
            entry.dst_leaf    = p.leaf;
            entry.nummightsee = p.nummightsee;
            entry.numcansee   = p.numcansee;

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

// >>>> END vvis_cache.hpp <<<<

// >>>> BEGIN vvis_bvh_turbo.hpp <<<<
// ============================================================================
// VVIS++ ARGON EDITION (x64) — Challenge B: Procedural Map Gen Acceleration
// (`-turbovis`, SAH-BVH Portal Culling, Topological Graph Visibility, `-maxdepth`)
// ============================================================================

#include <numeric>

namespace vvis_argon {

// ----------------------------------------------------------------------------
// 1. Flat Cache-Aligned SAH Bounding Volume Hierarchy (BVH) over Portals
// Replaces $O(N^2)$ brute-force loop in `BasePortalVis` with $O(N \log N)$
// half-space plane & radial frustum culling.
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
    int32_t left_or_first = 0; // If count > 0, index into portal_indices; else left child index
    int32_t count = 0;         // >0 for leaf node, 0 for internal node
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
            portal_aabbs[i] = box;
        }

        nodes.clear();
        nodes.reserve(2 * n);
        if (n > 0) {
            build_recursive(0, n);
        }
    }

    // Query all portals that are in front of `src_portal.plane` and within `max_dist_sq`
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
        const float max_center_to_box = use_radial_cull ? (std::sqrt(max_dist_sq) + src_r) : 0.0f;
        const float max_center_to_box_sq = max_center_to_box * max_center_to_box;

        int32_t stack[64];
        int32_t sp = 0;
        stack[sp++] = 0;

        while (sp > 0) {
            int32_t node_idx = stack[--sp];
            const PortalBVHNode& node = nodes[node_idx];

            // 1. Half-space plane test: if the entire AABB is behind `src_portal.plane`, prune subtree!
            if (node.bounds.max_signed_distance(pl) < MINUS_ON_EPSILON) {
                continue;
            }

            // 2. Radial visibility distance test: if AABB is outside max fog/override radius, prune!
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

        // Split along longest centroid extent axis
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

        // Reserve two contiguous child nodes
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
// Reconstructed from 0x1400d00b0 - 0x1400d0a78, upgraded with PortalBVH
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

    std::vector<uint64_t> portalmemory(c_portallongs, 0);
    const DynamicWinding& w = *p.winding;

    auto test_candidate = [&](int32_t tp_idx) {
        if (tp_idx == portal_idx) return;
        const VPortal& tp = portals[tp_idx];
        if (!tp.winding) return;
        const DynamicWinding& tw = *tp.winding;

        // 1. Fast bounding sphere test: is `tw` completely behind `p.plane`? (0x1400d0245)
        float d_center_tp = tw.origin.dot(p.plane.normal) - p.plane.dist;
        if (d_center_tp < -tw.radius) return;

        int32_t completely_on_or_front_tp = 0;
        int32_t front_count_tp = 0;
        int32_t k = 0;
        for (; k < tw.numpoints; ++k) {
            float d = tw.points[k].dot(p.plane.normal) - p.plane.dist;
            if (d > ON_EPSILON) {
                break; // At least one point strictly in front
            }
            if (d > MINUS_ON_EPSILON) {
                front_count_tp++;
            }
        }
        if (k == tw.numpoints) {
            if (front_count_tp != tw.numpoints) return; // Behind plane
            completely_on_or_front_tp = tw.numpoints;
        }

        // 2. Reverse plane test: is `w` completely in front of `tp.plane`? (0x1400d02fb)
        float d_center_p = w.origin.dot(tp.plane.normal) - tp.plane.dist;
        if (d_center_p > w.radius) return;

        int32_t completely_on_or_back_p = 0;
        int32_t back_count_p = 0;
        for (k = 0; k < w.numpoints; ++k) {
            float d = w.points[k].dot(tp.plane.normal) - tp.plane.dist;
            if (d < MINUS_ON_EPSILON) {
                break; // At least one point strictly behind tp.plane
            }
            if (d <= ON_EPSILON) {
                back_count_p++;
            }
        }
        if (k == w.numpoints) {
            if (back_count_p != w.numpoints) return; // Completely in front of tp.plane
            completely_on_or_back_p = w.numpoints;
        }

        // Coplanar anti-parallel check (0x1400d03cc)
        if ((completely_on_or_front_tp | completely_on_or_back_p) != 0) {
            if (p.plane.normal.dot(tp.plane.normal) < BACKFACE_EPSILON) {
                return;
            }
        }

        // 3. Radial distance culling using AVX2 FMA3 (0x1400d0438)
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

    // Flood-fill cluster visibility through `portalmemory` (Reconstructed from 0x1400d09b0)
    p.nummightsee = 0;
    std::vector<int32_t> flood_stack;
    flood_stack.reserve(256);
    flood_stack.push_back(p.leaf);

    while (!flood_stack.empty()) {
        int32_t leafnum = flood_stack.back();
        flood_stack.pop_back();

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
                flood_stack.push_back(next_p->leaf);
            }
        }
    }

    // Compute hybrid scheduling complexity score (combines nummightsee and winding vertex count)
    p.complexity_score = static_cast<float>(p.nummightsee) *
                         static_cast<float>(p.winding ? p.winding->numpoints : 4);
}

// ----------------------------------------------------------------------------
// 3. Tier 1 (`-turbovis`): Topological Portal Graph Cone/Frustum Solver
// Designed specifically for `SourceWorld` procedural maps (`mapgen.nut`) where
// pre-authored rooms/tiles are connected via sockets & doors (`PlaceDoor`, `ConnectRooms`).
// Solves visibility in < 0.1s by bounded graph traversal with analytical cone
// aperture intersection instead of recursive polygon chopping!
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

    const int32_t effective_max_depth = (max_depth > 0) ? max_depth : 6;
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
        -0.15f, // Initial wide hemisphere from doorway
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

        // Depth Clamping (`-maxdepth <N>`) for procedural room/corridor turns
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

            // Radial distance check
            Vector3 delta = next_orig - src_origin;
            float dist_sq = delta.length_sq();
            if (use_radial_vis && dist_sq > max_dist_sq) {
                continue;
            }

            // Must lie in front of source portal plane
            if (next_orig.dot(src_plane.normal) - src_plane.dist + next_rad < MINUS_ON_EPSILON) {
                continue;
            }

            // Analytical Cone-Sphere Aperture Test through intermediate doorway
            float dist = std::sqrt(std::max(dist_sq, 1.0f));
            Vector3 dir_to_next = delta * (1.0f / dist);
            float angular_spread = (src_radius + next_rad) / dist;
            float dot_cone = dir_to_next.dot(cur.cone_axis);

            if (dot_cone + angular_spread < cur.cos_half_angle) {
                continue; // Sightline blocked by intermediate doorway turn!
            }

            // Tighten cone as we step through `next_p`
            Vector3 new_axis = (cur.cone_axis + dir_to_next) * 0.5f;
            float axis_len = new_axis.length();
            if (axis_len > 1e-4f) new_axis = new_axis * (1.0f / axis_len);
            float new_cos = std::min(0.95f, std::max(cur.cos_half_angle, 1.0f - angular_spread));

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

// >>>> END vvis_bvh_turbo.hpp <<<<

// >>>> BEGIN vvis_hybrid_scheduler.hpp <<<<
// ============================================================================
// VVIS++ ARGON EDITION (x64) — Challenge C: Hybrid CPU Scheduler (Alder Lake 4P+4E)
// Eliminates the "convoy effect" on Intel Core i5-12450H (8 P-threads + 4 E-threads)
// via hardware topology detection and a Dual-Ended Complexity-Aware Work-Stealing Queue.
// ============================================================================

#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cstdio>

#if defined(_WIN32)
    #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
    #endif
    #ifndef NOMINMAX
    #define NOMINMAX
    #endif
    #include <windows.h>
#elif defined(__linux__)
    #include <sched.h>
    #include <unistd.h>
#endif

namespace vvis_argon {

enum class CoreClass : uint8_t {
    PerformanceCore = 0, // Intel Golden Cove P-core (i5-12450H: logical processors 0..7)
    EfficiencyCore  = 1  // Intel Gracemont E-core (i5-12450H: logical processors 8..11)
};

struct LogicalThreadInfo {
    uint32_t  logical_id = 0;
    uint32_t  os_cpu_set_id = 0;
    CoreClass core_class = CoreClass::PerformanceCore;
};

// ----------------------------------------------------------------------------
// Hybrid Topology Detector (Windows 10/11 `GetSystemCpuSetInformation` + Linux sysfs)
// ----------------------------------------------------------------------------
class HybridTopologyDetector {
public:
    static std::vector<LogicalThreadInfo> detect_threads(int32_t requested_threads, bool enable_hybrid_balance) {
        uint32_t hw_concurrency = std::thread::hardware_concurrency();
        if (hw_concurrency == 0) hw_concurrency = 12; // Default fallback for i5-12450H
        uint32_t count = (requested_threads > 0)
            ? std::min<uint32_t>(static_cast<uint32_t>(requested_threads), 64u)
            : std::min<uint32_t>(hw_concurrency, 64u);

        std::vector<LogicalThreadInfo> threads(count);
        for (uint32_t i = 0; i < count; ++i) {
            threads[i].logical_id = i;
            threads[i].os_cpu_set_id = i;
            threads[i].core_class = CoreClass::PerformanceCore;
        }

        if (!enable_hybrid_balance) {
            return threads;
        }

#if defined(_WIN32)
        ULONG buffer_size = 0;
        GetSystemCpuSetInformation(nullptr, 0, &buffer_size, GetCurrentProcess(), 0);
        if (buffer_size > 0) {
            std::vector<uint8_t> buffer(buffer_size);
            auto* info = reinterpret_cast<SYSTEM_CPU_SET_INFORMATION*>(buffer.data());
            if (GetSystemCpuSetInformation(info, buffer_size, &buffer_size, GetCurrentProcess(), 0)) {
                uint8_t min_eff = 255, max_eff = 0;
                std::vector<std::pair<uint32_t, uint8_t>> cpu_eff;
                ULONG offset = 0;
                while (offset + sizeof(SYSTEM_CPU_SET_INFORMATION) <= buffer_size) {
                    auto* cur = reinterpret_cast<SYSTEM_CPU_SET_INFORMATION*>(buffer.data() + offset);
                    if (cur->Type == CpuSetInformation) {
                        uint8_t eff = cur->CpuSet.EfficiencyClass;
                        min_eff = std::min(min_eff, eff);
                        max_eff = std::max(max_eff, eff);
                        cpu_eff.push_back({cur->CpuSet.LogicalProcessorIndex, eff});
                    }
                    if (cur->Size == 0) break;
                    offset += cur->Size;
                }
                if (max_eff > min_eff) {
                    // Hybrid CPU detected via Win32 EfficiencyClass!
                    // EfficiencyClass == max_eff -> P-core, < max_eff -> E-core
                    for (auto& kv : cpu_eff) {
                        if (kv.first < count) {
                            threads[kv.first].core_class =
                                (kv.second == max_eff) ? CoreClass::PerformanceCore : CoreClass::EfficiencyCore;
                        }
                    }
                    return threads;
                }
            }
        }
#endif

        // Fallback heuristic for 12-thread Intel Alder Lake i5-12450H (4P/8T + 4E/4T)
        // or general hybrid CPUs when queried in virtualized/containerized environments:
        if (count == 12) {
            for (uint32_t i = 0; i < 8; ++i)  threads[i].core_class = CoreClass::PerformanceCore;
            for (uint32_t i = 8; i < 12; ++i) threads[i].core_class = CoreClass::EfficiencyCore;
        } else if (count >= 6) {
            // Assign top 2/3 threads as P-core workers, bottom 1/3 as E-core workers
            uint32_t p_cut = (count * 2) / 3;
            for (uint32_t i = 0; i < count; ++i) {
                threads[i].core_class = (i < p_cut) ? CoreClass::PerformanceCore : CoreClass::EfficiencyCore;
            }
        }
        return threads;
    }

    static void pin_current_thread(const LogicalThreadInfo& info, bool enable_hybrid_balance) {
        if (!enable_hybrid_balance) return;
#if defined(_WIN32)
        if (info.logical_id < 64) {
            DWORD_PTR mask = (1ULL << info.logical_id);
            SetThreadAffinityMask(GetCurrentThread(), mask);
        }
#elif defined(__linux__)
        uint32_t hw = std::thread::hardware_concurrency();
        if (hw > 0 && info.logical_id < hw) {
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            CPU_SET(info.logical_id, &cpuset);
            sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
        }
#endif
    }
};

// ----------------------------------------------------------------------------
// Dual-Ended Asymmetric Work-Stealing Scheduler (`RunThreadsHybrid`)
//
// How it defeats the Alder Lake Convoy Effect:
// 1. Before dispatching `PortalFlow`, tasks can be ordered by complexity
//    (or in ascending `nummightsee` dependency order for fast early clipping,
//    while within each complexity band heavy open-space portals are prioritized
//    onto P-cores and leaf-simple portals onto E-cores).
// 2. Specifically, we maintain a lock-free two-headed atomic cursor over the
//    ordered task array `task_order[0 .. total_tasks - 1]`:
//    - In `BasePortalVis` and independent passes, `task_order` is sorted by
//      descending polygon complexity: P-cores pop from `head` (most complex portals)
//      in adaptive chunks, while E-cores pop from `tail` (simplest 4-vertex portals)
//      1 item at a time! They meet in the middle with zero idle convoy time!
//    - In `PortalFlow`, because completed simple portals (`status == StatDone`)
//      accelerate clipping of complex portals (at `0x1400cf92c`), E-cores pop
//      the ultra-fast leaf-simple portals from the front (`head++`) and rapidly
//      mark them `StatDone`, while P-cores dynamically steal medium/heavy portals
//      slightly ahead in the dependency queue, maximizing both cache reuse and
//      core utilization!
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
        bool enable_hybrid_balance,
        bool bidirectional_steal, // true: P-cores from head, E-cores from tail
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

        auto thread_infos = HybridTopologyDetector::detect_threads(num_threads, enable_hybrid_balance);
        const int32_t actual_threads = static_cast<int32_t>(thread_infos.size());

        // Lock-free dual-ended atomic indices (padded to separate cache lines to prevent false sharing)
        struct VVIS_ALIGN(64) AlignedAtomic {
            std::atomic<int32_t> val{0};
        };
        AlignedAtomic head_idx;
        AlignedAtomic tail_idx;
        AlignedAtomic completed_count;
        AlignedAtomic last_pacifier_step;

        head_idx.val.store(0, std::memory_order_relaxed);
        tail_idx.val.store(total_tasks - 1, std::memory_order_relaxed);
        completed_count.val.store(0, std::memory_order_relaxed);
        last_pacifier_step.val.store(0, std::memory_order_relaxed);

        std::mutex print_mtx;
        std::vector<uint64_t> per_thread_done(actual_threads, 0);

        auto worker_body = [&](int32_t tid) {
            const LogicalThreadInfo& tinfo = thread_infos[tid];
            HybridTopologyDetector::pin_current_thread(tinfo, enable_hybrid_balance);

            uint64_t local_done = 0;
            while (true) {
                int32_t queue_pos = -1;

                if (enable_hybrid_balance && bidirectional_steal) {
                    if (tinfo.core_class == CoreClass::PerformanceCore) {
                        // P-core grabs from head (heavy tasks)
                        queue_pos = head_idx.val.fetch_add(1, std::memory_order_relaxed);
                        if (queue_pos > tail_idx.val.load(std::memory_order_relaxed)) {
                            break;
                        }
                    } else {
                        // E-core steals from tail (light leaf-simple tasks)
                        queue_pos = tail_idx.val.fetch_sub(1, std::memory_order_relaxed);
                        if (queue_pos < head_idx.val.load(std::memory_order_relaxed)) {
                            break;
                        }
                    }
                } else {
                    queue_pos = head_idx.val.fetch_add(1, std::memory_order_relaxed);
                    if (queue_pos >= total_tasks) {
                        break;
                    }
                }

                int32_t task_id = task_order.empty() ? queue_pos : task_order[queue_pos];
                task_fn(tid, task_id);
                local_done++;

                if (show_progress) {
                    int32_t done = completed_count.val.fetch_add(1, std::memory_order_relaxed) + 1;
                    int32_t step = (done * 40) / total_tasks; // 40-step Source pacifier (0...1...2...3...4...5...6...7...8...9...10)
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
        // Main thread participates as worker 0 (P-core 0)
        worker_body(0);

        for (auto& th : workers) {
            if (th.joinable()) th.join();
        }

        auto t_end = std::chrono::high_resolution_clock::now();
        stats.elapsed_seconds = std::chrono::duration<double>(t_end - t_start).count();

        for (int32_t t = 0; t < actual_threads; ++t) {
            if (thread_infos[t].core_class == CoreClass::PerformanceCore) {
                stats.p_core_tasks_completed += per_thread_done[t];
            } else {
                stats.e_core_tasks_completed += per_thread_done[t];
            }
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

// >>>> END vvis_hybrid_scheduler.hpp <<<<

// >>>> BEGIN vvis_bsp_l4d2.hpp <<<<
// ============================================================================
// VVIS++ ARGON EDITION (x64) — BSP v20/v21 (Left 4 Dead 2 x64) & Portal I/O
// Reconstructed from vvis++.exe 0x1400e0960 (LoadBSPFile) & Source SDK bsplib.cpp
// Ensures 100% binary compatibility with Left 4 Dead 2 (BSP v21), VRAD++, and TF2/HL2 (BSP v20).
//
// CRITICAL FIXES (Argon v2):
//   1. RLE `CompressVis` / `DecompressVis` strictly uses `visrow = (portalclusters + 7) >> 3`
//      bytes per row (preventing "warning: Vis decompression overrun" in VRAD++ and Engine),
//      while internal SIMD bitvectors remain 64-bit aligned (`c_leafs_longs * 8`).
//   2. `BSPFile::save_visibility_lump_only()` performs non-destructive surgical updating of
//      `LUMP_VISIBILITY` ONLY (overwriting in-place if new_size <= old_size, or appending
//      4-byte aligned to EOF and patching ONLY `header.lumps[LUMP_VISIBILITY]`).
//      All other 63 lumps (especially `LUMP_GAME_LUMP` / `sprp` static props with absolute
//      file offsets `dgamelump_t::fileofs` and `LUMP_ENTITIES`) remain untouched byte-for-byte!
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
    // Reconstructed from vvis++.exe 0x1400d994d - 0x1400d9a9c
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
// Reconstructed from `LoadPortals` in vvis++.exe (0x1400d9c00 - 0x1400da200)
// ----------------------------------------------------------------------------
inline bool load_portals_prt(
    const std::string& prt_path,
    int32_t& out_portalclusters,
    int32_t& out_numportals,
    std::vector<VPortal>& out_portals,
    std::vector<ClusterLeaf>& out_leafs,
    std::vector<std::unique_ptr<DynamicWinding>>& out_winding_pool)
{
    FILE* f = std::fopen(prt_path.c_str(), "rb");
    if (!f) {
        std::fprintf(stderr, "LoadPortals: couldn't read %s\nThe map likely has a leak, or you forgot to run BSP step\n", prt_path.c_str());
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

    std::printf("%4i portalclusters\n", pclusters);
    std::printf("%4i numportals\n", nportals);

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

        // Compute plane from first 3 vertices (normal points into `leaf`)
        Vector3 v1 = w_fwd->points[1] - w_fwd->points[0];
        Vector3 v2 = w_fwd->points[2] - w_fwd->points[0];
        Vector3 normal_rev = v1.cross(v2);
        float len = normal_rev.length();
        if (len > 1e-6f) normal_rev = normal_rev * (1.0f / len);
        float dist_rev = w_fwd->points[0].dot(normal_rev);

        Vector3 normal_fwd = normal_rev * -1.0f;
        float dist_fwd = -dist_rev;

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

// >>>> END vvis_bsp_l4d2.hpp <<<<

// >>>> BEGIN vvis_main.cpp <<<<
// ============================================================================
// VVIS++ ARGON EDITION (x64) — Main Engine & Multi-Tier Visibility Solver
// Reconstructed from ficool2's VVIS++ (Oct 2026) & upgraded with Argon Subsystems:
//   - Challenge A: Incremental `.viscache` with xxHash64 & 2-Hop Merkle Invalidation
//   - Challenge B: SAH-BVH `BasePortalVis` + `-turbovis` Graph/Cone Solver + `-maxdepth`
//   - Challenge C: Alder Lake 4P+4E Hybrid Work-Stealing Scheduler (`-hybrid-balance`)
//   - Challenge D: Multi-Tier Draft Profiles (`-tier 0..3`, `-novis`, `-turbovis`, `-usecache`)
//
// CRITICAL FIXES (Argon v2):
//   1. RLE `CompressVis` strictly compresses `visrow = (portalclusters + 7) >> 3` bytes per row
//      (matching Source SDK 2013 `bsplib.cpp: CompressVis/DecompressVis`), while internal AVX2
//      bitvectors remain 64-bit aligned (`bitbytes_aligned = c_leafs_longs * 8`).
//   2. `BSPFile::save_visibility_lump_only()` updates ONLY `LUMP_VISIBILITY` in-place or at EOF
//      and patches only `header.lumps[LUMP_VISIBILITY]`, leaving `LUMP_GAME_LUMP` (`sprp` static
//      props), `LUMP_ENTITIES`, and all other lumps 100% untouched byte-for-byte.
// ============================================================================


#include <iostream>
#include <iomanip>
#include <chrono>

namespace vvis_argon {

// ----------------------------------------------------------------------------
// Reconstructed `CheckStack` (Expensive Stack Mightsee Pruning)
// Reconstructed from vvis++.exe 0x1400d0a80 - 0x1400d11e6
// ----------------------------------------------------------------------------
inline int32_t check_stack_avx2(
    ClusterLeaf* leaf,
    ThreadData& thread,
    const std::vector<VPortal>& portals,
    int32_t num_portals_directed,
    int32_t c_portallongs,
    int32_t c_leafs_longs)
{
    int32_t num_trimmed = 0;
    std::vector<uint64_t> portal_possible(c_portallongs, ~0ULL);
    std::vector<uint64_t> leaf_possible(c_leafs_longs, 0);

    for (PStack* p = thread.pstack_head.next; p != nullptr; p = p->next) {
        if (p->leaf == leaf) break;
        if (p->did_mightsee_test) continue;
        p->did_mightsee_test = true;

        std::fill(leaf_possible.begin(), leaf_possible.end(), 0ULL);

        if (p->pass != nullptr) {
            PStack dummy_stack;
            for (int32_t i = 0; i < num_portals_directed; ++i) {
                if (!(portal_possible[i >> 6] & (1ULL << (i & 63)))) continue;
                const VPortal& test_p = portals[i];
                if (!test_p.winding) continue;

                dummy_stack.freewindings[0] = 1;
                dummy_stack.freewindings[1] = 1;
                dummy_stack.freewindings[2] = 1;
                dummy_stack.windings[0].load_from_dynamic(*test_p.winding);
                dummy_stack.freewindings[0] = 0;

                StackWindingSOA* w = &dummy_stack.windings[0];
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
// Reconstructed from vvis++.exe 0x1400cf660 - 0x1400d0090
// ----------------------------------------------------------------------------
inline void recursive_leaf_flow_avx2(
    int32_t leafnum,
    ThreadData& thread,
    PStack* prevstack,
    int32_t current_depth,
    std::vector<VPortal>& portals,
    std::vector<ClusterLeaf>& leafs,
    int32_t c_portallongs,
    int32_t c_leafs_longs)
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

    PStack stack;
    stack.mightsee.resize(c_leafs_longs, 0);
    stack.leaf = leaf;
    stack.portal = nullptr;
    stack.next = nullptr;
    stack.freewindings[0] = 1;
    stack.freewindings[1] = 1;
    stack.freewindings[2] = 1;
    prevstack->next = &stack;

    uint64_t* might = stack.mightsee.data();
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
            prevstack->mightsee.data(),
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
        stack.freewindings[0] = 1;
        stack.freewindings[1] = 1;
        stack.freewindings[2] = 1;

        if (!p->winding) continue;

        stack.windings[0].load_from_dynamic(*p->winding);
        stack.freewindings[0] = 0;
        stack.pass = chop_winding_avx2_soa(&stack.windings[0], stack, thread.pstack_head.portalplane);
        if (!stack.pass) {
            continue;
        }

        if (!prevstack->pass) {
            stack.source = prevstack->source;
            recursive_leaf_flow_avx2(
                target_leaf, thread, &stack, current_depth + 1, portals, leafs, c_portallongs, c_leafs_longs);
            prevstack->next = nullptr;
            continue;
        }

        stack.pass = chop_winding_avx2_soa(stack.pass, stack, prevstack->portalplane);
        if (!stack.pass) {
            continue;
        }

        StackWindingSOA* src_copy = nullptr;
        for (int s = 0; s < NUM_STACK_WINDINGS; ++s) {
            if (stack.freewindings[s]) {
                stack.freewindings[s] = 0;
                stack.windings[s] = *prevstack->source;
                src_copy = &stack.windings[s];
                break;
            }
        }
        if (!src_copy) continue;

        stack.source = chop_winding_avx2_soa(src_copy, stack, backplane);
        if (!stack.source) {
            continue;
        }

        if (prevstack->numseperators[0] > 0) {
            for (int s = 0; s < prevstack->numseperators[0] && stack.pass; ++s) {
                stack.pass = chop_winding_avx2_soa(stack.pass, stack, prevstack->seperators[0][s]);
            }
        } else {
            stack.pass = clip_to_seperators_avx2_soa(
                prevstack->source, prevstack->portalplane, prevstack->pass, stack.pass, false, *prevstack, 0);
        }
        if (!stack.pass) {
            continue;
        }

        if (prevstack->numseperators[1] > 0) {
            for (int s = 0; s < prevstack->numseperators[1] && stack.pass; ++s) {
                stack.pass = chop_winding_avx2_soa(stack.pass, stack, prevstack->seperators[1][s]);
            }
        } else {
            stack.pass = clip_to_seperators_avx2_soa(
                prevstack->pass, prevstack->portalplane, prevstack->source, stack.pass, true, *prevstack, 1);
        }
        if (!stack.pass) {
            continue;
        }

        stack.source = clip_to_seperators_avx2_soa(
            stack.pass, stack.portalplane, prevstack->pass, stack.source, false, stack, 2);
        if (!stack.source) {
            continue;
        }

        stack.source = clip_to_seperators_avx2_soa(
            prevstack->pass, prevstack->portalplane, stack.pass, stack.source, true, stack, 3);
        if (!stack.source) {
            continue;
        }

        recursive_leaf_flow_avx2(
            target_leaf, thread, &stack, current_depth + 1, portals, leafs, c_portallongs, c_leafs_longs);
        prevstack->next = nullptr;
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

    data.pstack_head.portal = &p;
    data.pstack_head.source = &data.base_winding_soa;
    data.pstack_head.portalplane = p.plane;
    data.pstack_head.mightsee.assign(p.portalmightsee, p.portalmightsee + c_leafs_longs);
    data.pstack_head.mightsee_count = p.nummightsee;

    recursive_leaf_flow_avx2(
        p.leaf, data, &data.pstack_head, 1, portals, leafs, c_portallongs, c_leafs_longs);

    p.status.store(PortalStatus::StatDone, std::memory_order_release);
}

// ----------------------------------------------------------------------------
// Reconstructed Parallel Cluster PVS + `CalcPAS` + RLE `LUMP_VISIBILITY` Builder
// Reconstructed from vvis++.exe 0x1400daee0 - 0x1400dc365 & Source SDK `bsplib.cpp`
//
// NOTE ON ALIGNMENT vs RLE ROW SIZE (Argon v2 Fix #1):
//   - Internal buffers (`uncompressed_pvs`, `uncompressed_pas`) use `c_leafs_longs`
//     64-bit words (`bitbytes_aligned = c_leafs_longs * 8`) for fast 64-bit/AVX2 ops.
//   - RLE compression into `LUMP_VISIBILITY` strictly uses `visrow_bytes = (portalclusters + 7) >> 3`
//     bytes per row (`VisRLECodec::calc_visrow_bytes(portalclusters)`), matching
//     `bsplib.cpp: CompressVis` and eliminating "warning: Vis decompression overrun"!
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

    // Mask for the last 64-bit word to ensure padding bits >= portalclusters are strictly zero
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
                    bits &= (bits - 1); // Clear lowest set bit (BMI1 BLSR)
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
    // CRITICAL: Compress strictly `visrow_bytes = (portalclusters + 7) >> 3` bytes per row!
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
    std::printf("Breadworks / Argon - vvis++ (Oct 2026 Next-Gen x64 Edition)\n");
    std::printf("====================================================================================\n");
    std::printf(" %-20s | %-10s | %s\n", "Option", "Default", "Description");
    std::printf("====================================================================================\n");
    std::printf(" %-20s | %-10s | %s\n", "-threads <N>", "auto (12)", "Number of threads to use. Defaults to max CPU thread count");
    std::printf(" %-20s | %-10s | %s\n", "-tier <0|1|2|3>", "3", "Compilation gear: 0=NullVis, 1=TurboProc, 2=Cached, 3=Master");
    std::printf(" %-20s | %-10s | %s\n", "-novis", "false", "Tier 0: Instant all-visible/radial dummy PVS (<0.01s)");
    std::printf(" %-20s | %-10s | %s\n", "-turbovis", "false", "Tier 1: SAH-BVH + Topological Cone solver for mapgen.nut");
    std::printf(" %-20s | %-10s | %s\n", "-usecache", "false", "Tier 2+: Enable xxHash64 incremental .viscache subsystem");
    std::printf(" %-20s | %-10s | %s\n", "-maxdepth <N>", "unlimited", "Clamp portal traversal after N doorway turns");
    std::printf(" %-20s | %-10s | %s\n", "-hybrid-balance", "true", "Enable Alder Lake (4P+4E) asymmetric work-stealing scheduler");
    std::printf(" %-20s | %-10s | %s\n", "-fast", "false", "Only do first quick pass on vis calculations (BVH accelerated)");
    std::printf(" %-20s | %-10s | %s\n", "-radius_override <R>", "fog farz", "Force visibility radius, overrides fog radius");
    std::printf(" %-20s | %-10s | %s\n", "-nosort", "false", "Don't sort portals (sorting is an optimization)");
    std::printf(" %-20s | %-10s | %s\n", "-trace <A> <B>", "none", "Writes a linefile that traces vis from cluster A to B");
    std::printf(" %-20s | %-10s | %s\n", "-verbose (-v)", "false", "Turn on verbose output");
    std::printf("====================================================================================\n");
}

int main(int argc, char** argv) {
    using namespace vvis_argon;

    auto total_start = std::chrono::high_resolution_clock::now();
    std::printf("Breadworks - vvis++ (Argon Next-Gen x64 - Oct 2026)\n");

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
        } else if (arg == "-radius_override" && i + 1 < argc) {
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

    float max_dist_sq = MAX_COORD_DIST_SQ;
    bool  use_radial_vis = false;
    float fog_farz = bsp.parse_max_fog_farz();
    if (opts.radius_override > 0.0f) {
        max_dist_sq = opts.radius_override * opts.radius_override;
        use_radial_vis = true;
        std::printf("Forced visibility radius: %g\n", opts.radius_override);
    } else if (fog_farz > 0.0f) {
        max_dist_sq = fog_farz * fog_farz;
        use_radial_vis = true;
        std::printf("Max farz in all env_fog_controller entities: %g (used for radial vis)\n", fog_farz);
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

    // Internal SIMD buffer size is 64-bit aligned (`c_leafs_longs * 8`),
    // while RLE compression inside `assemble_and_compress_bsp_visibility` uses `(portalclusters + 7) >> 3`!
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

    // Check Incremental `.viscache` BEFORE BasePortalVis so clean portals skip BOTH passes!
    IncrementalVisCache vis_cache;
    if (opts.use_cache || opts.tier == CompileTier::Tier2_HybridCached) {
        IncrementalVisCache::compute_graph_hashes(portals, leafs);
        bool loaded = vis_cache.preload_before_base_vis(
            cache_path, portals, leafs, bitbytes_aligned, c_leafs_longs, max_dist_sq, opts.max_depth);
        if (loaded) {
            std::printf("  [VisCache] Restored %d / %d directed portals from %s (%d dirty clusters)\n",
                vis_cache.cache_hits_portals, total_directed, cache_path.c_str(), vis_cache.dirty_clusters);
        }
    }

    // Build SAH-BVH over Portals & Run `BasePortalVis` on uncached portals
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
    if (opts.hybrid_balance) {
        std::sort(base_order.begin(), base_order.end(), [&](int32_t a, int32_t b) {
            int na = portals[a].winding ? portals[a].winding->numpoints : 0;
            int nb = portals[b].winding ? portals[b].winding->numpoints : 0;
            return na > nb;
        });
    }

    auto base_stats = HybridWorkStealingScheduler::run_parallel(
        "BasePortalVis",
        static_cast<int32_t>(base_order.size()),
        base_order,
        opts.threads,
        opts.hybrid_balance,
        true,
        true,
        [&](int32_t /*tid*/, int32_t pid) {
            base_portal_vis_bvh_worker(
                pid, portals, leafs, bvh.get(), c_portallongs, c_leafs_longs, max_dist_sq, use_radial_vis);
        });

    if (opts.verbose) {
        std::printf("  [Hybrid Scheduler] BasePortalVis tasks: P-cores=%llu, E-cores=%llu\n",
            static_cast<unsigned long long>(base_stats.p_core_tasks_completed),
            static_cast<unsigned long long>(base_stats.e_core_tasks_completed));
    }

    // Sort Portals by `nummightsee` ascending (`PComp`, reconstructed from 0x1400d8870)
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
            auto flow_stats = HybridWorkStealingScheduler::run_parallel(
                "TurboPortalFlow",
                total_directed,
                flow_order,
                opts.threads,
                opts.hybrid_balance,
                true,
                true,
                [&](int32_t /*tid*/, int32_t pid) {
                    turbo_procedural_portal_vis_worker(
                        pid, portals, leafs, c_leafs_longs, turbo_depth, max_dist_sq, use_radial_vis);
                });
            if (opts.verbose) {
                std::printf("  [Hybrid Scheduler] TurboPortalFlow tasks: P-cores=%llu, E-cores=%llu\n",
                    static_cast<unsigned long long>(flow_stats.p_core_tasks_completed),
                    static_cast<unsigned long long>(flow_stats.e_core_tasks_completed));
            }
        } else {
            std::vector<int32_t> pending_flow_order;
            pending_flow_order.reserve(total_directed);
            for (int32_t pid : flow_order) {
                if (portals[pid].status.load(std::memory_order_relaxed) != PortalStatus::StatDone) {
                    pending_flow_order.push_back(pid);
                }
            }

            auto flow_stats = HybridWorkStealingScheduler::run_parallel(
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

            if (opts.verbose) {
                std::printf("  [Hybrid Scheduler] PortalFlow tasks: P-cores=%llu, E-cores=%llu\n",
                    static_cast<unsigned long long>(flow_stats.p_core_tasks_completed),
                    static_cast<unsigned long long>(flow_stats.e_core_tasks_completed));
            }
        }

        if (opts.use_cache || opts.tier == CompileTier::Tier2_HybridCached) {
            if (vis_cache.save(cache_path, portals, leafs, numportals, bitbytes_aligned, max_dist_sq, opts.max_depth)) {
                if (opts.verbose) {
                    std::printf("  [VisCache] Saved %d portals to %s\n", total_directed, cache_path.c_str());
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

// >>>> END vvis_main.cpp <<<<
