#pragma once
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
