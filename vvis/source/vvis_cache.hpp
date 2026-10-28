#pragma once
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

#include "vvis_argon.hpp"
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
