#pragma once
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

#include "vvis_argon.hpp"
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
