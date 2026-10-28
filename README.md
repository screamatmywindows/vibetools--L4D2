# vibevvis#

Fast, modern 64-bit VVIS compiler for Source Engine 1 / Left 4 Dead 2 with spatial-resilient caching (`.viscache`).

---

## Why vibevvis#?
This VVIS is better than `ficool2 vvis++` because it features a true **spatial-resilient cache (`.viscache`)**.
If you move a single brush, adjust geometry in a room, or edit small sections of your map, it does not re-compute portal vis flow from scratch for 10 minutes. Instead, it hits the spatial cache and finishes incremental builds in under **0.1 - 0.5 seconds**!

> **Disclaimer**: This compiler has not been tested on every existing Source map in existence. If you discover bugs, vis holes, or overculling — please open an **Issue** here on GitHub. I will fix them (with vibe coding of course, what did you think, you ended up in a fairy tale? hahahaha).

---

## Installation & Usage

1. Extract all contents from `vvis/` (`vibevvis#.exe` and all included `.dll` files) into your game's `bin/x64/` folder.
2. In your Hammer / Hammer++ compile parameters, **ALWAYS add `-usecache` tag**:
   ```bat
   $bin_dir/x64/vibevvis#.exe -usecache "$path/$file"
   ```

---

## Command-line Parameters:
| Flag | Description |
| :--- | :--- |
| **`-usecache`** | **MANDATORY for cache!** Enables spatial-resilient `.viscache` (0.1s iterative rebuilds). |
| **`-fast`** | Fast draft vis pass. |
| **`-radius_override <N>`**| Radial fog/cull distance override (minimum safe clamp 8192). |
| **`-threads <N>`** | Override worker thread count (defaults to hardware concurrency). |
| **`-verbose` / `-v`** | Detailed portal flow diagnostic output. |

---

## Directory Structure
- `vvis/` — Precompiled `vibevvis#.exe` and required x64 `.dll` files.
- `vvis/source/` — Full standalone C++ source code, headers, CMakeLists, and MSVC build script (`build_msvc2022.bat`).
