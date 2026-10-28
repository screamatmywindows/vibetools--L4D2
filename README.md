# vibevvis#
Fastest 64-bit VVIS for Source 1 / L4D2 with spatial `.viscache`
## Why it cooks
Better than `ficool2 vvis++` because it actually has a working cache. Moved 1 brush? It won't freeze your PC for 15-60s — it hits `.viscache` and finishes in **0.1 - 3s** huh
> **Disclaimer**: Barely tested ngl. Bugs or vis holes? Drop an **Issue**. ll fix it (with vibecoding ofc, what did you expect, a fairy tale? lmao)
## Setup & Hammer
1. Drop everything from `vvis/` (`vibevvis#.exe` + DLLs) into `bin/x64/`.
2. In Hammer / Hammer++ compile settings, slap **`-usecache`** or it'll do a slow cold run:
   ```bat
   -usecache -game $gamedir $path\$file
   ```
## Flags
- `-usecache` — **MANDATORY** for spatial cache (instant rebuilds).
- `-fast` — Draft vis run.
- `-radius_override <N>` — Fog/cull clamp (safe floor 8192).
- `-threads <N>` — Thread count override.
