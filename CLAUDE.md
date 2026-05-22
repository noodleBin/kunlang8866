# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository layout: two copies

This workspace holds two sibling copies of the **Century** autonomous-driving stack:

- `88/` — the active, complete copy. Has `century.sh`, `docs/`, and a set of project skills under `88/skills/`. **Default to working here.**
- `66/` — an older/partial copy. Lacks `century.sh`; it uses `fast_build.sh` directly and has some `*.shbk` backup scripts.

Neither directory is a git repository in this checkout. Paths inside scripts and configs are hardcoded to `/century` (the deployed mount point), not `/home/wb/7car/88` — see "Runtime path convention" below.

## What Century is

Century is a fork of Baidu Apollo. It runs on the **Cyber RT** middleware (`cyber/`) and is organized as Apollo-style modules under `modules/` (perception, planning, control, prediction, routing, canbus, localization, drivers, dreamview, etc.). Components are loaded as `.so` shared libraries and wired together via Cyber **DAG** files; the `mainboard` binary launches a DAG. It targets `x86_64` and `aarch64` (Orin), and is built/run inside a Docker dev container.

## Build, test, lint (run from `88/`)

`century.sh` is the single entrypoint; it sources `scripts/century.bashrc` and forwards to `scripts/century_*.sh`. Do not bypass it without a concrete reason.

```bash
bash century.sh build          # standard build
bash century.sh build_opt      # optimized
bash century.sh build_dbg      # debug symbols
bash century.sh build_gpu      # CUDA/TensorRT (config=gpu)
bash century.sh build_opt_gpu  # optimized GPU — the perception path
bash century.sh build planning # module-only build (faster iteration)
bash century.sh test           # bazel test
bash century.sh lint           # cpplint / clang-format
bash century.sh check          # build + test + lint gate — run before delivery
bash century.sh clean          # clear stale bazel cache/outputs
bash century.sh --help         # authoritative list of commands & module names
```

Underneath it is **Bazel** (`WORKSPACE` declares `workspace(name = "apollo")`; targets are `//cyber/...` and `//modules/...`). GPU builds pull in `--config=cuda --config=tensorrt` (CUDA 11.4) per `.century.bazelrc`. The bazel cache lives under `/century/.cache/bazel`; full rebuilds require deleting `.cache` and take ~1h vs ~3–5min incremental.

**Fast iterative build** (builds a curated target list, not the whole tree):
```bash
bash fast_build.sh             # general targets
bash fast_build.sh perception  # perception targets only — preferred for perception work
```
`fast_build.sh` defines `ALL_TARGETS` / `MASTER_TARGETS` / `SLAVE_TARGETS` — the master/slave split reflects multi-domain-controller deployment (one box runs control/planning/canbus, another runs perception/localization/drivers).

## Docker dev environment

Builds and runtime are expected inside the dev container:
```bash
bash docker/scripts/dev_start.sh   # start container
bash docker/scripts/dev_into.sh    # enter it (interactive, needs a TTY)
source cyber/setup.bash            # MUST source before building/running
```
For non-interactive/agent/CI use, `dev_into.sh` fails with "input device is not a TTY"; use exec instead:
```bash
docker exec -u "<user>" "century_dev_<user>" /bin/bash -lc \
  'cd /century && source cyber/setup.bash && bash fast_build.sh perception'
```

## Running the system

```bash
bash launch_century_system.sh   # full stack; auto-selects tasks per platform (uname -m)
bash stop.sh                    # stop / reset everything
bash node_monitor.sh            # process guard — restarts crashed nodes
bash scripts/bootstrap.sh       # start Dreamview web UI (then launch modules from the browser)
```
`launch_century_system.sh` calls `stop.sh`, `disk_clear.sh`, `create_link.sh`, sets RT scheduling, then launches modules as `mainboard -d <module>/dag/<x>.dag`. Individual pipelines have dedicated scripts: `scripts/perception.sh`, `launch_lidar.sh`, `launch_camera.sh`, etc.

Inspect live channels with `cyber_monitor`. Expected topics are listed in `scripts/topics.txt`; key perception outputs are `/century/perception/obstacles` and `/century/perception/traffic_light`.

## Runtime path convention (important)

All DAGs, launch files, and `conf.json`/`env.json` reference absolute paths under **`/century/...`**, the deployment mount, not the repo's actual location. On a domain controller `/century` is the project root (an external disk on Orin, local disk on x86). When reading a DAG/config that points at `/century/modules/...`, map it to `88/modules/...` in this checkout. `create_link.sh` sets up the symlinks the runtime expects.

## Perception pipeline (most actively developed area)

Modules: `modules/perception/{lidar,camera,fusion,lidar_tracking,onboard,production,...}`. Wiring lives under `modules/perception/production/`:
- launch files: `production/launch/perception_{all,lidar}.launch`
- DAG: `production/dag/dag_streaming_perception_lidar.dag`
- config: `production/conf/perception/lidar/config_manager.config` (detector/component names, `input_channel_name`, `output_channel_name`, `enable_hdmap`)

Channel names in the config must match readers/writers in the DAG. Helper tools: `modules/tools/perception/print_perception.py`, `replay_perception.py`.

## C++ coding standards (enforced — see `88/skills/century-cpp-coding-review`)

Simplified Google C++ style with project-specific rules. Notable, non-obvious ones:
- **Constant-left for `==` only**: write `if (kReady == status)`, `if (nullptr == ptr)`. Does NOT apply to `!=`, `<`, `>`, `<=`, `>=`.
- 2-space indent, 80-col lines, braces mandatory on all `if`/`for`/`while`.
- Names `CamelCase`; constants `kCamelCase`.
- No commented-out/dead code; no `std::cout`/`printf`/`cerr` debug prints — use the logging framework.
- Prefer `vec[i]` over `.at()`, `emplace_back` over `push_back`; no object construction inside loop bodies.
- **No Chinese characters in code** (identifiers/comments/strings must be ASCII); Chinese is fine in `*.md` docs.
- No hardcoded topic names/paths/thresholds — use named `constexpr`/config.
- Functions hard-capped at 200 lines (target <100).
- Include order: main header → C → C++ stdlib → third-party → gtest → other project headers → `.pb.h` → `cyber/`+`modules/` headers.

Config for linters: `CPPLINT.cfg`, `.clang-format`. Run `bash century.sh check` (or `bash century.sh format`) before committing.

## Project skills

`88/skills/` contains runbooks that go deeper than this file. Consult them for their domains:
`century-project-workflow` (general runbook), `century-build-workflow` (build triage), `century-perception-pipeline-runner` and the `century-perception-{lidar,camera,trafficlight}-only` variants, `century-drivable-area-integration`, `century-cpp-coding-review`, `autonomous-driving-perception-expert`.

## Docs

`docs/specs/Century_3.0_Software_Architecture*.md` and `docs/specs/Century_3.5_Software_Architecture.md` describe the overall architecture; `docs/howto/` has run guides; `docs/FAQs/` covers build/perception/calibration issues. The top-level `README.md` (Chinese) covers deployment on Orin / x86 domain controllers and Docker.
