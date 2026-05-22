---
name: century-build-workflow
description: Use when user asks to "compile century", "build module", "build_opt_gpu", "build failed", or "how to build in Docker". Always trigger for Century build, rebuild, or build troubleshooting requests.
---

# Century Build Workflow

Use this skill for all Century compilation tasks, including full build, module build, and build failure triage.

## Source-of-Truth Files

- `century.sh`
- `scripts/century_build.sh`
- `fast_build.sh`
- `scripts/century_test.sh`
- `scripts/century_clean.sh`
- `docs/specs/century_build_and_test_explained.md`
- `README.md`

## Command Matrix

- Full build: `bash century.sh build`
- Optimized build: `bash century.sh build_opt`
- Debug build: `bash century.sh build_dbg`
- CPU build: `bash century.sh build_cpu`
- GPU build: `bash century.sh build_gpu`
- Optimized GPU build: `bash century.sh build_opt_gpu`
- Module-only build: `bash century.sh build <module>` (example: `bash century.sh build planning`)
- Fast iterative build (general): `bash fast_build.sh`
- Fast iterative build (perception): `bash fast_build.sh perception`
- Build + test + lint gate: `bash century.sh check`
- Show command help: `bash century.sh --help`

## Recommended Perception Compile Path

Use this path by default for perception compilation:

1. Enter Docker dev container:
   - `bash docker/scripts/dev_into.sh`
2. Initialize Cyber environment in container:
   - `source cyber/setup.bash`
3. Compile perception quickly:
   - `bash fast_build.sh perception`

If container is not started yet, run `bash docker/scripts/dev_start.sh` first.

For non-interactive sessions (agent/CI/no TTY), use Docker exec fallback:

- `docker exec -u "<user>" "century_dev_<user>" /bin/bash -lc 'cd /century && source cyber/setup.bash && bash fast_build.sh perception'`

## Standard Build Procedure

1. Confirm you are at repo root and use `century.sh` as the entrypoint.
2. If needed, enter Docker dev environment first:
   - `bash docker/scripts/dev_start.sh`
   - `bash docker/scripts/dev_into.sh`
3. In Docker shell, source runtime environment before build:
   - `source cyber/setup.bash`
4. Pick build mode by target:
   - GPU-dependent perception path: `build_opt_gpu`
   - Common dev build: `build` or `build_opt`
   - Repro/debug symbols: `build_dbg`
5. For fast iteration, build specific modules first:
   - `bash century.sh build cyber`
   - `bash century.sh build perception`
   - `bash century.sh build planning`
   - `bash fast_build.sh perception` (preferred for perception iteration)
6. Before merge or delivery, run:
   - `bash century.sh check`

## Build Internals to Respect

- `century.sh` forwards to `scripts/century_build.sh`.
- `scripts/century_build.sh` decides CPU/GPU mode and expands targets.
- Build target syntax maps to Bazel target groups under `//modules/...` and `//cyber/...`.
- Some targets are conditionally disabled by architecture, compiler version, or environment.

## Failure-Stage Troubleshooting

### Stage 1: Environment or platform gate fails

- Check architecture support and platform checks from `century.sh`.
- Check memory warning path (`minimal memory requirement`) and reduce workload if needed.
- If build is launched on host (not in container) and fails with `/century/.cache/bazel` permission or missing path, switch to container workflow first.

### Stage 1.5: Container entry method mismatch

- If `bash docker/scripts/dev_into.sh` fails with `the input device is not a TTY`, do not retry the same command in non-interactive mode.
- Use the Docker exec fallback command from `Recommended Perception Compile Path`.

### Stage 2: CPU/GPU mode mismatch

- Avoid mixing CPU and GPU config flags in one command.
- If GPU unavailable, switch to CPU build path.

### Stage 3: Module target or dependency issue

- Verify module name exists under `modules/<name>`.
- Retry module-only build to isolate failure before full build.

### Stage 4: Dirty build cache or stale outputs

- Use `bash century.sh clean`.
- Re-run selected build mode.

### Stage 5: Quality gate failure

- Run and separate concerns:
  - `bash century.sh test <module>`
  - `bash century.sh lint`
  - `bash century.sh check`

## Guardrails

- Do not bypass `century.sh` unless there is a concrete reason.
- Do not guess module names or build options; read from `century.sh --help`.
- Prefer module build first when debugging compile failures.
- In Docker build workflows, do not skip `source cyber/setup.bash` before compiling.
- For perception fast build, run inside container at `/century` instead of host path.

## Output Format For User

Always return:

1. Selected build mode and why
2. Exact command(s)
3. Scope (full project or module)
4. Failure stage (if any) and evidence
5. Next actionable command
