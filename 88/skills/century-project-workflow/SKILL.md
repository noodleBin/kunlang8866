---
name: century-project-workflow
description: Use when user asks to build, run, debug, or navigate the Century project repository. Trigger on phrases like "build century", "run century", "how to start modules", "check repo structure", and "century engineering workflow".
---

# Century Project Workflow

Use this skill as the default engineering runbook for the Century repository.

## When to Use

- User asks how to build or check the project.
- User asks how to launch the full stack or key modules.
- User asks where key scripts, docs, and module entrypoints are.
- User asks for a safe workflow before coding or submitting changes.

## Source-of-Truth Paths

- `century.sh`
- `launch_century_system.sh`
- `stop.sh`
- `node_monitor.sh`
- `scripts/perception.sh`
- `scripts/topics.txt`
- `docs/howto/README.md`
- `modules/` (module roots)

## Standard Engineering Workflow

### 1) Confirm baseline context

- Confirm current repo root and module scope.
- Read project runbook files before proposing commands.
- Reuse existing scripts before introducing new command chains.

### 2) Build and quality gates

Prefer official project commands:

- `bash century.sh build`
- `bash century.sh build_opt_gpu` (GPU perception path)
- `bash century.sh test`
- `bash century.sh check`

If user scope is module-specific, target module builds/tests through the same command entrypoint.

### 3) Launch and runtime checks

- Full system: `bash launch_century_system.sh`
- Stop/reset: `bash stop.sh`
- Process guard and restarts: `bash node_monitor.sh`

When diagnosing runtime issues, verify expected topics from `scripts/topics.txt` and expected DAG/process commands from launch scripts.

### 4) Module-level drill-down

- Move from top-level script -> module launch/dag -> module configs.
- For perception-specific requests, chain into `century-perception-pipeline-runner`.
- For drivable-area interface work, chain into `century-drivable-area-integration`.

### 5) Reporting discipline

Return concrete evidence, not assumptions:

- exact command used
- exact file path inspected
- observed behavior or output
- next fix with one actionable command

## Guardrails

- Do not invent scripts or file paths.
- Do not replace canonical `century.sh` workflow with ad-hoc alternatives unless required.
- Do not perform destructive git or filesystem operations unless explicitly requested.

## Output Format For User

Always return:

1. Goal and scope
2. Commands executed (or proposed)
3. Files inspected or changed
4. Verification result
5. Next step
