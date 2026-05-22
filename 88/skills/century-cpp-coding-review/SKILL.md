---
name: century-cpp-coding-review
description: Enforce Century autonomous driving C++ coding standards based on a simplified Google C++ Style Guide
license: MIT
compatibility: opencode
metadata:
  audience: autonomous-driving-engineers
  language: cpp
  framework: century
---

## Purpose

Enforce Century Autonomous Driving Project C++ rules during coding and code
review. These rules are mandatory and violations are rejected.


## What I do

- Check compliance with Apollo module and directory conventions
- Validate component lifecycle usage
- Identify perception-specific architectural issues
- Flag unsafe pointer ownership and threading risks

## Mandatory Rules

1. Follow simplified Google C++ style:
- Type names use `CamelCase`
- Functions and variables use `CamelCase`
- Constants use `kCamelCase`
- Use 2-space indentation and no tabs
- Keep max line length at 80

2. No commented-out code:
- Delete dead code; do not keep disabled statements in comments

3. No debug prints:
- Do not use `std::cout`, `printf`, or `std::cerr` for debugging
- Do not keep debug print lines even in comments
- Use structured logging framework only

4. Performance-first containers:
- Use `vec[i]`, not `vec.at(i)`, unless explicit bounds check is required
- Use `emplace_back(...)`, not `push_back(...)`

5. No trivial comments:
- Comments must explain why, not what

6. Constant-left equality == only
- Apply this rule only to the equality operator ==.
- Do not apply this rule to !=, <, >, <=, or >=.
#### Rule
For comparisons using ==, constants, literals, nullptr, and enum values must be placed on the left-hand side.
- **USE**: `if (5 == x)`
- **NEVER**: `if (x == 5)`
- **USE**: `if (nullptr == ptr)`
- **NEVER**: `if (ptr == nullptr)`
- **USE**: `if (kReady == status)`
- **NEVER**: `if (status == kReady)`
#### Not enforced for other operators
The following forms are allowed and should not be flagged:
- `if (x != 5)`
- `if (x < 5)`
- `if (x > 5)`
- `if (x <= limit)`
- `if (x >= limit)`
- `if (ptr != nullptr)`
- `if (frame->world_cloud->empty())`
#### Rationale
Using constant-left style only for `==` helps prevent accidental assignment mistakes while preserving readability for other comparison operators.

7. No Chinese characters in code:
- Identifiers, comments, and strings in code must be ASCII English
- Chinese is only acceptable in external `*.md` documents

8. Self-documenting variable names:
- Variable names must describe purpose directly

9. No custom object creation in loops:
- Do not create class objects inside loop bodies
- Primitive local variables are allowed in loops

10. Braces are mandatory:
- Always use `{}` with `if`, `for`, and `while`, including single-line bodies

11. Violation reporting is mandatory:
- Every violation must cite exact rule number
- Always provide corrected code
- Do not use optional wording

12. Function length limits:
- Hard limit: no function may exceed 200 lines
- Target: each function should be under 100 lines
- If over 200 lines, refactor into helper functions under 100 lines each

13. Implicit boolean checks are allowed

- Boolean expressions may be used directly in conditions.
- Do **not** require explicit comparison with `true` or `false`.

#### Allowed
- `if (is_ready) {}`
- `if (!is_valid) {}`
- `while (has_data) {}`
- `return success;`

#### Also allowed
- `if (true == flag) {}`
- `if (false == flag) {}`

#### Do not flag
- Direct use of a boolean variable or boolean-returning expression in a condition.
- Negated boolean expressions such as `if (!flag)`.

#### Rationale
Implicit boolean checks are idiomatic in C++ and improve readability.
Requiring explicit comparison with `true` or `false` adds noise without improving clarity.

14. No hardcoded business or configuration values

- Do not hardcode topic names, file paths, thresholds, model-specific tuning values, or other project/business constants directly in logic.
- Extract reusable values into named `constexpr`/`const` symbols, configuration, or existing shared definitions.
- Short-lived literals that are inherently self-explanatory are allowed, such as `0`, `1`, `nullptr`, empty containers, and loop increments.

#### Flag
- `if (score > 37.5f) { ... }`
- `reader_.Init("/apollo/sensor/camera/front_6mm/image");`
- `const int kRetry = 3;  // inside a narrow function when project already has shared config`

#### Prefer
- `if (score > kScoreThreshold) { ... }`
- `reader_.Init(camera_topic_);`
- Centralized constants or config-backed values with names that explain intent.

#### Rationale
Hardcoded values hide intent, make tuning error-prone, and create inconsistent behavior across modules.

## Include Order

Apply this include order:

1. Main header (priority 0)
2. C system headers (priority 1)
3. C++ standard library (priority 2)
4. System/third-party libraries (priority 3)
5. Test libraries like gtest (priority 4)
6. Other project headers (priority 5)
7. Protobuf headers (`.pb.h`) (priority 6)
8. Century project headers (`cyber/`, `modules/`) (priority 7)

## Build and Validation

- Build with Bazel via `century.sh build`
- Follow project `CPPLINT.cfg` and `.clang-format`
- Run `bash century.sh check` before commit

## Required Review Workflow

When reviewing or editing C++ code:

1. Scan for all 14 rules.
2. For each issue, report:
- Rule number and title
- Non-compliant code
- Corrected code
3. Use mandatory language only. Do not use words such as:
- consider
- recommend
- suggest
- optional
4. Do not skip violations.

## Required Review Output Format

Use this exact structure:

```text
Rule Violations Found:

1. Rule X Violation: <short title>
   Found: <non-compliant code>
   Correction: <corrected code>
```

## When to use me

Use this during code review or before merging perception-related PRs.
