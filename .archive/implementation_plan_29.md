# Implementation Plan

## Task summary

Source: GitHub issue #29
Base branch: develop

Update the four `irurueta-*` compile-scope dependencies in `pom.xml` to their latest patch releases:
`irurueta-numerical` 1.6.0 → 1.6.1, `irurueta-geometry` 1.6.0 → 1.6.1, `irurueta-units` 1.4.0 → 1.4.1 and
`irurueta-algebra` 1.4.0 → 1.4.1. The issue is unlabeled and is treated as a `feature`-prefixed chore (branch
`feature/29-update-dependencies`).

Approach (chosen without asking, following the repository's own precedent in `CHANGELOG.md`): patch bumps only, no
library source changes expected; the existing test suite is the regression check; the change is recorded in
`CHANGELOG.md` under `[Unreleased]` in the same style as earlier dependency bumps.

Open clarifications: none

## Acceptance criteria coverage

| # | Criterion | Covered by tasks |
|---|---|---|
| 1 | "Update dependencies in pom.xml file to latest versions of irurueta-numerical 1.6.1, irurueta-geometry 1.6.1, irurueta-units 1.4.1, irurueta-algebra 1.4.1" | Task 1 |
| 2 | derived: the project compiles and all existing tests pass against the new versions | Task 3 |
| 3 | derived: the dependency bump is documented in the changelog | Task 2 |

## Current code state

- Java 21 library, Maven single module (no wrapper), `pom.xml` at the repository root, version `1.9.0-SNAPSHOT`.
- `pom.xml` lines ~54-76: four `com.irurueta` dependencies, `compile` scope: `irurueta-numerical` 1.6.0,
  `irurueta-geometry` 1.6.0, `irurueta-units` 1.4.0, `irurueta-algebra` 1.4.0.
- `CHANGELOG.md` has an empty `## [Unreleased]` section (line 8); prior bumps are recorded as
  `### Changed` bullets ("Updated dependencies ... from X to Y.").
- Antora docs in `docs/` (`docs/modules/ROOT/pages/`); no page references the dependency versions (only generated
  output under `docs/build/` does, which is not edited). `whats-new.adoc` is release-time content, handled by
  `iru-release`, not by this change.

## Implementation steps

### Group 1 — Dependency bump (Parallelizable: yes)

- [x] Task 1. Bump the four dependency versions in `pom.xml` _(java)_ — pom.xml only, no tests/coverage applicable (no Java classes touched); validation deferred to Group 2.
  - [x] Task 1.1. Set `irurueta-numerical` and `irurueta-geometry` `<version>` to `1.6.1`
  - [x] Task 1.2. Set `irurueta-units` and `irurueta-algebra` `<version>` to `1.4.1`

- [x] Task 2. Record the change in `CHANGELOG.md` _(untagged, docs only)_ — CHANGELOG.md only.
  - [x] Task 2.1. Under `## [Unreleased]` add `### Changed` with the bullet: "Updated dependencies
    `irurueta-numerical` and `irurueta-geometry` from 1.6.0 to 1.6.1, and `irurueta-units` and `irurueta-algebra`
    from 1.4.0 to 1.4.1."

### Group 2 — Verification (Parallelizable: yes)

- [x] Task 3. Verify the build against the new versions _(java)_
    - Verified: 990 tests pass (0 failures); line coverage 97.60%, branch 85.86%; Checkstyle 6573 / PMD 59 / SpotBugs 193 (unchanged from baseline). No Java sources changed.
  - [x] Task 3.1. Delegate to `iru-gate-runner` (`Skill iru-java-test`, full suite) to compile and run all tests;
    fix any compile/test failure caused by the new versions (none expected for patch releases)
  - [x] Task 3.2. Delegate to `iru-gate-runner` (`Skill iru-java-coverage`) and confirm coverage stays at or above
    80%
  - [x] Task 3.3. Delegate to `iru-gate-runner` (`Skill iru-java-code-quality`) and confirm no new
    Checkstyle/PMD/SpotBugs issues
