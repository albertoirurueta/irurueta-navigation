---
name: iru-dotnet-generate-all-tests
description: For a .NET/C# project or solution only, explore the whole codebase to find every class that has no corresponding unit test class, or whose measured line coverage falls below 80%, using the `iru-dotnet-coverage` skill to get the exact percentage per class — then generate new tests (or extend existing ones) until each reaches that bar, following this project's own test framework and style. Invoke as `/iru-dotnet-generate-all-tests`. Stops immediately, without attempting anything, if the repository isn't a .NET/C# project (no `.sln`/`.csproj` found). Use whenever the user wants comprehensive unit test coverage brought up across an entire .NET codebase in one pass, instead of writing tests for one class or task at a time.
model: sonnet
allowed-tools: Read Edit Write Bash(dotnet *) Bash(git status *) Bash(git diff *) Bash(find *) Bash(grep *) Bash(ls *) Skill Agent AskUserQuestion
---

# .NET Generate All Tests

Bring an entire .NET/C# codebase's unit test coverage up to a minimum bar (80% line coverage) in one pass: find
every class with no test class at all or with measured coverage below the bar, then write or extend tests for
each until it clears it. This skill is .NET/C# only — it never runs against, or falls back to, any other
language. It only adds/extends test code; it does not add license headers, doc comments, or run a code-quality
pass (those are separate skills' jobs), and it only touches production source code if a class is genuinely
untestable as written (see Step 5) — which it reports rather than silently "fixing" by redesigning the class.

## Step 1 — Confirm this is a .NET project

```bash
find . \( -name "*.sln" -o -name "*.csproj" \) -not -path "*/bin/*" -not -path "*/obj/*"
```

- **None found**: this skill applies only to .NET/C# projects. Tell the user plainly and stop — do not attempt
  any other language's equivalent or partial exploration.
- **Found**: continue. If there are multiple `.sln` files and no obvious single target, ask the user which
  solution to scope this run to (same convention as `iru-dotnet-test`/`iru-dotnet-coverage`).

## Step 2 — Discover every candidate class and the existing test convention

Use the `Explore` agent for this — a repository of any real size has too many source files to read directly one
by one, and this step needs a structural map, not a design review.

- Identify every **main-source** class (excluding test projects, `bin/`, `obj/`, generated code such as
  `*.Designer.cs`/`*.g.cs`, `AssemblyInfo.cs`/`GlobalUsings.g.cs`, and EF Core `Migrations/` folders). Use
  judgment on classes with no real logic to exercise (a pure marker interface, an empty partial class, a
  top-level `Program.cs`/`Main` that only wires up DI and has no branching logic of its own) — note these as
  "excluded, no testable logic" in the final report rather than silently dropping them or forcing trivial tests
  onto them.
- Identify every **existing test project** (a project referencing `Microsoft.NET.Test.Sdk` plus `xunit`/`nunit`/
  `MSTest.TestFramework`) and the naming convention it already uses to map a source class to its test class
  (e.g. `Foo` → `FooTests` or `FooTest`, and whether test classes mirror the source project's
  namespace/folder layout 1:1). If **no test project exists anywhere** in the repository, stop and tell the
  user: this skill extends an existing test project's coverage, it doesn't scaffold a new one from nothing —
  ask (`AskUserQuestion`) whether a test project actually exists under a naming convention this pass didn't
  recognize before concluding there really is none.
- For every candidate class, resolve whether a matching test class file already exists by that convention, and
  its path if so.

Build a plain list: fully-qualified class name → source file path → matching test file path (or "none").

## Step 3 — Measure real coverage for every candidate class in one pass

Delegate this to `iru-dotnet-coverage` via the `iru-gate-runner` agent, so a large `coverage.cobertura.xml` never lands
directly in this conversation's context, and so the whole suite is only executed once regardless of how many
classes are in scope:

```
Agent({
  description: "Measure coverage for every candidate class",
  subagent_type: "iru-gate-runner",
  prompt: "Invoke Skill({skill: \"dotnet-coverage\", args: \"<TargetClass1,TargetClass2,...>\"}) with no test
    selector, so the whole suite runs once and every class is measured in the same pass. Report back, per target
    class: its exact line coverage percentage: and, ONLY for classes below 80%, the specific uncovered line
    numbers too. For classes at or above 80%, report just the percentage — do not include per-line detail for
    those, to keep the report compact.",
  run_in_background: false
})
```

If the candidate list from Step 2 is very large (rough guide: more than ~50 classes), split it into a few
batches of target-class arguments passed in the same `iru-dotnet-coverage` call structure above so no single
`iru-gate-runner` report becomes unwieldy — this still only requires one full-suite run per batch, not one per
class.

If a class has no matching test file (Step 2) it will simply show 0% (or be entirely absent from the coverage
XML) — treat "absent from the report" the same as "0%, no coverage," per `iru-dotnet-coverage`'s own Step 4.

## Step 4 — Classify

From Step 3's results, split every candidate class into:

- **Missing tests**: no matching test file (Step 2), regardless of reported percentage.
- **Below bar**: a matching test file exists, but line coverage is under 80%.
- **Already sufficient**: line coverage is 80% or above — skip these entirely, count them for the final report.

## Step 5 — Generate or extend tests, one class at a time

For every class in the "missing tests" or "below bar" buckets, write (or extend) its unit tests. Since each
class's test file is independent of every other's, dispatch one sub-agent per class **concurrently** (in
batches of a manageable size — a handful at a time rather than dozens at once — if the bucket is large):

```
Agent({
  description: "Write/extend tests for <ClassName>",
  prompt: "Read <source-file-path> in full — this is the class to cover. <If a test file already exists at
    <test-file-path>: read it too; these lines are currently uncovered: <line numbers from Step 3> — extend this
    file with tests that exercise them.> <If no test file exists: create one at <path implied by this project's
    naming/folder convention from Step 2>.> Match this project's existing test style exactly: same test
    framework (xUnit/NUnit/MSTest — whichever this repository already uses), same mocking library only if one is
    already used elsewhere (Moq/NSubstitute), same assertion style. Cover the class's actual behavior, including
    edge cases (null/invalid-argument handling implied by its own validation logic, boundary conditions,
    branches) — not superficial calls that merely execute lines without asserting real behavior. Do not modify
    the class under test unless it is genuinely untestable as written (e.g. a hard dependency on a static/
    non-injectable collaborator with no seam to substitute it) — if so, make the minimal change needed to add a
    seam (e.g. extracting an interface, injecting a dependency) and say so explicitly rather than reporting it as
    a plain test addition. Report back: the test file touched (new or extended), what was added, and whether the
    class under test needed a minimal testability change.",
  run_in_background: false
})
```

Collect every sub-agent's report before moving on.

## Step 6 — Re-measure and iterate on shortfalls

Re-run Step 3's delegated `iru-dotnet-coverage` pass, scoped to just the classes touched in Step 5, to confirm each
now meets 80%. For any class still below the bar:

- Identify the specific lines still uncovered (already returned by this re-measurement).
- Dispatch one more Step 5-style agent for just that class, focused on the remaining uncovered lines.
- Re-measure once more.

Cap this at two extension attempts per class. A class still below 80% after that is a genuine residual gap —
report it as such (Step 8), with the exact lines still uncovered, rather than looping indefinitely. A common
legitimate reason: defensive code paths that are difficult or unsafe to trigger from a unit test (e.g. an
`OutOfMemoryException` catch) — name these explicitly if that's what remains uncovered.

## Step 7 — Run the full suite once to confirm no regressions

Delegate to `iru-gate-runner`: `Agent({description: "Run full test suite", subagent_type: "iru-gate-runner", prompt:
"Invoke Skill({skill: \"dotnet-test\"}) (fall back to `dotnet test` directly if unavailable). If everything
passed, report back only that all tests passed. If anything failed, report back only the failing test names,
the failure reason, and the stack trace for each."})`. Fix any regression this run's new/extended tests
introduced (a genuine bug in a generated test, not a pre-existing failure), then re-invoke until it reports all
tests passed. A pre-existing failure unrelated to any class this run touched is out of scope — note it in Step
8 rather than fixing it.

## Step 8 — Report

Summarize:

- Total candidate classes found (Step 2), and how many were excluded as having no testable logic, with names.
- How many were already at or above 80% and left untouched.
- For each class in the "missing tests"/"below bar" buckets: before/after line coverage, and whether its test
  file was newly created or extended.
- Any class that required a minimal testability change to the class under test (Step 5), named explicitly.
- Any class still below 80% after Step 6's two attempts, with the exact lines still uncovered and, where
  apparent, why (e.g. an untestable defensive branch).
- The full-suite result from Step 7.

Do not run a code-quality, license-header, or doc-comment pass — those are `iru-dotnet-code-quality`, `iru-check-license`,
and `iru-dotnet-docfx`'s jobs respectively, not this skill's.
