---
name: iru-dotnet-coverage
description: Generate a code coverage report for a .NET project or solution via `dotnet test --collect:"XPlat Code Coverage"` (the Coverlet collector), scoped to the test selector given (same selector syntax as the `iru-dotnet-test` skill), then read the generated `coverage.cobertura.xml` to report the exact line-coverage percentage for the specific class(es) touched — using that same file's per-line `hits` data to show which lines/branches are still uncovered. Invoke as `/iru-dotnet-coverage <selector> [TargetClass1,TargetClass2,...]`, or `/iru-dotnet-coverage` with no argument to run the full suite and report whole-solution coverage. Use whenever the user wants to check, or verify a coverage bar (e.g. "at least 80% line coverage") for a class or set of classes, instead of eyeballing `dotnet test` output.
model: haiku
---

# .NET Coverage

Run Coverlet (via `dotnet test`'s built-in "XPlat Code Coverage" collector) against a scoped (or full) test run
and report the exact, measured coverage percentage for the class(es) the user cares about. This skill only
measures and reports coverage — it does not add tests or modify source/test code unless asked to as a follow-up.

## Step 1 — Determine the test scope

If the user gave a selector, it uses the exact same syntax as the `iru-dotnet-test` skill's Step 1 table (class,
class+method, several classes/methods, wildcard/namespace pattern, trait/category expression, project/solution
path). If no selector was given, run the whole suite.

Important: coverage is only recorded for code actually exercised by the tests that ran in this invocation. If a
class under test is covered by tests spread across multiple test classes, the selector must include all of them
— scoping to just one will undercount that class's real coverage. When in doubt about whether a selector covers
everything relevant to the target class, prefer a broader selector (e.g. the whole namespace, or the full suite)
over a narrow one.

## Step 2 — Determine the target class(es) to report on

- If the user (or the calling skill) explicitly named target class(es), use those.
- Otherwise, infer them from the test selector by convention: a test class `FooTests` targets `Foo`,
  `FooTests.TestSomething` still targets `Foo`. For several test classes, infer one target class per test class.
- If the selector is a wildcard, namespace pattern, or trait/category expression, target classes can't be
  inferred by name — report on every class touched by that run (Step 4 already surfaces all of them), or ask the
  user which class(es) they care about if the run touches many unrelated ones.

## Step 3 — Run the coverage collection

```bash
dotnet test [<path-to-.sln-or-.csproj>] --filter "<expression>" --collect:"XPlat Code Coverage" --results-directory ./TestResults
```

Omit `--filter "<expression>"` entirely to run the full suite.

Notes:
- This requires the `coverlet.collector` NuGet package on the test project(s) being run — it ships by default in
  projects created from the `mstest`/`nunit`/`xunit` `dotnet new` templates. If a test project is missing it,
  `--collect` silently produces nothing to read; check the `.csproj` for `<PackageReference Include="coverlet.
  collector" .../>` first, and if it's absent, tell the user to add it (`dotnet add package coverlet.collector`)
  rather than reporting a false 0%.
- Each run writes a new GUID-named subfolder under the results directory containing `coverage.cobertura.xml`.
  Delete the results directory first (`rm -rf ./TestResults`) so there's no risk of reading a stale report left
  over from an earlier run — this is the .NET equivalent of the `clean` goal wiping JaCoCo's previous `jacoco.exec`.
- If the build or test run fails, report that failure — don't attempt to read a coverage report that wasn't
  (re)generated.

## Step 4 — Read the exact coverage from `coverage.cobertura.xml`

Find the generated file (`find ./TestResults -name coverage.cobertura.xml`, since it's nested under a run-guid
folder). For a small, known number of target classes from Step 2 (the common case — including a plain "are we
above N%?" check), extract just their `<class>...</class>` block(s) instead of reading the whole file — this
report carries full per-line `hits` data for every class in the run, which can be large on a solution with many
classes:

```bash
awk '/<class name="<FullyQualifiedName>"/,/<\/class>/' <path-to-coverage.cobertura.xml>
```

If a target class is split across multiple `<class>` entries (partial classes), this range pattern naturally
picks up each occurrence separately — sum the underlying `<line>` counts across all of them (see below). If Step
2 couldn't narrow down to specific class names (a wildcard/namespace-pattern/full-suite run touching many
classes), read the whole file instead — composing this extraction for many classes isn't worth it once there are
more than a handful.

Its overall structure is:

```xml
<coverage line-rate="..." branch-rate="...">
  <packages>
    <package name="...">
      <classes>
        <class name="MyProject.Services.Foo" filename="..." line-rate="..." branch-rate="...">
          <methods>...</methods>
          <lines>
            <line number="12" hits="1" branch="false"/>
            <line number="13" hits="0" branch="true" condition-coverage="50% (1/2)"/>
            ...
          </lines>
        </class>
      </classes>
    </package>
  </packages>
</coverage>
```

For each target class from Step 2, find its `<class>` element (match the `name` attribute, the fully-qualified
type name, e.g. `MyProject.Services.Foo`; nested/partial types may appear as separate `<class>` entries) and
compute:

```
line coverage %   = count(<line hits!="0">) / count(<line>) * 100
branch coverage % = count(<line branch="true" hits!="0">) / count(<line branch="true">) * 100   (n/a if there are no branch lines, e.g. records with no conditional logic)
```

Prefer counting `<line>` elements directly over trusting the pre-computed `line-rate`/`branch-rate` attributes at
face value — they're accurate for a single `<class>` entry, but if a target class is split across multiple
`<class>` entries (partial classes), sum the underlying `<line>` counts across all of them rather than averaging
the attributes. Use the XML, not a generated HTML report, as the source of truth, exactly as the `iru-java-coverage`
skill reads JaCoCo's CSV rather than its HTML tables.

If a target class doesn't appear in the XML at all, it means no code in that class executed during this run
(either it's dead/unreachable from the tests that ran, or the selector didn't include tests that exercise it) —
report 0%, not an error.

## Step 5 — Report results

State, per target class, the line coverage percentage (and branch coverage if relevant to the discussion),
computed from actual XML numbers — never estimate or guess. If the user's bar is "at least N%": say clearly
whether each class meets it.

For classes under the bar, point to exactly what's uncovered directly from the same XML — no separate HTML report
is needed, unlike JaCoCo, since Cobertura's `<lines>` list already carries per-line hit data: every `<line
number="N" hits="0"/>` under that class is an uncovered line, and every `<line branch="true" hits="1"
condition-coverage="50% (1/2)"/>` is a partially-covered branch. List those exact line numbers when the user wants
to know precisely which lines/branches to target with new tests, rather than just the aggregate percentage.

Do not add tests, modify source/test code, or re-run anything to try to raise coverage — that's a follow-up the
user (or another skill) drives explicitly.
