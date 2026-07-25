---
name: iru-dotnet-test
description: Run a .NET project or solution's test suite via the .NET CLI (`dotnet test`), optionally scoped to specific test classes, methods, namespaces, or trait/category filters, and optionally targeted at a specific project (`.csproj`) or solution (`.sln`) file. Invoke as `/iru-dotnet-test` to run the full suite (auto-detecting the solution/project in the working directory), or `/iru-dotnet-test <selector>` where `<selector>` names a class (`FooTests`), a class+method (`FooTests.TestSomething`), a wildcard/namespace pattern, a trait/category expression, or a project/solution path. Use whenever the user wants to run, re-run, or narrow down .NET unit tests instead of a full `dotnet test` across every project.
model: haiku
---

# .NET Test

Run the project or solution's tests through the .NET CLI, scoping the run to whatever the user asked for. This
skill only runs tests and reports results — it does not fix failures unless asked to as a follow-up.

## Step 1 — Determine scope from the argument

The skill may be invoked with no argument (run every test in the discovered solution/project) or with a selector
describing what to run, and optionally which project or solution to run it against.

### Locating the solution or project

If the working directory (or repository root) contains a single `.sln` file, `dotnet test` targets it by default
— no need to name it explicitly. If there are multiple `.sln` files, or multiple test projects (e.g.
`*.Tests.csproj`, `*.UnitTests.csproj`) and no `.sln`, ask which one to target unless the user's selector already
names it or it can be inferred unambiguously (e.g. only one test project contains the class named in the
selector). Pass the chosen file as the first positional argument to `dotnet test`.

### Selector mapping

| What the user wants | Parameter | Example |
|---|---|---|
| Everything | *(none)* | `dotnet test` |
| A specific project or solution | positional path | `dotnet test MySolution.sln` / `dotnet test tests/Foo.Tests/Foo.Tests.csproj` |
| One test class | `--filter` with `FullyQualifiedName~` | `dotnet test --filter "FullyQualifiedName~FooTests"` |
| One method in a class | `--filter` with `FullyQualifiedName~Class.Method` | `dotnet test --filter "FullyQualifiedName~FooTests.TestSomething"` |
| Several methods in one class | `--filter` with `\|` between `FullyQualifiedName~` clauses | `dotnet test --filter "FullyQualifiedName~FooTests.Method1\|FullyQualifiedName~FooTests.Method2"` |
| Several classes | `--filter` with `\|` between `FullyQualifiedName~` clauses | `dotnet test --filter "FullyQualifiedName~FooTests\|FullyQualifiedName~BarTests"` |
| Classes matching a name pattern | `--filter` with `~` (contains match) | `dotnet test --filter "FullyQualifiedName~ServiceTests"` |
| All classes in a namespace | `--filter` with the namespace as a `~` prefix | `dotnet test --filter "FullyQualifiedName~MyProject.Services"` |
| Excluding some classes/methods | `--filter` with `!~` | `dotnet test --filter "FullyQualifiedName!~SlowTests"` |
| Tests tagged with a category/trait | `--filter` with the trait's property name | `dotnet test --filter "Category=fast"` (NUnit), `dotnet test --filter "TestCategory=fast"` (MSTest), or `dotnet test --filter "Category=fast"` for an xUnit `[Trait("Category","fast")]` |
| Excluding a category/trait | `--filter` with `!=` | `dotnet test --filter "Category!=slow"` |
| Trait/category boolean expression | `--filter` with `&`, `\|`, `!` | `dotnet test --filter "Category=fast&Priority=1"` |

Notes:
- `~` means "contains" (substring match); `=` means exact match; `!~`/`!=` negate either form.
- `FullyQualifiedName` is `Namespace.ClassName.MethodName` — matching a namespace or class prefix with `~` selects
  every test under it.
- Which trait/category property names are recognized depends on the test framework/adapter in use: MSTest uses
  `TestCategory` and `Priority`, NUnit uses `Category`, xUnit exposes whatever key was passed to
  `[Trait("Key", "Value")]`. Check the test project's `<PackageReference>` entries (`MSTest.TestFramework`,
  `NUnit`, `xunit`) if unsure which applies.
- If the selector doesn't match any test, `dotnet test` reports "No test matches the given testcase filter" and
  exits successfully having run zero tests, rather than failing the build the way Maven Surefire does for an
  unmatched `-Dtest` selector — call this out explicitly in the report (Step 3) rather than treating it as a pass.

## Step 2 — Build and run the command

Compose the full `dotnet test` invocation from the working directory at the repository root (or the solution/
project directory if there's no root-level `.sln`):

```bash
dotnet test [<path-to-.sln-or-.csproj>] --filter "<expression>"
```

Combine a project/solution path with a filter expression when both are needed, e.g.:

```bash
dotnet test tests/Foo.Tests/Foo.Tests.csproj --filter "FullyQualifiedName~FooTests"
```

Run it with the Bash tool. Do not add unrelated flags (e.g. `--no-build`, `-v`, `--logger`) unless the user asks
for them or a prior step in this conversation already established they're needed.

## Step 3 — Report results

Summarize the `dotnet test` output concisely:
- Total tests run, passed, failed, skipped (the CLI prints a `Passed! - Failed: 0, Passed: X, Skipped: Y,
  Total: Z` or `Failed!` summary line per test project).
- If everything passed, say so briefly — don't paste the full log.
- If there are failures, show the failing test names and the relevant assertion/exception output (the console
  output usually includes it inline; check the `.trx` log under `TestResults/` if `--logger trx` was used and
  output was truncated), but don't dump unrelated passing-test noise.
- Do not attempt to fix failing tests or modify source/test code unless the user asks for that as a next step.
