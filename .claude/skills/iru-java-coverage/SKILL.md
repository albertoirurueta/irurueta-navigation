---
name: iru-java-coverage
description: Generate a JaCoCo code coverage report via `mvn clean jacoco:prepare-agent test jacoco:report`, scoped to the test selector given (same selector syntax as the `iru-java-test` skill), then read the generated `target/site/jacoco/jacoco.csv` to report the exact line-coverage percentage for the specific class(es) touched — falling back to the per-class `target/site/jacoco/**/<ClassName>.html` report to show which lines/branches are still uncovered. Invoke as `/iru-java-coverage <selector> [TargetClass1,TargetClass2,...]`, or `/iru-java-coverage` with no argument to run the full suite and report whole-project coverage. Use whenever the user wants to check, or verify a coverage bar (e.g. "at least 80% line coverage") for a class or set of classes, instead of eyeballing Surefire output.
model: haiku
---

# Java Coverage

Run JaCoCo against a scoped (or full) test run and report the exact, measured coverage percentage for the
class(es) the user cares about. This skill only measures and reports coverage — it does not add tests or modify
source/test code unless asked to as a follow-up.

## Step 1 — Determine the test scope

If the user gave a selector, it uses the exact same syntax as the `iru-java-test` skill's Step 1 table (class,
class+method, comma list, wildcard, package pattern, tag expression). If no selector was given, run the whole
suite.

Important: JaCoCo only records coverage for code actually exercised by the tests that ran in this invocation. If
a class under test is covered by tests spread across multiple test classes, the selector must include all of
them (comma-separated) — scoping to just one of them will undercount that class's real coverage. When in doubt
about whether a selector covers everything relevant to the target class, prefer a broader selector (e.g. the
whole package, or the full suite) over a narrow one.

## Step 2 — Determine the target class(es) to report on

- If the user (or the calling skill) explicitly named target class(es), use those.
- Otherwise, infer them from the test selector by convention: a test class `FooTest` targets `Foo`,
  `FooTest#methodName` still targets `Foo`. For a comma list of test classes, infer one target class per test
  class.
- If the selector is a wildcard, package pattern, or tag expression, target classes can't be inferred by name —
  report on every class touched by that run (Step 4 already surfaces all of them), or ask the user which class(es)
  they care about if the run touches many unrelated ones.

## Step 3 — Run the coverage build

```bash
mvn clean jacoco:prepare-agent test jacoco:report -Dtest=<selector>
```

Omit `-Dtest=<selector>` entirely to run the full suite. Do not add profile flags —
they only affect the `package`-phase source/javadoc jar attachment, which this command never reaches, so they
have no effect on the test run or the coverage report and just add noise.

This regenerates `target/site/jacoco/` from scratch each time (the `clean` goal wipes the previous `jacoco.exec`
and report), so there's no risk of reading a stale report from an earlier run.

If the build fails (compilation error or test failure), report that failure — don't attempt to read a coverage
report that wasn't (re)generated.

## Step 4 — Read the exact coverage from `jacoco.csv`

Its header is:

```
GROUP,PACKAGE,CLASS,INSTRUCTION_MISSED,INSTRUCTION_COVERED,BRANCH_MISSED,BRANCH_COVERED,LINE_MISSED,LINE_COVERED,COMPLEXITY_MISSED,COMPLEXITY_COVERED,METHOD_MISSED,METHOD_COVERED
```

For a small, known number of target classes from Step 2 (the common case), extract just their row(s) instead of
reading the whole file — a multi-module project's CSV can have many rows:

```bash
awk -F, 'NR==1 || $3=="<ClassName1>" || $3=="<ClassName2>"' target/site/jacoco/jacoco.csv
```

Matching the `CLASS` column exactly (not a substring grep) avoids a false match against an unrelated class whose
name merely contains the target's (e.g. `FooBar` when the target is `Foo`) — nested/inner classes appear as
separate rows, e.g. `Outer$Inner`, and need their own explicit condition if relevant. If Step 2 couldn't narrow
down to specific class names (a wildcard/package-pattern/full-suite run touching many classes), read the whole
file instead — composing a long `awk` condition for many classes isn't worth it once there are more than a
handful.

For each target class, once you have its row, compute:

```
line coverage %      = LINE_COVERED / (LINE_COVERED + LINE_MISSED) * 100
branch coverage %     = BRANCH_COVERED / (BRANCH_COVERED + BRANCH_MISSED) * 100   (n/a if both are 0, e.g. enums/records with no branches)
instruction coverage % = INSTRUCTION_COVERED / (INSTRUCTION_COVERED + INSTRUCTION_MISSED) * 100
```

Use the CSV, not the HTML `index.html` tables, as the source of truth for the percentage — the HTML package-level
table's "Cov." columns are instruction and branch coverage only; it never prints a line-coverage percentage
directly, only raw missed/total line counts, so computing from the CSV is more direct and less error-prone than
parsing HTML.

If a target class doesn't appear in the CSV at all, it means no code in that class executed during this run
(either it's dead/unreachable from the tests that ran, or the selector didn't include tests that exercise it) —
report 0%, not an error.

## Step 5 — Report results

State, per target class, the line coverage percentage (and branch coverage if relevant to the discussion), computed
from actual CSV numbers — never estimate or guess. If the user's bar is "at least N%": say clearly whether each
class meets it.

For classes under the bar, point to the per-class HTML report for line-level detail on exactly what's uncovered:
`target/site/jacoco/<package-with-dots>/<ClassName>.html` (e.g.
`target/site/jacoco/com.example.myapp/Foo.html`). Read its companion `.java.html` file
(same name, e.g. `Foo.java.html`) for the actual highlighted source: each line is a
`<span class="nc" id="L<n>">` (not covered), `<span class="pc" id="L<n>">` (partially covered branch), or
`<span class="fc" id="L<n>">` (fully covered) — `grep -o 'class="nc" id="L[0-9]*"' <file>` gives the exact
uncovered line numbers when the user wants to know precisely which lines/branches to target with new tests,
rather than just the aggregate percentage.

Do not add tests, modify source/test code, or re-run anything to try to raise coverage — that's a follow-up the
user (or another skill, e.g. `iru-code`) drives explicitly.
