---
name: iru-java-code-quality
description: Run this project's static-analysis Maven plugins — Checkstyle, PMD, and SpotBugs — via `mvn clean compile checkstyle:checkstyle pmd:pmd spotbugs:spotbugs`, then read the generated `target/checkstyle-result.xml`, `target/pmd.xml`, and `target/spotbugsXml.xml` reports to summarize every issue found (file, line, rule/check, message), grouped by tool. Invoke as `/iru-java-code-quality` to check the whole project, or `/iru-java-code-quality <path-or-class-name>` to scope the summary to a specific file/class — the underlying Maven run always analyzes the whole project (none of these plugins support a per-class selector), only the reported summary is filtered. Use whenever the user wants a lint/static-analysis pass called out separately from `iru-java-test`/`iru-java-coverage`, instead of running the full `mvn site` build.
model: haiku
---

# Java Code Quality

Run Checkstyle, PMD, and SpotBugs directly (not via `mvn site`) and report every issue they find. This skill only
runs static analysis and reports results — it does not fix issues unless asked to as a follow-up.

## Step 1 — Run the static analysis plugins

```bash
mvn clean compile checkstyle:checkstyle pmd:pmd spotbugs:spotbugs
```

- `clean compile` first is required: SpotBugs analyzes compiled bytecode under `target/classes`, and fails
  silently ("No files found to run spotbugs, check compile phase has been run" — build still succeeds, but no
  `spotbugsXml.xml` is produced) if that directory is missing or stale relative to the current source.
- Checkstyle and PMD only need source. Each picks up whatever ruleset the project's `pom.xml` configures for it
  (a custom `configLocation`/`rulesets` file, or the plugin's own defaults if the pom doesn't override them).
- Running the three goals directly, instead of `mvn site`, skips the slower javadoc/JXR/Surefire-report
  generation the full site build also does — this is meant to be a quick, focused quality pass.
- Do not add profile flags, they only affect package-phase source/javadoc jar attachment and has
  no effect on these goals.

If the build fails outright (e.g. a compile error), report that and stop — the reports below won't have been
freshly (re)generated, so don't read stale copies from an earlier run.

## Step 2 — Read each generated report, scoped to the request if given

Three XML reports land under `target/`, one per tool. Console output only shows progress, not the full issue
list, so these need to be read directly — but the Maven run in Step 1 always analyzed the whole project
regardless of scope (none of these plugins expose a per-file CLI selector like Surefire's `-Dtest`), so a scoped
invocation should extract just the matching entries rather than reading each full report and filtering
afterward — these reports can get large on a project with many files/issues.

**With no argument** (project-wide): read each of the three files in full.

**With a specific file/class name**: extract only the matching block(s) instead of reading the whole file:

- **Checkstyle** (`target/checkstyle-result.xml`) and **PMD** (`target/pmd.xml`) both wrap each file's issues in
  a `<file name="...">...</file>` block — extract just the matching block(s):
  `awk '/<file name="[^"]*\/<Name>\.java"/,/<\/file>/' target/checkstyle-result.xml` (same for `pmd.xml`).
- **SpotBugs** (`target/spotbugsXml.xml`) doesn't group issues by file the same way — grep for the class name
  with enough surrounding context to capture the whole `<BugInstance>` block it belongs to:
  `grep -B 8 -A 4 'classname="[^"]*\.<Name>"' target/spotbugsXml.xml` (widen the context window if a match looks
  cut off; a `<BugInstance>` rarely runs past a dozen lines). Infer the base package from any matching entry
  rather than assuming one, and also match a nested class (e.g. `Foo$Inner` when `<Name>` is `Foo`).

An empty extraction for a tool means that tool found nothing for this file/class, not that the report is
missing — only fall back to reading the whole file if the extraction itself looks wrong (e.g. no `<file>`/
`classname` entry anywhere matches the expected pattern at all, suggesting a naming mismatch rather than a clean
result).

**Checkstyle** entries look like:
```xml
<file name="/abs/path/Foo.java">
  <error line="28" column="12" severity="error" message="..." source="com.puppycrawl.tools.checkstyle.checks.sizes.LineLengthCheck"/>
</file>
```
A `<file>` with no `<error>` children is clean. The rule name is the last segment of `source` (e.g.
`LineLengthCheck`).

**PMD** entries look like:
```xml
<file name="/abs/path/Foo.java">
  <violation beginline="130" endline="130" rule="UnnecessaryModifier" ruleset="Code Style"
             package="com.example.pkg" class="Foo" method="bar" priority="3">
    message text
  </violation>
</file>
```
`priority` is PMD's own 1 (highest) to 5 (lowest) scale; the pom doesn't set `<minimumPriority>`, so all five
levels are reported.

**SpotBugs** entries look like:
```xml
<BugInstance category="BAD_PRACTICE" priority="2" type="CT_CONSTRUCTOR_THROW">
  <ShortMessage>...</ShortMessage>
  <LongMessage>...</LongMessage>
  <Class classname="com.example.pkg.Foo" primary="true">
    <SourceLine classname="com.example.pkg.Foo" start="36" end="50" sourcefile="Foo.java"/>
  </Class>
</BugInstance>
```
`priority` here is SpotBugs' own scale (1 = High, 2 = Medium, 3 = Low) — distinct from PMD's. A `BugInstance` may
list secondary classes/methods/fields in addition to the primary one; report against the primary `<Class>`'s
`<SourceLine>` for file/line.

## Step 3 — Report

Group findings by tool, then by file. For each issue, give file:line, the rule/check name, and the message.
State a total count per tool at the top and an overall total; if a tool found zero issues (in the requested
scope), say so briefly rather than omitting it silently.

Do not attempt to fix any of the issues found, or modify source/checkstyle/pmd/spotbugs configuration — that's a
follow-up the user drives explicitly.
