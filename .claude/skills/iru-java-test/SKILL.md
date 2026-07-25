---
name: iru-java-test
description: Run a Java project's JUnit test suite via the Maven Surefire plugin (`mvn test`), optionally scoped to specific test classes, methods, packages, or tags. Invoke as `/iru-java-test` to run the full suite, or `/iru-java-test <selector>` where `<selector>` names a class (`FooTest`), a class+method (`FooTest#testSomething`), a wildcard/package pattern, or a tag expression. Use whenever the user wants to run, re-run, or narrow down unit tests instead of a full `mvn clean test`.
model: haiku
---

# Java Test

Run the project's JUnit tests through Maven Surefire, scoping the run to whatever the user asked for. This
skill only runs tests and reports results — it does not fix failures unless asked to as a follow-up.

## Step 1 — Determine scope from the argument

The skill may be invoked with no argument (run everything) or with a selector describing what to run. Map the
selector to the correct Surefire parameter:

| What the user wants | Parameter | Example |
|---|---|---|
| Everything | *(none)* | `mvn test` |
| One test class | `-Dtest` | `mvn test -Dtest=FooTest` |
| One method in a class | `-Dtest` with `#` | `mvn test -Dtest=FooTest#testSomething` |
| Several methods in one class | `-Dtest` with `+` between method names after `#` | `mvn test -Dtest=FooTest#method1+method2` |
| Several classes | `-Dtest` comma-separated | `mvn test -Dtest=FooTest,BarTest` |
| Classes matching a name pattern | `-Dtest` with `*` wildcard | `mvn test -Dtest=*ServiceTest` |
| All classes in a package | `-Dtest` with the package path and wildcard | `mvn test -Dtest=com.example.pkg.**` |
| Excluding some classes/methods | `-Dtest` with `!` prefix (combine with other `-Dtest` entries via comma) | `mvn test -Dtest=!SlowTest` |
| Tests tagged with a JUnit 5 `@Tag` | `-Dgroups` | `mvn test -Dgroups="fast"` |
| Excluding a JUnit 5 tag | `-DexcludedGroups` | `mvn test -DexcludedGroups="slow"` |
| Tag boolean expression | `-Dgroups` with `&`, `|`, `!` | `mvn test -Dgroups="fast & !flaky"` |

Notes:
- The `-Dtest` value accepts the class simple name without the `.java`/`.class` extension.
- `#method` selection and `-Dgroups`/`-DexcludedGroups` tag filtering require JUnit 4.x, JUnit 5, or TestNG (not
  plain JUnit 3). Check the project's `pom.xml` for its test framework/dependency versions if unsure which
  syntax applies; JUnit 5 `@Tag` values map directly to `-Dgroups`/`-DexcludedGroups`, TestNG groups likewise.
- If the selector the user gave doesn't match any known test (typo in class/method name), Surefire fails the
  build with "No tests matching pattern...". Add `-DfailIfNoSpecifiedTests=false` only if the user explicitly
  wants a missing-selector to be a no-op rather than a failure — don't add it by default, since silently running
  zero tests is usually not what's wanted.

## Step 2 — Build and run the command

Compose the full Maven invocation from the working directory at the repository root:

```bash
mvn test -Dtest=<selector>
```

Combine multiple parameters with spaces if both a class/method selector and a tag filter are needed, e.g.:

```bash
mvn test -Dtest=FooTest -Dgroups="fast"
```

Run it with the Bash tool. Do not add unrelated flags (e.g. `-o`, `-q`, skip-checks) unless the user asks for
them or a prior step in this conversation already established they're needed.

## Step 3 — Report results

Summarize the Surefire output concisely:
- Total tests run, failures, errors, skipped (Surefire prints a `Tests run: X, Failures: Y, Errors: Z, Skipped: W`
  line per class and a totals line at the end).
- If everything passed, say so briefly — don't paste the full log.
- If there are failures/errors, show the failing test names and the relevant assertion/exception output (from
  `target/surefire-reports/*.txt` if the console output was truncated), but don't dump unrelated passing-test
  noise.
- Do not attempt to fix failing tests or modify source/test code unless the user asks for that as a next step.
