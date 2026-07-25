---
name: iru-setup-java-gitignore
description: Create or update the root `.gitignore` file for a Java project — compiled classes, logs, packaged
  artifacts (jar/war/ear/zip), JVM crash logs, the build tool's output directory (Maven `/target/` or Gradle
  `build/`/`.gradle/`), IntelliJ/.idea entries, and, when detected, project-specific generated files such as a
  build-info properties file written by a Groovy/Maven plugin or an Antora docs build output directory. Invoke as
  `/iru-setup-java-gitignore`. Only creates `.gitignore` when one doesn't exist yet — if one already exists, this skill
  never overwrites it silently: it proposes an updated version, shows the user a diff, and only writes it on
  explicit approval. Ships with this repository's own `.gitignore` as the concrete reference template (genericized
  so its Hermes-specific paths don't leak into other repositories), reused across repositories. Use whenever a Java
  project needs a `.gitignore` bootstrapped from scratch, or an existing one checked against this house template
  without touching it unless the user approves.
model: haiku
---

# Setup Java Gitignore

Create or refresh the root `.gitignore` for a Java project, using this repository's own `.gitignore` as the
concrete reference template. Never overwrite an existing `.gitignore` without showing the user exactly what would
change and getting explicit approval first — a `.gitignore` can carry hand-added, project-specific entries that
must not be silently dropped.

## Step 1 — Check whether `.gitignore` already exists at the repository root

- **Not found**: continue to Step 2. Because there's nothing to lose, the final write in Step 5 happens without
  needing approval — go straight from composing the content to writing it, then report in Step 6.
- **Found**: read its current content now — it matters later, both to avoid proposing duplicate entries and
  because Step 4 must preserve any hand-written, project-specific lines this skill doesn't know how to regenerate.
  Warn the user up front that a `.gitignore` already exists and this skill will propose changes for them to review
  rather than overwrite it silently. Continue to Step 2.

## Step 2 — The universal base template

Every Java project gets this baseline, adapted from this repository's own `.gitignore` (the parts of it that are
not specific to Hermes):

```gitignore
# Compiled class file
*.class

# Log file
*.log

# BlueJ files
*.ctxt

# Mobile Tools for Java (J2ME)
.mtj.tmp/

# Package Files #
*.jar
*.war
*.nar
*.ear
*.zip
*.tar.gz
*.rar

# virtual machine crash logs, see http://www.java.com/en/download/help/error_hotspot.xml
hs_err_pid*
replay_pid*
```

This section is unconditional — include it verbatim regardless of build tool or IDE.

## Step 3 — Detect project-specific additions

Don't guess; only add a section below when the thing it targets is actually present in the target repository.

- **Build tool output directory**:
  - `pom.xml` at the root → Maven project: add
    ```gitignore
    /target/
    ```
  - `build.gradle` or `build.gradle.kts` at the root → Gradle project: add
    ```gitignore
    /build/
    .gradle/
    ```
  - Both can exist in unusual multi-tool setups — include both blocks if so; if neither exists, skip this section
    and note it in Step 6 (there's no build tool to infer an output directory for yet).

- **IntelliJ IDEA (`.idea/`)**: if a `.idea/` directory exists, or there's no strong signal either way (IntelliJ is
  the default assumption for a Java project), use this repository's own convention: track most of `.idea/` (shared
  code-style and inspection settings are useful to commit) but ignore the machine-specific or noisy files:
  ```gitignore
  /.idea/codeStyles/codeStyleConfig.xml
  /.idea/.gitignore
  /.idea/compiler.xml
  /.idea/copilot.data.migration.agent.xml
  /.idea/copilot.data.migration.edit.xml
  /.idea/encodings.xml
  /.idea/jarRepositories.xml
  /.idea/misc.xml
  /.idea/vcs.xml
  ```
  If the user says they'd rather ignore all of `.idea/` outright (some teams prefer that instead of this
  repository's partial-tracking convention), use `/.idea/` as a single line instead — ask via `AskUserQuestion` if
  it's not already clear from an existing `.gitignore`'s current handling of `.idea/`.

- **VS Code**: if `.vscode/` exists, add `.vscode/` (Java projects on VS Code rarely benefit from committing
  workspace settings the way IntelliJ's code style does).

- **Eclipse**: if `.classpath`, `.project`, or `.settings/` exist, add `.classpath`, `.project`, and `.settings/`.

- **Generated build-info properties file**: grep `pom.xml` (or `build.gradle*`) for a Groovy/Maven (or equivalent
  Gradle) build step that writes a `build-info.properties`-style file, mirroring how this repository's own
  `groovy-maven-plugin` execution writes
  `src/main/resources/com/irurueta/hermes/build-info.properties` — the pattern to look for is a build script step
  that computes version/commit/branch metadata and writes it under `src/main/resources/<package-path>/`. If found,
  extract the actual output path from that script (don't hardcode Hermes's `com/irurueta/hermes` package) and add
  the matching line, e.g.:
  ```gitignore
  src/main/resources/<detected/package/path>/build-info.properties
  ```
  If no such generated-file step exists, skip this entry entirely.

- **Antora documentation build output**: if `docs/antora-playbook.yml` exists (see the `iru-setup-antora` skill), add:
  ```gitignore
  /docs/build/
  ```
  If the docs toolchain also has its own `docs/package.json` (Node-based Antora build), also add
  `/docs/node_modules/`. Skip both if there's no Antora docs setup.

- Don't invent additional entries beyond what's actually detected — if the project doesn't have a docs folder,
  Eclipse files, or a build-info step, the composed `.gitignore` should simply not mention them.

## Step 4 — Compose the proposed content

Concatenate Step 2's base template with whichever Step 3 sections actually matched, each under its own comment
header for readability (e.g. `# Maven`, `# IntelliJ IDEA`, `# Antora docs`). If Step 1 found an existing
`.gitignore`, merge rather than duplicate: keep any of its lines that aren't already covered by the composed
template (hand-added project-specific ignores must survive), and don't repeat a line that's already present
verbatim.

## Step 5 — If `.gitignore` already existed, get approval before writing

Skip this step entirely if Step 1 found no existing file — proceed straight to Step 6's write.

- Show the user the proposed new `.gitignore` content as a diff against the current file (writing the draft to a
  temp path and running `git diff --no-index <current> <draft>` gives a clean unified diff).
- Ask via `AskUserQuestion` whether to: (a) accept and write the proposed content, (b) skip and leave the existing
  `.gitignore` untouched. There is no partial-apply option here — if the user wants only some of the proposed lines,
  let them say so in free text and revise the draft before writing.
- Only proceed to Step 6 on explicit acceptance.

## Step 6 — Write `.gitignore` and report

Write the composed content (Step 4, incorporating any edits from Step 5's review) to `.gitignore` at the repository
root. Then summarize: whether the file was newly created or updated, which conditional sections from Step 3 were
included versus skipped and why (e.g. "no Eclipse section — no `.classpath`/`.project` found", "no build-info
entry — no such generated-file step in `pom.xml`"), and — if `.gitignore` already existed — whether the user
accepted or skipped the proposed changes.
