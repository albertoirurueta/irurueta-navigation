---
name: iru-setup-readme
description: Create or update the root `README.md` for a repository — a brief description, badges (CI status, SonarCloud/SonarQube, etc.), a project status table (language, versions, license, CI, quality tools), documentation links (Antora site, Maven site report, SonarCloud dashboard, CHANGELOG), installation instructions (Maven/Gradle/npm/etc. dependency snippets, matched to the project's actual build tool), and a short "how it works" section with a runnable example. Invoke as `/iru-setup-readme`. Explores the repository's actual state (`pom.xml`/`build.gradle`/`package.json`, git remote, GitHub Actions workflows, Sonar config, Antora docs, `CHANGELOG.md`, `LICENSE`, source code) to fill in every section — a section whose source material doesn't exist yet (e.g. a brand-new, mostly empty repository) is omitted rather than filled with placeholders or invented content. If `README.md` already exists, warns the user and shows the proposed new content as a diff before writing anything, so they can accept or skip the changes. Use whenever a repository needs a README bootstrapped or refreshed from what's actually there, instead of hand-writing it.
model: sonnet
---

# Setup Readme

Create or refresh a repository's root `README.md` by exploring what actually exists in the repository —
build files, CI workflows, quality-tool config, documentation, license, and source code — and composing only the
sections that real material supports. Never invent version numbers, badge URLs, dashboard links, or example code
that don't correspond to something actually present.

## Step 1 — Check whether `README.md` already exists

Look at the repository root for `README.md`.

- **Not found**: continue to Step 2; the final write in Step 10 happens without needing approval first (there is
  nothing to lose), though the composed content is still worth a quick summary in Step 11.
- **Found**: warn the user up front that a `README.md` already exists and this skill will propose changes for
  them to review rather than overwriting it silently. Read it now — its content matters later: Step 9 shows a
  diff against it, and any hand-written section that doesn't map onto Steps 3–8 (e.g. a "Contributing" or
  "Acknowledgements" section unrelated to this skill's scope) should be preserved in the proposed version rather
  than dropped. Continue to Step 2.

## Step 2 — Gather project identity

- **Repository name and host**: `git remote get-url origin`; if it's GitHub, extract `<owner>/<repo>` — used for
  badge URLs, GitHub Pages links, and compare links throughout. If there's no remote, or it isn't GitHub, skip
  every GitHub-specific link/badge later and note this in Step 11's report rather than guessing a URL scheme.
- **Description/tagline**: prefer, in order, a `<description>` in `pom.xml`, a `"description"` in `package.json`,
  or an existing Antora `docs/modules/ROOT/pages/index.adoc` overview paragraph. If none exist, ask the user for a
  one-sentence description rather than inventing one — this is the first thing a reader sees.
- **License**: look for `LICENSE`/`LICENSE.txt`/`LICENSE.md` at the root. Identify common licenses by their actual
  text (Apache-2.0, MIT, BSD, GPL family) rather than assuming; if the file exists but doesn't match a license you
  can confidently name, still link to it but describe it generically ("see `LICENSE`") rather than guessing a name.
  If no license file exists, omit the License section entirely — don't claim a license the repository doesn't
  declare.
- **Primary language and build tool**: check for `pom.xml` (Java/Maven), `build.gradle`/`build.gradle.kts`
  (Java or Kotlin/Gradle), `package.json` (Node), `pyproject.toml`/`setup.py` (Python), `Cargo.toml` (Rust), etc.
  A repository can have more than one (e.g. a Maven library with an Antora docs site under `docs` that has its own
  `package.json` for the doc toolchain only) — identify the build tool for the library/application itself, not
  incidental tooling.
- **Current version(s)**: for Maven, the `<version>` in `pom.xml` (immediately under the project's own
  `artifactId`, not a dependency/plugin). For Gradle, the `version` in `build.gradle`. For npm, the `"version"` in
  `package.json`. Also find the latest released version: `git tag --sort=-v:refname | head -1` (or `gh release
  list --limit 1` if on GitHub with `gh` available). If the current version has no `-SNAPSHOT`/prerelease suffix
  and matches the latest tag, there's only one version to show, not a "current dev / latest release" pair.

## Step 3 — Gather badge sources

Only include a badge when its underlying resource actually exists — don't fabricate a badge for a CI workflow or
SonarCloud project that isn't configured yet.

- **CI status**: list `.github/workflows/*.yml`. For each workflow that looks like a build/test pipeline (not,
  say, a stale/dependabot-only workflow), add a status badge:
  `https://github.com/<owner>/<repo>/actions/workflows/<file>/badge.svg`, linking to
  `https://github.com/<owner>/<repo>/actions/workflows/<file>`.
- **SonarCloud/SonarQube**: check `sonar-project.properties` or `sonar.*` properties in `pom.xml`/`build.gradle`
  for `sonar.projectKey` (and `sonar.organization` if using SonarCloud). If found and hosted on SonarCloud, add the
  standard metric badges (bugs, code smells, coverage, duplicated lines density, lines of code, maintainability
  rating, quality gate status, reliability rating, security rating, technical debt, vulnerabilities), each of the
  form `https://sonarcloud.io/api/project_badges/measure?project=<projectKey>&metric=<metric>`, linking to
  `https://sonarcloud.io/dashboard?id=<projectKey>`. If Sonar config exists but points at a self-hosted SonarQube
  instance instead, link the dashboard but skip the SonarCloud-specific badge images (they're a SonarCloud-only
  feature) and note this in Step 11.
- **Package registry badges** (npm version, Maven Central version, etc.): include only if the project is actually
  published there — e.g. don't add an npm badge for a library that's never been published to the npm registry.
- If none of the above exist yet, skip the badges section of the README entirely rather than leaving an empty
  heading.

## Step 4 — Build the project status table

Compose a small table from whatever Steps 2–3 actually resolved; omit rows with no data rather than writing "N/A":

| Row | Source                                                                                                                         |
| --- |--------------------------------------------------------------------------------------------------------------------------------|
| Language | Detected primary language + version (e.g. `Java 21`)                                                                           |
| Build tool | Maven / Gradle / npm / etc.                                                                                                    |
| Current development version | The unreleased/SNAPSHOT version from the build file, if different from the latest release                                      |
| Latest release | The latest git tag / release version                                                                                           |
| License | The license identified in Step 2, if any                                                                                       |
| CI | Which CI system and what it runs (e.g. "GitHub Actions for release and `develop` branch builds")                               |
| Quality | Which tools actually run — only list ones with real config found (SonarCloud, JaCoCo, Checkstyle, SpotBugs, PMD, ESLint, etc.) |

## Step 5 — Gather documentation links

Include a link only when the thing it points to actually exists in the repository (or is confirmed published):

- **Antora documentation site**: if `docs/antora.yml` and `docs/antora-playbook.yml` exist, link to the published
  site. If a GitHub Pages workflow step publishes it (grep workflows for `gh-pages`/`ghaction-github-pages`), the
  URL is normally `https://<owner>.github.io/<repo>`; if you can't confirm it's actually published, still include
  the link but note in Step 11 that it's inferred from convention, not confirmed live.
- **Maven/Gradle site report**: if the CI workflow merges a Maven `site` (or Gradle equivalent) report into the
  published docs (e.g. a `mvn-site` subpath alongside the Antora output, matching this repository's own
  `develop`/`main` workflow pattern), link to it at that subpath.
- **SonarCloud/SonarQube dashboard**: reuse the link built in Step 3.
- **Changelog**: if `CHANGELOG.md` exists at the root, link to it directly (`CHANGELOG.md`).
- Omit any of the above whose source doesn't exist — don't add a "Documentation" section at all if none of these
  resolve.

## Step 6 — Compose installation instructions

Match the snippet to the build tool detected in Step 2 — don't show a Maven snippet for a Gradle project or vice
versa:

- **Maven**: a `<dependency>` block with the real `groupId`/`artifactId`, using the latest release version; if the
  current build-file version is a distinct `-SNAPSHOT`, show both under "Latest release" / "Latest snapshot"
  (mirroring how a snapshot repository would need to be added — only mention that if the project's own POM/
  settings actually reference one).
- **Gradle**: the equivalent `implementation '<groupId>:<artifactId>:<version>'` (or Kotlin DSL form if
  `build.gradle.kts`).
- **npm/yarn**: `npm install <package-name>` (or `yarn add`), using the real `package.json` name.
- **Python**: `pip install <package-name>`, using the real project name from `pyproject.toml`/`setup.py`.
- If the build tool isn't one of the above, or the project isn't actually published/publishable yet (no
  registry/repository config found), omit the Installation section and note the gap in Step 11 rather than
  guessing coordinates.

## Step 7 — Write the "how it works" section with an example

This is the most judgment-heavy part — ground it in what the code actually does, don't invent behavior:

- Identify the main source entry points (the primary public classes/interfaces/functions a consumer would use —
  for a library, usually a small number of top-level types; for an application, its README-worthy commands/
  endpoints) without reading the whole source tree directly: use the `Explore` agent to locate them first, then
  read only those specific files.
- If Antora docs already exist (`docs/modules/ROOT/pages/index.adoc`, `getting-started.adoc`, or similar), prefer
  reusing/adapting their conceptual explanation and example code rather than writing a divergent one from scratch
  — the README and the docs site describing the same library differently is a maintenance trap.
  If a docs example exists, verify it still matches the current API shape before reusing it — the docs page might
  itself be stale.
- Write a short paragraph (a few sentences) on the core concept, plus one minimal, realistic code example that
  would actually compile/run against the current API — check public method/constructor signatures rather than
  guessing them.
- If the repository has no meaningful source yet (a brand-new, scaffolded-but-empty project), omit this section
  entirely rather than writing a placeholder example.

## Step 8 — Compose the remaining structural sections

Include each only when Steps 2–7 gave it real content:

- **Title + tagline**: repository name as an `# H1`, one-sentence description from Step 2 underneath.
- **Badges**: from Step 3, directly under the title.
- **Project Status**: table from Step 4.
- **Documentation**: links from Step 5.
- **Installation**: snippet(s) from Step 6.
- **How It Works** / **Quick Example**: prose + code from Step 7.
- **License**: one line naming the license from Step 2, linking to the `LICENSE` file.

If the existing `README.md` read in Step 1 had additional sections this skill doesn't generate (e.g.
"Contributing", "Acknowledgements", a project-specific comparison table like "Choosing a Detector"), keep them
verbatim in the proposed version, in their original relative position, rather than silently dropping
hand-written content this skill doesn't know how to regenerate.

## Step 9 — If `README.md` already existed, get approval before writing

Skip this step entirely if Step 1 found no existing file — proceed straight to Step 10.

- Show the user the full proposed new `README.md` content (or a diff against the current file — writing the
  draft to a temp path and running `git diff --no-index <current> <draft>` gives a clean unified diff) so they
  can see exactly what would change section by section.
- Ask via `AskUserQuestion` whether to: (a) accept and write the proposed content, (b) skip and leave the
  existing `README.md` untouched. There is no partial-apply option here — if the user wants to keep some sections
  and change others, let them say so in free text and revise the draft before writing.
- Only proceed to Step 10 on explicit acceptance.

## Step 10 — Write `README.md`

Write the composed content (Step 8, incorporating any edits from Step 9's review) to `README.md` at the
repository root.

## Step 11 — Report

Summarize: which sections were included versus omitted and why (e.g. "no Installation section — no build/package
config found", "no CI badges — no GitHub Actions workflows present"), any fact that had to be inferred rather than
confirmed (e.g. an Antora Pages URL assumed from convention), and — if `README.md` already existed — whether the
user accepted or skipped the proposed changes. Remind the user to double-check the "How It Works"/example section
specifically, since it's the part most dependent on this skill's reading of the current API rather than on
mechanically-derived facts like versions or badge URLs.
