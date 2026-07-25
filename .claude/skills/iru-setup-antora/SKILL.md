---
name: iru-setup-antora
description: Set up an Antora documentation site in a repository that doesn't have one yet — installs Node.js/Antora prerequisites, scaffolds `docs/antora.yml` and `docs/antora-playbook.yml`, and builds a minimal ROOT module with `index.adoc`, `installation.adoc`, and `reference.adoc` pages wired into `nav.adoc`. Invoke as `/iru-setup-antora`. Idempotent: safe to re-run on a repository that already has some or all of this in place — it fills gaps without overwriting existing pages, config, or nav entries. Use whenever a repository needs Antora documentation bootstrapped from scratch, instead of hand-wiring `antora.yml`/`antora-playbook.yml`/the module skeleton one file at a time.
model: haiku
---

# Antora Setup

Bootstrap a minimal, working Antora documentation site for a repository: verify/install the Node.js toolchain,
install Antora plus its supported extensions under `docs/`, write `antora.yml` and `antora-playbook.yml`, create
the `ROOT` module with three starter pages, and confirm the site actually builds. This skill makes no assumptions
about the repository's language or build system — discover the project name, version, and install instructions
fresh each run. It is additive: if some or all of this already exists (as it does in this very repository), skip
what's already in place and only fill genuine gaps, without overwriting existing pages or reordering existing nav
entries.

## Step 1 — Verify Node.js is installed

Run `node -v` and `npm -v`.

- **Both print a version**: Node.js is already usable — continue to Step 2. Antora 3.x needs Node.js 20+; if the
  reported Node version is older, warn the user but let them decide whether to upgrade before continuing.
- **Either command fails**: Node.js isn't installed. Show the user the install instructions below and stop —
  don't attempt to install Node.js yourself (it changes shell/global state outside this repository). Ask them to
  install it and re-invoke `/iru-setup-antora` once `node -v` and `npm -v` both work.

  **macOS / Linux** (via `nvm`):
  ```bash
  curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.5/install.sh | bash
  \. "$HOME/.nvm/nvm.sh"
  nvm install 24
  ```

  **Windows** (via Chocolatey, run in an elevated PowerShell):
  ```powershell
  powershell -c "irm https://community.chocolatey.org/install.ps1|iex"
  choco install nodejs --version="24.18.0"
  ```

  Point the user at https://nodejs.org/en/download for other install methods (installers, other package
  managers) if they'd rather not use `nvm`/Chocolatey.

## Step 2 — Survey what already exists

Don't assume a clean slate. Before creating anything, check:

- Does a `docs/` directory already exist at the repository root? If yes, is it already an Antora module (does
  `docs/antora.yml` exist)?
- Does `docs/antora-playbook.yml` (or another playbook file referenced from `docs/package.json` scripts) exist?
- Does `docs/ui-bundle.zip` already exist next to it?
- Does `docs/modules/ROOT/` exist, and if so what pages does `docs/modules/ROOT/nav.adoc` already list?
- Is `docs/package.json` already tracking `antora` and the three extensions from Step 3 as dependencies?

Build a concrete gap list from this survey (e.g. "antora.yml missing", "playbook exists but has no mermaid
extension", "ROOT module exists with 5 pages already, no page covers installation"). Every later step acts only
on genuine gaps — a fully-set-up repository (this one, `hermes`, already has all of this) should result in Step
13 reporting "already set up," not in duplicated or overwritten files.

## Step 3 — Install Antora and its extensions

If `docs/` doesn't exist yet, create it (`mkdir docs`). From inside `docs/`:

```bash
cd docs
npm i -D -E antora
npm i @antora/lunr-extension
npm i @sntke/antora-mermaid-extension
npm i @djencks/asciidoctor-mathjax
```

- `npm i -D -E antora` pins an exact Antora version as a dev dependency (`-E` = exact, no `^`/`~` range) —
  matching how this repo pins it in `docs/package.json`.
- The three extensions add: full-text search (`@antora/lunr-extension`), Mermaid diagram rendering
  (`@sntke/antora-mermaid-extension`), and MathJax equation rendering (`@djencks/asciidoctor-mathjax`).
- If `docs/package.json` already lists a package from Step 2's survey, skip reinstalling it — `npm i` is safe to
  re-run regardless, but there's no need to if the dependency is already present at a working version.

## Step 4 — Write `docs/antora.yml`

Determine three values first:

- **`name`**: a short slug for the project — the Antora *component* name, used in `start_page` references like
  `<name>::index.adoc`. Derive it from the project identifier (Maven `artifactId`, npm package `name`, Go module
  base name, etc.), lowercased with no spaces.
- **`title`**: the human-readable project name (e.g. from a README's top heading, `pom.xml`'s `<name>`, or
  `package.json`'s `name` capitalized).
- **`version`**: the current release version, preferring, in order: (1) the latest GitHub release
  (`gh release view --json tagName -q .tagName` or `gh api repos/{owner}/{repo}/releases/latest`), (2) the latest
  git tag (`git tag --sort=-v:refname | head -1`), (3) the project manifest's version with any pre-release suffix
  like `-SNAPSHOT` stripped. If none of these resolve, ask the user for a starting version rather than guessing.

Write (or, if `antora.yml` already exists, only correct genuinely wrong fields rather than rewriting the whole
file):

```yaml
name: <project-slug>
version: <current-release-version>
title: <Human Readable Project Title>
nav:
  - modules/ROOT/nav.adoc
```

Concrete example (`docs/antora.yml`):

```yaml
name: <name-of-repository>
version: 1.3.0
title: <Name-of-repository>
nav:
  - modules/ROOT/nav.adoc
```

## Step 5 — Write `docs/antora-playbook.yml`

The `site`/`content` sections are project-specific; the `ui`/`antora.extensions`/`asciidoc` sections are fixed
boilerplate this skill applies as-is (they wire up the three extensions installed in Step 3) — don't improvise
alternatives for them. If the file already exists, only add missing pieces (e.g. an extension block) rather than
rewriting sections that already match this shape.

- `site.title`: same human-readable title as Step 4.
- `site.start_page`: `<component-name>::index.adoc`, using the `name` chosen in Step 4.
- `content.sources[0].url`: literally `..` — the playbook lives in `docs/`, one level below the repository root
  that Antora needs as its content source root.
- `content.sources[0].start_path`: literally `docs` — tells Antora the actual module content (this `antora.yml`
  and `modules/`) lives under `docs/` within that content root, not at the repository root itself.
- `ui.bundle.url`: literally `ui-bundle.zip` — a local, relative path resolved from wherever the playbook file
  itself lives. This skill ships its own `ui-bundle.zip` alongside this `SKILL.md`; copy that file into `docs/`
  (next to the `antora-playbook.yml` being written) so the relative path resolves. This avoids depending on the
  upstream GitLab CI artifact URL at build time.

```yaml
site:
  title: <Human Readable Project Title>
  start_page: <project-slug>::index.adoc
content:
  sources:
    - url: ..
      branches: HEAD
      start_path: docs
ui:
  bundle:
    url: ui-bundle.zip
    snapshot: true
  supplemental_files:
    - path: ui.yml
      contents: |
        static_files:
        - .nojekyll
    - path: .nojekyll
antora:
  extensions:
    - require: "@antora/lunr-extension"
    - require: '@sntke/antora-mermaid-extension'
      mermaid_library_url: https://cdn.jsdelivr.net/npm/mermaid@11/dist/mermaid.esm.min.mjs
      script_stem: header-scripts
      mermaid_initialize_options:
        start_on_load: true
asciidoc:
  attributes:
    mathjax-tex-tags: ams
    mathjax-tex-packages: ams
  extensions:
    - '@djencks/asciidoctor-mathjax'
```

Concrete example, from this repository (`docs/antora-playbook.yml`), differs from the template above only in the
`site` block (`title: <Name-of-repository>`, `start_page: name-of-repository::index.adoc`) — every other section is copied verbatim, including the local `ui-bundle.zip` copied into `docs/`.

## Step 6 — Create the `ROOT` module skeleton

Create, if not already present:

```
docs/
  modules/
    ROOT/
      nav.adoc
      pages/
        index.adoc
        installation.adoc
        reference.adoc
```

Only create directories/files that Step 2's survey found missing. If a `ROOT` module already exists with
differently-named pages that already cover one of these three roles (e.g. this repository's own
`getting-started.adoc` already plays the "installation" role, and `index.adoc`/`reference.adoc` already exist),
don't create a duplicate page for that role — note in Step 13's report that the role is already covered under a
different name instead.

## Step 7 — Write `index.adoc` (repository overview)

Explore the codebase before writing this page. Read the README in full — it's usually compact and central to
what this page needs. For the top-level source layout and main public entry points/classes/exports, don't read
every file directly: use the `Explore` agent (the same one the `iru-explore` skill uses for this exact kind of broad
structural search) to build a picture of module layout and the small number of top-level public types/exports a
newcomer would actually care about, and read only those specific files directly once identified. This page should
give a newcomer: what the project is, why it exists, the core concept(s) it's built around, and a pointer to the
other two starter pages. Use a Mermaid diagram (`[mermaid]` block) if a flow or architecture is genuinely clearer
as a diagram, and a MathJax equation (`\( ... \)` / `\[ ... \]`) only if the project's core logic involves
something worth stating precisely as math — most projects won't need one, don't force it.

Every generated page from this skill (`index.adoc`, `installation.adoc`, `reference.adoc`) must carry this exact
disclaimer, placed right after the page's `= Title` heading:

```
NOTE: This documentation was generated with the assistance of AI. Please report any inaccuracies.
```

This repository's own `docs/modules/ROOT/pages/index.adoc` is a concrete example of the shape to aim for: a short
overview, a Mermaid flowchart of the core algorithm, a couple of concept sections, and closing pointers to the
next pages.

## Step 8 — Write `installation.adoc` (installation guide)

Discover the actual install path for this project's ecosystem rather than assuming Maven: a Maven/Gradle
dependency snippet (group/artifact/version), an npm/pip/cargo/go-get install command, or build-from-source steps
if the project isn't published as a package. Include the disclaimer note from Step 7. If the project publishes
both a latest release and a latest snapshot/pre-release (check the manifest version's suffix), show both, as this
repository's `getting-started.adoc` does for its Maven coordinates.

## Step 9 — Write `reference.adoc` (links of interest)

Discover what generated reports and external dashboards this repository actually has before listing them — don't
list a report that isn't actually produced anywhere. Check for: a Javadoc/API-doc generation step, a coverage
tool (JaCoCo, Istanbul/nyc, coverage.py) and where its report is published, a test-report plugin (Surefire, JUnit
HTML reporter), a SonarCloud/SonarQube project, and the repository's own GitHub URL. Link only what's confirmed to
exist (check CI config, `pom.xml`/`package.json` plugins, or existing GitHub Pages settings), in a table like this
repository's `reference.adoc`:

```asciidoc
= Reference

NOTE: This documentation was generated with the assistance of AI. Please report any inaccuracies.

These reports are generated during the build.

== Generated Reports

[cols="1,3",options="header"]
|===
|Report
|Description

|<link>
|<description>
|===
```

## Step 10 — Update `nav.adoc`

Add an `xref:` entry for each page created in Steps 7–9 that doesn't already have one. Preserve the order and
content of any existing entries — append new entries in a sensible position (overview first, then the rest)
rather than reordering what's already there:

```asciidoc
* xref:index.adoc[]
* xref:installation.adoc[]
* xref:reference.adoc[]
```

## Step 11 — Ignore the build output

Check the repository's `.gitignore` for an entry covering `docs/build/`. If missing, add one (this repository's
own `.gitignore` has `/docs/build/`). Do **not** gitignore `docs/node_modules/`, `docs/package.json`, or
`docs/package-lock.json` — those are committed, matching how this repository tracks them.

## Step 12 — Build the site

From `docs/`:

```bash
cd docs
npx antora antora-playbook.yml
```

- **Build succeeds**: the static site lands at `docs/build/site`; note the path to `docs/build/site/index.html`
  for the report.
- **Build fails**: read the Antora/Asciidoctor error (broken `xref:`, bad YAML, malformed AsciiDoc table, invalid
  Mermaid/MathJax syntax) and fix the offending file before finishing — don't report success with a broken build.

## Step 13 — Report

Summarize, grouped by what Step 2 found already present vs. what this run created or filled in:

- Node.js/Antora/extensions: already present vs. newly installed.
- `antora.yml` / `antora-playbook.yml`: already present and correct, corrected, or newly written.
- `ui-bundle.zip`: already present at `docs/` vs. newly copied in from this skill's directory.
- Each of the three starter pages: newly created, or already covered by an existing page under a different name
  (name it).
- `nav.adoc` and `.gitignore`: entries added, if any.
- Build result from Step 12, with the path to the built `index.html`.

Remind the user to review the generated `index.adoc`/`installation.adoc`/`reference.adoc` content themselves
before trusting it, and that `docs/build/` should stay untracked while `docs/node_modules/`, `docs/package.json`,
and `docs/package-lock.json` should be committed.
