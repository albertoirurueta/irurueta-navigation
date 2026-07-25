---
name: iru-setup-java-library-repository
description: End-to-end bootstrap for a brand-new Java/Maven library repository — collects the project's identity (groupId, artifactId, base package, developer name/email/organizationUrl, license) and pipeline parameters (integration branch, Java version, publishing server id, sign/extras profile ids, Maven settings file) once, then orchestrates `iru-setup-java-library` (pom.xml + source folders), `iru-setup-antora` (documentation site), `iru-setup-java-gitignore` (root `.gitignore`), `iru-setup-java-github-workflows` (CI/CD workflows), `iru-setup-changelog` (root CHANGELOG.md), and `iru-setup-readme` (root README.md) in that order so nothing is asked twice. Invoke as `/iru-setup-java-library-repository`. Each parameter has the same default as the skill it feeds (`develop`, `21`, `central`, `build-extras`, `sign`, `mvnsettings.xml`) and can be overridden. Use when starting a new Java library repository from nothing and you want the full pom/docs/gitignore/CI/changelog/README scaffold in one pass, instead of running the skills separately and re-answering the same questions each time.
model: haiku
---

# Setup Java Library Repository

Bootstrap a brand-new Java/Maven library repository in one pass by orchestrating six existing skills:
`iru-setup-java-library` (writes `pom.xml` and the `src/main/java|resources`/`src/test/java` folders),
`iru-setup-antora` (scaffolds the Antora documentation site), `iru-setup-java-gitignore` (writes or updates the root
`.gitignore`), `iru-setup-java-github-workflows` (writes `develop.yml`/`main.yml`), `iru-setup-changelog` (bootstraps a
root `CHANGELOG.md`), and `iru-setup-readme` (bootstraps a root `README.md`). This skill does not duplicate their
logic — it collects the shared parameters once and passes each skill only what it needs, so the user isn't asked
the same question repeatedly.

Each of Steps 3–8 invokes its sub-skill via the `iru-isolated-skill-executor` agent rather than calling `Skill(...)`
directly. Every sub-skill already re-derives whatever it needs straight from the filesystem this orchestrator has
just written to (as each step below notes), so this orchestrator only ever needs a short completion summary back
— nothing is lost by keeping each sub-skill's own exploration, file reads, and internal reasoning out of this
orchestrator's own context. Without this, all six sub-skills' full transcripts would accumulate in one
conversation over the course of a single bootstrap run.

## Step 1 — Collect the project's identity

Ask the user directly (plain conversation — these are free-text project-identity fields, not a bounded choice):

- **groupId** — e.g. `com.example`.
- **artifactId** — e.g. `my-library`.
- **Base package** — the library's base Java package (e.g. `com.example.mylibrary`); need not equal `groupId`.
- **Developer name**, **developer email**, **organizationUrl**.

Then resolve the **license** with `AskUserQuestion` (mirrors `iru-setup-java-library`'s own choice, resolved once
here so it isn't asked again):

- Apache License 2.0 (recommended) — `The Apache License, Version 2.0` /
  `http://www.apache.org/licenses/LICENSE-2.0.txt`
- MIT License — `The MIT License` / `https://opensource.org/license/mit`
- No license (proprietary / all rights reserved)
- Other — ask for the license's display name and URL directly afterward

Leave `version` unset here — `iru-setup-java-library` applies its own `1.0.0-SNAPSHOT` default and isn't worth asking
about twice at this level.

## Step 2 — Collect the pipeline parameters

Ask the user directly, presenting each with its default so a plain "yes"/blank reply accepts it:

| Parameter | Default           |
|---|-------------------|
| Integration branch | `develop`         |
| Java version | `21`              |
| Publishing server id | `central`         |
| Extras profile id | `build-extras`    |
| Sign profile id | `sign`            |
| Maven settings file | `mvnsettings.xml` |

These map directly onto `iru-setup-java-github-workflows`'s own six placeholders — same names, same defaults — so
there's nothing to translate before Step 4.

## Step 3 — Run `iru-setup-java-library`

Format Step 1's answers as `key: value` lines and invoke via `iru-isolated-skill-executor`:

```
Agent({
  description: "Run setup-java-library",
  subagent_type: "iru-isolated-skill-executor",
  prompt: "Invoke Skill({skill: \"setup-java-library\", args: \"groupId: <group-id>\\nartifactId: <artifact-id>\\n
    package: <base-package>\\ndeveloper-name: <developer-name>\\ndeveloper-email: <developer-email>\\n
    organization-url: <organization-url>\\nlicense: <license-name, or 'none'>\"}). Report back: whether pom.xml
    and the source folders were created fresh or already existed (and, if so, whether the user chose to stop),
    and any value it resolved on its own (e.g. version, repository info) worth noting in the final summary.",
  run_in_background: false
})
```

`iru-setup-java-library` parses these itself (its own Step 0) and only asks the user about anything genuinely left
out (e.g. `version`, or repository info it infers from git directly) — `AskUserQuestion` surfaces to the user the
same way whether invoked directly or from inside this sub-agent. If it reports that `pom.xml` already existed and
the user chose to stop, stop this skill here too — there's no coherent project identity to build Antora docs or
CI workflows around yet.

## Step 4 — Run `iru-setup-antora`

Invoke via `iru-isolated-skill-executor`: `Agent({description: "Run setup-antora", subagent_type:
"isolated-skill-executor", prompt: "Invoke Skill({skill: \"setup-antora\"}) with no args — it derives the Antora
component name, title, and version straight from the repository's pom.xml, so nothing needs to be passed through.
Report back: which files/pages were created vs. already present, and whether the site build succeeded.",
run_in_background: false})`.

Run this *before* Step 6, not after, even though the user described these two in the other order: `iru-setup-antora`
is quick to detect as "already done" and `iru-setup-java-github-workflows`'s own survey (Step 1 there) checks whether
`docs/antora.yml`/`docs/antora-playbook.yml` already exist — running Antora setup first means that check finds
everything in place instead of flagging a gap it would otherwise ask about.

## Step 5 — Run `iru-setup-java-gitignore`

Invoke via `iru-isolated-skill-executor`: `Agent({description: "Run setup-java-gitignore", subagent_type:
"isolated-skill-executor", prompt: "Invoke Skill({skill: \"setup-java-gitignore\"}) with no args — it takes none,
deriving everything it needs by exploring the repository directly. Report back only whether .gitignore was
created fresh or updated, and what categories of entries it added.", run_in_background: false})`. Run this after
Steps 3–4, not before: `iru-setup-java-gitignore` detects the build tool and any generated build-info file from the
`pom.xml` Step 3 just wrote, and detects the Antora docs build output from the `docs/antora-playbook.yml` Step 4
just scaffolded — running it earlier would miss both and produce a thinner `.gitignore` than the repository's
actual shape supports. Run it *before* Step 6 (`iru-setup-java-github-workflows`), so `target/`, `.idea/`, and the
Antora build output are already ignored before CI config and any generated reports show up locally. For a
genuinely brand-new repository, `.gitignore` won't already exist, so `iru-setup-java-gitignore`'s own approval step is
skipped and it writes directly — nothing further to confirm here.

## Step 6 — Run `iru-setup-java-github-workflows`

Format Step 2's answers, plus the `groupId`/`artifactId` already collected in Step 1, as `key: value` lines and
invoke via `iru-isolated-skill-executor`:

```
Agent({
  description: "Run setup-java-github-workflows",
  subagent_type: "iru-isolated-skill-executor",
  prompt: "Invoke Skill({skill: \"setup-java-github-workflows\", args: \"integration-branch: <integration-branch>\\n
    java-version: <java-version>\\npublishing-server-id: <publishing-server-id>\\n
    extras-profile-id: <extras-profile-id>\\nsign-profile-id: <sign-profile-id>\\n
    settings-file: <settings-file>\\ngroup-id: <group-id>\\nartifact-id: <artifact-id>\"}). Report back: whether
    develop.yml/main.yml/sync.yml were created fresh or already existed (and, if so, whether the user chose to
    stop), the required GitHub secrets it listed, and any open gap it flagged (missing Central plugin, missing
    signing profile, missing Antora setup, README/Antora wording mismatch, etc.).",
  run_in_background: false
})
```

Passing `group-id`/`artifact-id` through here lets `iru-setup-java-github-workflows` skip re-reading them from
`pom.xml` in its own Step 1, since this orchestrator already collected both in its own Step 1.
`iru-setup-java-github-workflows` parses these itself (its own Step 1) and only surveys/asks about facts these
parameters don't cover (static-analysis plugins, SonarQube config, branch-model confirmation, etc.). If it reports
that `develop.yml`/`main.yml` already existed and the user chose to stop, note that in Step 9's report rather than
treating it as a failure of this skill — `pom.xml`, the Antora docs, and the `.gitignore` from Steps 3–5 are still
valid on their own.

## Step 7 — Run `iru-setup-changelog`

Invoke via `iru-isolated-skill-executor`: `Agent({description: "Run setup-changelog", subagent_type:
"isolated-skill-executor", prompt: "Invoke Skill({skill: \"setup-changelog\"}) with no args — it takes none,
working entirely from the repository's own git tag/GitHub Release history. Report back only whether
CHANGELOG.md was created (and the version range reconstructed) or already existed.", run_in_background: false})`.
Run this before `iru-setup-readme` (Step 8): it's independent of `pom.xml`, the Antora docs, the `.gitignore`, and the
workflows, so its position relative to them doesn't matter functionally, but for a genuinely brand-new repository
(no tags yet) it's the fastest to resolve — it either bootstraps a minimal `## [Unreleased]`-only file or stops
per the user's choice, per its own Step 2. If `CHANGELOG.md` already exists, it stops immediately and reports
that — treat that the same way as the stop cases in Steps 3 and 6: not a failure of this skill, just something to
note in Step 9's report. Running it before `iru-setup-readme` means the README's own Documentation section (which
links to `CHANGELOG.md` if present) sees it already in place.

## Step 8 — Run `iru-setup-readme`

Invoke via `iru-isolated-skill-executor`: `Agent({description: "Run setup-readme", subagent_type:
"isolated-skill-executor", prompt: "Invoke Skill({skill: \"setup-readme\"}) with no args — it takes none, deriving
everything it needs by exploring the repository directly (pom.xml, the Antora docs, .gitignore, the workflows and
Sonar config, and CHANGELOG.md). Report back only which sections were included vs. omitted and why.",
run_in_background: false})`. Run this last, after every other skill: `iru-setup-readme`'s badges, documentation links,
and project-status table are only as complete as what already exists on disk when it runs, so giving it the
finished state of Steps 3–7 to explore produces a fuller README than running it earlier would. For a genuinely
brand-new repository, most of the CI/Sonar/changelog-derived sections will still be sparse or absent at this point
(no commits/tags/CI runs yet) — that's expected; `iru-setup-readme` omits what it can't confirm rather than inventing
it, and the README can be regenerated later once the repository has real history. Since `README.md` won't already
exist for a brand-new repository, `iru-setup-readme`'s own approval step (its Step 9) is skipped and it writes
directly — nothing further to confirm here.

## Step 9 — Report

Summarize the outcome of all six delegated skills together: the resolved project identity and license, the
pipeline parameters used, which of `pom.xml`/source folders, the Antora docs site, the `.gitignore`, the two
workflow files, `CHANGELOG.md`, and `README.md` were created versus left untouched (per any stop choice in Steps 3,
6, or 7), and the required GitHub secrets `iru-setup-java-github-workflows` listed. Note which README sections
`iru-setup-readme` omitted for lack of material (expected for a brand-new repository). Repeat its closing warning here
too: **review every generated file before building, committing, or relying on CI** — `mvn validate` should
succeed, the license should match an actual `LICENSE` file if one was chosen, the workflow's inferred
branch/profile/publishing values should be double checked before the first real push or release, and the README's
"How It Works" example should be verified once real source code exists.
