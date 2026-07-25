---
name: iru-release
description: Convert the current SNAPSHOT version into a final release. Must be run from the `develop` branch (gitflow) — warns and stops otherwise. Asks the user for the release version (current SNAPSHOT stripped, a patch bump, or a major bump) and the next upcoming SNAPSHOT version, creates a `release_x.y.z` branch off `develop`, updates the version in `pom.xml`, README.md, `docs/antora.yml`, and any Antora page showing dependency snippets, creates/updates `docs/modules/ROOT/pages/whats-new.adoc` and the root `CHANGELOG.md` with a summary of changes since the previous release, opens a pull request for the release branch with a `iru-release` label attached, and uses the `iru-pr-description` skill to fill in its description. Invoke as `/iru-release`. Use whenever the user wants to cut a new release of this library.
model: sonnet
---

# Release

Turn the current development SNAPSHOT into a release: pick the version numbers, cut a `release_x.y.z` branch,
update every place that names the current release/snapshot version, draft the "what's new" notes, and open a pull
request. This mirrors the repository's existing release history (`release_1.0.0` … `release_1.3.0`, each a PR
into `main`) — follow that convention unless the user says otherwise.

## Step 1 — Verify this is being run from `develop`

This is a precondition and must be checked before anything else in this skill, including reading versions or
touching any file.

- Run `git branch --show-current`.
- If it is not exactly `develop`, stop immediately: warn the user that this skill follows gitflow and must be run
  from the `develop` branch, tell them which branch they're actually on, and ask them to `git checkout develop`
  (pulling/updating it first if needed) and re-invoke `/iru-release`. Do not proceed to any later step, do not offer
  to branch from the current branch instead, and do not read or modify any file first.
- Only continue to Step 2 once the current branch is confirmed to be `develop`.

## Step 2 — Gather current state

- Read the project version from `pom.xml` (the `<version>` immediately under
  `<artifactId>hermes</artifactId>`, not a dependency/plugin version). It must end in `-SNAPSHOT` — if it doesn't,
  tell the user there's nothing to release and stop.
- Find the latest released version: `git tag --sort=-v:refname | head -1` (falls back to "no previous release" if
  there are no tags — a first release).
- Check the working tree is clean (`git status`). If there are uncommitted changes, stop and ask the user to
  commit or stash them first — a release branch should not carry unrelated in-progress work.
- Determine the PR's target/base branch: it must be `main` or `master`, whichever this repository actually has —
  never anything else. Check `git ls-remote --heads origin main master` (or `git branch -a` if offline) rather
  than assuming; older repositories tend to use `master`, newer ones `main`. If both somehow exist, prefer `main`.
  If neither exists, stop and ask the user which branch releases should target — don't guess. This is where the
  release branch's pull request will point (Step 13) — the release branch itself always originates from
  `develop`, per Step 1.

## Step 3 — Ask for the release version

Compute three candidates and present them with `AskUserQuestion` (previews help here — show the resulting
version string for each):

- **Current SNAPSHOT without the suffix** (e.g. `1.4.0-SNAPSHOT` → `1.4.0`) — recommended default; this repo's
  convention is to already bump the SNAPSHOT to the next planned minor right after the previous release, so this
  is normally the right one.
- **A new patch version** — latest tag with its patch component incremented (e.g. latest tag `1.3.0` → `1.3.1`),
  for when the changes since the last release turned out to be small enough not to warrant the pre-planned minor.
- **A new major version** — latest tag with its major incremented and minor/patch reset to `0` (e.g. `1.3.0` →
  `2.0.0`), for when the changes are breaking.

`AskUserQuestion` always offers a free-text "Other" option too, so an arbitrary version is always possible. Treat
whatever is chosen/typed as the release version for the rest of this skill; validate it looks like `X.Y.Z`.

## Step 4 — Ask for the upcoming SNAPSHOT version

Using the release version chosen in Step 3, compute three bump candidates (minor/patch/major) for the *next*
development version, and ask via `AskUserQuestion` which to use — default/recommended: minor bump, matching this
repo's established convention (e.g. release `1.4.0` → next `1.5.0-SNAPSHOT`). The chosen bump type plus
`-SNAPSHOT` is the upcoming version referenced in README/docs text for the rest of this skill.

Note for the final report (Step 15): this skill only writes the release version into `pom.xml` — it does not bump
`pom.xml` itself to the upcoming SNAPSHOT. That bump happens automatically once the release is published: the
`Sync` GitHub workflow (`.github/workflows/sync.yml`) opens a `sync_x.y.z` pull request into `develop` that merges
the released branch back in and bumps `pom.xml`/`README.md`/the Antora docs to the next snapshot. The upcoming
version gathered here is used for the human-readable "latest snapshot" mentions in README/docs, and should match
the minor-bump default that workflow computes (major.(minor+1).0-SNAPSHOT) unless there's a specific reason to
diverge.

## Step 5 — Create the release branch

- Confirm `release_<version>` doesn't already exist locally or on `origin` (`git branch --list`, `git ls-remote
  --heads origin release_<version>`) — stop and ask the user how to proceed if it does.
- `git checkout -b release_<version>` from `develop` (confirmed to be the current branch in Step 1).

## Step 6 — Update `pom.xml`

Replace the project's `<version>...-SNAPSHOT</version>` with `<version><release-version></version>` (the specific
tag right after `<artifactId>hermes</artifactId>` — don't touch dependency/plugin versions elsewhere in the file).

## Step 7 — Update `README.md`

Check whether the `iru-setup-readme` skill is available (present under `.claude/skills/iru-setup-readme`).

- **Available**: invoke `Skill({skill: "iru-setup-readme"})`. `pom.xml` already carries the release version from Step
  6, so `iru-setup-readme`'s own exploration picks it up directly; since `README.md` already exists, `iru-setup-readme`
  will show its own diff and ask for approval (its Step 9) — review that diff here rather than assuming it's
  correct. Watch specifically for the "latest release" version: at this point in the release flow the release tag
  doesn't exist yet (it's only created once this branch is merged and published), so `iru-setup-readme`'s git-tag-based
  detection will still see the *previous* release as "latest" and may not know the upcoming SNAPSHOT version at
  all (only Step 4 of this skill does). If the proposed README doesn't already show "Latest release" → the release
  version and "Latest snapshot"/"Current development version" → the upcoming SNAPSHOT version chosen in Step 4,
  correct those specific mentions by hand before accepting, using the manual approach below as reference — don't
  accept a diff that regresses those two facts.
- **Not available**: fall back to updating every version reference by hand so the README reflects the new
  release:
  - The "Add the following dependency" / Installation code blocks: "Latest release" → the release version, "Latest
    snapshot" → the upcoming SNAPSHOT version.
  - The Project Status table rows `Current development version` → the upcoming SNAPSHOT version, and
    `Latest release shown here` → the release version.

## Step 8 — Update `docs/antora.yml`

Set `version:` to the release version (matching this repo's convention of the Antora component version tracking
the latest actual release, not a SNAPSHOT).

## Step 9 — Update Antora pages with version snippets

Search the docs pages for the same kind of dependency snippet the README has:

```bash
grep -rl "<version>" docs/modules/ROOT/pages/*.adoc
```

For each match (in this repo, `getting-started.adoc`), update its "Latest release" and "Latest snapshot" `<version>`
values the same way as Step 7. Don't touch pages that don't mention a version.

## Step 10 — Determine what changed, then update `whats-new.adoc` and `CHANGELOG.md`

Both files describe the same release from the same underlying analysis — do the analysis once, then write it into
each file in its own house style. Don't let them drift: same set of changes, same version, same date.

- Gather what actually changed since the previous release: if a previous tag was found in Step 2, delegate the
  commit/diff reading to the `iru-change-summarizer` agent rather than reading `git log`/diffs inline — the same job
  `iru-setup-changelog` uses it for, just for a single range here instead of a fan-out across many past releases:
  `Agent({description: "Summarize changes since <previous-tag>", subagent_type: "iru-change-summarizer", prompt:
  "Summarize <previous-tag>..develop for a Keep a Changelog entry. Source directories for this repository: (the
  library's main source, excluding repository tooling/CI/editor-assistant config). Report back only a short
  bullet list of user-facing changes, grouped by Added/Changed/Deprecated/Removed/Fixed/Security where
  applicable."})`. Use the returned bullet list as the basis for both files below. If there is no previous tag,
  this is the first release — write a short section describing the library's initial capabilities instead of a
  diff-based changelog (no agent call needed, there's no range to summarize).
- **`docs/modules/ROOT/pages/whats-new.adoc`**: check whether it already exists. Compose a new, brief section for
  the release version being cut (heading + short bullet list, one line per change, emojis optional per this
  repo's existing README style — e.g. ✨ for new features, 🐛 for fixes, ♻️ for behavioral changes). Insert it at
  the top of the page (newest first) above any existing sections — don't rewrite or drop prior sections. If the
  page doesn't exist yet, create it following this module's conventions (see `index.adoc` for the house style: a
  `= Title` heading and, since this page's content is being drafted by this skill, the same AI-assistance
  disclaimer note `index.adoc` already carries), and add an `xref:whats-new.adoc[]` entry to
  `docs/modules/ROOT/nav.adoc`, placed right after `xref:index.adoc[]` so it's the first thing a reader sees after
  the overview.
- **`CHANGELOG.md`** (repository root): follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) — an
  `## [Unreleased]` section always sits at the top, followed by one `## [<version>] - <YYYY-MM-DD>` section per
  release, newest first, each with `### Added`/`### Changed`/`### Fixed`/`### Removed` subsections as applicable
  (omit empty subsections). Rename `## [Unreleased]` to `## [<release-version>] - <today's date>` using the same
  change list just composed for `whats-new.adoc` (same substance, terser Keep a Changelog wording — no need to
  match sentence-for-sentence), then add a fresh empty `## [Unreleased]` section above it. Update the compare
  links at the bottom of the file: retarget the `[Unreleased]` link to `compare/<release-version>...HEAD` and add
  a new `[<release-version>]: compare/<previous-tag>...<release-version>` link (or `releases/tag/<release-version>`
  if there's no previous tag). If `CHANGELOG.md` doesn't exist yet, create it with this same structure, an
  `[Unreleased]` section, and one entry for the release being cut.

## Step 11 — Review, then commit

Show the user a summary of every file changed (`git status`, `git diff --stat`) before committing. Commit with a
message matching this repo's convention: `Release <version>` (see `git log --oneline` on the previous release
branches for the exact style).

## Step 12 — Confirm before pushing and opening the PR

Pushing a branch and opening a pull request are visible, shared-state actions — confirm with the user before
proceeding, summarizing: the branch name, the release version, the upcoming SNAPSHOT version, the target base
branch, and the files changed. Only continue once they say to go ahead.

## Step 13 — Push and open the pull request

- `git push -u origin release_<version>`.
- Ensure a `iru-release` label exists (`gh label list` — create it with `gh label create release --description
  "Release pull request" --color <any hex>` if it's missing; this repo doesn't have one by default).
- Open the PR against the `main`/`master` branch determined in Step 2 — never `develop` or any other branch —
  titled `Release <version>` to match this repo's history, with a minimal placeholder body (the real description
  comes next) and the `iru-release` label attached: `gh pr create --base <main-or-master> --head release_<version>
  --title "Release <version>" --body "Release <version>." --label release`. If `gh pr create` for some reason
  doesn't accept `--label` (older `gh` versions), fall back to `gh pr edit <number> --add-label release`
  immediately after creating it — the PR must not be left without the label.

## Step 14 — Fill in the PR description

Invoke `Skill({skill: "iru-pr-description"})`. It will find the PR just opened in Step 13 (same branch), draft a
description from the actual diff, show it, and — once confirmed — update the PR body via `gh pr edit`.

## Step 15 — Report

Summarize: the release version and upcoming SNAPSHOT version chosen, the branch and PR URL, which files were
updated, and what was written to `whats-new.adoc` and `CHANGELOG.md`. Remind the user that once this PR is merged
and a GitHub release is published from it, the `Sync` workflow will automatically open a follow-up PR into
`develop` bumping the development snapshot (per the note in Step 4) — no manual sync step is needed unless that
workflow's minor-bump default doesn't match the upcoming SNAPSHOT version chosen here.
