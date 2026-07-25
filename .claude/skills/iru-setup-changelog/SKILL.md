---
name: iru-setup-changelog
description: Bootstrap a root `CHANGELOG.md` for a repository that doesn't have one yet, by exploring its actual release history (git tags and, if hosted on GitHub, GitHub Releases) and backfilling one entry per past release in Keep a Changelog format. Invoke as `/iru-setup-changelog`. Skips entirely — does nothing and reports why — if `CHANGELOG.md` already exists at the repository root; this is a one-time bootstrap, not an idempotent fill-gaps tool. Use whenever a repository has release history but no changelog file yet and the user wants one backfilled from that real history instead of started blank.
model: sonnet
---

# Setup Changelog

Create a root `CHANGELOG.md` for a repository by reconstructing its release history from git tags, GitHub
Releases, and the actual code changes between them — not by guessing or leaving placeholder entries. This skill
makes no assumptions about language or build system. It is explicitly **not** idempotent: if `CHANGELOG.md`
already exists, stop immediately rather than trying to merge or extend it (an existing changelog may already
follow its own format and cadence — see Step 1).

## Step 1 — Check whether `CHANGELOG.md` already exists

Look at the repository root for `CHANGELOG.md` (case-insensitive; also treat `CHANGELOG.rst`/`CHANGELOG.txt`/
`HISTORY.md` as "already exists" — this repository already maintains changelog-equivalent content under a
different name).

- **Found**: stop here. Tell the user a changelog already exists at that path and this skill only bootstraps one
  from scratch — it does not touch or extend an existing one. Do not read further, do not propose edits, do not
  continue to Step 2.
- **Not found**: continue to Step 2.

## Step 2 — Discover the release history

- List git tags oldest-to-newest with their creation dates: `git for-each-ref --sort=creatordate --format
  '%(refname:short) %(creatordate:short)' refs/tags`.
- **No tags at all**: there is no release history to reconstruct. Ask the user (via `AskUserQuestion` if
  available) whether to still create a minimal `CHANGELOG.md` containing just an `## [Unreleased]` section (Step
  6's format, with all commits so far treated as unreleased work), or to stop entirely since there's nothing to
  backfill. Respect either choice; don't invent version numbers.
- **Tags found**: determine whether the repository is hosted on GitHub (`git remote get-url origin` matches
  `github.com`) and the `gh` CLI is available. If so, fetch richer data per tag:
  `gh release view <tag> --json tagName,name,body,publishedAt` (a tag may have no matching GitHub Release — that's
  fine, fall back to the tag's own creator date and derive content from commits instead).
- If the repository isn't on GitHub, or `gh` isn't available/authenticated, work from tags and commits alone —
  don't fail the skill over missing GitHub access, just note in the final report that GitHub Release bodies
  weren't available as a source.

## Step 3 — For each release, determine its actual source of content

Process oldest-to-newest so later "since previous release" ranges are well-defined. For each tag `T` with
predecessor `P` (or the repository's first commit, if `T` is the oldest tag):

- **A non-empty GitHub Release body exists for `T`**: prefer it as the primary source — it's already
  human-curated. Use it to compose that release's entries (Step 4), lightly reformatted to Keep a Changelog style
  rather than copied verbatim if its structure doesn't already fit.
- **No usable Release body** (missing, empty, or GitHub Releases unavailable): derive the change set yourself.
  Read `git log P..T --oneline` and the actual diffs for commits that look meaningful (not just the commit
  subjects — terse messages like "Updated pom.xml" don't tell you whether it's a real dependency bump or a
  one-line fix) to understand real, user-facing changes: new/changed/removed public APIs or behavior, bug fixes,
  breaking changes. Confirm with `git diff --stat P T -- <source-dirs>` whether the release actually touched
  library/application source at all, or was purely docs/CI/tooling — say so plainly in that release's entry rather
  than fabricating a feature that didn't happen (a release can legitimately be "build tooling and CI updates" with
  nothing else).
- Skip changes that don't affect users of the project (repository tooling, CI config, editor/assistant
  configuration, formatting-only diffs) unless they're genuinely the entire content of that release — then say so
  rather than inventing something more exciting.
- For the **oldest** tag with no predecessor, treat everything up to it as the initial release: summarize the
  project's capabilities at that point rather than walking individual early commits one by one.
- If the repository has a long tag history (a few dozen releases or more), this step can get context-heavy.
  Dispatch one `iru-change-summarizer` agent per release (or small batch of releases) instead of reading every diff
  inline yourself: `Agent({description: "Summarize changes for <tag-range>", subagent_type: "iru-change-summarizer",
  prompt: "Summarize <predecessor>..<tag> for a Keep a Changelog entry. Source directories for this repository:
  <source-dirs>. Report back only a short bullet list of user-facing changes, grouped by
  Added/Changed/Deprecated/Removed/Fixed/Security where applicable."})`. Collect each returned bullet list here,
  keyed by tag, rather than reading every diff inline yourself.

## Step 4 — Compose each release's entries

For each tag, write one or more Keep a Changelog subsections under its version heading, using only the
subsections that actually apply:

- `### Added` — new features, APIs, capabilities.
- `### Changed` — behavior changes to existing functionality (including tooling/build changes worth surfacing).
- `### Deprecated` — soon-to-be-removed features, if any were flagged as such at the time.
- `### Removed` — removed features/APIs.
- `### Fixed` — bug fixes.
- `### Security` — security-relevant fixes.

Keep each bullet to one line, concrete about what changed rather than restating the commit message verbatim.

## Step 5 — Determine each version's date and ordering

For each tag, the date shown in its heading is, in preference order: (1) the GitHub Release's `publishedAt` date
if one was found in Step 2, (2) the tag's own creation date from Step 2's `git for-each-ref` output. Order
sections newest-first with an empty `## [Unreleased]` at the very top, above the newest release.

## Step 6 — Write `CHANGELOG.md`

```markdown
# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [<newest-version>] - <YYYY-MM-DD>

### Added

- ...

## [<previous-version>] - <YYYY-MM-DD>

### Changed

- ...

[Unreleased]: <compare-link-newest-to-HEAD>
[<newest-version>]: <compare-link-previous-to-newest>
...
[<oldest-version>]: <release-or-tag-link-for-oldest>
```

- If the remote is GitHub (`owner/repo` derived from `git remote get-url origin`), use GitHub compare links:
  `https://github.com/<owner>/<repo>/compare/<previous>...<version>` for every version with a predecessor, and
  `https://github.com/<owner>/<repo>/releases/tag/<oldest-version>` for the oldest one. The `[Unreleased]` link
  compares the newest tag to `HEAD`.
- If the remote isn't GitHub, or has no remote at all, omit the compare-link reference section entirely rather
  than guessing a URL scheme for an unknown host — note this in the final report.

## Step 7 — Report

Summarize: how many releases were reconstructed and their version range, which releases used a GitHub Release
body as their source vs. derived analysis of commits/diffs, any release noted as tooling/docs-only rather than a
feature release, and whether compare links were included or omitted (and why). Remind the user to review the
backfilled entries themselves — this is a best-effort reconstruction of history, not a guarantee of completeness
— and that going forward, keeping `CHANGELOG.md` current on each new release is a separate, ongoing concern (in
this repository, that's handled by the `iru-release` skill's Step 10; other repositories should wire up their own
equivalent).
