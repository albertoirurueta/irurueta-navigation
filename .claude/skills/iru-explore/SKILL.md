---
name: iru-explore
description: Explore this repository codebase, optionally grounded in a specific GitHub issue or Jira ticket used as the task to investigate, and detect the programming language(s) and framework(s) in play — both in the existing codebase and in the requested change — plus whether Antora documentation exists and where, which platform hosts the repository (GitHub, Bitbucket, Azure DevOps, or TFS), and whether a `.archive/` directory holds implementation plans from previously resolved tasks that resemble the current one. Works even when the current working directory isn't a git repository at all — it warns the user that git-remote-dependent tooling (e.g. `gh` inferring an owner/repo, `git log` history) won't work, then keeps going with whatever sources don't need one: a Jira ticket, a connected documentation MCP, and any files actually present in the working directory. Invoke as `/iru-explore <ticket-id>` where `<ticket-id>` is either a GitHub issue ID (e.g. `42`) or a Jira key (e.g. `PROJ-123`) — the skill auto-detects which. If invoked as `/iru-explore` with no argument, ask the user for a ticket ID; if they decline or give none, just explore the current codebase. Looks up GitHub issues with the `gh` CLI (or GitHub MCP tools if `gh` is unavailable) and Jira tickets via any connected Jira MCP tools — for a Jira ticket, also fetches its epic and any linked/related tickets (one hop out), and downloads and reads any `implementation_plan_*.md` attached to the ticket, its epic, or its linked tickets (the naming convention `iru-code`/`iru-issue` archive and attach a completed plan under), for additional context — and, if a documentation MCP (e.g. Confluence, Notion) is connected, searches it for pages relevant to the ticket or codebase for extra context. Use for onboarding, understanding "what would it take to fix issue #N"/"ticket PROJ-123", or getting oriented before planning work. Its "Tech stack" and "Related past implementation plans" findings (languages/frameworks, Antora docs location, repository host, relevant archived plans) are meant to be reused by later skills in the same conversation (e.g. `iru-plan`, `iru-code`, `iru-update-docs`, code review, branch/PR creation) to pick language/framework-appropriate flows, best practices, the right git-platform tooling, and proven prior plan structure.
model: opus
---

# Explore

Produce a grounded understanding of this repository codebase and, when available, of a specific GitHub issue or
Jira ticket — without writing a plan or making code changes. This skill is read-only research: its output is a
report, not a diff.

## Step 1 — Resolve the ticket ID and its type

The skill may be invoked with a ticket ID as its argument (e.g. `/iru-explore 42` or `/iru-explore PROJ-123`).

- **Argument provided**: detect which system it belongs to:
  - Matches a Jira key pattern — one or more uppercase letters/digits starting with a letter, a hyphen, then
    digits (e.g. `PROJ-123`, `ABC2-45`) → **Jira**.
  - A bare number, optionally prefixed with `#` (e.g. `42`, `#42`) → **GitHub issue**.
  - If it matches neither pattern clearly, ask the user (via `AskUserQuestion`) whether it's a GitHub issue ID or
    a Jira key.
- **No argument provided**: ask the user (via `AskUserQuestion`) for a ticket ID to focus the exploration on,
  making clear they can skip this and just have the codebase explored instead. Offer something like:
  - "Provide a GitHub issue ID or Jira key" (free text via "Other")
  - "No ticket — just explore the codebase" (recommended default if they seem in a hurry)
  - If they decline or give none, proceed directly to Step 4 (codebase-only exploration).

## Step 2 — Detect the repository host and available tooling

Determine which platform hosts this repository, so later skills that operate on it (branch creation, pull/merge
requests, commit history, CI status, etc.) know which CLI/API to use instead of assuming GitHub by default.

**First, confirm the current working directory is actually inside a git repository at all**
(`git rev-parse --is-inside-work-tree`, or check for a `.git` directory/file up the tree) — this skill must still
be useful when it isn't, e.g. run from a plain folder of docs, a fresh checkout-in-progress, or a directory that
was never `git init`'d:

- **Not a git repository**: tell the user plainly, once, that no repository was detected in the current working
  directory — record `Repository host: none — not a git repository` for Step 7 and skip the rest of this step
  (there is no remote to inspect). This also means any git-dependent tooling later in this skill (`gh` inferring
  an owner/repo from the local remote, `git log` for recent history in Step 4) will not work; note that
  explicitly too, as a warning, not a hard stop — continue exploring with whatever sources don't need a
  repository: a Jira ticket (Step 3) fetched via Jira MCP tools, a documentation MCP (Step 6), and any files
  actually present in the working directory (Step 4 — Antora docs, `CLAUDE.md`/`README`, source files, all work
  the same whether or not the directory happens to be a git repo).
- **Is a git repository**: continue below as normal.
- Get the remote URL: `git remote get-url origin` (fall back to `git remote -v`, or to whichever remote the
  current branch tracks, if `origin` isn't the remote in use).
- Match the URL against known host patterns:
  - `github.com` (or a GitHub Enterprise Server domain — look for a `/api/v3/`-style setup, or ask if uncertain)
    → **GitHub**. Tooling: `gh` CLI, or GitHub MCP tools (`ToolSearch` "github").
  - `bitbucket.org` (cloud), or a self-hosted domain with a `/scm/<project>/<repo>.git` path (the Bitbucket
    Server/Data Center convention) → **Bitbucket**. Tooling: there's no ubiquitous CLI equivalent to `gh` here —
    search for Bitbucket MCP tools first (`ToolSearch` "bitbucket"), otherwise fall back to the Bitbucket REST
    API.
  - `dev.azure.com` or `<org>.visualstudio.com` → **Azure DevOps (cloud)**. Tooling: `az repos`/`az boards` (the
    Azure CLI with the `azure-devops` extension) if installed, or Azure DevOps MCP tools (`ToolSearch`
    "azure devops").
  - A self-hosted domain with a `/tfs/` path segment, or `_git` in the path without a `dev.azure.com`/
    `visualstudio.com` domain → **Azure DevOps Server / Team Foundation Server (TFS)**. Same tooling family as
    Azure DevOps (`az repos` can often target a collection URL directly), but confirm the collection URL and API
    version before assuming parity — older on-prem TFS versions expose a narrower REST API surface than cloud
    Azure DevOps.
  - If the URL doesn't clearly match any of these (e.g. a mirrored or internal git host), ask the user (via
    `AskUserQuestion`) which platform it is rather than guessing — a wrong assumption here would misdirect every
    later git-platform operation.
- If there is no remote configured at all (a purely local repository), note that explicitly — later skills that
  need a hosted platform (PR/merge-request creation, etc.) won't have one to target.
- Record the detected host — and, for Azure DevOps/TFS or self-hosted Bitbucket, the base URL/collection — for
  the report in Step 6. This detection is what later skills should key off instead of re-deriving it or
  defaulting to GitHub-specific tooling (`gh`, "Closes #N", etc.) unconditionally.

This is independent of Step 1: it detects where the **code repository** is hosted, not where the **ticket**
lives — a Jira ticket can reference work in a GitHub, Bitbucket, or Azure DevOps repo, and vice versa.

## Step 3 — Fetch the ticket

### GitHub issue

Determine the repository (owner/name) from the git remote, e.g.:

```bash
gh repo view --json owner,name -q '.owner.login + "/" + .name'
```

- **If Step 2 found no git repository at all** (or a repository with no remote configured), `gh repo view` has
  nothing to infer owner/repo from and will fail — this is expected, not a bug. Ask the user for the target
  repository explicitly (`owner/repo`), then fetch with `--repo` supplied directly:
  ```bash
  gh issue view <issue-id> --repo <owner/repo> --json number,title,body,labels,comments,state,url
  ```
  If they don't know it or decline, warn them that the GitHub issue can't be fetched without a repository
  context, then continue with a codebase-only or Jira-only exploration (Step 4 onward) rather than stopping the
  whole skill — a missing repository should degrade this one lookup, not the rest of the exploration.

Otherwise, fetch the issue with the `gh` CLI directly:

```bash
gh issue view <issue-id> --json number,title,body,labels,comments,state,url
```

If `gh` is not installed/authenticated, search for GitHub MCP tools instead (`ToolSearch` with a query like
"github issue") and use those to fetch the same information — these may still work without a local repository if
the tool takes an explicit owner/repo parameter rather than inferring it from `git remote`. If neither is
available, tell the user and continue with a codebase-only exploration (Step 4), noting the issue could not be
retrieved.

### Jira ticket

Search for connected Jira MCP tools (`ToolSearch` with a query like "jira issue" or "jira ticket"). If one or
more are found, use them to fetch the ticket by key: summary/title, description, issue type, status, labels/
components, comments, URL, and its issue links (`issuelinks`, plus the epic/parent relationship).

If no Jira MCP tool is available, tell the user plainly that this environment has no Jira connection, then ask
(via `AskUserQuestion`) whether to:
- paste the relevant ticket content (title/description/comments) directly so exploration can proceed grounded
  in it, or
- continue with a codebase-only exploration (Step 4), noting the ticket could not be retrieved.

Don't guess at Jira REST endpoints or attempt to `WebFetch` a Jira URL directly — Jira instances are normally
authenticated, and an unauthenticated fetch will just fail or return a login page.

**Also fetch the epic and any linked tickets, one hop out**, since a single ticket's own fields often omit
context that lives on its epic or on tickets it references — a plan grounded only in the leaf ticket can miss a
constraint the epic states once for all its children, or misunderstand what "blocks"/"relates to" actually means
without reading the other side of that link:

- **Epic**: check whether the ticket belongs to an epic — the classic "Epic Link" field, or the `parent` field
  in team-managed/next-gen projects. If it does, fetch that epic ticket too (summary, description, status, and
  its own labels/components) — an epic frequently carries the overall goal, cross-cutting acceptance criteria, or
  links to design docs that no individual child ticket repeats.
- **Linked/related tickets**: for each entry in the ticket's `issuelinks` (e.g. "relates to", "blocks", "is
  blocked by", "duplicates", "clones") and any subtasks, fetch that linked ticket's summary, description, and
  status — enough to know concretely what the link means for this task, not just that it exists. Skip fetching
  its own comments unless the summary/description alone leaves the relationship unclear.
- Keep this to one hop from the primary ticket — don't recursively follow the epic's or a linked ticket's own
  links.
- If fetching the epic or a linked ticket fails (no permission, deleted, not found), note that briefly and
  continue with what was retrieved rather than blocking the whole exploration on it.

**Also check for attached implementation plans**, on the ticket itself and on its epic/linked tickets fetched
above: list each ticket's attachments (via the same Jira MCP tool set — `ToolSearch` "jira attachment" if the
issue-fetch tool doesn't already return an attachment list) and look for any file named `implementation_plan_*.md`.
This is exactly the naming convention `iru-code`'s own archiving step uses, and `iru-issue` attaches that archived
file back onto the ticket it resolved (per its own Step 7) — so a match here is a previously-completed
implementation plan for work the ticket-filer or a prior agent run judged similar or related, travelling with the
ticket itself rather than needing a local `.archive/` in this checkout.

- Download and read each matched attachment in full — it's the same kind of precedent this skill's `.archive/`
  check in Step 4 looks for locally, just sourced from the ticket instead of the current repository.
- If more than one is attached (e.g. an epic with several resolved child tickets, each having attached its own
  plan), read all of them — they're typically small and each is a distinct, useful precedent.
- If the connected Jira MCP tool set has no way to list or download attachment content, note that explicitly and
  continue without it — this is a best-effort enrichment, not a blocker for the rest of exploration.
- If nothing matches `implementation_plan_*.md` on the ticket or its epic/linked tickets, note that briefly too
  rather than silently omitting the check.

### Either way

Read the ticket's title, body/description, labels, and any comments that clarify scope or add constraints —
plus, for a Jira ticket, the epic's and any linked tickets' summaries/descriptions, and any downloaded
`implementation_plan_*.md` attachments, fetched above. Note explicitly: what behavior is requested/reported, any
acceptance criteria (including any stated only on the epic), and any files, classes, error messages, languages,
frameworks, or platforms mentioned — across the primary ticket, whatever epic/linked tickets were fetched, and
any attached implementation plans read.

## Step 4 — Explore the codebase

This step must work generically on any repository — and equally on a working directory that per Step 2 isn't a
git repository at all — don't assume a language, framework, or prior familiarity with the project. Start from
first principles each time; every check below is file-based and works the same whether or not `.git` exists.

- Get the lay of the land first: check for a `CLAUDE.md`, `README`, or `AGENTS.md` (use as a starting index,
  never as a substitute for reading the actual files, and treat it skeptically if it looks stale), list the
  top-level directories, and identify the language(s), build/package tooling, and entry points (main modules,
  services, public API surface).
- Check whether Antora documentation exists and where: look for an `antora.yml` (commonly under `docs/`, but
  confirm the actual path rather than assuming) and, alongside it, an `antora-playbook.yml` and a
  `modules/ROOT/pages` (or other module) tree. Record the docs root path if found — later skills (e.g.
  `iru-update-docs`) need this location, and its content is itself a language/framework signal (see Step 5).
- Check for a `.archive/` directory at the repository root and give it special attention: the `iru-code` skill
  archives every completed `implementation_plan.md` there (typically as
  `implementation_plan_<ticket-or-timestamp-id>.md`) once its task groups are done, so it's a library of
  previously resolved tasks. If it exists, list its contents (`ls .archive`) and skim each archived plan's
  title/summary for one whose topic, named files/classes, or task area overlaps with the current ticket (or,
  with no ticket, the general topic of this exploration) — matching on keywords rather than reading every file
  in full. Read the most relevant one or two in full: a matching archived plan shows how a similar task was
  previously scoped, broken into dependency-aware task groups, and what its "Current code state" notes
  captured — treat it as a concrete example to ground the current task's own plan against, not as a template
  to copy verbatim, since the current change is never identical. If `.archive/` doesn't exist, or nothing in it
  is relevant, note that explicitly rather than silently skipping the check.
- Use the `Explore` agent (or direct `Read`/`grep` for small, targeted lookups) to build a picture of the
  most significant parts of the code structure: how the project is organized into modules/packages, the core
  abstractions and how they relate (inheritance, composition, key interfaces), where the main control flow or
  request/data path runs, and where tests live and how they map to source.

**If a ticket was fetched**, narrow the exploration to only the areas it implicates — do not do a full general
tour. Concretely:
  - Search for the classes/functions/files/error messages/behaviors named or described in the ticket.
  - Read those files plus their closest structural neighbors (the base class/interface they implement, direct
    callers/callees) — only as much surrounding context as needed to understand how that area currently
    behaves.
  - Check existing tests covering that area to learn current expected behavior and edge cases already handled.
  - Skip unrelated parts of the codebase, even if they'd normally be worth a general orientation pass.

**If there is no ticket** (declined or unavailable), do a general orientation pass instead: map out the overall
architecture — the major components/modules, how they depend on each other, key abstractions and design
patterns in use, and, if Step 2 found this is actually a git repository, anything notable in recent git history
(`git log --oneline -20`) that suggests active areas of work. Skip the git-history check entirely (not a
failure, just nothing to check) if Step 2 found no git repository.

## Step 5 — Detect the technology stack

Identify the programming language(s) and framework(s) in play, both for the existing codebase and for whatever
change is being requested. This is not a vague "looks like Java" impression — ground it in concrete evidence, and
call out enough detail (per-module, if the repo has more than one stack) that later skills can act on it directly.

### Codebase languages and frameworks

Use whichever of these signals are present — manifests/config are the most reliable, source-level signals are the
fallback when manifests are absent or ambiguous:

- **Build/package manifests** are the primary signal:
  - `pom.xml`, `build.gradle`/`build.gradle.kts` → Java/Kotlin; check dependencies for `spring-boot-starter*`
    (Spring/Spring Boot), `micronaut-*`, `quarkus-*`, or an Android Gradle plugin (`com.android.application`/
    `com.android.library`) — and within Android, `androidx.compose` deps → Jetpack Compose vs. XML/View-based UI.
  - `*.csproj`/`*.fsproj`/`*.sln` → C#/.NET (or F#); check for `Microsoft.AspNetCore.*` (ASP.NET Core), `*.Maui`/
    Xamarin (cross-platform mobile), WPF/WinForms project types.
  - `package.json` → JavaScript/TypeScript; check `dependencies`/`devDependencies` for `react`, `vue`, `angular`,
    `next`, `express`, `nestjs`, etc.
  - `requirements.txt`, `pyproject.toml`, `Pipfile`, `setup.py` → Python; check for `django`, `flask`, `fastapi`.
  - `Podfile`, `Package.swift`, `*.xcodeproj`/`*.xcworkspace` → Swift and/or Objective-C; check `import SwiftUI`
    vs. `import UIKit` in source files for the UI framework in use, and `.m`/`.mm`/`.h` file presence for
    Objective-C alongside `.swift`.
  - `CMakeLists.txt`, `Makefile`, `*.vcxproj` with `.c`/`.cpp`/`.h`/`.hpp` sources → C/C++.
  - `go.mod` → Go. `Cargo.toml` → Rust. `Gemfile` → Ruby.
- **Source-level fallback**: when manifests are absent, sparse, or ambiguous (e.g. a monorepo with multiple
  stacks, or a manifest that doesn't disambiguate framework choice), sample actual source files — file
  extensions, `import`/`using`/`#include` statements, and characteristic APIs — to confirm or fill the gap.
- **Antora documentation, if present** (per Step 4): read `antora.yml` (component `name`/`title`) and the docs
  pages themselves — an `installation.adoc`/`reference.adoc`/`getting-started.adoc` will typically spell out the
  language and build tool explicitly (e.g. a Maven/Gradle/npm/NuGet dependency snippet, a `pod` entry, an
  `import` example). Treat this as a corroborating signal, not a substitute for the manifest/source checks
  above — but when manifests are ambiguous or a module lacks one of its own, the docs are often the clearest
  statement of intent, and are worth quoting directly rather than re-deriving from source alone. Note any
  mismatch between what the docs claim and what the manifests/source actually show (e.g. stale docs describing a
  since-migrated framework) instead of silently preferring one over the other.
- **Multiple stacks**: if the repository contains more than one language/framework (e.g. a backend service plus
  a mobile client, or a monorepo of independent packages), record the stack per module/directory rather than
  flattening it into one list.

### Languages/frameworks implicated by the requested change

If a ticket was fetched, read its title/description/comments for explicit or implicit tech signals — named
languages, frameworks, or platforms (e.g. "iOS", "Android", "the web dashboard", "the API"), specific file/class
names, or code snippets whose syntax identifies a language. Then cross-reference against the codebase stack just
detected:

- If the ticket's signals match a part of the existing stack, note which module/language/framework the change
  targets.
- If the ticket names something **not** currently present in the codebase (e.g. a request to add a new mobile
  app to a backend-only repo, or to introduce a new framework), flag this explicitly as a new
  language/framework the work would introduce — don't silently assume it matches the existing stack.
- If nothing in the ticket indicates a specific target and the repo has a single stack, note that the change is
  presumed to apply to that stack. If the repo has multiple stacks and the ticket is genuinely ambiguous about
  which one it targets, say so rather than guessing.

If there is no ticket, this sub-step is just the codebase stack detected above — there is no requested change to
cross-reference yet.

## Step 6 — Check for a documentation MCP

Before reporting, check whether any documentation/knowledge-base MCP tools are connected in this session (e.g.
Confluence, Notion, or similar — search with `ToolSearch` using queries like "confluence", "notion",
"documentation", "wiki"). These are optional, environment-specific connectors, not something every repository or
user has configured.

- **If one or more are found**, use them to search for pages relevant to:
  - the ticket's title/keywords and any named components (if a ticket was fetched in Step 3), or
  - the key components/domain areas identified in Step 4 (if there is no ticket).

  Keep the search targeted — a handful of keyword queries against the ticket/codebase context, not a broad
  crawl of the whole space. Fetch and skim any clearly relevant pages (specs, design docs, architecture decision
  records, runbooks) that could sharpen a later implementation plan. Note what was found: page titles/links and
  a one-line summary of the relevant content, so it can be folded into Step 7's report.
- **If nothing relevant turns up** even though a documentation MCP is connected, say so briefly rather than
  omitting the mention.
- **If no documentation MCP is connected**, note that briefly and move on — this step is a best-effort
  enrichment and must never block exploration when no such tool exists.

## Step 7 — Report findings

Summarize for the user in plain text (no file output unless asked):

- Always include a **Tech stack** section, structured so later skills invoked in this same conversation (e.g.
  `iru-plan`, `iru-code`, code review, branch/PR creation) can read it directly instead of re-detecting it themselves:
  ```
  ## Tech stack
  - Languages (codebase): <e.g. Java, Kotlin>
  - Frameworks (codebase): <e.g. Spring Boot (backend/), Jetpack Compose (android/)>
  - Languages/frameworks (requested change): <e.g. Kotlin, Jetpack Compose — matches android/ module>
  - Antora docs: <e.g. docs/ (antora.yml component "my-lib") — or "none found">
  - Repository host: <e.g. GitHub (github.com/org/repo) — or Bitbucket / Azure DevOps / TFS, with base URL — or
    "no remote configured" — or "none — not a git repository">
  ```
  If Step 2 found the working directory isn't a git repository at all, use that last form and add a one-line
  warning restating what that disabled this run (git-remote-dependent tooling like unscoped `gh issue view`,
  and `git log`-based history in Step 4) versus what still ran normally (Jira, a documentation MCP, and any
  file-based checks in Step 4/5).
  If the repo has multiple modules/stacks, break the codebase lines out per module. If the requested change
  introduces something new (per Step 5), say so explicitly here rather than folding it silently into the
  existing list. Always state the Antora docs and Repository host lines, even when the answer is "none found"/"no
  remote", so later skills like `iru-update-docs` or ones creating branches/PRs don't need to re-check.
- If Step 6 found a connected documentation MCP, include a **Related documentation** section listing each
  relevant page found (title, link, one-line summary) — or a one-line note that none was relevant/connected.
  This is what a later `iru-plan` step should draw on for additional context beyond the codebase and ticket.
- Include a **Related past implementation plans** section covering both the `.archive/` check from Step 4 and,
  for a Jira ticket, the attached-plan check from Step 3: name any archived or downloaded plan(s) used as
  precedent (file name/source — local `.archive/` vs. downloaded from the ticket/epic/linked ticket — its
  original topic, and what carries over — task breakdown, granularity, or approach), or state explicitly that
  neither source had anything relevant (`.archive/` doesn't exist or nothing in it was relevant, and no
  `implementation_plan_*.md` attachment was found). This is what a later `iru-plan` step should draw on to ground
  a new plan's structure in a proven prior one.
- If a ticket was explored: restate the task in your own words, list the specific files/classes/methods
  relevant to it, explain how the current code behaves in the area the ticket touches, and flag anything
  ambiguous or missing from the ticket that would block implementation. For a Jira ticket, also name its epic
  (if any) and any linked/related tickets fetched in Step 3, with a one-line summary of what each adds (a shared
  constraint from the epic, what a "blocks"/"relates to" link actually implies here) — or state plainly that the
  ticket has no epic and no links if that's the case.
- If codebase-only: summarize the architecture areas covered and anything notable found (e.g. inconsistencies,
  undocumented behavior, missing test coverage) relevant to general orientation.
- Do not propose an implementation plan or start editing code — if the user wants that next, they will ask for
  it (e.g. via planning mode) as a separate step.
