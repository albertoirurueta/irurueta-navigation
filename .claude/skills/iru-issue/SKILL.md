---
name: iru-issue
description: End-to-end kickoff for a tracked ticket — a GitHub issue or a Jira ticket, auto-detected like `iru-explore` — requires a ticket ID, unlike `iru-explore`/`iru-plan` this skill stops if none is given. Fetches the ticket, classifies it as a feature or a hotfix from its labels/issue-type/content, creates a `feature/<ticket-id>` or `hotfix/<ticket-id>` branch off the current branch, then runs the `iru-explore` and `iru-plan` skills for that ticket. Once a reviewable `implementation_plan.md` exists, asks whether to hand off implementation to the `iru-code` skill (run in an isolated sub-agent) or stop for manual review; once implementation is done, attaches the archived implementation plan back onto the original ticket (as a GitHub issue comment, or a Jira attachment/comment) for future context if one was actually archived, skipping that step otherwise, then pushes the branch, opens a pull request (using whichever tooling matches the repository's host — GitHub, Bitbucket, Azure DevOps, or TFS — as detected by `iru-explore`) back to the branch `/iru-issue` was run from, uses the `iru-pr-description` skill to fill in its description, then runs the `iru-pr-review` skill (passing both the new PR's id and the ticket id) to leave review comments on it. If `iru-code`'s security gate (`iru-check-security`) ever flags a new or newly-unaudited secret during implementation, this skill stops instead — it warns the user with the specifics and never pushes the branch or opens a PR for that run, even if `iru-code` itself went on to resolve the gate and finish. Invoke as `/iru-issue <ticket-id>` where `<ticket-id>` is either a GitHub issue ID (e.g. `42`) or a Jira key (e.g. `PROJ-123`). Use when starting work on a tracked ticket and you want branch creation, exploration, planning, implementation, PR creation, and an initial code review done in one pass.
model: sonnet
---

# Issue

Take a tracked ticket — a GitHub issue or a Jira ticket — from "just filed" to "pull request open for review, with
review comments already left" in one pass. This skill orchestrates `iru-explore`, `iru-plan`, `iru-code`, `iru-pr-description`, and
`iru-pr-review` — it does not duplicate their logic, it sequences them around branch creation, ticket triage, and PR
creation.

## Step 1 — Resolve the ticket ID and its type (required)

Unlike `iru-explore`/`iru-plan`, this skill has no "no ticket" mode — a ticket ID is mandatory, and it may be either a
GitHub issue ID or a Jira key.

- **Argument provided** (e.g. `/iru-issue 42` or `/iru-issue PROJ-123`): detect which system it belongs to, the same way
  `iru-explore` does:
  - Matches a Jira key pattern — one or more uppercase letters/digits starting with a letter, a hyphen, then
    digits (e.g. `PROJ-123`, `ABC2-45`) → **Jira**.
  - A bare number, optionally prefixed with `#` (e.g. `42`, `#42`) → **GitHub issue**.
  - If it matches neither pattern clearly, ask the user (via `AskUserQuestion`) whether it's a GitHub issue ID or
    a Jira key.
  - Once resolved, go to Step 2.
- **No argument provided**: ask the user via `AskUserQuestion` for the GitHub issue ID or Jira key to work on
  (free text).
  - If they provide one, detect its type as above and continue to Step 2.
  - If they decline or give no ID, stop here entirely. Tell the user this skill requires a ticket ID and that
    they can re-invoke it with one (e.g. `/iru-issue 42` or `/iru-issue PROJ-123`). Do not fall back to a codebase-only
    exploration or any other partial behavior.

## Step 2 — Fetch the ticket

### GitHub issue

Determine the repository (owner/name) from the git remote:

```bash
gh repo view --json owner,name -q '.owner.login + "/" + .name'
```

Then fetch the issue:

```bash
gh issue view <ticket-id> --json number,title,body,labels,comments,state,url
```

If `gh` is not installed/authenticated, search for GitHub MCP tools instead (`ToolSearch` with a query like
"github issue") and use those to fetch the same fields. If the issue can't be fetched by any means — including if
`gh repo view` reveals this repository isn't actually hosted on GitHub — tell the user and stop; every later step
(classification, branch naming, `iru-explore`, `iru-plan`) depends on it.

### Jira ticket

Search for connected Jira MCP tools (`ToolSearch` with a query like "jira issue" or "jira ticket"). If one or
more are found, use them to fetch the ticket by key: summary/title, description, issue type, status, labels/
components, comments, and URL.

If no Jira MCP tool is available, ask the user (via `AskUserQuestion`) to paste the relevant ticket content
(title/description/comments) directly. Unlike `iru-explore`, this skill has no codebase-only fallback — without
either a live fetch or pasted content it cannot classify, plan, or name a branch for the ticket. If the user has
neither available, tell them this skill requires the ticket's content and stop.

Don't guess at Jira REST endpoints or attempt to `WebFetch` a Jira URL directly — Jira instances are normally
authenticated, and an unauthenticated fetch will just fail or return a login page.

### Either way

Read the ticket's title, body/description, labels (or issue type, for Jira), and any comments that clarify scope
or add constraints. Note explicitly: what behavior is requested/reported, any acceptance criteria, and any files,
classes, or error messages mentioned. Every later step refers to this as "the ticket."

## Step 3 — Classify the ticket as feature or hotfix

Decide whether the ticket describes a new feature or a hotfix/bug fix, in this order of precedence:

1. **Labels/iru-issue type first**: for a GitHub issue, labels like `bug`, `fix`, `hotfix`, `regression` → hotfix;
   labels like `feature`, `enhancement`, `feature-request` → feature. For a Jira ticket, its issue-type field
   (e.g. `Bug` → hotfix; `Story`/`Task`/`New Feature` → feature) plus any labels serve the same role. Use
   whichever category the ticket's actual labels/iru-issue type most clearly map to.
2. **Title/body content** if labels/iru-issue type are absent or don't clearly indicate either category: language
   describing broken/incorrect existing behavior (e.g. "fails", "crash", "regression", "broken", "incorrect",
   "error when…") points to hotfix; language describing new, not-yet-existing capability (e.g. "add", "support
   for", "new", "implement", "as a user I want…") points to feature.
3. **If still genuinely ambiguous** after both checks (e.g. conflicting signals, or the ticket reads as a bit of
   both): ask the user via `AskUserQuestion` to pick feature or hotfix, showing the ticket title and your
   reasoning for why it's ambiguous, with your best guess as the recommended default.

State which category was chosen and why (labels/iru-issue type vs. content) before moving on — this determines the
branch prefix in Step 4.

## Step 4 — Create the branch

- Capture the current branch name first (`git branch --show-current`) and remember it as `<base-branch>` — this is
  the branch `/iru-issue` was invoked from (typically `develop`, `main`, or `master`), and is the destination the pull
  request in Step 8 will target.
- Compute the branch name: `feature/<ticket-id>` if Step 3 classified it as a feature, `hotfix/<ticket-id>` if a
  hotfix.
- Check it doesn't already exist locally or on `origin` (`git branch --list <branch-name>`, `git ls-remote --heads
  origin <branch-name>`). If it does, tell the user and ask (`AskUserQuestion`) whether to check it out instead of
  creating it, pick a different name, or stop — don't silently overwrite or reuse it.
- Otherwise, create it from `<base-branch>`: `git checkout -b <branch-name>`. This is a local, easily reversible
  action (no push), consistent with how this repository's other skills branch without extra confirmation — so no
  need to ask before this specific step, only report which branch was created and from what base afterward.

## Step 5 — Explore and plan

Run, in order, waiting for each to finish before starting the next:

1. `Skill({skill: "iru-explore", args: "<ticket-id>"})`
2. `Skill({skill: "iru-plan", args: "<ticket-id>"})`

`iru-explore`'s own Step 2 detects and reports which platform hosts this repository (GitHub, Bitbucket, Azure DevOps,
or TFS) as part of its "Tech stack" summary — Step 8 below relies on that detection instead of re-deriving it or
assuming GitHub.

`iru-plan`'s own Step 2 already checks whether exploration for this ticket happened earlier in the conversation and
skips re-running `iru-explore` if so — since Step 5.1 just did that exploration, `iru-plan` should detect and reuse it
rather than duplicating the work. That's expected; don't intervene.

If either skill reports it cannot proceed (e.g. `iru-plan` needed a clarification the user declined to resolve), stop
here and surface that to the user rather than pushing ahead to Step 6 with an incomplete plan.

## Step 6 — Choose how to proceed with implementation

Once `implementation_plan.md` exists at the repository root, ask the user via `AskUserQuestion`:

- **Proceed automatically with the `iru-code` skill** — hands off implementation now; a pull request is opened
  automatically once it finishes cleanly (Step 8) — unless `iru-code`'s security gate ever fires during the run, in
  which case this skill stops and warns the user instead of pushing anything (see below).
- **Review manually first** — the user reads/edits `implementation_plan.md` themselves and runs `/iru-code` later,
  whenever they're ready.

Handle the choice as follows:

- **Review manually**: stop here (skip Steps 7 and 8). Tell the user the branch that was created, that
  `implementation_plan.md` is ready for review at the repository root, and that once they've run `/iru-code`
  themselves and are happy with the result, they can invoke the `iru-pr-description` skill (or ask you to) to push the
  branch and open a pull request back to `<base-branch>`.
- **Proceed automatically**: implementation must run with a clean context — carrying this whole
  exploration/planning transcript into `iru-code`'s execution would waste context and risk it anchoring on
  intermediate exploration detail instead of the finished plan. Delegate the entire `iru-code` run to the
  `iru-isolated-skill-executor` agent rather than running it inline, since it starts from a fresh context by
  construction and — unlike clearing this conversation directly — lets this skill keep running afterward to attach
  the archived plan to the ticket and open the PR (Steps 7 and 8):

  ```
  Agent({
    description: "Implement ticket <ticket-id> via the code skill",
    subagent_type: "iru-isolated-skill-executor",
    prompt: "Invoke Skill({skill: \"code\"}) to execute implementation_plan.md at the repository root end to end.
      Report back: which tasks completed, the files touched, the coverage and code-quality outcome, whether the
      plan was archived, and — critically — whether `iru-code`'s Step 7 security gate (backed by `iru-check-security`)
      ever fired during the run, i.e. whether any new or newly-unaudited secret was ever detected, even if it was
      later resolved and the skill went on to finish. If the skill stops for a genuine blocker instead of
      finishing, report that blocker instead.",
    run_in_background: false
  })
  ```

  Wait for the sub-agent to finish (`run_in_background: false` — Step 7 needs its outcome before attaching the
  plan, and Step 8 needs it before opening a PR). If it reports a blocker instead of completion (e.g. `iru-code`
  stopped for a decision only the user can make, or a build stayed broken), surface that to the user and stop
  here — do not proceed to Step 7 or Step 8 against unfinished or broken work (an incomplete run also won't have
  reached `iru-code`'s own archiving step, so Step 7 would have nothing to attach anyway).

  **If the sub-agent reports that the security gate ever fired** — regardless of whether `iru-code` went on to
  resolve it and finish cleanly — treat this as a hard stop for this skill too, distinct from an ordinary
  blocker: do not proceed to Step 7 or Step 8. A branch that ever had a secret flagged on it is not one this skill
  should push, open a pull request for, or post ticket comments/attachments about automatically, since an
  accepted-as-false-positive judgment call or an incompletely-rotated credential could otherwise get exposed —
  pushed to a shared remote, or pasted into a ticket comment — without further human review. Warn the user
  explicitly: state what `iru-check-security` flagged (file, line, secret type, from the sub-agent's report), that
  the branch `<branch-name>` was created and implementation finished locally, but that this skill is **not**
  pushing the branch, opening a PR, or attaching anything to the ticket. Tell them to verify the finding
  themselves (confirm the secret was actually removed/rotated, or is genuinely a false positive per
  `iru-check-security`'s own audit step), and that once satisfied it's safe they can push and open the PR
  themselves, or re-invoke `iru-pr-description` to do so. Then go to Step 9 to report, skipping Steps 7 and 8
  entirely.

## Step 7 — Attach the archived implementation plan to the ticket

Once `iru-code` (invoked in Step 6) reports successful completion, check whether it archived
`implementation_plan.md` to `.archive/` — `iru-code`'s own Step 9 only archives once every task's checkbox is
checked and both its quality and security gates resolved, and names the file after the ticket this skill is
already working on, per that step's naming convention: the bare issue number for a GitHub issue (e.g.
`implementation_plan_42.md`), or the Jira key as-is for a Jira ticket (e.g. `implementation_plan_PROJ-123.md`).

```bash
ls .archive/implementation_plan_<ticket-id>.md
```

- **Missing** (no matching file — e.g. `iru-code` used a timestamp instead because it couldn't identify the
  ticket's origin, even though this skill knows it): skip this step entirely and continue to Step 8 — there is
  nothing to attach, and this is not worth surfacing as an error on its own.
- **Present**: attach it to the original ticket, so that a future exploration of this same ticket — e.g.
  `iru-explore`'s own comment-reading step, or someone opening the ticket cold to use it as precedent for a new
  task — has the full implementation plan available as context, not just the ticket's original description:
  - **GitHub issue**: GitHub's issue API has no generic file-attachment endpoint, so post the plan's content as a
    comment instead — this keeps it readable directly on the ticket, including for anyone who later fetches it
    via `iru-explore`'s own comment-reading step:
    ```bash
    gh issue comment <ticket-id> --body-file .archive/implementation_plan_<ticket-id>.md
    ```
    If `gh` is unavailable, use the equivalent GitHub MCP comment tool instead, passing the archived file's full
    content as the comment body.
  - **Jira ticket**: search for a connected Jira MCP tool that supports adding an attachment to a ticket
    (`ToolSearch` with a query like "jira attachment" or "jira upload"). If one is found, use it to attach the
    archived file itself. If no attachment-capable tool is available but a comment tool is, fall back to posting
    the plan's content as a comment instead (same rationale as the GitHub case), and note in Step 9's report that
    it was added as a comment rather than a true attachment, for lack of an attachment-capable tool.
  - If neither an attachment nor a comment can be posted by any available means (e.g. no Jira MCP tool at all is
    connected), tell the user this step couldn't be completed and why, then continue to Step 8 — this is a
    best-effort enrichment, not a blocker for opening the pull request.
- Note the outcome (attached, added as comment instead, or skipped and why) so it can be folded into Step 9's
  report.

## Step 8 — Push the branch and open the pull request

Only reached when Step 6's sub-agent reports successful completion **and** its security gate never fired — never
push or open a PR off the back of a run where `iru-check-security` ever flagged something, per Step 6.

1. **Confirm with the user** before taking any visible/shared-state action: show the branch name, the destination
   (`<base-branch>` from Step 4), and a summary of what the sub-agent changed (from its report). Pushing and
   opening a PR are exactly the kind of actions this repository's other skills (`iru-release`, `iru-pr-description`) always
   confirm before taking — do the same here.
2. Push the branch: `git push -u origin <branch-name>` — a plain git operation, unaffected by which platform hosts
   the remote.
3. Identify the repository host from Step 5's exploration — `iru-explore`'s own Step 2 already detected and reported
   it (as `Repository host: ...` in its Tech stack summary). Use that instead of re-detecting it or assuming
   GitHub. If it couldn't be determined there (e.g. no remote configured), tell the user and stop here rather than
   guessing which tool to use.

   Open the pull request **as a draft** with a minimal placeholder body that keeps the ticket linked, mirroring
   how the `iru-release` skill opens its PR before handing off to `iru-pr-description`. It stays in draft so the user has
   an explicit chance to review before marking it ready — this skill never converts it out of draft itself. Pick
   the tool that matches the detected host:
   - **GitHub**: the `gh` CLI (or GitHub MCP tools if `gh` is unavailable):
     ```bash
     gh pr create --draft --base <base-branch> --head <branch-name> --title "<ticket-title> (#<ticket-id>)" \
       --body "Closes #<ticket-id>."
     ```
     Use `--body "Closes #<ticket-id>."` only when the ticket is a GitHub issue — that auto-close keyword is
     GitHub-specific. If the ticket is a Jira key on a GitHub repo, use `--body "Refs <ticket-id>."` instead;
     Jira tickets close through their own automation, not a foreign platform's PR body keyword.
   - **Bitbucket**: there's no CLI as ubiquitous as `gh` here — search for Bitbucket MCP tools first (`ToolSearch`
     "bitbucket") and use those; otherwise call the Bitbucket REST API directly (`POST
     /2.0/repositories/<workspace>/<repo>/pullrequests` for Cloud, or the Server/Data Center equivalent) with the
     draft flag, title, a description of `Refs <ticket-id>.`, the source branch, and the destination branch.
   - **Azure DevOps / TFS**: `az repos pr create --draft true --target-branch <base-branch> --source-branch
     <branch-name> --title "<ticket-title> (<ticket-id>)" --description "Refs <ticket-id>."` (the Azure CLI with
     the `azure-devops` extension, pointed at the right org/project or on-prem collection URL), or Azure DevOps
     MCP tools if connected.
4. Invoke `Skill({skill: "iru-pr-description"})`. It will detect the PR just opened (same branch, via its own Step 2),
   draft a description from the actual diff, and ask whether to replace the placeholder body — confirm yes so the
   real description lands.
5. After `iru-pr-description` finishes, check the PR body still references the ticket (a "Closes #<ticket-id>." or
   "Refs <ticket-id>." line, per whichever convention Step 8.3 used) with the same host tooling used to create it
   (e.g. `gh pr view <number> --json body` for GitHub) — `iru-pr-description` fully replaces the body with its own
   draft and may drop the placeholder's ticket reference. If the reference is gone, append it back with that same
   tooling (e.g. for GitHub: `gh pr edit <number> --body "$(gh pr view <number> --json body -q .body)"$'\n\n'"Closes
   #<ticket-id>."`) so the ticket still links back — and, for a GitHub issue, auto-closes — when the PR merges.
6. Invoke `Skill({skill: "iru-pr-review", args: "<number> <ticket-id>"})` to get an initial code review left on the PR
   — passing the ticket id explicitly (rather than relying on `iru-pr-review`'s own body/title detection) since this
   skill already knows with certainty which ticket the new PR closes. `iru-pr-review` runs its own confirmation before
   posting anything, so no separate confirmation is needed here; wait for it to finish before moving to Step 9.
   If it reports it couldn't proceed (e.g. no PR could be fetched, which shouldn't happen since it was just
   created), surface that to the user rather than treating it as a hard failure of this skill — the PR itself is
   still open and usable.

## Step 9 — Report

Summarize for the user: the ticket ID and title, the classification (feature/hotfix) and why, the branch created
and its base, and the final outcome — one of:

- manual review pending (Step 6), or
- the security gate fired during implementation (Step 6): the sub-agent's implementation summary, what
  `iru-check-security` flagged, and the explicit reminder that the branch was **not** pushed, no PR was opened,
  and nothing was attached to the ticket — restate what the user needs to verify before doing so themselves, or
- the sub-agent's implementation summary, whether the archived plan was attached to the ticket (or added as a
  plain comment, or skipped, per Step 7), the PR URL, confirmation the ticket is linked, and `iru-pr-review`'s
  outcome (findings posted, or the drafted review if the user declined to post) (Step 8).

If Step 8 ran, close with an explicit warning, not just a passing mention: the pull request was opened **as a
draft** and still needs the user's own review before it's marked ready and merged — `iru-pr-review`'s automated pass
does not substitute for that. Include the PR link, obtained via whichever host tooling created it in Step 8 — e.g.
`gh pr create`'s own output or `gh pr view <number> --json url -q .url` for GitHub; the API response's web link for
Bitbucket; `az repos pr create`'s own output or `az repos pr show --id <number> --query "_links.web.href" -o tsv`
for Azure DevOps/TFS — so the user can open it directly.
