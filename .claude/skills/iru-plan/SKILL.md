---
name: iru-plan
description: Generate an implementation_plan.md at the repository root detailing the tasks needed to complete a piece of work, optionally grounded in a specific GitHub issue or Jira ticket. Invoke as `/iru-plan <ticket-id>` to plan for that ticket (a GitHub issue ID or Jira key, auto-detected like `iru-explore`), or `/iru-plan` with a description in the same message to plan manually described work. Runs the `iru-explore` skill first to ground the plan in the actual codebase and ticket, unless that exploration already happened earlier in this conversation. Records the source GitHub issue or Jira ticket, if any, in the plan's Task summary so later steps in the pipeline can track which tracked ticket the plan resolves. If `implementation_plan.md` already exists at the repository root, determines whether it matches the requested task and asks the user whether to resume it as-is, discard and regenerate it, or update it while preserving already-completed tasks. Groups tasks into dependency-aware task groups — each marked parallelizable or not — so a downstream `iru-code`-style skill can validate once per group instead of once per task. Records which language/framework (e.g. `java`, `dotnet`) each task and sub-task belongs to, using the same keys as this repository's installed `<language>-code-one-task` skills, so a downstream execution skill knows which one to invoke per task without re-inferring it. Use when the user wants a concrete, reviewable step-by-step plan before code changes begin.
model: opus
---

# Plan

Produce a concrete, step-by-step implementation plan for a task — grounded in the real state of the codebase —
and write it to `implementation_plan.md` at the repository root. This skill does not modify source code; its
only output artifact is the plan file itself. It uses a complex model on purpose: every downstream skill in the
`iru-code` pipeline assumes the hard reasoning already happened here, so planning mistakes are the most expensive ones
to make in the whole pipeline.

## Step 1 — Determine what is being planned

The skill may be invoked with a ticket ID as its argument — a GitHub issue ID (e.g. `/iru-plan 42`) or a Jira
key (e.g. `/iru-plan PROJ-123`), auto-detected the same way `iru-explore` does — or with a free-form description
of work to plan (e.g. `/iru-plan add retry support to the HTTP client`), or with neither.

- **Ticket ID provided**: use it as the ticket ID (noting whether it's a GitHub issue or a Jira ticket), go to
  Step 2.
- **Free-form task description provided** (in the invocation or immediately preceding user messages): use that
  as the task definition, go to Step 2.
- **Neither provided**: ask the user (via `AskUserQuestion`) whether they want to plan around a GitHub issue or
  Jira ticket ID, or a manually described task. If they give neither, ask them to briefly describe the task — a
  plan cannot be produced without knowing what to plan.

## Step 2 — Reconcile with an existing `implementation_plan.md`

Check whether `implementation_plan.md` already exists at the repository root before doing any further work.

- **Missing**: nothing to reconcile — continue to Step 3.
- **Present**: read it in full (its "Task summary" and checkbox state in particular) and compare it against the
  task determined in Step 1 — same ticket id/reference, same described files/classes/behavior, same general topic.
  You don't need a full codebase exploration to make this call; a reasonable topical read of both is enough.
  Then tell the user your assessment (clearly matches / clearly does not match / unclear) and ask, via
  `AskUserQuestion`, what to do:
  - **Skip planning and leave `implementation_plan.md` as-is** — useful when a previous `/iru-code` run was halted
    partway through and the user just wants to resume execution at the current point. If chosen, stop this skill
    entirely: report the plan's current checkbox progress (tasks/groups done vs. remaining) and tell the user to
    run `/iru-code` directly. Do not touch the file.
  - **Discard the current file and create a new one** — useful when a previous run finished but could not reach
    its own archiving step, leaving a stale completed (or unrelated) plan behind. If chosen, continue to Step 3
    as a fresh plan; the file will be overwritten in Step 6.
  - **Continue exploring/planning and update `implementation_plan.md`, preserving completed work** — continue to
    Step 3, but when drafting in Step 6, carry forward every task/sub-task already marked `[x]` (and its progress
    note) into the regenerated plan at the equivalent position, rather than re-adding it as pending. Only tasks
    that are new or still unchecked get re-derived from this run's exploration/planning.
  Respect whichever the user picks; don't guess on their behalf even if the mismatch (or match) looks obvious.

## Step 3 — Ground the plan via the `iru-explore` skill

If a ticket ID is involved, or the task otherwise warrants understanding unfamiliar parts of the codebase, this
plan must be grounded in a prior exploration, not produced from a cold start.

- **Check first**: look back in the current conversation for exploration already performed for this same ticket
  ID or task (either via the `iru-explore` skill or equivalent research already done in this session). If that
  exploration is present and looks reasonably current, reuse it and skip straight to Step 4 — do not repeat it.
- **Otherwise**: invoke the `iru-explore` skill via the `Skill` tool, passing the ticket ID as its argument if one
  exists (`Skill({skill: "iru-explore", args: "<ticket-id>"})`), or with no argument if this is a manually described
  task that still needs codebase orientation. Wait for it to complete before continuing.
- If the task is trivial and self-contained enough that no codebase orientation is needed at all (e.g. the user
  already pasted all relevant context, or the change is confined to a single already-known file), exploration
  may be skipped — use judgment, but default to exploring when unsure.

## Step 4 — Resolve ambiguity, but only when necessary

After grounding, decide whether anything is genuinely ambiguous or under-specified in a way that would make the
plan wrong or force a guess with real consequences (e.g. conflicting requirements, a choice between materially
different architectures, a missing decision only the user can make like which library/framework to standardize
on, or scope that could reasonably mean two very different things).

- If something is ambiguous: ask the user via `AskUserQuestion`, but keep it tight — 1-4 focused questions,
  each with a recommended default option based on codebase conventions and common best practices for the
  language/framework/architecture in play. Do not ask about things you can reasonably infer or that have an
  obvious best-practice answer.
- If nothing is genuinely ambiguous: do **not** ask the user anything. Proceed straight to drafting the plan,
  choosing the best-practice approach yourself given the language, frameworks, libraries, and existing
  architectural conventions found in Step 3 (check the repository's own `CLAUDE.md` or equivalent contributor
  docs, if present, for conventions to follow). State the approach you chose and why in the plan itself so the
  user can see and challenge the reasoning during review, rather than being asked upfront.

## Step 5 — Determine language/framework keys and group tasks by dependency

Downstream, a `iru-code`-style skill executes this plan one task **group** at a time by invoking `iru-code-one-task-group`,
which in turn dispatches each task in the group to a language-specific `<key>-code-one-task-group`/
`<key>-code-one-task` skill (e.g. `iru-java-code-one-task`, `iru-dotnet-code-one-task`). Both the language key and the
grouping need to be explicit and unambiguous in the plan — don't leave either to be re-inferred later.

**Language/framework keys:**

- Find which `*-code-one-task` skills are actually installed in this repository:
  `find .claude/skills -maxdepth 1 -type d -name "*-code-one-task"`. The directory name minus its `iru-` prefix
  and its `-code-one-task` suffix is the key (e.g. `iru-java-code-one-task` → `java`, `iru-dotnet-code-one-task`
  → `dotnet`) — the `iru-` prefix is only this catalog's marketplace-collision namespace, not part of the
  language/framework key itself. Use these exact keys — don't invent a synonym (e.g. write `dotnet`, not `csharp`
  or `net`, if `iru-dotnet-code-one-task` is what's installed).
- Determine each task's language/framework from the exploration in Step 3 (the `iru-explore` skill already detects the
  language(s)/framework(s) in play) and from the concrete file(s) each task names.
- If a task's actual language/framework has no matching `*-code-one-task` skill installed, still record the real
  language/framework (e.g. `python`, `typescript`) rather than forcing it into an unrelated key — a downstream
  skill can then fall back to generic tooling or flag it as unsupported, but the plan must stay accurate about
  what the task actually is.
- If a task's language/framework genuinely can't be determined (e.g. a non-code task like documentation-only or
  config-only changes, or exploration simply didn't surface enough to tell), do **not** block on it or guess —
  still add the task to the plan, just without any language/framework tag. A missing tag is a valid, expected
  state; a downstream skill treats an untagged task as one it must implement directly rather than dispatch to a
  language-specific skill.
- Tag **every** top-level task with its key right after the bold title, even when the whole plan shares a single
  language/framework — e.g. `[ ] **Task 1. Some task** _(dotnet)_`. A sub-task inherits its parent task's tag by
  default and doesn't need its own unless it genuinely differs from its parent (rare — e.g. a task that adds a
  Java class plus a companion doc-only sub-task).

**Task groups:**

- Group tasks by their real dependencies, not by arbitrary batch size. Two tasks belong in the same group only if
  their implementation doesn't require one to see the other's finished result — e.g. adding two independent,
  unrelated classes, or implementing a class alongside its own tests. Two tasks belong in *different*, ordered
  groups when one genuinely depends on the other having landed first — e.g. a concrete implementation that
  depends on an abstraction added by an earlier task, or a task that edits a file another task in the same
  candidate group would also edit (their tests especially: two tasks whose test suites touch the same test
  file/fixture cannot be safely implemented and validated in parallel).
- Within a group, task **implementation** (code + its own tests) can generally run in parallel even when the
  group's final validation (running all tests, checking coverage, checking quality) must happen once, together,
  after every task in the group has been implemented — that consolidated validation is what a downstream
  `<key>-code-one-task-group` skill performs once per group instead of once per task.
- Mark every group with whether its tasks can be implemented in parallel (`Parallelizable: yes` or
  `Parallelizable: no — <reason>`, e.g. `no — Task 3 extends the class Task 2 introduces`). A group of exactly
  one task is always `Parallelizable: yes` (trivially — there's nothing else in the group to conflict with).
- Order groups themselves so each is buildable/testable on top of the previous one, same as task ordering already
  required below.

## Step 6 — Draft `implementation_plan.md`

Write the plan to a file named `implementation_plan.md` at the repository root (per Step 2: overwrite it if the
user chose to discard, merge into it if the user chose to preserve completed work, or don't reach this step at
all if the user chose to skip planning). Structure it as:

1. **Task summary** — a brief, plain-language restatement of what is being requested and why (from the ticket
   and/or user's description). If choices were made on the user's behalf per Step 4, state them here with a
   one-line rationale. If this plan was grounded in a tracked GitHub issue or Jira ticket (per Step 1), state
   its source explicitly as its own line, e.g. `Source: GitHub issue #42` or `Source: Jira PROJ-123`, so that a
   future run (e.g. `iru-code`'s archiving step, or someone auditing `.archive/`) can tell which ticket this plan
   resolves. Omit this line entirely for a manually described task with no ticket — don't fabricate one.
2. **Current code state** — a brief summary of the relevant existing architecture/classes/files as found during
   exploration: what exists today, how it is structured, and where the change needs to land. Reference concrete
   file paths and class/method names from the actual repository, not vague descriptions.
3. **Implementation steps** — organized into the task groups determined in Step 5, each group its own subsection
   naming the group number and its `Parallelizable` verdict, containing that group's tasks (each broken into
   sub-tasks as needed). Every task and sub-task must be explicit and actionable, not aspirational:
   - Name the exact file(s) to create or modify.
   - Describe the exact change (new class/function/field, signature, behavior, which hook/interface it
     implements, what test to add, what documentation is required, etc.).
   - Where it clarifies the intent, include a short illustrative code example (a signature, a snippet, a test
     case sketch) — enough to remove guesswork, not a full implementation.
   - Number tasks and sub-tasks continuously across the whole plan (not restarting per group) using the pattern
     `- [ ] Task <N>. <Description>` for a top-level task and `- [ ] Task <N>.<M>. <Description>` for its
     sub-tasks, e.g.:
     ```
     - [ ] Task 1. Add the `Widget` abstraction _(java)_
       - [ ] Task 1.1. Define the `Widget` interface in `Widget.java`
       - [ ] Task 1.2. Add `WidgetTest` covering the interface's default methods
     - [ ] Task 2. Implement `SimpleWidget` _(java)_
     ```
     The `iru-code-one-task-group` skill (and its language-specific delegates) check these boxes off as it completes
     each one — every task/sub-task line needs its own `[ ]`, never a shared checkbox.
   - Order tasks within a group so each is buildable/testable on top of the previous one where a real ordering
     dependency exists within the group; across groups, ordering already comes from Step 5.
   - Call out verification steps explicitly where this repository's conventions require them (e.g. "update
     the docs", "run the linter/static-analysis profile"). For running the tests touched by a task, call out
     delegating to the `iru-gate-runner` agent (e.g. `Agent({description: "Run tests for <selector>", subagent_type:
     "gate-runner", prompt: "Invoke Skill({skill: \"<test-skill>\", args: \"<selector>\"}) ..."})`, if
     `iru-gate-runner` is installed in this repository's `.claude/agents/` — otherwise the equivalent generic
     sub-agent delegation) rather than running the test-running skill or command directly in the main
     conversation, so test output doesn't consume the main context window. Scope it to the relevant test(s) using
     a repository-specific test-running skill if one is available in this repository; otherwise have the
     sub-agent run the equivalent test-runner command for this repository's language/build tool directly.

Keep the plan concrete and skimmable: prefer nested numbered/bulleted lists over prose paragraphs.

## Step 7 — Ask for review

Once `implementation_plan.md` is written, tell the user it's ready, give a one- or two-sentence summary of the
approach — including how many task groups were formed and which, if any, are marked non-parallelizable and why —
and explicitly ask them to review the file and let you know if anything needs to change before implementation
starts. Do not begin implementing the plan unless the user asks you to.
