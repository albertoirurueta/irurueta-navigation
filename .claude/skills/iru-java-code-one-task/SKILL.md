---
name: iru-java-code-one-task
description: Implement a single task from a Java `implementation_plan.md`-style task list — just the implementation and its tests. Re-checks the current code state, implements exactly what the task specifies, writes/updates JUnit tests, then — if the task didn't stop on a blocker — immediately checks off this task's own checkbox (and its sub-tasks') in `implementation_plan.md` with a "group validation pending" note, so an interrupted run doesn't re-attempt work that already landed, before handing back a short summary (files touched, tests added/updated, and any blocker) for the caller to record. Does not capture a quality baseline, add license headers, update Javadoc, run coverage/quality checks, or replace the pending note with the final validation outcome — those still happen once per task group in `iru-java-code-one-task-group`, which is what invokes this skill once per task in a bucket (in parallel when the bucket allows it) and finalizes each checkbox's note once group validation passes. Invoke as `/iru-java-code-one-task <task description>`, passing the task's own text — including its exact `implementation_plan.md` checkbox line(s), so this skill can find and flip them — as the argument. Equivalent to `iru-dotnet-code-one-task` for Java/Maven projects.
model: sonnet
allowed-tools: Read Edit Write Bash(mvn *) Bash(git status *) Bash(git diff *) Bash(git log *) Bash(find *) Bash(grep *) Bash(ls *)
---

# Implement one plan task

Carry out a single task's implementation, tests, and checkbox update against the real codebase — nothing else.
This is the narrowest execution unit in the `iru-code` pipeline: `iru-java-code-one-task-group` calls it once per
task in a bucket (potentially many at once, in parallel agents), then handles license headers, Javadoc, and all
test/coverage/quality validation itself, once for the whole bucket, instead of repeating that validation here for
every single task. It uses a medium model on purpose — the plan already carries the hard reasoning; this skill
just executes one task's implementation.

## Step 1 — Re-check the current code state

Read the actual current content of the file(s) the task touches before editing — don't assume any "current code
state" notes passed in with the task are still accurate; other tasks in the same bucket may be landing
concurrently, or the file may have changed for unrelated reasons.

## Step 2 — Implement exactly what the task specifies

The named file(s), the described class/method/field, the behavior, the hook/interface it implements. Follow this
repository's conventions from `CLAUDE.md` (Java version, `final`/`var` type inference for locals (only if code
already uses it), full Javadoc with `@param`/`@return`/`@throws` on every public/protected member,
`IllegalArgumentException` for invalid arguments, no new runtime dependencies). Don't add anything the task
didn't ask for — no speculative abstractions, no unrelated cleanup.

## Step 3 — Write or update the tests

Follow the existing test style in the same package (JUnit 5, Mockito only where already used). Cover the
new/changed behavior, including edge cases implied by the Javadoc `@throws` contracts (e.g. null/invalid-argument
cases). Do not run the test suite yourself — `iru-java-code-one-task-group` runs it once for the whole bucket in its
own consolidated validation pass.

## Step 4 — When to interrupt the user

Keep interruptions rare — most tasks should complete unattended. Stop and use `AskUserQuestion` (or plain text if
no real choice is being offered) only when:

- Implementing the task reveals it's ambiguous in a way that changes correctness or scope and can't be safely
  inferred — conflicting instructions, a choice only the user can make (e.g. which of two APIs to break), or a
  missing decision. Don't ask about things with an obvious best-practice answer.
- The task's described approach appears infeasible against the real code as it stands (e.g. it names a type or
  member that doesn't exist and isn't a trivial typo) — not something you can resolve by writing the code
  slightly differently.

Do not stop merely because the task is nontrivial — implement it. Do not report the task done to work around a
blocker; report it as blocked instead (Step 6), with enough detail — what was tried, what failed, why — for the
caller to surface it.

## Step 5 — Check off this task's checkbox now

If this task did not stop on a blocker (Step 4), update `implementation_plan.md` at the repository root before
reporting back — don't leave this for the caller. Re-read the file fresh (not any earlier cached view — other
tasks in the same bucket may be landing concurrently) and locate this task's own checkbox line by the exact text
passed in with the task. Flip its `[ ]` to `[x]`, and do the same for every sub-task checkbox that was part of
the work just completed, adding a short note naming the files touched, e.g. `- [x] Task 2. **Implement
`SimpleWidget`** — implemented, tests added; group validation pending.` — the same "pending" wording
`iru-java-code-one-task-group` uses, since the coverage/quality outcome isn't known until it validates the whole
bucket in its own Step 3/4. Edit only this task's own line(s) — never rewrite surrounding lines or other tasks'
checkboxes, since sibling tasks in a parallel bucket may be editing this same file at nearly the same moment.
This is what lets an interrupted run resume without re-attempting a task whose implementation and tests already
landed, even if the interruption happened before `iru-java-code-one-task-group` got to run its own group-wide
validation.

If this task stopped on a blocker instead, leave its checkbox untouched — there's nothing finished to mark done.

## Step 6 — Report the outcome

Hand control back to the caller with a short summary: the file(s) touched, the tests added/updated, and whether
this task stopped on a blocker per Step 4. This is what `iru-java-code-one-task-group` records for this task before
running its own consolidated license/Javadoc/test/coverage/quality validation across the whole bucket; this
skill has already checked off this task's own checkbox with a pending-validation note (Step 5) — the caller only
needs to replace that note with the final validation outcome once the whole bucket passes, not create the
checkbox entry itself. This skill itself never captures a quality baseline and never runs tests/coverage/quality
checks.
