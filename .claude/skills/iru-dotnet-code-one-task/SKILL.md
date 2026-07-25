---
name: iru-dotnet-code-one-task
description: Implement a single task from a .NET `implementation_plan.md`-style task list — just the implementation and its tests. Re-checks the current code state, implements exactly what the task specifies, writes/updates test-framework tests (xUnit/NUnit/MSTest, whichever the project uses), then hands back a short summary (files touched, tests added/updated, and any blocker) for the caller to record. Does not capture a quality baseline, add license headers, update doc comments, run coverage/quality checks, or read/write `implementation_plan.md` — those now happen once per task group in `iru-dotnet-code-one-task-group`, which is what invokes this skill once per task in a bucket (in parallel when the bucket allows it). Invoke as `/iru-dotnet-code-one-task <task description>`, passing the task's own text (what to build, the named file(s)/type(s)/method(s), the described behavior, and any sub-tasks) as the argument. Equivalent to `iru-java-code-one-task` for .NET/C# projects.
model: sonnet
allowed-tools: Read Edit Write Bash(dotnet *) Bash(git status *) Bash(git diff *) Bash(git log *) Bash(find *) Bash(grep *) Bash(ls *)
---

# Implement one plan task (.NET)

Carry out a single task's implementation and tests against the real codebase — nothing else. This is the
narrowest execution unit in the `iru-code` pipeline: `iru-dotnet-code-one-task-group` calls it once per task in a bucket
(potentially many at once, in parallel agents), then handles license headers, doc comments, and all test/
coverage/quality validation itself, once for the whole bucket, instead of repeating that validation here for
every single task. It uses a medium model on purpose — the plan already carries the hard reasoning; this skill
just executes one task's implementation.

## Step 1 — Re-check the current code state

Read the actual current content of the file(s) the task touches before editing — don't assume any "current code
state" notes passed in with the task are still accurate; other tasks in the same bucket may be landing
concurrently, or the file may have changed for unrelated reasons.

## Step 2 — Implement exactly what the task specifies

The named file(s), the described class/method/property, the behavior, the interface/base class it implements.
Follow this repository's conventions from `CLAUDE.md` (target framework/language version, `var`/nullable
reference type usage (only if code already uses it), full XML doc comments (`<summary>`/`<param>`/`<returns>`/
`<exception>`) on every public/protected member, `ArgumentException`/`ArgumentNullException` for invalid
arguments, no new NuGet dependencies). Don't add anything the task didn't ask for — no speculative abstractions,
no unrelated cleanup.

## Step 3 — Write or update the tests

Follow the existing test style in the same project/namespace — whichever test framework this repository already
uses (xUnit, NUnit, or MSTest), and Moq/NSubstitute only where already used. Cover the new/changed behavior,
including edge cases implied by the XML doc `<exception>` contracts (e.g. null/invalid-argument cases). Do not
run the test suite yourself — `iru-dotnet-code-one-task-group` runs it once for the whole bucket in its own
consolidated validation pass.

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
blocker; report it as blocked instead (Step 5), with enough detail — what was tried, what failed, why — for the
caller to surface it.

## Step 5 — Report the outcome

Hand control back to the caller with a short summary: the file(s) touched, the tests added/updated, and whether
this task stopped on a blocker per Step 4. This is what `iru-dotnet-code-one-task-group` records for this task before
running its own consolidated license/doc/test/coverage/quality validation across the whole bucket; this skill
itself never touches `implementation_plan.md`, never captures a quality baseline, and never runs tests/coverage/
quality checks.
