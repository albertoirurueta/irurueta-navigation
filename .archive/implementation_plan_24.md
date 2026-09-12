# Implementation Plan

## Task summary

Source: GitHub issue #24

The bibliography section of the Antora reference page lists books and papers used as sources for this
library's algorithms, but several entries have no link to the book's official editorial page or the paper's
official download page. Add those links so readers can reach the official source directly.

This work is done on `feature/24`, branched from `release_1.8.1` (per the issue's explicit instruction), so the
fix ships in the docs for the most recently published version. It must later be merged into `master`, then into
`develop` — that merge chain is outside the scope of this plan/branch and is handled separately once this PR is
approved and merged.

Of the 14 bibliography entries in `docs/modules/ROOT/pages/reference.adoc`, 10 already carry an appropriate
link (DOI, arXiv, GitHub, project site, or Wikipedia) and need no change. Four are missing a link; the official
URL for each was identified via web research during planning (documented per task below) rather than left as a
placeholder or guess:

| Anchor | Missing link |
|---|---|
| `bib-groves2013` | Artech House official book page |
| `bib-borkowski1989` | Springer (official publisher) article page |
| `bib-higham2002` | SIAM ePubs official book page |
| `bib-hereman-murphy` | Official PDF hosted on the author's Colorado School of Mines faculty page |

This is a documentation-only change (AsciiDoc); no source code, tests, or build tooling are affected.

## Current code state

- `docs/modules/ROOT/pages/reference.adoc` — the Antora reference page. Its `[[bibliography]]` section (lines
  36-115) lists 14 entries, each an AsciiDoc anchor (`[[bib-<key>]]`) followed by a citation paragraph. Entries
  that already have a link embed it inline as `https://...[link text]` or `link:https://...[link text]`
  (see `bib-karney2013`, `bib-geographiclib`, `bib-wikipedia-trilateration` for the existing style/convention).
- The four entries needing a link, and their current text:
  - `bib-groves2013` (reference.adoc:42-45): "Groves, P. D., _Principles of GNSS, Inertial, and Multi-sensor
    Integrated Navigation Systems_, 2nd ed., Artech House, 2013." — no link at all.
  - `bib-borkowski1989` (reference.adoc:52-57): "Borkowski, K. M., "Accurate algorithms to transform geocentric
    to geodetic coordinates," _Bulletin Géodésique_, vol. 63, no. 1, pp. 50-56, 1989." — no link at all.
  - `bib-higham2002` (reference.adoc:84-86): "Higham, N. J., _Accuracy and Stability of Numerical Algorithms_,
    2nd ed., SIAM, 2002." — no link at all.
  - `bib-hereman-murphy` (reference.adoc:102-104): "Hereman, W., and W. S. Murphy Jr., "Determination of a
    Position in Three Dimensions Using Trilateration and Approximate Distances."" — no link, and no venue/year
    stated either.
- No `.claude/skills/*-code-one-task` language key applies (this is a plain-text doc edit) — the task below is
  left untagged, per `iru-plan`'s convention for non-code tasks, so `iru-code` implements it directly rather than
  dispatching to a language-specific skill.
- `iru-build-docs` skill exists and can build the Antora site to verify the edited page has no AsciiDoc syntax
  errors; `iru-gate-runner` agent exists (used here to keep the doc-build output out of the main context window).

## Implementation steps

### Group 1 — Add the four missing bibliography links (Parallelizable: yes — single small file, sequential edits are trivial and don't depend on each other's outcome)

- [x] Task 1. Update `docs/modules/ROOT/pages/reference.adoc` bibliography entries with official links — files
  touched: `docs/modules/ROOT/pages/reference.adoc` only; tests: n/a (documentation-only change); coverage: n/a;
  code-quality outcome: no issues, AsciiDoc link syntax matches surrounding entries and Antora build is clean;
  license-header generation: skipped (n/a for `.adoc` doc pages, no header convention applies to this file type).
  - [x] Task 1.1. `bib-groves2013` (reference.adoc:42-47) — added
    `https://us.artechhouse.com/Principles-of-GNSS-Inertial-and-Multisensor-Integrated-Navigation-Systems-Second-Edition-P2046.aspx[Artech House]`
    inline after "Artech House, 2013"; explanatory prose unchanged.
  - [x] Task 1.2. `bib-borkowski1989` (reference.adoc:54-59) — added
    `https://link.springer.com/article/10.1007/BF02520228[Springer]` inline after "pp. 50-56, 1989"; explanatory
    text about the Appendix C equations unchanged.
  - [x] Task 1.3. `bib-higham2002` (reference.adoc:86-89) — added
    `https://epubs.siam.org/doi/book/10.1137/1.9780898718027[SIAM]` inline after "SIAM, 2002".
  - [x] Task 1.4. `bib-hereman-murphy` (reference.adoc:105-109) — added
    `https://people.mines.edu/whereman/wp-content/uploads/sites/260/2024/05/Murphy-Hereman-Trilateration-MCS-07-1995.pdf[Colorado School of Mines]`
    inline after the quoted title; explanatory text unchanged.
  - [x] Task 1.5. Re-read the full bibliography section (lines 36-120) after editing: link syntax is consistent
    with the surrounding entries (`https://...[text]` inline, no broken brackets), and no other entry was
    modified.
  - [x] Task 1.6. Verified via `iru-gate-runner` agent invoking `iru-build-docs`: the Antora build completed with
    exit code 0 and no errors/warnings; `reference.adoc` rendered cleanly to `reference.html`.
