---
name: iru-setup-java-github-workflows
description: Create or update the `develop.yml`, `main.yml`, and `sync.yml` GitHub Actions workflows for a Java/Maven library repository — build and test, run Checkstyle/PMD/SpotBugs static analysis, generate JaCoCo coverage and the Maven `site` report, run a SonarQube/SonarCloud scan via the `sonar-maven-plugin` (`mvn sonar:sonar`, so it runs under the same JDK the job already set up), build the Antora documentation site, publish both the Antora docs and the Maven site report to GitHub Pages, publish the release artifact to Maven Central, and — once a release is published — open a pull request that merges the released branch back into the integration branch and bumps the development snapshot version across `pom.xml`, `README.md`, and the Antora docs (via the companion `.github/scripts/sync_versions.py` script). Invoke as `/iru-setup-java-github-workflows`. Ships with generic example templates (derived from a real repository's workflows, genericized so they don't name any specific repository) embedded in this skill file, reusable across repositories. Creates all three workflows (and the sync script) from scratch if none exist; if any already exists, asks the user whether to stop or attempt an update using the templates as reference. Accepts the six pipeline parameters (integration branch, Java version, publishing server id, extras/sign profile ids, settings file) pre-resolved via `args` (`key: value` lines) so an orchestrating skill like `iru-setup-java-library-repository` can supply them without re-prompting. Use whenever a Java/Maven repository needs this CI/CD release pipeline bootstrapped or brought in line with this house pattern, instead of hand-writing the YAML.
model: haiku
---

# Setup Java GitHub Workflows

Scaffold (or update) three GitHub Actions workflows for a Java/Maven library:

- **`develop.yml`** — runs on every push to the integration branch. Build, test, static analysis, coverage, a
  SonarQube/SonarCloud scan (via the `sonar-maven-plugin`, not a standalone `sonar-scanner` step), an Antora docs
  build, and a publish to GitHub Pages + Maven Central.
- **`main.yml`** — the same pipeline, but triggered when a GitHub Release is published rather than on every push.
- **`sync.yml`** — triggered when a GitHub Release is published from the stable branch. Opens a pull request into
  the integration branch that merges the released branch back in and bumps the development snapshot version in
  `pom.xml`, `README.md`, and the Antora docs, via the companion script `.github/scripts/sync_versions.py`. This
  closes the gap where a release branch's version/doc bump never makes it back into the integration branch without
  a manual follow-up.

This skill is designed for Maven projects — the templates in Step 3 assume Maven coordinates, profiles, and
`mvn` commands. If `pom.xml` is missing at the repository root, don't stop automatically: warn the user that this
skill's templates assume Maven and most of Step 1's survey won't resolve, then use `AskUserQuestion` to ask
whether to stop here or continue anyway. If they choose to continue, proceed through the remaining steps but treat
every Maven-derived placeholder in Step 3/4 (Java version, publishing server id, signing/extras profile ids,
`mvn` commands) as an open gap to ask the user about directly, and call this out prominently again in Step 8's
report.

## Step 1 — Survey the target repository

This skill can be invoked stand-alone (`/iru-setup-java-github-workflows`) or as a step inside another skill, such as
`iru-setup-java-library-repository`, which resolves the six pipeline parameters below itself (plus, since it
collects them anyway in its own Step 1, `groupId`/`artifactId`) and passes them through `args` as `key: value`
lines, one per line, e.g.:

```
integration-branch: develop
java-version: 17
publishing-server-id: central
extras-profile-id: build-extras
sign-profile-id: sign
settings-file: mvnsettings.xml
group-id: com.example
artifact-id: my-library
```

Parse any such lines from `args` first. For each of `integration-branch`, `java-version`,
`publishing-server-id`, `extras-profile-id`, `sign-profile-id`, `settings-file`, `group-id`, and `artifact-id`
found there, use that value directly — skip the corresponding fact-finding bullet below entirely for it, since
it's already resolved (and, for `group-id`/`artifact-id`, skips re-reading them from `pom.xml`). If `args` is
absent or doesn't look like this format, treat everything as unset and gather every fact below as usual.

Gather the remaining facts before writing anything; every placeholder in Step 3's templates must be resolved from
a real answer, not guessed. If `pom.xml` is missing and the user chose to continue anyway, skip the
`pom.xml`-derived facts below and ask the user for equivalent values directly instead:

- **Maven coordinates** (skip `groupId`/`artifactId` if supplied via `args`): read whichever of
  `groupId`/`artifactId`/`version` weren't already supplied from `pom.xml` — `version` is always read here, since
  it isn't known yet at the point `iru-setup-java-library-repository` collects its own Step 1 fields.
- **Java version** (skip if supplied via `args`): read `maven.compiler.source`/`maven.compiler.target` (or
  `maven.compiler.release`) from `pom.xml` properties.
- **Central/Nexus publishing plugin** (skip the `publishing-server-id` half if supplied via `args`): grep
  `pom.xml` for `central-publishing-maven-plugin` (the current Sonatype Central Publishing Portal plugin) or
  `nexus-staging-maven-plugin` (the older OSSRH plugin). Record the `publishingServerId`/`serverId` value it
  declares (e.g. `central`) — the workflow's `setup-java` `server-id` and the Maven settings file's `<server><id>`
  must both match this value. If neither plugin is configured, tell the user Maven Central publishing isn't
  wired up in `pom.xml` yet and ask whether to continue anyway (the generated deploy step will need adjustment
  once that's added).
- **GPG signing profile** (skip if `sign-profile-id` supplied via `args`): find the Maven profile that binds
  `maven-gpg-plugin`'s `sign` goal. Record its profile id (e.g. `sign`).
- **Extras profile** (skip if `extras-profile-id` supplied via `args`): find the profile — typically active by
  default — that attaches sources/javadoc jars (e.g. via `maven-source-plugin` + `maven-javadoc-plugin`). Record
  its id (e.g. `build-extras`). This is the profile the plain test/coverage run excludes (`-P '!<id>'`) so a quick
  `mvn test` doesn't rebuild javadoc every time, while the final `deploy` includes it.
- **Static analysis / coverage plugins**: confirm `jacoco-maven-plugin`, `maven-checkstyle-plugin`,
  `spotbugs-maven-plugin`, and `maven-pmd-plugin` are configured under `<reporting>` — these feed the `mvn site`
  report. Not required to proceed, but note any that are missing so the user knows that section of the site report
  will be empty.
- **SonarQube/SonarCloud config**: check `pom.xml` for the `sonar-maven-plugin`
  (`org.sonarsource.scanner.maven:sonar-maven-plugin`) and its `sonar.organization`/`sonar.projectKey`/
  `sonar.host.url` properties. This house pattern runs the scan via `mvn sonar:sonar` in the workflow (Step 3), so
  the config lives in `pom.xml` itself — not a standalone `sonar-project.properties` file (that file, and the
  `warchant/setup-sonar-scanner` action plus a separate `sonar-scanner` CLI step, are the older pattern this skill
  no longer generates). If the plugin or properties are missing, this skill still wires up the `Run SonarCloud
  analysis` step (Step 3), but someone must add them to `pom.xml` before it will run successfully — flag this and
  offer to add them in Step 5 if the user can give you `sonar.organization`/`sonar.projectKey` now. If a stray
  `sonar-project.properties` still exists from before this house pattern changed, note in Step 8 that it's now
  redundant (nothing reads it once `pom.xml` carries the same settings) and offer to remove it.
- **Antora docs**: check for `docs/antora.yml` and `docs/antora-playbook.yml`. If either is missing, the docs step
  in Step 3 has nothing to build against — tell the user to run the `iru-setup-antora` skill first, or confirm they'll
  do so before merging this workflow.
- **Maven settings file used for deploy** (skip if `settings-file` supplied via `args`): check for an XML file
  referenced by a `--settings` flag in any existing deploy step, defaulting to `mvnsettings.xml` at the repository
  root if none is referenced yet. If it doesn't exist, this skill creates it (Step 6).
- **Branch names** (skip the integration branch if `integration-branch` supplied via `args`): confirm the
  integration branch (commonly `develop`) and the stable branch (commonly `main`) actually match this
  repository's branching model — run `git branch -a` or ask, don't assume gitflow defaults. Both are needed for
  `sync.yml` (Step 3): the stable branch is what it checks the release was published from, the integration branch
  is where its pull request lands.
- **README/Antora version-snippet wording** (only needed for `sync.yml`'s companion script,
  `sync_versions.py`): rather than reading `README.md` and every matched Antora page in full, grep for just the
  marker text and its surrounding lines — `grep -B2 -A2 -i "current development version\|latest release\|latest
  snapshot" README.md`, and for Antora pages, `grep -rl "<version>" docs/modules/ROOT/pages/*.adoc` to find which
  pages mention a version at all, then `grep -B2 -A2 "<version>" <matched-page>` on each one found. From that
  context, find the exact row label used for the development-snapshot row (e.g. `Current development version`)
  and the exact marker text preceding each `<version>` tag (e.g. `Latest release`/`Latest snapshot`, possibly with
  a trailing qualifier like `Latest release shown here`). `sync_versions.py`'s regexes must match this
  repository's actual wording, not just the `iru-setup-readme` skill's default convention — note any deviation now so
  Step 4 can adjust the template correctly. If `README.md` doesn't exist yet, or was not produced by the
  `iru-setup-readme` skill, tell the user `sync_versions.py`'s README step will need hand-adjustment once the file
  exists.

## Step 2 — Decide how to proceed if workflows already exist

Check whether `.github/workflows/develop.yml`, `.github/workflows/main.yml`, `.github/workflows/sync.yml`, and
`.github/scripts/sync_versions.py` already exist. Treat `sync.yml` and its script as one unit for this check —
either both exist or neither should.

- **None exist**: skip this step and go straight to Step 4 — create everything from scratch.
- **Any exists**: use `AskUserQuestion` to ask whether to (a) stop here and leave everything untouched, or
  (b) continue and attempt to update the existing file(s) using this skill's templates as reference.
  - **Stop**: report which file(s) already exist and end here — make no changes.
  - **Continue**: proceed to Step 4, but treat each existing file as the base to edit, not as something to
    overwrite wholesale.
    - For `develop.yml`/`main.yml`: preserve any step that isn't one of the eight pipeline stages this skill owns
      (build/test, static analysis, coverage, site report, SonarQube, Antora build, GitHub Pages publish, Maven
      Central publish) — e.g. a Slack notification step or an extra test matrix stays untouched. Only add missing
      stages or correct outdated ones (wrong action version, wrong profile id, etc.), and don't reorder steps this
      skill doesn't own.
    - For `sync.yml`/`sync_versions.py`: preserve any file the script updates beyond `pom.xml`/`README.md`/the
      Antora docs that a prior customization added, and preserve any extra workflow step (e.g. a Slack
      notification) the same way. Only correct the pipeline stages this skill owns (branch/version computation,
      the merge, the version bump, the PR creation) or fix a stale placeholder value.

## Step 3 — Reference templates

These are genericized examples — based on a real Java/Maven repository's actual `develop.yml`/`main.yml` — showing
the full pipeline: checkout → JDK setup with signing configured → test + coverage + site report → SonarQube scan →
Antora docs build → merge docs with the site report → publish to GitHub Pages → deploy to Maven Central. Copy the
structure; resolve every `<placeholder>` using Step 1's survey before writing the real files in Step 4.

### `develop.yml` template

```yaml
name: Develop

on:
  push:
    branches: [ <integration-branch> ]

jobs:
  build:
    name: Build and execute tests
    runs-on: ubuntu-latest
    steps:
      - name: Check out code
        uses: actions/checkout@v5
        with:
          fetch-depth: 0

      - name: Set up JDK <java-version>
        uses: actions/setup-java@v5
        with:
          distribution: adopt
          java-version: <java-version>
          server-id: <publishing-server-id> # Value of the distributionManagement/repository/id field of the pom.xml
          server-username: OSSRH_USERNAME # env variable for username in deploy
          server-password: OSSRH_PASSWORD # env variable for token in deploy
          gpg-private-key: ${{ secrets.SIGNING_KEY }} # Value of the GPG private key to import
          gpg-passphrase: SIGNING_PASSWORD # env variable for GPG private key passphrase

      - name: Run tests
        run: |
          mvn clean jacoco:prepare-agent install jacoco:report javadoc:jar source:jar -P '!<extras-profile-id>'
          mvn site -Djacoco.skip -DskipTests -P '!<extras-profile-id>'

      - name: Run SonarCloud analysis
        env:
          # to get access to secrets.SONAR_TOKEN, provide GITHUB_TOKEN
          GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}
        run: mvn sonar:sonar -Dsonar.token=${{ secrets.SONAR_TOKEN }}

      - name: Install Node
        uses: actions/setup-node@v4
        with:
          node-version: 24

      - name: Install Antora
        run: |
          mkdir docs && cd docs
          npm i -D -E antora
          npm i @antora/lunr-extension
          npm i @sntke/antora-mermaid-extension
          npm i @djencks/asciidoctor-mathjax

      - name: Build Antora docs
        run: cd docs && npx antora antora-playbook.yml

      - name: Merge documentation
        run: |
          touch ./docs/build/site/.nojekyll
          cp -r ./docs/build/site ./doc
          cp -r ./target/site ./doc/mvn-site

      - name: Deploy to GitHub Pages
        if: success()
        uses: crazy-max/ghaction-github-pages@v5
        with:
          target_branch: gh-pages
          build_dir: ./doc
        env:
          GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}

      - name: Deploy to maven central
        env:
          OSSRH_USERNAME: ${{ secrets.OSSRH_USERNAME }}
          OSSRH_PASSWORD: ${{ secrets.OSSRH_PASSWORD }}
          SIGNING_KEY_ID: ${{ secrets.SIGNING_KEY_ID }}
          SIGNING_PASSWORD: ${{ secrets.SIGNING_PASSWORD }}
          SONATYPE_STAGING_PROFILE_ID: ${{ secrets.SONATYPE_STAGING_PROFILE_ID }}
        run: mvn deploy -P <sign-profile-id>,<extras-profile-id> --settings <settings-file> -DskipTests
```

### `main.yml` template

Identical pipeline; the only difference is the trigger — a GitHub Release being published instead of a push:

```yaml
name: Release

on:
  release:
    # Runs this workflow when a new GitHub release is created
    types: [released]

jobs:
  build:
    name: Build and execute tests
    runs-on: ubuntu-latest
    steps:
      # ... identical to every step under develop.yml's `build` job above, verbatim ...
```

Notes on placeholders that are genuinely project-specific and must come from Step 1, not from these templates:

- `<integration-branch>`: the branch `develop.yml` triggers on (e.g. `develop`).
- `<java-version>`: e.g. `17`.
- `<publishing-server-id>`: the id shared by the Central/Nexus plugin config and the settings file's `<server>`.
- `<sign-profile-id>` / `<extras-profile-id>`: the two profile ids found in Step 1.
- `<settings-file>`: the Maven settings XML path used for the deploy step (Step 6 creates it if absent).

### `sync.yml` template

Runs once a GitHub Release is published, provided it was published from the stable branch. Opens a pull request
that merges the released branch back into the integration branch and bumps the development snapshot version via
the companion `sync_versions.py` script (below):

```yaml
name: Sync

# After a release is published from <stable-branch>, open a pull request that merges it back into
# <integration-branch> and bumps the development snapshot version, so <integration-branch> never has to wait for a
# manual sync branch.
on:
  release:
    types: [released]

permissions:
  contents: write
  pull-requests: write

jobs:
  sync:
    name: Open sync PR from the released branch into <integration-branch>
    runs-on: ubuntu-latest
    if: github.event.release.target_commitish == '<stable-branch>'
    steps:
      - name: Check out code
        uses: actions/checkout@v5
        with:
          fetch-depth: 0

      - name: Compute versions and branch names
        id: versions
        run: |
          set -euo pipefail
          RELEASE_VERSION="${{ github.event.release.tag_name }}"
          RELEASE_VERSION="${RELEASE_VERSION#v}"

          IFS='.' read -r MAJOR MINOR PATCH <<< "$RELEASE_VERSION"
          if [ -z "${MAJOR:-}" ] || [ -z "${MINOR:-}" ] || [ -z "${PATCH:-}" ]; then
            echo "::error::Release tag '$RELEASE_VERSION' is not in X.Y.Z form; cannot compute the next snapshot."
            exit 1
          fi

          NEXT_MINOR=$((MINOR + 1))
          NEXT_SNAPSHOT="${MAJOR}.${NEXT_MINOR}.0-SNAPSHOT"
          SYNC_BRANCH="sync_${MAJOR}.${NEXT_MINOR}.0"

          echo "release_version=$RELEASE_VERSION" >> "$GITHUB_OUTPUT"
          echo "next_snapshot=$NEXT_SNAPSHOT" >> "$GITHUB_OUTPUT"
          echo "sync_branch=$SYNC_BRANCH" >> "$GITHUB_OUTPUT"

      - name: Skip if this release was already synced
        id: guard
        run: |
          set -euo pipefail
          if git ls-remote --exit-code --heads origin "${{ steps.versions.outputs.sync_branch }}" >/dev/null 2>&1; then
            echo "::notice::${{ steps.versions.outputs.sync_branch }} already exists on origin; skipping."
            echo "exists=true" >> "$GITHUB_OUTPUT"
          else
            echo "exists=false" >> "$GITHUB_OUTPUT"
          fi

      - name: Configure git identity
        if: steps.guard.outputs.exists == 'false'
        run: |
          git config user.name "github-actions[bot]"
          git config user.email "41898282+github-actions[bot]@users.noreply.github.com"

      - name: Create sync branch from <integration-branch>
        if: steps.guard.outputs.exists == 'false'
        run: |
          git fetch origin <integration-branch> <stable-branch>
          git checkout -b "${{ steps.versions.outputs.sync_branch }}" "origin/<integration-branch>"

      - name: Merge the released branch into the sync branch
        if: steps.guard.outputs.exists == 'false'
        run: |
          git merge --no-ff --no-edit \
            -m "Merge <stable-branch> (${{ steps.versions.outputs.release_version }}) into <integration-branch>" \
            "origin/<stable-branch>"

      - name: Set up Python
        if: steps.guard.outputs.exists == 'false'
        uses: actions/setup-python@v5
        with:
          python-version: '3.x'

      - name: Bump the development snapshot version
        if: steps.guard.outputs.exists == 'false'
        env:
          RELEASE_VERSION: ${{ steps.versions.outputs.release_version }}
          NEXT_SNAPSHOT: ${{ steps.versions.outputs.next_snapshot }}
        run: python3 .github/scripts/sync_versions.py

      - name: Commit the version bump
        if: steps.guard.outputs.exists == 'false'
        run: |
          git add pom.xml README.md docs/antora.yml docs/modules/ROOT/pages
          git diff --cached --quiet || git commit -m "Sync ${{ steps.versions.outputs.next_snapshot }}"

      - name: Push sync branch
        if: steps.guard.outputs.exists == 'false'
        run: git push origin "${{ steps.versions.outputs.sync_branch }}"

      - name: Ensure the sync label exists
        if: steps.guard.outputs.exists == 'false'
        env:
          GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}
        run: |
          gh label create sync --description "Sync pull request from a release branch back into <integration-branch>" --color 0e8a16 \
            || gh label list --search sync >/dev/null

      - name: Open pull request into <integration-branch>
        if: steps.guard.outputs.exists == 'false'
        env:
          GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}
        run: |
          gh pr create \
            --base <integration-branch> \
            --head "${{ steps.versions.outputs.sync_branch }}" \
            --title "Sync ${{ steps.versions.outputs.next_snapshot }}" \
            --body "Merges \`<stable-branch>\` back into \`<integration-branch>\` after releasing \`${{ steps.versions.outputs.release_version }}\`, and bumps the development snapshot version to \`${{ steps.versions.outputs.next_snapshot }}\` in \`pom.xml\`, \`README.md\`, and the Antora docs." \
            --label sync
```

Notes specific to this template:

- `<stable-branch>`: the branch releases are published from (e.g. `main`, sometimes `master`) — resolved in Step 1,
  used both in the `if:` guard and the merge source.
- The snapshot bump always assumes a minor-version bump with the patch reset to `0` (e.g. release `1.3.0` → next
  snapshot `1.4.0-SNAPSHOT`) — this matches the conventional gitflow pattern where a release branch is cut from a
  pre-bumped `-SNAPSHOT` on the integration branch. If the target repository does patch-level releases too, flag
  this as an open gap in Step 8 rather than silently guessing which bump the user wants.
- `CHANGELOG.md` is deliberately not touched by this workflow: whatever process turns the release branch's
  `[Unreleased]` section into a dated release section (by hand, or via a skill like this repository's own
  `/iru-release`) already happens before the tag is published, so it arrives on `<integration-branch>` unchanged via
  the merge step above.
- The `permissions:` block is necessary but not sufficient — flag in Step 7 that the repository's Settings → Actions
  → General → "Workflow permissions" must also allow GitHub Actions to create pull requests, or `gh pr create` will
  fail with a permissions error even though the token has the right scopes declared here.

### `sync_versions.py` template

Companion script for `sync.yml`, at `.github/scripts/sync_versions.py`. Reads `RELEASE_VERSION`/`NEXT_SNAPSHOT`
from the environment and rewrites every version reference the merge step doesn't already carry forward. The
`<artifact-id>` and README/Antora regexes below are placeholders — resolve them from Step 1's survey of this
repository's actual `pom.xml` artifact id and README/Antora wording before writing the real file:

```python
#!/usr/bin/env python3
"""Rewrites version references after a release, for the Sync workflow.

Reads RELEASE_VERSION and NEXT_SNAPSHOT from the environment and updates
pom.xml, README.md, docs/antora.yml, and any Antora page that carries the
same "Latest release" / "Latest snapshot" dependency snippets as README.md.

CHANGELOG.md is intentionally left untouched: its release section and fresh
"[Unreleased]" heading are written before the tag is published, and arrive on
<integration-branch> via the merge that precedes this script, not by bumping
a version string.
"""
import os
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


def replace_dependency_snippets(text, release_version, next_snapshot):
    """Replaces the <version> tag following a "Latest release"/"Latest snapshot" marker line."""
    lines = text.splitlines(keepends=True)
    pending = None
    for i, line in enumerate(lines):
        lower = line.strip().lower()
        if lower.startswith("latest release"):
            pending = release_version
        elif lower.startswith("latest snapshot"):
            pending = next_snapshot
        elif pending and "<version>" in line:
            lines[i] = re.sub(r"(<version>)[^<]*(</version>)", rf"\g<1>{pending}\g<2>", line)
            pending = None
    return "".join(lines)


def update_pom(release_version, next_snapshot):
    path = REPO_ROOT / "pom.xml"
    text = path.read_text()
    updated, count = re.subn(
        r"(<artifactId><artifact-id></artifactId>\s*\n\s*<version>)[^<]*(</version>)",
        rf"\g<1>{next_snapshot}\g<2>",
        text,
        count=1,
    )
    if count != 1:
        sys.exit("pom.xml: could not find the <artifact-id> <version> element to bump")
    path.write_text(updated)


def update_readme(release_version, next_snapshot):
    path = REPO_ROOT / "README.md"
    text = path.read_text()
    # Adjust these two row-label regexes to match this repository's actual README table wording
    # (resolved in Step 1) if it differs from setup-readme's default "Current development version" /
    # "Latest release" labels.
    text = re.sub(
        r"(\| Current development version \| `)[^`]*(` \|)",
        rf"\g<1>{next_snapshot}\g<2>",
        text,
    )
    text = re.sub(
        r"(\| Latest release[^|]*\| `)[^`]*(` \|)",
        rf"\g<1>{release_version}\g<2>",
        text,
    )
    text = replace_dependency_snippets(text, release_version, next_snapshot)
    path.write_text(text)


def update_antora_component_version(release_version):
    path = REPO_ROOT / "docs" / "antora.yml"
    if not path.exists():
        return
    text = path.read_text()
    updated, count = re.subn(r"(?m)^version:.*$", f"version: {release_version}", text, count=1)
    if count == 1:
        path.write_text(updated)


def update_antora_pages(release_version, next_snapshot):
    pages_dir = REPO_ROOT / "docs" / "modules" / "ROOT" / "pages"
    if not pages_dir.exists():
        return
    for path in sorted(pages_dir.glob("*.adoc")):
        text = path.read_text()
        if "<version>" not in text:
            continue
        updated = replace_dependency_snippets(text, release_version, next_snapshot)
        if updated != text:
            path.write_text(updated)


def main():
    release_version = os.environ["RELEASE_VERSION"]
    next_snapshot = os.environ["NEXT_SNAPSHOT"]

    update_pom(release_version, next_snapshot)
    update_readme(release_version, next_snapshot)
    update_antora_component_version(release_version)
    update_antora_pages(release_version, next_snapshot)


if __name__ == "__main__":
    main()
```

Notes specific to this template:

- `<artifact-id>`: the Maven `artifactId` from Step 1's survey — the `update_pom` regex anchors on the
  `<artifactId>` element immediately preceding the `<version>` to bump, so it never touches a dependency or plugin
  version elsewhere in `pom.xml`.
- `update_antora_component_version` and `update_antora_pages` both no-op cleanly if `docs/antora.yml` or
  `docs/modules/ROOT/pages` don't exist, so this script is safe to write even before the `iru-setup-antora` skill has
  run — no need to gate its creation on Antora being present.
- The Antora component version (`docs/antora.yml`) is intentionally set to the release version, not the next
  snapshot — matching the convention (used by this repository's own `/iru-release`-style workflow, if present) that
  the Antora component version tracks the latest actual release.

## Step 4 — Fill the templates

Substitute every placeholder from Step 1's survey. If any required fact wasn't resolvable there (e.g. no signing
profile exists yet, or no Central/Nexus plugin is configured), don't invent a value — ask the user or note it as an
open gap in Step 8's report instead of silently guessing.

## Step 5 — Handle missing supporting files

Before writing the workflows, make sure the files they depend on exist:

- **`docs/antora.yml` / `docs/antora-playbook.yml` missing**: recommend running the `iru-setup-antora` skill now; the
  Antora build step in the workflow will fail without them.
- **`sonar-maven-plugin`/`sonar.*` properties missing from `pom.xml`**: ask the user for `sonar.organization` (if
  using SonarCloud), `sonar.projectKey`, and `sonar.host.url` (default `https://sonarcloud.io` unless they run
  self-hosted SonarQube), then add them directly to `pom.xml`. Look up the current latest
  `org.sonarsource.scanner.maven:sonar-maven-plugin` version from Maven Central rather than hardcoding a version
  here — this plugin releases fairly often. Add to `<properties>`:

  ```xml
  <sonar.organization><org></sonar.organization>
  <sonar.projectKey><project-key></sonar.projectKey>
  <sonar.host.url><sonar-host-url></sonar.host.url>
  <sonar.coverage.jacoco.xmlReportPaths>${project.build.directory}/site/jacoco/jacoco.xml</sonar.coverage.jacoco.xmlReportPaths>
  ```

  and add the plugin itself to `<build><plugins>` (it needs no `<executions>` — the workflow invokes its
  `sonar:sonar` goal directly, not through a lifecycle phase):

  ```xml
  <!-- SonarCloud analysis -->
  <plugin>
    <groupId>org.sonarsource.scanner.maven</groupId>
    <artifactId>sonar-maven-plugin</artifactId>
    <version><sonar-maven-plugin-version></version>
  </plugin>
  ```

  `sonar.coverage.jacoco.xmlReportPaths` must point at wherever the `jacoco-maven-plugin`'s `report` execution
  writes its XML (confirm the actual path already configured in `pom.xml` — `${project.build.directory}/site/jacoco/jacoco.xml`
  is this house pattern's default) so the scan doesn't rely on autodetection finding it. `sonar.java.source` needs
  no explicit setting: the plugin reads `maven.compiler.source`/`maven.compiler.release` automatically, so the
  scan always tracks whatever Java version `pom.xml` already targets.

- **Maven settings file (`<settings-file>` from Step 3) missing**: create it, matching the `server-id` resolved in
  Step 1:

  ```xml
  <?xml version="1.0" encoding="UTF-8"?>
  <settings>
    <servers>
      <server>
        <id><publishing-server-id></id>
        <username>${env.OSSRH_USERNAME}</username>
        <password>${env.OSSRH_PASSWORD}</password>
      </server>
    </servers>

    <profiles>
      <profile>
        <id>ossrh</id>
        <activation>
          <activeByDefault>true</activeByDefault>
        </activation>
        <properties>
          <gpg.executable>gpg</gpg.executable>
          <gpg.keyname>${env.SIGNING_KEY_ID}</gpg.keyname>
          <gpg.passphrase>${env.SIGNING_PASSWORD}</gpg.passphrase>
        </properties>
      </profile>
    </profiles>

    <pluginGroups>
      <pluginGroup>org.sonatype.plugins</pluginGroup>
    </pluginGroups>
  </settings>
  ```

- **`.gitignore` missing an entry for `/docs/build/` and `/target/`**: add them if absent, so the build output the
  workflow generates never gets committed by accident.

## Step 6 — Write the workflow files

Write (or, per Step 2, carefully update) `.github/workflows/develop.yml`, `.github/workflows/main.yml`, and
`.github/workflows/sync.yml` with the filled-in templates from Step 4, plus `.github/scripts/sync_versions.py` and
any other supporting files created in Step 5. `sync.yml` and `sync_versions.py` are written together — never write
one without the other.

## Step 7 — List required repository secrets and settings

Report the secrets that must exist under the target repository's Settings → Secrets and variables → Actions —
this skill cannot create them itself:

| Secret | Purpose |
|---|---|
| `SONAR_TOKEN` | Auth token for the SonarQube/SonarCloud scan, passed as `-Dsonar.token` to `mvn sonar:sonar` |
| `OSSRH_USERNAME` / `OSSRH_PASSWORD` | Maven Central (Sonatype) publishing credentials |
| `SIGNING_KEY` / `SIGNING_KEY_ID` / `SIGNING_PASSWORD` | GPG private key, its key id, and its passphrase, for artifact signing |
| `SONATYPE_STAGING_PROFILE_ID` | Only needed with the legacy `nexus-staging-maven-plugin`; if the target repo uses `central-publishing-maven-plugin` this secret is unused and can be left unset or removed from the deploy step |

`GITHUB_TOKEN` needs no setup — GitHub Actions provides it automatically. `sync.yml` uses it too (no extra secret),
but also needs the repository setting under Settings → Actions → General → "Workflow permissions" set to allow
GitHub Actions to create pull requests — the `permissions: contents: write / pull-requests: write` block in the
workflow itself is not sufficient on its own. Call this out explicitly in Step 8's report, since it's easy to miss
and `sync.yml` will fail silently-ish (a `gh pr create` permissions error) on the very first real release without it.

## Step 8 — Report and warn

Summarize what happened: whether the workflows (and `sync_versions.py`) were created fresh, updated in place, or
left untouched (Step 2's stop path); whether `pom.xml`'s `sonar-maven-plugin`/`sonar.*` properties and the settings
XML were added versus already present (and whether a stale `sonar-project.properties` was flagged for removal);
and any open gaps noted in Steps 1/4 (missing Central plugin, missing signing profile, missing Antora setup,
README/Antora wording that didn't match `sync_versions.py`'s default regexes, patch-level releases the minor-bump
default doesn't handle, etc.).

Finish with an explicit warning: **the user must review the generated (or updated) workflow files before relying on
them.** Branch names, profile ids, the Java version, and the Maven Central publishing setup were inferred from this
repository's current state and may need correction — and because this pipeline handles signing keys and publishes
artifacts publicly, a bad assumption here has real consequences. Recommend a dry run (e.g. pushing to a throwaway
branch with the trigger temporarily adjusted, or manually triggering via `workflow_dispatch` if added) before
trusting it on the real `develop`/iru-release flow. For `sync.yml` specifically, recommend verifying it against a real
(or test) release before relying on it — confirm the computed next-snapshot version and the exact files it
touches match expectations, and confirm the "Workflow permissions" repository setting from Step 7 is enabled.
