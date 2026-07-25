---
name: iru-setup-java-library
description: Generate a `pom.xml` at the repository root for a new Java library — asks for groupId, artifactId, version (default `1.0.0-SNAPSHOT`), the library's base Java package, license, developer name/email/organizationUrl, and whether to wire up a SonarQube/SonarCloud scan via the `sonar-maven-plugin`, then infers repository name/URL/SCM/inception year from the current git repository — and scaffolds the standard Maven source layout (`src/main/java/<package>`, `src/main/resources/<package>`, `src/test/java/<package>`) for that package. Invoke as `/iru-setup-java-library`. Ships with an explicit example `pom.xml` (derived from a real repository's pom, genericized) embedded in this skill file: test-only dependencies limited to junit-jupiter, junit-platform-launcher, mockito-core, and mockito-junit-jupiter, the same `sign`/`build-extras` profiles with the same plugins and versions, and a `groovy-maven-plugin` build-info step whose output path is rewired to the given library package. If `pom.xml` already exists, asks the user whether to stop or continue to regenerate it from the example. Accepts pre-resolved inputs via `args` (`key: value` lines) so an orchestrating skill like `iru-setup-java-library-repository` can supply them without re-prompting. Use whenever a new Java/Maven library repository needs its `pom.xml` and base folder structure bootstrapped from this house template, instead of hand-writing it.
model: haiku
---

# Setup Java Library

Generate `pom.xml` at the repository root for a new Java library, using an explicit example template embedded in
this skill (Step 4) that is genericized from a real repository's `pom.xml` — same test dependencies, same
`sign`/`build-extras` profiles, same build/reporting plugins — with only the project-identity fields (groupId,
artifactId, version, package, license, developer, repository info) filled in per repository. It also scaffolds the
standard Maven source folders for the library's base package.

## Step 0 — Check for pre-supplied parameters

This skill can be invoked stand-alone (`/iru-setup-java-library`) or as a step inside another skill, such as
`iru-setup-java-library-repository`, which resolves these same inputs itself (e.g. after asking the user once, or
applying its own defaults) and passes them through `args` as `key: value` lines, one per line, e.g.:

```
groupId: com.example
artifactId: my-library
package: com.example.mylibrary
developer-name: Jane Doe
developer-email: jane@example.com
organization-url: https://github.com/jane
license: Apache License 2.0
```

Parse any such lines from `args` now. Every field found here is resolved — skip asking about it in Step 2. Only
fields genuinely missing from `args` (including `version`, which callers typically leave for this skill's own
default) still need a question. If `args` is absent or doesn't look like this format, treat everything as unset
and ask normally.

## Step 1 — Check for an existing `pom.xml`

- **No `pom.xml` at the repository root**: skip straight to Step 2.
- **`pom.xml` already exists**: warn the user it will be replaced, then use `AskUserQuestion` to ask whether to
  stop (leave the existing file untouched) or continue (regenerate it from this skill's example).
  - **Stop**: report that `pom.xml` already exists and end here — make no changes.
  - **Continue**: before asking the questions in Step 2, read the existing `pom.xml` and use its current
    `groupId`, `artifactId`, `version`, `description`, license, developer, and main source package as the
    *defaults* you offer the user for each question that Step 0 didn't already resolve — a value supplied via
    `args` always wins over a value merely found in the existing file, since it reflects what the caller (or the
    user, just now) explicitly asked for. Tell the user up front that regenerating from the template replaces the
    whole file — any dependencies or plugins beyond the four test dependencies and the standard profiles/plugins
    in Step 4's example will be lost unless the user re-adds them afterward (call this out again in Step 7).

## Step 2 — Collect the required inputs

For any field Step 0 already resolved from `args`, use that value directly and don't ask about it again. For
everything else, ask the user directly (plain conversation, pre-filling defaults found in Step 1 if updating an
existing file):

- **groupId** — e.g. `com.example`.
- **artifactId** — e.g. `my-library`.
- **version** — default `1.0.0-SNAPSHOT` if the user doesn't give one.
- **Library package** — the base Java package for the library's main source (e.g. `com.example.mylibrary`). This
  need not equal `groupId`; ask for it explicitly. Used in Step 3's `src/main/java/<package>` layout and in the
  `groovy-maven-plugin` step of Step 5.
- **Developer name**, **developer email**, **organizationUrl** — populate the `<developers>` block.

Then, unless Step 0 already resolved a `license` value from `args`, ask about the **license** with
`AskUserQuestion` (small bounded choice, so a proper multiple-choice question fits, unlike the free-text fields
above):

- Apache License 2.0 (recommended) — `The Apache License, Version 2.0` /
  `http://www.apache.org/licenses/LICENSE-2.0.txt`
- MIT License — `The MIT License` / `https://opensource.org/license/mit`
- No license (proprietary / all rights reserved) — omit the `<licenses>` block entirely
- Other — ask for the license's display name and URL directly afterward

If a license is chosen (anything but "No license"), remind the user in Step 7 to also add a matching `LICENSE`
file at the repository root if one doesn't exist yet — the `iru-check-license` skill can verify/backfill file headers
against it once it's there.

Then use `AskUserQuestion` to ask whether to wire up a SonarQube/SonarCloud scan (this house pattern runs it via
the `sonar-maven-plugin`, invoked as `mvn sonar:sonar` — typically from the CI workflow the
`iru-setup-java-github-workflows` skill generates, so it's worth setting up here even if that workflow comes
later):

- Yes, SonarCloud (recommended) — ask for the `sonar.organization` key, offering `<owner>-github` (the owner parsed
  in Step 3) as the suggested default, since that's the key SonarCloud assigns by default when an organization is
  created by importing from GitHub. Default `sonar.projectKey` to `<owner>_<repo>` (also from Step 3) and
  `sonar.host.url` to `https://sonarcloud.io`, confirming both with the user rather than assuming silently.
- Yes, self-hosted SonarQube — ask for `sonar.host.url` directly (no sensible default), plus `sonar.organization`
  only if that server has organizations enabled, and `sonar.projectKey`.
- No — omit the `sonar.*` properties and the `sonar-maven-plugin` entry from Step 4's template entirely.

## Step 3 — Infer repository information

Don't ask for these — derive them from the current repository:

- **Owner/repo and host**: parse `git remote get-url origin` (handles both `git@host:owner/repo.git` and
  `https://host/owner/repo.git` forms). Use the parsed host/owner/repo for:
  - `<url>`: `https://<host>/<owner>/<repo>`
  - `<scm><connection>`/`<developerConnection>`: `scm:git@<host>:<owner>/<repo>.git`
  - `<scm><url>`: `git@<host>:<owner>/<repo>.git`
  If there's no `origin` remote yet, ask the user for the intended repository URL instead of leaving it blank.
- **Description**: if the `gh` CLI is available and authenticated, try `gh repo view --json description -q
  .description`; if that yields nothing (no remote repo yet, `gh` unavailable, or empty description), ask the user
  for a one-line description instead.
- **Inception year**: `git log --reverse --format=%ad --date=format:%Y | head -1` for the first commit's year;
  fall back to the current year if the repository has no commits yet.

## Step 4 — Reference template

This is a genericized example — based on a real Java/Maven library's actual `pom.xml` — kept verbatim below except
for the placeholders. Preserve every dependency, profile, plugin, version, and reporting entry exactly as shown;
only substitute `<placeholder>` values using Steps 2–3.

```xml
<project xmlns="http://maven.apache.org/POM/4.0.0"
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:schemaLocation="http://maven.apache.org/POM/4.0.0 http://maven.apache.org/xsd/maven-4.0.0.xsd">
  <modelVersion>4.0.0</modelVersion>

  <groupId><group-id></groupId>
  <artifactId><artifact-id></artifactId>
  <version><version></version>
  <packaging>jar</packaging>

  <name>${project.groupId}:${project.artifactId}</name>
  <description><description></description>
  <url><repository-url></url>
  <inceptionYear><inception-year></inceptionYear>

  <!-- Omit this whole <licenses> block if the user chose "No license" in Step 2 -->
  <licenses>
    <license>
      <name><license-name></name>
      <url><license-url></url>
    </license>
  </licenses>
  <developers>
    <developer>
      <name><developer-name></name>
      <email><developer-email></email>
      <organizationUrl><organization-url></organizationUrl>
    </developer>
  </developers>
  <scm>
    <connection>scm:git@<host>:<owner>/<repo>.git</connection>
    <developerConnection>scm:git@<host>:<owner>/<repo>.git</developerConnection>
    <url>git@<host>:<owner>/<repo>.git</url>
  </scm>

  <dependencies>
    <dependency>
      <groupId>org.junit.jupiter</groupId>
      <artifactId>junit-jupiter</artifactId>
      <version>5.13.4</version>
      <scope>test</scope>
    </dependency>
    <dependency>
      <groupId>org.junit.platform</groupId>
      <artifactId>junit-platform-launcher</artifactId>
      <version>1.14.3</version>
      <scope>test</scope>
    </dependency>
    <dependency>
      <groupId>org.mockito</groupId>
      <artifactId>mockito-core</artifactId>
      <version>5.19.0</version>
      <scope>test</scope>
    </dependency>
    <dependency>
      <groupId>org.mockito</groupId>
      <artifactId>mockito-junit-jupiter</artifactId>
      <version>5.19.0</version>
      <scope>test</scope>
    </dependency>
  </dependencies>

  <properties>
    <project.build.sourceEncoding>UTF-8</project.build.sourceEncoding>
    <maven.compiler.source>17</maven.compiler.source>
    <maven.compiler.target>17</maven.compiler.target>
    <github.global.server>github</github.global.server>
    <github.global.oauth2Token>${env.GITHUB_OAUTH_TOKEN}</github.global.oauth2Token>
    <!-- Omit these four sonar.* properties if the user opted out of SonarCloud/SonarQube in Step 2 -->
    <sonar.organization><sonar-organization></sonar.organization>
    <sonar.projectKey><sonar-project-key></sonar.projectKey>
    <sonar.host.url><sonar-host-url></sonar.host.url>
    <sonar.coverage.jacoco.xmlReportPaths>${project.build.directory}/site/jacoco/jacoco.xml</sonar.coverage.jacoco.xmlReportPaths>
  </properties>

  <profiles>
    <profile>
      <id>sign</id>
      <build>
        <plugins>
          <plugin>
            <groupId>org.apache.maven.plugins</groupId>
            <artifactId>maven-gpg-plugin</artifactId>
            <version>3.2.8</version>
            <executions>
              <execution>
                <id>sign-artifacts</id>
                <phase>verify</phase>
                <goals>
                  <goal>sign</goal>
                </goals>
              </execution>
            </executions>
          </plugin>
        </plugins>
      </build>
    </profile>
    <profile>
      <id>build-extras</id>
      <activation>
        <activeByDefault>true</activeByDefault>
      </activation>
      <build>
        <plugins>
          <plugin>
            <groupId>org.apache.maven.plugins</groupId>
            <artifactId>maven-source-plugin</artifactId>
            <version>3.3.1</version>
            <executions>
              <execution>
                <id>attach-sources</id>
                <goals>
                  <goal>jar-no-fork</goal>
                </goals>
              </execution>
            </executions>
          </plugin>
          <plugin>
            <groupId>org.apache.maven.plugins</groupId>
            <artifactId>maven-javadoc-plugin</artifactId>
            <version>3.11.3</version>
            <executions>
              <execution>
                <id>attach-javadocs</id>
                <goals>
                  <goal>jar</goal>
                </goals>
              </execution>
            </executions>
          </plugin>
        </plugins>
      </build>
    </profile>
  </profiles>

  <!-- default profile -->
  <build>
    <plugins>
      <!-- unit tests plugins -->
      <plugin>
        <groupId>org.apache.maven.plugins</groupId>
        <artifactId>maven-surefire-plugin</artifactId>
        <version>3.5.4</version>
      </plugin>
      <plugin>
        <groupId>org.apache.maven.plugins</groupId>
        <artifactId>maven-failsafe-plugin</artifactId>
        <version>3.5.4</version>
      </plugin>
      <!-- code coverage plugin -->
      <plugin>
        <groupId>org.jacoco</groupId>
        <artifactId>jacoco-maven-plugin</artifactId>
        <version>0.8.13</version>
        <executions>
          <execution>
            <id>jacoco-initialize</id>
            <goals>
              <goal>prepare-agent</goal>
            </goals>
          </execution>
          <execution>
            <id>jacoco-site</id>
            <phase>package</phase>
            <goals>
              <goal>report</goal>
            </goals>
          </execution>
        </executions>
      </plugin>
      <!-- code quality plugins -->
      <plugin>
        <groupId>com.github.spotbugs</groupId>
        <artifactId>spotbugs-maven-plugin</artifactId>
        <version>4.9.6.0</version>
      </plugin>
      <!-- maven site -->
      <plugin>
        <groupId>org.apache.maven.plugins</groupId>
        <artifactId>maven-site-plugin</artifactId>
        <version>3.21.0</version>
      </plugin>
      <!-- save build info into generated package -->
      <plugin>
        <groupId>org.codehaus.gmaven</groupId>
        <artifactId>groovy-maven-plugin</artifactId>
        <version>2.1.1</version>
        <executions>
          <execution>
            <phase>validate</phase>
            <goals>
              <goal>execute</goal>
            </goals>
            <configuration>
              <source>
                <![CDATA[
                import java.util.Date
                import java.util.Properties
                import java.text.SimpleDateFormat
                import java.io.File
                import java.io.FileWriter

                println("Saving build info...")
                def dateFormatter = new SimpleDateFormat("yy-MM-dd HH:mm:ss")
                def buildTimestamp = dateFormatter.format(new Date())

                String groupId = "${project.groupId}"
                String artifactId = "${project.artifactId}"
                String version = "${project.version}"

                //JENKINS
                def buildNumber = System.getenv("BUILD_NUMBER")
                def commit = System.getenv("GIT_COMMIT")
                def branch = System.getenv("GIT_BRANCH")

                //TRAVIS CI
                if (buildNumber == null) {
                  buildNumber = System.getenv("TRAVIS_BUILD_NUMBER")
                }
                if (commit == null) {
                  commit = System.getenv("TRAVIS_COMMIT")
                }
                if (branch == null) {
                  branch = System.getenv("TRAVIS_BRANCH")
                }

                //GITLAB
                if (buildNumber == null) {
                  buildNumber = System.getenv("CI_JOB_ID")
                }
                if (commit == null) {
                  commit = System.getenv("CI_COMMIT_SHA")
                }
                if (branch == null) {
                  branch = System.getenv("CI_COMMIT_REF_NAME")
                }

                def props = new Properties()
                props.setProperty("BUILD_TIMESTAMP", buildTimestamp)
                props.setProperty("GROUP_ID", groupId)
                props.setProperty("ARTIFACT_ID", artifactId)
                props.setProperty("VERSION", version)
                if (buildNumber != null) {
                  props.setProperty("BUILD_NUMBER", buildNumber)
                }
                if (commit != null) {
                  props.setProperty("COMMIT", commit)
                }
                if (branch != null) {
                  props.setProperty("BRANCH", branch)
                }

                File dir = new File("src/main/resources/<library-package-path>")
                dir.mkdirs()
                File file = new File(dir, "build-info.properties")
                FileWriter writer = new FileWriter(file)
                props.store(writer, null)
                writer.close()
                println("Build info saved at " + file)
                ]]>
              </source>
            </configuration>
          </execution>
        </executions>
      </plugin>
      <!-- deploys artifact to snapshots repository -->
      <plugin>
        <groupId>org.sonatype.central</groupId>
        <artifactId>central-publishing-maven-plugin</artifactId>
        <version>0.8.0</version>
        <extensions>true</extensions>
        <configuration>
          <publishingServerId>central</publishingServerId>
          <autoPublish>true</autoPublish>
        </configuration>
      </plugin>
      <!-- SonarCloud analysis; omit this plugin entry if the user opted out in Step 2 -->
      <plugin>
        <groupId>org.sonarsource.scanner.maven</groupId>
        <artifactId>sonar-maven-plugin</artifactId>
        <version><sonar-maven-plugin-version></version>
      </plugin>
    </plugins>
  </build>

  <reporting>
    <plugins>
      <!-- add javadoc report into site -->
      <plugin>
        <groupId>org.apache.maven.plugins</groupId>
        <artifactId>maven-javadoc-plugin</artifactId>
        <version>3.11.3</version>
        <configuration>
          <show>private</show>
          <nohelp>true</nohelp>
          <detectJavaApiLink>true</detectJavaApiLink>
        </configuration>
      </plugin>
      <!-- adds unit tests report into site -->
      <plugin>
        <groupId>org.apache.maven.plugins</groupId>
        <artifactId>maven-surefire-report-plugin</artifactId>
        <version>3.5.4</version>
      </plugin>
      <!-- adds code coverage report into site -->
      <plugin>
        <groupId>org.jacoco</groupId>
        <artifactId>jacoco-maven-plugin</artifactId>
        <version>0.8.13</version>
      </plugin>
      <!-- adds code quality reports into site -->
      <plugin>
        <groupId>org.apache.maven.plugins</groupId>
        <artifactId>maven-checkstyle-plugin</artifactId>
        <version>3.6.0</version>
        <configuration>
          <configLocation>checkstyle.xml</configLocation>
        </configuration>
        <reportSets>
          <reportSet>
            <reports>
              <report>checkstyle</report>
            </reports>
          </reportSet>
        </reportSets>
      </plugin>

      <plugin>
        <groupId>com.github.spotbugs</groupId>
        <artifactId>spotbugs-maven-plugin</artifactId>
        <version>4.9.6.0</version>
      </plugin>
      <plugin>
        <groupId>org.apache.maven.plugins</groupId>
        <artifactId>maven-pmd-plugin</artifactId>
        <version>3.27.0</version>
      </plugin>
      <!--
      adds cross reference into source for easier navigation of source in
      reports
      -->
      <plugin>
        <groupId>org.apache.maven.plugins</groupId>
        <artifactId>maven-jxr-plugin</artifactId>
        <version>3.6.0</version>
      </plugin>
    </plugins>
  </reporting>
</project>
```

Note on `<library-package-path>`: this is the library package from Step 2 with dots turned into slashes (e.g.
`com.example.mylibrary` → `com/example/mylibrary`) — it must match wherever the library's own code will later read
`build-info.properties` as a classpath resource, the same way this repository's `BuildInfo` class reads it from
`com/irurueta/hermes`.

## Step 5 — Fill the template

Substitute every placeholder using Steps 2–3:

- `<group-id>`, `<artifact-id>`, `<version>`, `<description>`, `<developer-name>`, `<developer-email>`,
  `<organization-url>` — straight from Step 2's answers.
- `<repository-url>`, `<host>`, `<owner>`, `<repo>`, `<inception-year>` — straight from Step 3's inference.
- `<license-name>` / `<license-url>` — from Step 2's license choice; if "No license" was chosen, remove the entire
  `<licenses>...</licenses>` block rather than leaving empty placeholder tags.
- `<library-package-path>` — the library package from Step 2, with `.` replaced by `/`.
- `<sonar-organization>` / `<sonar-project-key>` / `<sonar-host-url>` — from Step 2's SonarCloud/SonarQube answer;
  if the user opted out, remove the four `sonar.*` properties and the `sonar-maven-plugin` plugin entry entirely
  rather than leaving empty placeholder tags.
- `<sonar-maven-plugin-version>` — only needed if SonarCloud/SonarQube was opted into: look up the current latest
  `org.sonarsource.scanner.maven:sonar-maven-plugin` version (e.g. via Maven Central's search API) rather than
  hardcoding a version here, since this plugin releases fairly often.

Every other line (dependencies, properties, both profiles, all build/reporting plugins and their versions) is
copied verbatim from Step 4 — this skill does not add, remove, or re-version any dependency or plugin beyond what
the template already shows.

## Step 6 — Write `pom.xml`

Write the filled-in template to `pom.xml` at the repository root (creating it fresh, or replacing it entirely per
the user's "continue" choice in Step 1).

## Step 7 — Scaffold the source folders

Using the same library package from Step 2 (with `.` replaced by `/`, the same `<library-package-path>` computed
in Step 5), create the standard Maven layout if not already present:

```
src/main/java/<library-package-path>/
src/main/resources/<library-package-path>/
src/test/java/<library-package-path>/
```

- Create each directory only if it doesn't already exist — don't touch a directory (or any files already inside
  it) that's already there, whether from a prior run of this skill or from existing library code.
- Git doesn't track empty directories, so these three will disappear from `git status`/a commit until a file
  exists inside them. Don't create placeholder files (e.g. `.gitkeep`) unless the user asks for that — note this
  in Step 8 instead, since the first source/test/resource file the user adds will make the directory stick.
- `src/main/resources/<library-package-path>` is exactly where the `groovy-maven-plugin` step in Step 4's template
  writes `build-info.properties` at build time — this folder existing ahead of time isn't required for that (the
  script calls `mkdir()`), but creating it now keeps the layout visibly complete from the start.

## Step 8 — Report and warn

Summarize what was generated: the resolved groupId/artifactId/version, the license chosen (or "none"), the
SonarCloud/SonarQube choice from Step 2 (and its resolved `sonar.organization`/`sonar.projectKey`/`sonar.host.url`,
or that it was omitted), the repository info inferred in Step 3, and which of the three source folders from Step 7
were created versus already present. Remind the user that empty directories won't show up in `git status` until
they contain a file. If Step 1's existing file was replaced, explicitly list what could have been lost — any
dependency or plugin that isn't one of the four test dependencies or the standard profiles/plugins in Step 4's
template — and tell the user to check `git diff pom.xml` for anything they need to re-add.

Finish with an explicit warning: **the user must review the generated `pom.xml` before building or committing.**
Confirm the license choice matches an actual `LICENSE` file (or that none is intended), that the inferred
repository URL/SCM values are correct, and that `mvn validate` (or `mvn compile`) succeeds before relying on this
file. If a license was chosen and no `LICENSE` file exists yet, suggest the `iru-check-license` skill to generate one
and backfill source headers. If SonarCloud/SonarQube was wired up, note that a `SONAR_TOKEN` repository secret and
an actual SonarCloud/SonarQube project matching `sonar.projectKey` still need to exist before `mvn sonar:sonar`
(typically run from the `iru-setup-java-github-workflows` skill's generated CI) will succeed.
