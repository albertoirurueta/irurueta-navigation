---
name: iru-setup-java-springboot-pom
description: Generate the Maven reactor for a DDD/hexagonal Spring Boot service — the root `pom.xml` (the single place every dependency and plugin version is declared) plus one `pom.xml` per module (`domain`, `application`, `infrastructure/database/<engine>`, `infrastructure/configuration`, `infrastructure/clients/<name>`, `infrastructure/producers`, `infrastructure/metrics`, `api/rest-server`, `api/grpc-server`, `api/graphql-server`, `api/consumers`, `boot`, `coverage`). Enforces Java 21 as a minimum via `maven.compiler.release`, `java.version`, and a `maven-enforcer-plugin` `requireJavaVersion` rule, and enforces the hexagonal dependency direction per module with `banned-dependencies`. Wires the full report/quality plugin set — Javadoc, Checkstyle, PMD, SpotBugs, Surefire, Failsafe, JaCoCo (per module plus a `report-aggregate` in `coverage`), JXR, Maven Site, `groovy-maven-plugin` for `build-info.properties`, and `sonar-maven-plugin` — with generated sources excluded from every analysis. Reads its inputs from the `springboot-stack.yml` manifest and from the reference pom fetched from the Spring Initializr API, so starter artifact names and BOM imports come from the API rather than from memory. Invoke as `/iru-setup-java-springboot-pom` (it reads `springboot-stack.yml` from the repository root), or with `args` (`stack-file:` / `initializr-pom:` lines) when called from `iru-setup-java-springboot`. Use whenever a Spring Boot service's multi-module Maven reactor needs generating or regenerating from the recorded stack, instead of hand-writing a dozen interdependent poms.
model: sonnet
---

# Setup Java Spring Boot Poms

Generate the whole Maven reactor for a hexagonal Spring Boot service. Two rules govern everything below:

1. **Every version lives in the root `pom.xml`.** Module poms declare `<groupId>`/`<artifactId>` only, with no
   `<version>`, resolving through the root's `<dependencyManagement>` and `<pluginManagement>`. A version in a
   module pom is a bug, not a shortcut.
2. **Starter and driver artifact names come from the Initializr reference pom, never from memory.** They change
   across Boot major versions — in Boot 4.x `spring-boot-starter-web` became `spring-boot-starter-webmvc`, the
   Testcontainers modules became `testcontainers-<engine>`, and each starter gained a
   `spring-boot-starter-<x>-test` companion. Copy what the reference pom actually says.

## Step 0 — Resolve inputs

Parse `args` for `key: value` lines:

```
stack-file: springboot-stack.yml
initializr-pom: /path/to/scratchpad/initializr-pom.xml
```

- **`stack-file`** — defaults to `springboot-stack.yml` at the repository root. Read it in full; it carries the
  project identity, the concurrency model, and every technology choice. If it doesn't exist, stop and tell the
  user to run `/iru-setup-java-springboot` first — this skill deliberately doesn't re-interview.
- **`initializr-pom`** — the reference pom. If it wasn't supplied, fetch it now yourself from the ids in the
  manifest's `initializr.dependencies`:

  ```bash
  curl -sG https://start.spring.io/pom.xml \
    --data-urlencode 'bootVersion=<project.springBootVersion>' \
    -d type=maven-build -d javaVersion=<project.javaVersion> \
    -d groupId=<project.groupId> -d artifactId=<project.artifactId> \
    -d packageName=<project.basePackage> \
    -d 'dependencies=<comma-separated ids>' -o <scratchpad>/initializr-pom.xml
  ```

  Confirm the response is XML, not a plain-text error, before using it.

From the reference pom, extract into a working table you'll consult for every module below:

- every starter/driver `groupId:artifactId` and its scope (compile / runtime / test / optional);
- every `<dependencyManagement>` BOM import plus its version property (e.g. `spring-cloud.version`,
  `spring-ai.version`) — these go into the root pom verbatim;
- any build plugin Initializr added and whether the Boot parent already manages its version;
- the `annotationProcessorPaths` entries.

Then classify every extracted dependency to the module that should own it, using the table in Step 2. A dependency
you can't confidently place goes in the `boot` module (which depends on everything and where a misplacement is
harmless) and gets reported as unplaced in Step 8 — never drop it silently.

If `pom.xml` already exists at the repository root, use `AskUserQuestion` to ask whether to stop or to regenerate.
On regenerate, keep any dependency, plugin, or profile the existing pom has that this skill's template doesn't,
and list those in Step 8 so the user can confirm each one survived.

## Step 1 — Root `pom.xml`

Substitute `<placeholders>` from the manifest. Include only the `<module>` entries for modules the manifest's
`modules:` list actually names.

```xml
<project xmlns="http://maven.apache.org/POM/4.0.0"
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:schemaLocation="http://maven.apache.org/POM/4.0.0 http://maven.apache.org/xsd/maven-4.0.0.xsd">
  <modelVersion>4.0.0</modelVersion>

  <parent>
    <groupId>org.springframework.boot</groupId>
    <artifactId>spring-boot-starter-parent</artifactId>
    <version><spring-boot-version></version>
    <relativePath/>
  </parent>

  <groupId><group-id></groupId>
  <artifactId><artifact-id>-parent</artifactId>
  <version><version></version>
  <packaging>pom</packaging>

  <name>${project.groupId}:${project.artifactId}</name>
  <description><description></description>
  <url>https://<host>/<owner>/<repo></url>
  <inceptionYear><inception-year></inceptionYear>

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

  <modules>
    <module>domain</module>
    <module>application</module>
    <module>infrastructure</module>
    <module>api</module>
    <module>boot</module>
    <module>coverage</module>
  </modules>

  <properties>
    <project.build.sourceEncoding>UTF-8</project.build.sourceEncoding>
    <!-- Java 21 is this catalog's enforced minimum; all three must agree. -->
    <java.version><java-version></java.version>
    <maven.compiler.release><java-version></maven.compiler.release>
    <maven.compiler.source><java-version></maven.compiler.source>
    <maven.compiler.target><java-version></maven.compiler.target>

    <!-- BOM versions copied verbatim from the Initializr reference pom; omit any not present there. -->
    <spring-cloud.version><from-reference-pom></spring-cloud.version>
    <spring-ai.version><from-reference-pom></spring-ai.version>

    <!-- Dependencies with no Initializr id and no Boot-managed version. Omit any the stack doesn't use. -->
    <mapstruct.version><latest></mapstruct.version>
    <lombok-mapstruct-binding.version><latest></lombok-mapstruct-binding.version>
    <caffeine.version><latest></caffeine.version>
    <mongock.version><latest></mongock.version>
    <liquibase-mongodb.version><latest></liquibase-mongodb.version>
    <liquibase-couchbase.version><latest></liquibase-couchbase.version>
    <liquibase-neo4j.version><latest></liquibase-neo4j.version>
    <spring-cloud-aws.version><latest></spring-cloud-aws.version>
    <archunit.version><latest></archunit.version>

    <!-- Plugin versions. Pinned explicitly for reproducible builds even where the Boot parent also manages
         them — that override is deliberate and shared with the rest of this catalog's Java skills. -->
    <maven-enforcer-plugin.version>3.5.0</maven-enforcer-plugin.version>
    <maven-surefire-plugin.version>3.5.4</maven-surefire-plugin.version>
    <maven-failsafe-plugin.version>3.5.4</maven-failsafe-plugin.version>
    <maven-javadoc-plugin.version>3.11.3</maven-javadoc-plugin.version>
    <maven-checkstyle-plugin.version>3.6.0</maven-checkstyle-plugin.version>
    <maven-pmd-plugin.version>3.27.0</maven-pmd-plugin.version>
    <spotbugs-maven-plugin.version>4.9.6.0</spotbugs-maven-plugin.version>
    <jacoco-maven-plugin.version>0.8.13</jacoco-maven-plugin.version>
    <maven-jxr-plugin.version>3.6.0</maven-jxr-plugin.version>
    <maven-site-plugin.version>3.21.0</maven-site-plugin.version>
    <groovy-maven-plugin.version>2.1.1</groovy-maven-plugin.version>
    <sonar-maven-plugin.version><latest></sonar-maven-plugin.version>
    <openapi-generator-maven-plugin.version><latest></openapi-generator-maven-plugin.version>
    <protobuf-maven-plugin.version><from-reference-pom-or-latest></protobuf-maven-plugin.version>
    <avro-maven-plugin.version><latest></avro-maven-plugin.version>
    <!-- GraphQL SDL -> Java. The same version is ALSO used for the io.github.kobylynskyi:graphql-java-codegen
         runtime dependency that generated GraphQL *client* code extends; keep the two in lockstep. -->
    <graphql-codegen-maven-plugin.version><latest></graphql-codegen-maven-plugin.version>
    <!-- Runs the AsyncAPI documentation generators (npm-based) from the Maven build; see
         iru-setup-java-springboot-apis. Needed when messaging and/or a GraphQL server is enabled. -->
    <frontend-maven-plugin.version><latest></frontend-maven-plugin.version>
    <!-- Escape hatches for the two npm-driven documentation steps, so an offline build can opt out.
         CI must leave both false or the published documentation silently goes stale. -->
    <asyncapi.docs.skip>false</asyncapi.docs.skip>
    <graphql.docs.skip>false</graphql.docs.skip>

    <!-- Analysis and coverage settings -->
    <checkstyle.config.location>${maven.multiModuleProjectDirectory}/checkstyle.xml</checkstyle.config.location>
    <sonar.organization><sonar-organization></sonar.organization>
    <sonar.projectKey><owner>_<repo></sonar.projectKey>
    <sonar.host.url>https://sonarcloud.io</sonar.host.url>
    <sonar.coverage.jacoco.xmlReportPaths>${maven.multiModuleProjectDirectory}/coverage/target/site/jacoco-aggregate/jacoco.xml</sonar.coverage.jacoco.xmlReportPaths>
    <!-- Generated sources are never the developer's code; keep them out of coverage and duplication metrics. -->
    <sonar.exclusions>**/generated-sources/**,**/generated/**</sonar.exclusions>
  </properties>

  <dependencyManagement>
    <dependencies>
      <!-- This project's own modules, so module poms never repeat ${project.version}. -->
      <dependency>
        <groupId>${project.groupId}</groupId>
        <artifactId><artifact-id>-domain</artifactId>
        <version>${project.version}</version>
      </dependency>
      <!-- ... one entry per module in the manifest's `modules:` list, same shape ... -->

      <!-- BOM imports copied from the Initializr reference pom (Spring Cloud, Spring AI, ...). -->
      <dependency>
        <groupId>org.springframework.cloud</groupId>
        <artifactId>spring-cloud-dependencies</artifactId>
        <version>${spring-cloud.version}</version>
        <type>pom</type>
        <scope>import</scope>
      </dependency>

      <!-- Non-Initializr dependencies. Include only what the stack uses. -->
      <dependency>
        <groupId>org.mapstruct</groupId>
        <artifactId>mapstruct</artifactId>
        <version>${mapstruct.version}</version>
      </dependency>
      <dependency>
        <groupId>com.github.ben-manes.caffeine</groupId>
        <artifactId>caffeine</artifactId>
        <version>${caffeine.version}</version>
      </dependency>
      <dependency>
        <groupId>com.tngtech.archunit</groupId>
        <artifactId>archunit-junit5</artifactId>
        <version>${archunit.version}</version>
        <scope>test</scope>
      </dependency>
      <!-- ... Mongock, the Liquibase extensions, and the AWS BOM as the stack requires ... -->
    </dependencies>
  </dependencyManagement>

  <dependencies>
    <!-- Project-wide: every module gets these. Nothing Spring-specific belongs here — `domain` inherits it too. -->
    <dependency>
      <groupId>org.projectlombok</groupId>
      <artifactId>lombok</artifactId>
      <optional>true</optional>
    </dependency>
    <dependency>
      <groupId>org.mapstruct</groupId>
      <artifactId>mapstruct</artifactId>
    </dependency>
    <!-- Test dependencies every module needs; artifact names taken from the reference pom. -->
    <dependency>
      <groupId>org.springframework.boot</groupId>
      <artifactId>spring-boot-starter-test</artifactId>
      <scope>test</scope>
    </dependency>
    <dependency>
      <groupId>org.mockito</groupId>
      <artifactId>mockito-junit-jupiter</artifactId>
      <scope>test</scope>
    </dependency>
  </dependencies>

  <build>
    <pluginManagement>
      <plugins>
        <plugin>
          <groupId>org.openapitools</groupId>
          <artifactId>openapi-generator-maven-plugin</artifactId>
          <version>${openapi-generator-maven-plugin.version}</version>
        </plugin>
        <plugin>
          <groupId><protobuf-plugin-group-from-reference-pom></groupId>
          <artifactId>protobuf-maven-plugin</artifactId>
          <version>${protobuf-maven-plugin.version}</version>
        </plugin>
        <plugin>
          <groupId>org.apache.avro</groupId>
          <artifactId>avro-maven-plugin</artifactId>
          <version>${avro-maven-plugin.version}</version>
        </plugin>
        <plugin>
          <groupId>io.github.kobylynskyi</groupId>
          <artifactId>graphql-codegen-maven-plugin</artifactId>
          <version>${graphql-codegen-maven-plugin.version}</version>
        </plugin>
        <plugin>
          <groupId>com.github.eirslett</groupId>
          <artifactId>frontend-maven-plugin</artifactId>
          <version>${frontend-maven-plugin.version}</version>
        </plugin>
        <plugin>
          <groupId>org.springframework.boot</groupId>
          <artifactId>spring-boot-maven-plugin</artifactId>
        </plugin>
      </plugins>
    </pluginManagement>

    <plugins>
      <!-- Java 21 minimum, enforced at build time and not just declared in a property -->
      <plugin>
        <groupId>org.apache.maven.plugins</groupId>
        <artifactId>maven-enforcer-plugin</artifactId>
        <version>${maven-enforcer-plugin.version}</version>
        <executions>
          <execution>
            <id>enforce-java</id>
            <goals>
              <goal>enforce</goal>
            </goals>
            <configuration>
              <rules>
                <requireJavaVersion>
                  <version>[<java-version>,)</version>
                  <message>This project requires Java <java-version> or newer.</message>
                </requireJavaVersion>
                <requireMavenVersion>
                  <version>[3.9.0,)</version>
                </requireMavenVersion>
              </rules>
            </configuration>
          </execution>
        </executions>
      </plugin>

      <!-- Lombok and MapStruct annotation processors. lombok-mapstruct-binding is required for the two to
           coexist: without it MapStruct runs before Lombok has generated getters and produces empty mappers.
           Order matters — lombok, then the binding, then mapstruct-processor. -->
      <plugin>
        <groupId>org.apache.maven.plugins</groupId>
        <artifactId>maven-compiler-plugin</artifactId>
        <configuration>
          <release>${maven.compiler.release}</release>
          <parameters>true</parameters>
          <annotationProcessorPaths>
            <path>
              <groupId>org.projectlombok</groupId>
              <artifactId>lombok</artifactId>
              <version>${lombok.version}</version>
            </path>
            <path>
              <groupId>org.projectlombok</groupId>
              <artifactId>lombok-mapstruct-binding</artifactId>
              <version>${lombok-mapstruct-binding.version}</version>
            </path>
            <path>
              <groupId>org.mapstruct</groupId>
              <artifactId>mapstruct-processor</artifactId>
              <version>${mapstruct.version}</version>
            </path>
            <path>
              <groupId>org.springframework.boot</groupId>
              <artifactId>spring-boot-configuration-processor</artifactId>
              <version>${spring-boot.version}</version>
            </path>
          </annotationProcessorPaths>
          <compilerArgs>
            <arg>-Amapstruct.defaultComponentModel=spring</arg>
            <arg>-Amapstruct.unmappedTargetPolicy=ERROR</arg>
          </compilerArgs>
        </configuration>
      </plugin>

      <!-- Unit tests: *Test. Integration tests are Failsafe's, below, and never run here. -->
      <plugin>
        <groupId>org.apache.maven.plugins</groupId>
        <artifactId>maven-surefire-plugin</artifactId>
        <version>${maven-surefire-plugin.version}</version>
        <configuration>
          <includes>
            <include>**/*Test.java</include>
          </includes>
          <excludes>
            <exclude>**/*IT.java</exclude>
          </excludes>
        </configuration>
      </plugin>

      <!-- Integration tests: *IT, bound to verify. These are the Testcontainers-backed ones. -->
      <plugin>
        <groupId>org.apache.maven.plugins</groupId>
        <artifactId>maven-failsafe-plugin</artifactId>
        <version>${maven-failsafe-plugin.version}</version>
        <configuration>
          <includes>
            <include>**/*IT.java</include>
          </includes>
        </configuration>
        <executions>
          <execution>
            <id>integration-tests</id>
            <goals>
              <goal>integration-test</goal>
              <goal>verify</goal>
            </goals>
          </execution>
        </executions>
      </plugin>

      <!-- Coverage. Generated sources are excluded so codegen output can't inflate or deflate the number. -->
      <plugin>
        <groupId>org.jacoco</groupId>
        <artifactId>jacoco-maven-plugin</artifactId>
        <version>${jacoco-maven-plugin.version}</version>
        <configuration>
          <excludes>
            <exclude><base-package-path>/**/generated/**</exclude>
            <exclude>**/*MapperImpl.class</exclude>
            <exclude>**/*Application.class</exclude>
          </excludes>
        </configuration>
        <executions>
          <execution>
            <id>jacoco-initialize</id>
            <goals>
              <goal>prepare-agent</goal>
            </goals>
          </execution>
          <execution>
            <id>jacoco-initialize-it</id>
            <goals>
              <goal>prepare-agent-integration</goal>
            </goals>
          </execution>
          <execution>
            <id>jacoco-report</id>
            <phase>verify</phase>
            <goals>
              <goal>report</goal>
              <goal>report-integration</goal>
            </goals>
          </execution>
        </executions>
      </plugin>

      <!-- Static analysis. All three analyse hand-written sources only. -->
      <plugin>
        <groupId>org.apache.maven.plugins</groupId>
        <artifactId>maven-checkstyle-plugin</artifactId>
        <version>${maven-checkstyle-plugin.version}</version>
        <configuration>
          <configLocation>${checkstyle.config.location}</configLocation>
          <sourceDirectories>
            <sourceDirectory>${project.build.sourceDirectory}</sourceDirectory>
          </sourceDirectories>
          <includeTestSourceDirectory>true</includeTestSourceDirectory>
        </configuration>
      </plugin>
      <plugin>
        <groupId>org.apache.maven.plugins</groupId>
        <artifactId>maven-pmd-plugin</artifactId>
        <version>${maven-pmd-plugin.version}</version>
        <configuration>
          <targetJdk>${maven.compiler.release}</targetJdk>
          <excludeRoots>
            <excludeRoot>${project.build.directory}/generated-sources</excludeRoot>
          </excludeRoots>
        </configuration>
      </plugin>
      <plugin>
        <groupId>com.github.spotbugs</groupId>
        <artifactId>spotbugs-maven-plugin</artifactId>
        <version>${spotbugs-maven-plugin.version}</version>
        <configuration>
          <excludeFilterFile>${maven.multiModuleProjectDirectory}/spotbugs-exclude.xml</excludeFilterFile>
        </configuration>
      </plugin>

      <!-- Maven site, the report aggregator -->
      <plugin>
        <groupId>org.apache.maven.plugins</groupId>
        <artifactId>maven-site-plugin</artifactId>
        <version>${maven-site-plugin.version}</version>
      </plugin>

      <!-- SonarCloud/SonarQube; omit if the stack opted out -->
      <plugin>
        <groupId>org.sonarsource.scanner.maven</groupId>
        <artifactId>sonar-maven-plugin</artifactId>
        <version>${sonar-maven-plugin.version}</version>
      </plugin>
    </plugins>
  </build>

  <reporting>
    <plugins>
      <plugin>
        <groupId>org.apache.maven.plugins</groupId>
        <artifactId>maven-javadoc-plugin</artifactId>
        <version>${maven-javadoc-plugin.version}</version>
        <configuration>
          <show>private</show>
          <nohelp>true</nohelp>
          <detectJavaApiLink>true</detectJavaApiLink>
          <doclint>none</doclint>
        </configuration>
      </plugin>
      <plugin>
        <groupId>org.apache.maven.plugins</groupId>
        <artifactId>maven-surefire-report-plugin</artifactId>
        <version>${maven-surefire-plugin.version}</version>
        <reportSets>
          <reportSet>
            <reports>
              <report>report</report>
              <report>failsafe-report-only</report>
            </reports>
          </reportSet>
        </reportSets>
      </plugin>
      <plugin>
        <groupId>org.jacoco</groupId>
        <artifactId>jacoco-maven-plugin</artifactId>
        <version>${jacoco-maven-plugin.version}</version>
      </plugin>
      <plugin>
        <groupId>org.apache.maven.plugins</groupId>
        <artifactId>maven-checkstyle-plugin</artifactId>
        <version>${maven-checkstyle-plugin.version}</version>
        <configuration>
          <configLocation>${checkstyle.config.location}</configLocation>
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
        <groupId>org.apache.maven.plugins</groupId>
        <artifactId>maven-pmd-plugin</artifactId>
        <version>${maven-pmd-plugin.version}</version>
      </plugin>
      <plugin>
        <groupId>com.github.spotbugs</groupId>
        <artifactId>spotbugs-maven-plugin</artifactId>
        <version>${spotbugs-maven-plugin.version}</version>
      </plugin>
      <!-- Cross-referenced source, so every report above can link straight to the offending line -->
      <plugin>
        <groupId>org.apache.maven.plugins</groupId>
        <artifactId>maven-jxr-plugin</artifactId>
        <version>${maven-jxr-plugin.version}</version>
      </plugin>
    </plugins>
  </reporting>
</project>
```

Notes on filling this in:

- `${lombok.version}` and `${spring-boot.version}` are provided by `spring-boot-starter-parent` — don't redeclare
  them. `${maven.multiModuleProjectDirectory}` resolves to the reactor root from any module, which is what makes a
  single shared `checkstyle.xml` and `spotbugs-exclude.xml` work.
- If the stack opted out of Sonar, drop the four `sonar.*` properties and the `sonar-maven-plugin` entry rather
  than leaving empty placeholders.
- Look up the current version for every `<latest>` marker (Maven Central's search API) rather than guessing —
  and record what you resolved, so Step 8 can report it.
- Also write `spotbugs-exclude.xml` at the repository root, excluding the generated-source packages (the
  OpenAPI/protobuf/AVRO output) — SpotBugs analyses bytecode, so `excludeRoots` doesn't help it the way it helps
  PMD, and generated DTOs will otherwise produce a large permanent finding list.

## Step 2 — Where each dependency belongs

Use this to place every artifact extracted in Step 0. The point is that a dependency lands in exactly one module,
so the hexagonal boundaries are enforced by the classpath and not merely by convention.

| Module | Owns these dependencies |
|---|---|
| `domain` | Nothing Spring, nothing infrastructural. Plain Java plus `jakarta.validation-api` if the domain uses constraint annotations, plus the shared Lombok/MapStruct/test dependencies inherited from the root. This module defines the **ports** (interfaces) — repositories, event publishers, clients, metrics — and knows no implementation. |
| `application` | `<a>-domain`. Optionally `spring-context`/`spring-tx` if use cases need `@Service`/`@Transactional`; prefer keeping even this out and wiring in `boot` when practical. |
| `infrastructure/database/<engine>` | `<a>-domain` plus that engine's Spring Data starter and driver, plus its migration tooling (Mongock, Liquibase + the relevant extension, or Flyway) and its Testcontainers module (`test` scope). One engine per module — never two. |
| `infrastructure/configuration` | `<a>-domain`, `spring-boot-starter` (for `@ConfigurationProperties`), `spring-boot-configuration-processor`, and the dynamic-configuration starter the manifest names (Spring Cloud Kubernetes Config, AWS Secrets Manager, or Spring Cloud Config Client + Bus). |
| `infrastructure/clients/<name>` | `<a>-domain` plus the HTTP client starter (`spring-restclient`/`spring-webclient`) for a REST client, the gRPC client starter for a gRPC one, or `spring-boot-starter-graphql` (for `HttpGraphQlClient`/`HttpSyncGraphQlClient`) **and** `io.github.kobylynskyi:graphql-java-codegen` at compile scope for a GraphQL one — that last artifact supplies the base classes the generated request/projection classes extend, and without it the module generates and then fails to compile. A downstream service reached over more than one protocol keeps one module with one generator execution per protocol. Also the OpenAPI, protobuf or GraphQL generator plugin execution for that client's own contract, and — at `test` scope — the Testcontainers module for whichever API mock `stack.apiMock` names (`io.github.microcks:microcks-testcontainers` or the WireMock Testcontainers module), so this module's own `*IT` can start just the mock instead of the whole compose stack. |
| `infrastructure/producers` | `<a>-domain`, `spring-cloud-stream`, `spring-cloud-stream-binder-kafka`, the AVRO serializer (`io.confluent:kafka-avro-serializer`) and `org.apache.avro:avro`, plus the AVRO generator plugin execution. |
| `infrastructure/metrics` | `<a>-domain`, `spring-boot-starter-actuator`, `micrometer-registry-prometheus` (and `micrometer-registry-otlp` if selected). |
| `api/rest-server` | `<a>-application`, `<a>-domain`, `spring-boot-starter-webmvc`/`-webflux` (whatever the reference pom names), `spring-boot-starter-validation`, plus the OpenAPI generator execution for `apis/rest-server/`. Add the Spring Security starters here if the service authorizes incoming HTTP. |
| `api/grpc-server` | `<a>-application`, `<a>-domain`, the gRPC server starter, plus the protobuf generator execution for `apis/grpc-server/`. |
| `api/graphql-server` | `<a>-application`, `<a>-domain`, `spring-boot-starter-graphql`, **and** the web transport starter the reference pom names for `stack.concurrency` (`spring-boot-starter-webmvc`/`-web` or `-webflux`) — the GraphQL starter carries no HTTP transport of its own — plus `spring-boot-starter-validation`, `spring-boot-starter-graphql-test` at `test` scope, the `graphql-codegen-maven-plugin` execution for `apis/graphql-server/`, the `maven-resources-plugin` execution that copies the SDL to `target/classes/graphql/` for runtime schema loading, and the `frontend-maven-plugin` executions that generate the SpectaQL reference (and optional Voyager schema graph) into `target/generated-docs/graphql-server/`. |
| `api/consumers` | `<a>-application`, `<a>-domain`, `spring-cloud-stream`, the Kafka binder, the AVRO serializer, plus the AVRO generator execution. |
| `boot` | Every other module, `spring-boot-starter`, the Spring AI starters if any, and anything Step 0 couldn't place. This is the only module with `spring-boot-maven-plugin` and the only one that produces an executable artifact. |
| `coverage` | Every other module at `test` scope, for JaCoCo `report-aggregate` only. No sources of its own. |

`spring-boot-docker-compose` (runtime, optional) belongs in `boot` — it starts the local compose stack
automatically during development.

## Step 3 — Module pom template

Every module pom follows this shape. Note the complete absence of `<version>` on dependencies.

```xml
<project xmlns="http://maven.apache.org/POM/4.0.0"
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:schemaLocation="http://maven.apache.org/POM/4.0.0 http://maven.apache.org/xsd/maven-4.0.0.xsd">
  <modelVersion>4.0.0</modelVersion>

  <parent>
    <groupId><group-id></groupId>
    <artifactId><parent-artifact-id></artifactId>
    <version><version></version>
    <relativePath><relative-path-to-parent-pom></relativePath>
  </parent>

  <artifactId><artifact-id>-<module-suffix></artifactId>
  <packaging>jar</packaging>
  <name>${project.groupId}:${project.artifactId}</name>
  <description><what this module is responsible for></description>

  <dependencies>
    <!-- Only what this module needs, per Step 2's table. No <version> anywhere. -->
  </dependencies>
</project>
```

- `<relativePath>` is `../pom.xml` for a top-level module, `../../pom.xml` for an `infrastructure/*` or `api/*`
  module, `../../../pom.xml` for an `infrastructure/database/<engine>` or `infrastructure/clients/<name>` module.
- The intermediate aggregators (`infrastructure/pom.xml`, `infrastructure/database/pom.xml`,
  `infrastructure/clients/pom.xml`, `api/pom.xml`) use `<packaging>pom</packaging>`, carry `<modules>` and no
  `<dependencies>`, and are pure grouping.

### Enforcing the dependency direction

Add a `banned-dependencies` enforcer execution to each module so a violation of the hexagon fails the build
instead of being caught in review. In `domain/pom.xml`:

```xml
<build>
  <plugins>
    <plugin>
      <groupId>org.apache.maven.plugins</groupId>
      <artifactId>maven-enforcer-plugin</artifactId>
      <executions>
        <execution>
          <id>enforce-domain-purity</id>
          <goals>
            <goal>enforce</goal>
          </goals>
          <configuration>
            <rules>
              <bannedDependencies>
                <searchTransitive>false</searchTransitive>
                <excludes>
                  <exclude>org.springframework:*</exclude>
                  <exclude>org.springframework.boot:*</exclude>
                  <exclude>org.springframework.data:*</exclude>
                  <exclude>${project.groupId}:<artifact-id>-application</exclude>
                  <exclude>${project.groupId}:<artifact-id>-infrastructure-*</exclude>
                  <exclude>${project.groupId}:<artifact-id>-api-*</exclude>
                </excludes>
                <message>The domain module must not depend on Spring or on any outer module. Define a port
                  (interface) here and implement it in the adapter module instead.</message>
              </bannedDependencies>
            </rules>
          </configuration>
        </execution>
      </executions>
    </plugin>
  </plugins>
</build>
```

Apply the same idea to the others, banning what each must not see:

- `application` — must not see any `infrastructure-*` or `api-*` module.
- `infrastructure/*` — must not see `application`, `api-*`, or a sibling `infrastructure-*` module.
- `api/*` — must not see any `infrastructure-*` module.
- `boot` — nothing banned; it is the composition root.

Note in Step 8 that banning `org.springframework:*` in `domain` is a deliberately strict DDD stance and the one
rule a team may reasonably want to relax (e.g. to use `spring-context` annotations in domain services); relaxing
it is a one-line change, whereas noticing the leak later is not.

### `coverage/pom.xml`

```xml
  <artifactId><artifact-id>-coverage</artifactId>
  <packaging>pom</packaging>
  <name>${project.groupId}:${project.artifactId}</name>
  <description>JaCoCo coverage aggregation across every module. No sources of its own.</description>

  <dependencies>
    <!-- one test-scope entry per module whose coverage should be aggregated -->
    <dependency>
      <groupId>${project.groupId}</groupId>
      <artifactId><artifact-id>-domain</artifactId>
    </dependency>
    <!-- ... -->
  </dependencies>

  <build>
    <plugins>
      <plugin>
        <groupId>org.jacoco</groupId>
        <artifactId>jacoco-maven-plugin</artifactId>
        <executions>
          <execution>
            <id>aggregate-report</id>
            <phase>verify</phase>
            <goals>
              <goal>report-aggregate</goal>
            </goals>
            <configuration>
              <dataFileIncludes>
                <dataFileInclude>**/jacoco*.exec</dataFileInclude>
              </dataFileIncludes>
              <outputDirectory>${project.build.directory}/site/jacoco-aggregate</outputDirectory>
            </configuration>
          </execution>
        </executions>
      </plugin>
    </plugins>
  </build>
```

`coverage` must be the **last** `<module>` in the root pom so every other module's `jacoco.exec` already exists
when `report-aggregate` runs. `dataFileIncludes` picks up both the unit (`jacoco.exec`) and integration
(`jacoco-it.exec`) execution data, so the aggregate — which is what Sonar reads — reflects both.

### `boot/pom.xml`

Additionally carries the executable-artifact and build-info plugins:

```xml
  <build>
    <plugins>
      <plugin>
        <groupId>org.springframework.boot</groupId>
        <artifactId>spring-boot-maven-plugin</artifactId>
        <configuration>
          <mainClass><base-package>.<ServiceName>Application</mainClass>
          <image>
            <name><image-registry>/<artifact-id>:${project.version}</name>
          </image>
        </configuration>
        <executions>
          <execution>
            <goals>
              <goal>repackage</goal>
            </goals>
          </execution>
        </executions>
      </plugin>

      <!-- Writes build-info.properties into this module's resources, the same house pattern as
           iru-setup-java-library, so the running service can report exactly what was deployed. -->
      <plugin>
        <groupId>org.codehaus.gmaven</groupId>
        <artifactId>groovy-maven-plugin</artifactId>
        <version>${groovy-maven-plugin.version}</version>
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

                //GITHUB ACTIONS
                def buildNumber = System.getenv("GITHUB_RUN_NUMBER")
                def commit = System.getenv("GITHUB_SHA")
                def branch = System.getenv("GITHUB_REF_NAME")

                //JENKINS
                if (buildNumber == null) {
                  buildNumber = System.getenv("BUILD_NUMBER")
                }
                if (commit == null) {
                  commit = System.getenv("GIT_COMMIT")
                }
                if (branch == null) {
                  branch = System.getenv("GIT_BRANCH")
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

                File dir = new File("${project.basedir}/src/main/resources/<base-package-path>")
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
    </plugins>
  </build>
```

`<base-package-path>` is the base package with dots replaced by slashes. The GitHub Actions branch is listed
first because that's what this catalog's generated workflows actually run under; the Jenkins/GitLab fallbacks are
kept from the `iru-setup-java-library` template so the same file works in a mirrored pipeline.

## Step 4 — Write the poms

Write the root pom, every aggregator pom, and every module pom named in the manifest's `modules:` list. Create the
directories as needed. Don't create source files here — that's `iru-setup-java-springboot-modules`; a pom for a
module with no sources yet is valid and builds cleanly.

## Step 5 — Write the shared analysis configuration

- **`checkstyle.xml`** at the repository root, if absent. Start from Sun or Google checks with the rules a Spring
  Boot service realistically can't satisfy relaxed (line length raised to 120, `JavadocPackage`/`HideUtilityClassConstructor`
  dropped, `FinalParameters` dropped). A ruleset the build immediately fails on gets disabled by the first
  developer who hits it, which is worse than a lenient one that stays enabled.
- **`spotbugs-exclude.xml`** at the repository root, if absent, excluding the generated-source packages:

  ```xml
  <FindBugsFilter>
    <Match>
      <Source name="~.*[\\/]generated-sources[\\/].*"/>
    </Match>
    <Match>
      <Class name="~.*\.generated\..*"/>
    </Match>
    <!-- MapStruct-generated implementations -->
    <Match>
      <Class name="~.*MapperImpl"/>
    </Match>
  </FindBugsFilter>
  ```

## Step 6 — Verify the reactor resolves

Delegate to the `iru-gate-runner` agent (a multi-module Maven log is exactly the kind of output that shouldn't land
in this context) and ask for a compact result:

```bash
mvn -q -N validate            # root pom alone parses and the enforcer rules pass
mvn -q validate               # every module pom parses and the reactor order resolves
mvn dependency:tree -q        # every dependency resolves; no version comes from outside the root pom
```

Scan the `dependency:tree` output for two specific problems and fix them before finishing:

- a **version declared in a module pom** — move it to the root's `<dependencyManagement>`;
- an **unmanaged transitive version conflict** Maven resolved by nearest-wins — pin it in the root's
  `<dependencyManagement>` so the choice is explicit.

If the enforcer's `requireJavaVersion` rule fails because the local JDK is older than the target, that's the rule
working: report it and tell the user which JDK the build needs, rather than lowering the target.

## Step 7 — Sanity-check the hexagon

Confirm by reading the written poms, not by assuming:

- `domain` depends on no other module of this project and on nothing from `org.springframework*`.
- No `infrastructure/*` module depends on `application`, on `api/*`, or on a sibling infrastructure module.
- No `api/*` module depends on an `infrastructure/*` module.
- `boot` depends on every other module.
- No module pom other than the root declares a `<version>` for a dependency.

Report any violation you had to introduce deliberately (e.g. a starter that drags Spring into `domain`) rather
than quietly relaxing the enforcer rule to make the build pass.

## Step 8 — Report

Summarize: the reactor's module list in build order; every version pinned in the root pom (separating those copied
from the Initializr reference pom from those looked up on Maven Central); which module each significant dependency
landed in; any dependency that couldn't be placed and therefore went to `boot`; whether `checkstyle.xml` and
`spotbugs-exclude.xml` were created or already present; the result of Step 6's three commands; and any hexagon
violation from Step 7. If an existing root pom was regenerated, list what was carried over from it so the user can
confirm nothing was lost.

Close by reminding the user that **the root pom is the only place a version may be declared** — adding one to a
module pom silently defeats the reactor's version discipline — and that `mvn dependency:tree` is the check for
that.
