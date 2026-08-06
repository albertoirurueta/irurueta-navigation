---
name: iru-setup-java-springboot-apis
description: Set up the contract-first API layer of a Spring Boot service — the single `apis/` directory at the repository root holding every contract (`apis/rest-server/` OpenAPI specs, `apis/grpc-server/` protobuf definitions, `apis/rest-client/<name>/` and `apis/grpc-client/<name>/` per downstream service, `apis/messaging/` AVRO schemas), a starter spec for each one, and the code-generation plugin executions wired into the module that consumes each contract — `openapi-generator-maven-plugin` (Spring server interfaces + DTOs for the REST server, client + DTOs per REST client, and HTML documentation), `protobuf-maven-plugin` (gRPC service base classes and stubs, plus HTML documentation via `protoc-gen-doc`), and `avro-maven-plugin` (Java classes from the AVRO schemas). Adds a linting/compatibility check per contract kind and keeps generated sources out of version control and out of static analysis. Reads `springboot-stack.yml` for which contracts the service actually needs and writes no version numbers, inheriting them all from the root pom. Invoke as `/iru-setup-java-springboot-apis`, or with `args` (`stack-file:` line) when called from `iru-setup-java-springboot`. Use after the reactor and module sources exist, whenever a Spring Boot service needs its OpenAPI/protobuf/AVRO contracts and their generators set up instead of hand-configuring three code generators across several module poms.
model: sonnet
---

# Setup Java Spring Boot APIs

Wire up the contract-first API layer. Two invariants:

1. **Every contract lives under `apis/` at the repository root** — never inside a module's `src/`. One directory
   is where a reviewer looks to see everything this service exposes and consumes, and it keeps a contract from
   being coupled to the module that happens to implement it today. `apis/` is not a Maven module.
2. **Generated code is never committed and never hand-edited.** It lands in each module's
   `target/generated-sources/`, is git-ignored, and is excluded from Checkstyle/PMD/SpotBugs/JaCoCo/Sonar. If
   something about the generated code is wrong, the fix is in the spec or the generator configuration.

## Step 0 — Resolve inputs

Parse `args` for `stack-file: <path>`, defaulting to `springboot-stack.yml`. Read it and determine exactly which
contracts are needed:

| Manifest field | Contract directory | Generated into | Generator |
|---|---|---|---|
| `stack.restServer: true` | `apis/rest-server/` | `api/rest-server` | `openapi-generator-maven-plugin` (`spring` generator) |
| `stack.grpcServer: true` | `apis/grpc-server/` | `api/grpc-server` | `protobuf-maven-plugin` |
| each `stack.restClients[]` | `apis/rest-client/<name>/` | `infrastructure/clients/<name>` | `openapi-generator-maven-plugin` (client generator) |
| each `stack.grpcClients[]` | `apis/grpc-client/<name>/` | `infrastructure/clients/<name>` | `protobuf-maven-plugin` |
| `stack.messaging.enabled: true` | `apis/messaging/` | `infrastructure/producers` and/or `api/consumers` | `avro-maven-plugin` |

If none apply, report that and stop — there's nothing to set up.

Verify the target module directories exist (from `iru-setup-java-springboot-pom`). If a module is missing, report it
and skip that contract rather than creating a module here.

Read the root `pom.xml` to confirm the generator plugins are already version-managed in `<pluginManagement>` (the
pom skill puts them there). **Declare no `<version>` in any module pom below** — if a version is missing from
`pluginManagement`, add it to the root pom rather than inline in the module.

## Step 1 — Create the `apis/` tree

```
apis/
  README.md
  rest-server/
    <service>-api-v1.yaml
  grpc-server/
    <service>_v1.proto
  rest-client/
    <name>/
      <name>-api.yaml
  grpc-client/
    <name>/
      <name>.proto
  messaging/
    <event-name>-v1.avsc
```

Create only the subdirectories the manifest calls for. Write `apis/README.md` explaining the layout, which module
each directory generates into, that generated code is never committed, and how to regenerate
(`mvn generate-sources`). This file is what stops the next developer from adding a spec in the wrong place.

Version contracts **in the filename and in the path inside the contract**, not by mutating a file in place: an
OpenAPI spec's paths carry `/v1/...`, a protobuf file declares `package <service>.v1`, an AVRO schema's namespace
ends in `.v1`. A breaking change means a `v2` file next to the `v1` one, with both generating and both served
until `v1` is retired. State this convention in `apis/README.md`.

## Step 2 — REST server contract and generator

Write a starter OpenAPI 3.1 spec at `apis/rest-server/<service>-api-v1.yaml` covering one resource, and make it
complete enough to be a real template rather than a stub: `info` with a version and description, a server entry,
one tagged path with parameters and request/response bodies, `components/schemas` with per-field `description`
and `required` lists, and — importantly — a documented **error schema** plus `4xx`/`5xx` responses that reference
it. If `stack.security.enabled`, add a `securitySchemes` entry (`bearerFormat: JWT`) and apply it.

Every field in every schema needs a `description`, and any field with a constraint needs it expressed in the spec
(`maxLength`, `pattern`, `minimum`, `format`) — those constraints become Bean Validation annotations on the
generated DTOs *and* they're what the documentation page reads. A spec without descriptions produces both
undocumented code and an undocumentable API.

Wire into `api/rest-server/pom.xml`:

```xml
<build>
  <plugins>
    <plugin>
      <groupId>org.openapitools</groupId>
      <artifactId>openapi-generator-maven-plugin</artifactId>
      <executions>
        <execution>
          <id>generate-rest-server-v1</id>
          <phase>generate-sources</phase>
          <goals>
            <goal>generate</goal>
          </goals>
          <configuration>
            <inputSpec>${maven.multiModuleProjectDirectory}/apis/rest-server/<service>-api-v1.yaml</inputSpec>
            <generatorName>spring</generatorName>
            <apiPackage><base>.api.rest.v1.generated.api</apiPackage>
            <modelPackage><base>.api.rest.v1.generated.model</modelPackage>
            <configOptions>
              <!-- interfaceOnly: generate the interfaces only; the controllers are hand-written and
                   implement them, so regeneration can never overwrite real logic. -->
              <interfaceOnly>true</interfaceOnly>
              <useSpringBoot3>true</useSpringBoot3>
              <useTags>true</useTags>
              <useJakartaEe>true</useJakartaEe>
              <useBeanValidation>true</useBeanValidation>
              <performBeanValidation>true</performBeanValidation>
              <openApiNullable>false</openApiNullable>
              <documentationProvider>none</documentationProvider>
              <!-- reactive: true emits Mono/Flux signatures. Set from stack.concurrency. -->
              <reactive><true-if-reactive></reactive>
              <serializableModel>true</serializableModel>
              <unhandledException>true</unhandledException>
            </configOptions>
          </configuration>
        </execution>
        <execution>
          <id>generate-rest-server-docs</id>
          <phase>generate-resources</phase>
          <goals>
            <goal>generate</goal>
          </goals>
          <configuration>
            <inputSpec>${maven.multiModuleProjectDirectory}/apis/rest-server/<service>-api-v1.yaml</inputSpec>
            <generatorName>html2</generatorName>
            <output>${project.build.directory}/generated-docs/rest-server/v1</output>
            <generateApiTests>false</generateApiTests>
            <generateModelTests>false</generateModelTests>
          </configuration>
        </execution>
      </executions>
    </plugin>
  </plugins>
</build>
```

Notes:

- `interfaceOnly=true` is the important setting. With it, the generator owns the interfaces and DTOs and the
  hand-written `@RestController` implements them — so `mvn generate-sources` is always safe to re-run. Without it,
  the generator emits controllers and every regeneration is a merge conflict.
- `reactive` must match `stack.concurrency`. Getting it wrong produces signatures the controllers can't implement,
  which surfaces as a confusing compile error rather than an obvious misconfiguration.
- The `html2` generator produces the standalone HTML documentation. It lands in
  `target/generated-docs/rest-server/v1` where the CI workflow can publish it to GitHub Pages next to the Antora
  site — the documentation page then links to it rather than duplicating the endpoint list by hand.
- Add one execution **per version file**. When `v2` arrives, it gets its own execution and its own
  `apiPackage`/`modelPackage` ending in `.v2.`, and both keep generating.

## Step 3 — gRPC server contract and generator

Write a starter `.proto` at `apis/grpc-server/<service>_v1.proto`: `syntax = "proto3"`, `package <service>.v1`,
`option java_package = "<base>.api.grpc.v1.generated"`, `option java_multiple_files = true`, one service with one
unary RPC and one server-streaming RPC, request/response messages, and an enum. Comment every message, every
field, and every RPC — protobuf comments are what `protoc-gen-doc` turns into the generated documentation, so an
uncommented field is an undocumented API field.

Field-number discipline belongs in the file as a comment: numbers are never reused or renumbered, a removed field
is marked `reserved`, and a deprecated field carries `[deprecated = true]` with the reason in its comment.

Wire into `api/grpc-server/pom.xml`, using the protobuf plugin **groupId that the root pom manages** — Spring
Boot 4's Initializr wires `io.github.ascopes:protobuf-maven-plugin`, while older setups used
`org.xolstice.maven.plugins:protobuf-maven-plugin`; the two have different configuration schemas, so read the root
pom to see which is in play and configure that one rather than assuming:

```xml
<plugin>
  <groupId><protobuf-plugin-group></groupId>
  <artifactId>protobuf-maven-plugin</artifactId>
  <configuration>
    <sourceDirectories>
      <sourceDirectory>${maven.multiModuleProjectDirectory}/apis/grpc-server</sourceDirectory>
    </sourceDirectories>
    <!-- plus the grpc-java protoc plugin, so service base classes are generated and not just messages -->
  </configuration>
  <executions>
    <execution>
      <goals>
        <goal>generate</goal>
      </goals>
    </execution>
  </executions>
</plugin>
```

Consult the plugin's own current documentation for the exact element names (`sourceDirectories` vs
`protoSourceRoot`, how the `grpc-java` plugin is declared, whether `protocVersion` is needed) instead of guessing —
a wrong element name here fails the build with an unhelpful "unknown parameter" error. Verify by running
`mvn generate-sources` in Step 7.

For **HTML documentation**, add a second execution using the `protoc-gen-doc` protoc plugin
(`io.github.pseudomuto:protoc-gen-doc`) with `--doc_opt=html,index.html`, writing into
`${project.build.directory}/generated-docs/grpc-server/v1`. If the protobuf plugin in use can't invoke a custom
protoc plugin cleanly, fall back to a `docker run pseudomuto/protoc-gen-doc` step and say so in the report — don't
silently ship no gRPC documentation, since the documentation page is specified to link to it.

## Step 4 — REST clients

Per entry in `stack.restClients`, put the downstream service's OpenAPI spec at
`apis/rest-client/<name>/<name>-api.yaml`. This is the **downstream team's contract, not yours** — the starter file
should be a minimal placeholder with a prominent comment saying to replace it with the real published spec, plus a
note in `apis/README.md` about keeping it in sync (ideally by pulling it from the downstream service's own
published artifact in CI rather than by hand).

Wire into `infrastructure/clients/<name>/pom.xml`:

```xml
<configuration>
  <inputSpec>${maven.multiModuleProjectDirectory}/apis/rest-client/<name>/<name>-api.yaml</inputSpec>
  <generatorName><generator></generatorName>
  <apiPackage><base>.infrastructure.clients.<name>.generated.api</apiPackage>
  <modelPackage><base>.infrastructure.clients.<name>.generated.model</modelPackage>
  <configOptions>
    <useJakartaEe>true</useJakartaEe>
    <openApiNullable>false</openApiNullable>
    <serializableModel>true</serializableModel>
  </configOptions>
</configuration>
```

`<generator>` follows `stack.concurrency`: `java` with `library=restclient` for blocking (matching the
`spring-restclient` starter), or `java` with `library=webclient` for reactive. Confirm the library name against
the generator's current option list rather than assuming — these names have changed across generator versions.
No HTML documentation execution is needed for clients: the contract belongs to the downstream service and is
documented there.

**Give every operation at least one request/response `example` pair.** This spec has a second consumer besides the
generator: the API mock in the compose stack (`stack.apiMock` — Microcks or WireMock, set up by
`iru-setup-java-springboot-testcontainers`) is built from this file, and it is the examples, not the schemas, that
become the mocked responses. A spec with complete schemas and no examples generates a perfectly good client and a
mock with nothing to say — which surfaces later as an integration test failing against an empty response body,
a long way from its cause. Name each example after the case it represents (`found`, `not-found`, `rate-limited`),
and cover the error responses too, since the adapter's exception translation is what most needs testing.

## Step 5 — gRPC clients

Per entry in `stack.grpcClients`, put the downstream `.proto` at `apis/grpc-client/<name>/<name>.proto`, with the
same "this is someone else's contract" comment. Wire the protobuf plugin into
`infrastructure/clients/<name>/pom.xml` with `java_package` under
`<base>.infrastructure.clients.<name>.generated`, generating the stubs. Generate only the client stubs, not
service base classes.

This `.proto` is also what the compose stack's mock serves. A protobuf file has no `examples` section, so the
sample responses live alongside it — Microcks reads them from a companion examples artifact next to the `.proto`
in the same directory (a Postman collection or a Microcks-format YAML; check the pinned version's supported
formats), while WireMock's gRPC extension needs a descriptor set built from this file plus hand-written mappings.
Whichever `stack.apiMock` names, put those example files **in this same directory**, so a downstream contract and
the responses mocked for it are updated in one place.

## Step 6 — AVRO messaging schemas

Only if `stack.messaging.enabled`. Write one starter schema per event at
`apis/messaging/<event-name>-v1.avsc`: a record with `namespace` ending in `.v1`, a `doc` on the record and on
**every** field, and — critically — a **default value on every optional field**. Defaults are what make schema
evolution work: a field added without one is a backward-incompatible change that the registry will reject and that
breaks every existing consumer.

Wire into whichever of `infrastructure/producers` and `api/consumers` exist (both, if the service produces and
consumes):

```xml
<plugin>
  <groupId>org.apache.avro</groupId>
  <artifactId>avro-maven-plugin</artifactId>
  <executions>
    <execution>
      <id>generate-avro-sources</id>
      <phase>generate-sources</phase>
      <goals>
        <goal>schema</goal>
      </goals>
      <configuration>
        <sourceDirectory>${maven.multiModuleProjectDirectory}/apis/messaging</sourceDirectory>
        <outputDirectory>${project.build.directory}/generated-sources/avro</outputDirectory>
        <stringType>String</stringType>
        <enableDecimalLogicalType>true</enableDecimalLogicalType>
        <fieldVisibility>PRIVATE</fieldVisibility>
      </configuration>
    </execution>
  </executions>
</plugin>
```

`stringType=String` matters: the default (`CharSequence`) makes generated classes awkward to map with MapStruct.

The Confluent AVRO serializer resolves schemas through the schema registry at runtime, so also record in the
report that the registry's **compatibility mode** (`BACKWARD` is the usual choice) has to be configured in the
registry itself — neither this scaffold nor the poms can set it. If a subject naming strategy other than the
default is wanted (e.g. `TopicRecordNameStrategy` for multiple event types per topic), it must be configured in the
producer/consumer binding properties; note whether you set one.

## Step 7 — Verify generation actually works

Delegate to `iru-gate-runner`:

```bash
mvn -q clean generate-sources    # every generator runs
mvn -q clean compile             # hand-written code compiles against the generated code
```

Then confirm by inspection:

- generated sources exist under each expected module's `target/generated-sources/`;
- the packages match what Step 2–6 configured, so the hand-written mappers and controllers resolve;
- generated HTML documentation exists under `target/generated-docs/`.

A generator that runs but produces nothing usually means a wrong `inputSpec`/`sourceDirectory` path — remember
these are absolute via `${maven.multiModuleProjectDirectory}`, because the contract lives outside the module.

Fix whatever fails. This is the step most likely to need real iteration, particularly the protobuf plugin's
configuration schema and the OpenAPI generator's `library` option names — check the plugin's actual documentation
rather than trying variations.

## Step 8 — Keep generated output out of git and out of analysis

- Confirm `.gitignore` covers `target/` (which already covers `generated-sources` and `generated-docs`). If any
  generator was configured to write outside `target/`, either change it to write inside `target/` — strongly
  preferred — or add an explicit ignore entry and explain why in the report.
- Confirm the root pom's Checkstyle `sourceDirectories`, PMD `excludeRoots`, SpotBugs `excludeFilterFile`, JaCoCo
  `excludes`, and `sonar.exclusions` all cover the generated packages. `iru-setup-java-springboot-pom` sets these
  up; verify they match the package names actually configured here, and correct them if the generated packages
  ended up somewhere the exclusions don't reach. Thousands of findings on generated DTOs is how a team learns to
  ignore the quality report entirely.

## Step 9 — Contract linting (recommended)

Add a lightweight check per contract kind so a malformed contract fails in CI rather than at generation time, and
report what you added:

- **OpenAPI** — the generator's own `validateSpec` option (on by default) catches structural errors. For style and
  breaking-change detection, note Spectral (`@stoplight/spectral-cli`) and `oasdiff` as the tools to add to CI;
  wire one in if the user wants it now.
- **protobuf** — `buf lint` and `buf breaking` are the standard; adding a `buf.yaml` is cheap and catches
  field-number reuse, which is otherwise a silent wire-compatibility break.
- **AVRO** — schema compatibility is checked by the registry. Note that
  `io.confluent:kafka-schema-registry-maven-plugin` can run `test-compatibility` against a real registry in CI,
  which catches an incompatible schema before deploy rather than at first publish.

## Step 10 — Report

Summarize:

- Every contract file created, with its path and what it describes.
- Every generator execution added, to which module pom, generating into which package, and where its HTML
  documentation lands.
- The `reactive`/`library` settings resolved from `stack.concurrency`, and the protobuf plugin flavour detected.
- The result of Step 7, including whether HTML documentation was actually produced for REST and gRPC.
- Whether the analysis exclusions in Step 8 already covered the generated packages or had to be corrected.
- What linting was wired versus merely recommended.

Warn explicitly:

- **The client specs under `apis/rest-client/` and `apis/grpc-client/` are placeholders for someone else's
  contract** — they must be replaced with the downstream service's real published spec before the client adapters
  mean anything, and kept in sync afterwards. They have two consumers, not one: the client generator *and* the
  compose stack's API mock (`stack.apiMock`), so a spec left as a placeholder produces both a useless client and a
  mock that agrees with it. Report which client specs still carry no `examples`, since those mock nothing.
- **Generated code must never be committed or edited.** Fix the spec or the generator configuration instead.
- The AVRO registry compatibility mode and the gRPC field-numbering discipline are enforced outside this
  repository (in the registry, and by review or `buf breaking`) — the scaffold can't enforce either.
- Every contract's descriptions and constraints feed the documentation pages, so an under-described spec produces
  under-described documentation. `/iru-update-java-springboot-documentation` reads these files.
