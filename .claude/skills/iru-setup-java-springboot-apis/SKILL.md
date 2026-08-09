---
name: iru-setup-java-springboot-apis
description: Set up the contract-first API layer of a Spring Boot service — the single `apis/` directory at the repository root holding every contract (`apis/rest-server/` OpenAPI specs, `apis/grpc-server/` protobuf definitions, `apis/graphql-server/` SDL schemas, `apis/rest-client/<name>/`, `apis/grpc-client/<name>/` and `apis/graphql-client/<name>/` per downstream service, `apis/messaging/` AVRO schemas *and* the AsyncAPI specification that describes the topics carrying them), a starter spec for each one, and the code-generation plugin executions wired into the module that consumes each contract — `openapi-generator-maven-plugin` (Spring server interfaces + DTOs for the REST server, client + DTOs per REST client, and HTML documentation), `protobuf-maven-plugin` (gRPC service base classes and stubs, plus HTML documentation via `protoc-gen-doc`), `graphql-codegen-maven-plugin` (resolver interfaces + model types for the GraphQL server, request/response/projection classes per GraphQL client, plus the SDL copied onto the runtime classpath Spring for GraphQL loads it from, and `spectaql`/`graphql-voyager` run from the Maven build for its static HTML reference and schema graph), `avro-maven-plugin` (Java classes from the AVRO schemas), and `@asyncapi/html-template` run from the Maven build via `frontend-maven-plugin` (HTML documentation from the AsyncAPI specification). Adds a linting/compatibility check per contract kind and keeps generated sources out of version control and out of static analysis. Reads `springboot-stack.yml` for which contracts the service actually needs and writes no version numbers, inheriting them all from the root pom. Invoke as `/iru-setup-java-springboot-apis`, or with `args` (`stack-file:` line) when called from `iru-setup-java-springboot`. Use after the reactor and module sources exist, whenever a Spring Boot service needs its OpenAPI/protobuf/GraphQL/AVRO/AsyncAPI contracts and their generators set up instead of hand-configuring five code generators across several module poms.
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
| `stack.graphqlServer: true` | `apis/graphql-server/` (SDL → code) | `api/graphql-server` | `graphql-codegen-maven-plugin` |
| `stack.graphqlServer: true` | `apis/graphql-server/` (SDL → docs) | documentation only — no Java | `spectaql` (+ optional `graphql-voyager`) via `frontend-maven-plugin` |
| each `stack.restClients[]` | `apis/rest-client/<name>/` | `infrastructure/clients/<name>` | `openapi-generator-maven-plugin` (client generator) |
| each `stack.grpcClients[]` | `apis/grpc-client/<name>/` | `infrastructure/clients/<name>` | `protobuf-maven-plugin` |
| each `stack.graphqlClients[]` | `apis/graphql-client/<name>/` | `infrastructure/clients/<name>` | `graphql-codegen-maven-plugin` |
| `stack.messaging.enabled: true` | `apis/messaging/` (AVRO schemas) | `infrastructure/producers` and/or `api/consumers` | `avro-maven-plugin` |
| `stack.messaging.enabled: true` | `apis/messaging/` (AsyncAPI spec) | documentation only — no Java | `@asyncapi/html-template` via `frontend-maven-plugin` |

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
  graphql-server/
    <service>-v1.graphqls
    docs/
      package.json
      spectaql-v1.yml
      build-voyager.mjs
  rest-client/
    <name>/
      <name>-api.yaml
  grpc-client/
    <name>/
      <name>.proto
  graphql-client/
    <name>/
      <name>-api.graphqls
  messaging/
    <service>-async-api-v1.yaml
    <event-name>-v1.avsc
    docs/
      package.json
```

`apis/messaging/` holds **two kinds of contract, not one**: the AVRO schemas define the *payloads*, and the
AsyncAPI specification defines the *interface* — which topics exist, in which direction the service uses each one,
which message each one carries, and which server (broker) they live on. The AVRO schemas are what
`avro-maven-plugin` compiles into Java; the AsyncAPI document is what the documentation generators read. Keeping
them in one directory is deliberate: a new event means a new `.avsc` *and* a new channel in the AsyncAPI file, and
a reviewer should see both in the same diff.

Create only the subdirectories the manifest calls for. Write `apis/README.md` explaining the layout, which module
each directory generates into, that generated code is never committed, and how to regenerate
(`mvn generate-sources`). This file is what stops the next developer from adding a spec in the wrong place.

Version contracts **in the filename and in the path inside the contract**, not by mutating a file in place: an
OpenAPI spec's paths carry `/v1/...`, a protobuf file declares `package <service>.v1`, an AVRO schema's namespace
ends in `.v1`, and a GraphQL SDL — which has no in-band version at all — is versioned by filename and by the
package its types generate into. A breaking change means a `v2` file next to the `v1` one, with both generating and both served
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

## Step 3a — GraphQL server contract and generator

Only if `stack.graphqlServer`. Write a starter SDL at `apis/graphql-server/<service>-v1.graphqls`: one `type` for
the aggregate, an `enum`, an `input` type, a `Query` with one field, and a `Mutation` with one field. Give **every
type, field, argument and enum value a description string** (`"""..."""` or `"..."` above the element) — SDL
descriptions are what the generated Javadoc carries and what any schema explorer shows, so an undescribed field is
an undocumented API field, exactly as with OpenAPI.

Version by file and by type-name suffix rather than by mutating in place. GraphQL has no URL path to carry a
version, so a breaking change means a `v2` SDL file whose types are generated into a `.v2.` package; both keep
generating and both stay served until `v1` is retired. Say so in `apis/README.md`.

Wire into `api/graphql-server/pom.xml`:

```xml
<plugin>
  <groupId>io.github.kobylynskyi</groupId>
  <artifactId>graphql-codegen-maven-plugin</artifactId>
  <executions>
    <execution>
      <id>generate-graphql-server-v1</id>
      <phase>generate-sources</phase>
      <goals>
        <goal>generate</goal>
      </goals>
      <configuration>
        <graphqlSchemaPaths>
          <path>${maven.multiModuleProjectDirectory}/apis/graphql-server/<service>-v1.graphqls</path>
        </graphqlSchemaPaths>
        <outputDir>${project.build.directory}/generated-sources/graphql-server</outputDir>
        <apiPackageName><base>.api.graphql.v1.generated.api</apiPackageName>
        <modelPackageName><base>.api.graphql.v1.generated.model</modelPackageName>
        <!-- Interfaces only, never implementations: the hand-written @Controller implements the
             resolver interface, so regeneration can't overwrite real logic. Same reasoning as
             interfaceOnly=true for the REST generator. -->
        <generateApis>true</generateApis>
        <generateClient>false</generateClient>
        <!-- The default is javax.validation.constraints.NotNull, which does NOT exist on Spring
             Boot 3+/Jakarta and fails compilation with "package javax.validation does not exist".
             This override is required, not cosmetic. -->
        <modelValidationAnnotation>jakarta.validation.constraints.NotNull</modelValidationAnnotation>
        <!-- The @Generated annotation embeds a build timestamp, making output non-reproducible. -->
        <addGeneratedAnnotation>false</addGeneratedAnnotation>
      </configuration>
    </execution>
  </executions>
</plugin>
```

The generator emits a `QueryResolver`/`MutationResolver` interface plus one `<Field><Operation>Resolver` interface
per field, and the model classes. Implement the resolver interfaces from a hand-written `@Controller` using
`@QueryMapping`/`@MutationMapping`/`@SchemaMapping`.

**Then copy the SDL onto the runtime classpath — this step is not optional.** Spring for GraphQL reads the schema
at startup from `spring.graphql.schema.locations`, whose default is `classpath:graphql/**/` with extensions
`.graphqls`/`.gqls`. The contract lives outside the module, so nothing puts it there by itself, and the failure
mode is a service that compiles perfectly and then refuses to start because the schema is empty:

```xml
<plugin>
  <groupId>org.apache.maven.plugins</groupId>
  <artifactId>maven-resources-plugin</artifactId>
  <executions>
    <execution>
      <id>copy-graphql-schema</id>
      <phase>generate-resources</phase>
      <goals>
        <goal>copy-resources</goal>
      </goals>
      <configuration>
        <!-- classpath:graphql/ — matches spring.graphql.schema.locations' default. -->
        <outputDirectory>${project.build.outputDirectory}/graphql</outputDirectory>
        <resources>
          <resource>
            <directory>${maven.multiModuleProjectDirectory}/apis/graphql-server</directory>
            <includes>
              <include>*.graphqls</include>
            </includes>
          </resource>
        </resources>
      </configuration>
    </execution>
  </executions>
</plugin>
```

Confirm in Step 7 that the file actually landed in `target/classes/graphql/`. If the service sets a non-default
`spring.graphql.schema.locations`, copy to wherever that points instead and note the divergence in the report.

Two more things worth setting and reporting:

- **`spring.graphql.schema.inspection.enabled`** (on by default in recent Boot lines) reports schema fields with no
  corresponding resolver at startup. Leave it on — it's how a field added to the SDL and never implemented gets
  noticed at boot rather than as a runtime `null`.
- **Query depth and complexity limits.** A GraphQL endpoint with neither lets any client issue a deeply nested
  query that walks the object graph until the service dies — the standard denial-of-service shape for GraphQL, and
  not a hypothetical one. There is no default limit. Add `MaxQueryDepthInstrumentation` and
  `MaxQueryComplexityInstrumentation` beans, and say in the report that the chosen numbers are a starting point the
  team must tune against their real schema.

## Step 3b — GraphQL documentation generation, wired into the build

Only if `api/graphql-server/` exists. `graphql-codegen-maven-plugin` generates *code*, not documentation, so unlike
REST (`html2`) and gRPC (`protoc-gen-doc`) the GraphQL contract would otherwise publish nothing. Close that gap the
same way `apis/messaging/` does: an npm toolchain pinned beside the contract, driven from the Maven build by
`frontend-maven-plugin`, writing into `target/generated-docs/` for the workflow to publish.

| Tool | Produces | Role |
|---|---|---|
| `spectaql` | a single self-contained `index.html` reference | the primary reference — types, fields, arguments, enums, inputs and operations, with every SDL description rendered |
| `graphql-voyager` | an interactive schema graph | optional; answers "how do these types connect?", which a flat reference can't |

The two are complements, not alternatives: SpectaQL is the page someone reads to integrate, Voyager is the picture
someone opens to understand the shape of the schema. Generate SpectaQL always; ask before adding Voyager, since it
roughly doubles the published bytes for that API.

Write `apis/graphql-server/docs/package.json`:

```json
{
  "name": "<service>-graphql-docs",
  "private": true,
  "version": "1.0.0",
  "description": "GraphQL documentation toolchain. Not published; not a Maven module.",
  "dependencies": {
    "spectaql": "<latest 3.x>",
    "graphql": "<latest 16.x>",
    "graphql-voyager": "<latest 2.x>"
  }
}
```

`npm install` there once and **commit `package-lock.json`**; add `apis/graphql-server/docs/node_modules/` to
`.gitignore`. Drop `graphql-voyager` from the dependencies if Voyager wasn't wanted — `graphql` stays either way,
since the Voyager build script needs it and SpectaQL wants a matching version.

**SpectaQL** is configured by a YAML file per contract version, at `apis/graphql-server/docs/spectaql-v1.yml`:

```yaml
spectaql:
  targetDir: <replaced on the command line>
  # One self-contained index.html, so publishing is a single file copy and the page
  # works from any path on GitHub Pages.
  oneFile: true
introspection:
  # The SDL is the input — no running server and no introspection endpoint needed,
  # which is what makes this work in CI.
  schemaFile: ../<service>-v1.graphqls
info:
  title: <Service> GraphQL API
  description: The GraphQL interface exposed by this service.
  version: "1.0.0"
  x-url: <the deployed /graphql endpoint>
```

**Voyager** has no CLI and takes an introspection result rather than SDL, so it needs a small build script at
`apis/graphql-server/docs/build-voyager.mjs`. It stays offline by copying Voyager's own standalone bundle next to
the page rather than loading it from a CDN:

```js
import { readFileSync, writeFileSync, mkdirSync, copyFileSync } from 'node:fs';
import { buildSchema, introspectionFromSchema } from 'graphql';

const [, , sdlPath, outDir, title] = process.argv;
// Voyager requires the `data` half of an introspection response — an object with a
// top-level __schema key, which is exactly what introspectionFromSchema returns.
const introspection = introspectionFromSchema(buildSchema(readFileSync(sdlPath, 'utf8')));

mkdirSync(outDir, { recursive: true });
for (const f of ['voyager.standalone.js', 'voyager.css']) {
  copyFileSync(`node_modules/graphql-voyager/dist/${f}`, `${outDir}/${f}`);
}
writeFileSync(`${outDir}/index.html`, `<!doctype html>
<html><head><meta charset="utf-8"><title>${title}</title>
<link rel="stylesheet" href="voyager.css" />
<style>body,html,#voyager{height:100%;margin:0;overflow:hidden}</style>
</head><body>
<div id="voyager">Loading…</div>
<script src="voyager.standalone.js"></script>
<script>
  GraphQLVoyager.renderVoyager(document.getElementById('voyager'), {
    introspection: ${JSON.stringify(introspection)}
  });
</script>
</body></html>
`);
```

Wire both into `api/graphql-server/pom.xml` with `frontend-maven-plugin`, exactly as Step 6b does for AsyncAPI —
same `install-node-and-npm` + `npm ci` prelude, same `<skip>` escape hatch (`${graphql.docs.skip}`), same
`generate-resources` phase as every other documentation generator in the reactor:

```xml
<execution>
  <id>graphql-docs-spectaql</id>
  <phase>generate-resources</phase>
  <goals><goal>npx</goal></goals>
  <configuration>
    <arguments>spectaql spectaql-v1.yml --target-dir ${project.build.directory}/generated-docs/graphql-server/v1</arguments>
  </configuration>
</execution>
<execution>
  <id>graphql-docs-voyager</id>
  <phase>generate-resources</phase>
  <goals><goal>npx</goal></goals>
  <configuration>
    <arguments>node build-voyager.mjs ${maven.multiModuleProjectDirectory}/apis/graphql-server/<service>-v1.graphqls ${project.build.directory}/generated-docs/graphql-server/v1/voyager <Service> GraphQL schema</arguments>
  </configuration>
</execution>
```

`workingDirectory` is `${maven.multiModuleProjectDirectory}/apis/graphql-server/docs` and `installDirectory` is
`${project.build.directory}`, as in Step 6b. The layout that results —
`generated-docs/graphql-server/v1/index.html` with `generated-docs/graphql-server/v1/voyager/index.html` beneath
it — publishes to `doc/api/graphql-server/v1/`, next to `doc/api/rest-server/v1/` and `doc/api/grpc-server/v1/`.
Add one pair of executions per SDL version file. Omit the Voyager execution entirely if it wasn't wanted rather
than leaving a disabled one behind.

Two things to confirm in Step 7 rather than assume:

- **The SpectaQL page shows the SDL descriptions.** An SDL with no description strings produces a structurally
  complete page that documents nothing — the same failure the OpenAPI spec has, and the reason Step 3a insists on
  describing every element.
- **The Voyager page actually draws the graph.** It is a client-side render, so a broken introspection payload
  yields a blank page with a console error and no build failure at all. Open it and look; the bundled
  `voyager.standalone.js` and `voyager.css` must sit beside `index.html` in the published output, since the page
  loads them by relative path.

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

## Step 5a — GraphQL clients

Per entry in `stack.graphqlClients`, put the downstream service's SDL at
`apis/graphql-client/<name>/<name>-api.graphqls`, with the same "this is someone else's contract, replace it with
the real published schema" comment as the other client directories. A GraphQL server can produce its own SDL
(`spring.graphql.schema.printer.enabled` exposes it, and any introspection-enabled endpoint can be dumped with
`gql-cli`/`apollo client:download-schema`), so say in `apis/README.md` that this file should be refreshed from the
downstream service rather than hand-edited.

Wire into `infrastructure/clients/<name>/pom.xml`:

```xml
<configuration>
  <graphqlSchemaPaths>
    <path>${maven.multiModuleProjectDirectory}/apis/graphql-client/<name>/<name>-api.graphqls</path>
  </graphqlSchemaPaths>
  <outputDir>${project.build.directory}/generated-sources/graphql-client-<name></outputDir>
  <apiPackageName><base>.infrastructure.clients.<name>.generated.api</apiPackageName>
  <modelPackageName><base>.infrastructure.clients.<name>.generated.model</modelPackageName>
  <!-- The mirror image of the server configuration: request/response and projection classes,
       no resolver interfaces. -->
  <generateApis>false</generateApis>
  <generateClient>true</generateClient>
  <modelValidationAnnotation>jakarta.validation.constraints.NotNull</modelValidationAnnotation>
  <addGeneratedAnnotation>false</addGeneratedAnnotation>
</configuration>
```

**Client generation needs a runtime dependency that server generation doesn't.** The generated
`<Op>Request`/`<Type>ResponseProjection` classes extend base types from
`io.github.kobylynskyi:graphql-java-codegen` — the same artifact as the plugin, at the same version. Add it as a
compile dependency of the client module (version-managed in the root pom like everything else). Without it the
module generates and then fails to compile with `cannot find symbol: class GraphQLResponseField`, which reads like
a generator bug rather than a missing dependency.

The adapter itself uses Spring's `HttpGraphQlClient` (reactive) or `HttpSyncGraphQlClient` (blocking), matching
`stack.concurrency`; the generated request objects serialise the query and the projections declare which fields
come back.

This SDL is also what the compose stack's mock serves when `stack.apiMock` is Microcks, which reads GraphQL
schemas directly. Put the example responses in this same directory alongside the SDL, as for the other client
kinds — a downstream contract and the responses mocked for it belong in one place. If `stack.apiMock` is WireMock,
note in the report that GraphQL stubs must match on the request body rather than the path (every operation is a
`POST /graphql`) and that the SDL is not consumed by the mock at all, so the two can drift.

## Step 6 — AVRO messaging schemas

Only if `stack.messaging.enabled`. Steps 6, 6a and 6b are all conditional on it.

Write one starter schema per event at
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

## Step 6a — The AsyncAPI specification

The AVRO schemas say what a message *looks like*. They say nothing about which topic carries it, in which
direction, under which consumer group, on which broker — so on their own they can't document the messaging
interface the way the OpenAPI spec documents the REST one. That's what the AsyncAPI document is for.

Write one at `apis/messaging/<service>-async-api-v1.yaml`, declaring the **current AsyncAPI version** — 3.1.0 at
the time of writing. Confirm what's current rather than copying that number: `npx asyncapi validate <spec>` reports
`asyncapi-latest-version` as an informational finding when a newer one exists, so the toolchain tells you directly.

```yaml
asyncapi: 3.1.0
info:
  title: <Service> Events API
  version: 1.0.0
  description: Events this service publishes and consumes.
servers:
  kafka:
    host: <broker-host>:9092
    protocol: kafka
    description: Kafka broker; schemas resolve through the Confluent Schema Registry.
channels:
  orderCreated:
    # `address` is the real topic name — it must match the
    # `spring.cloud.stream.bindings.<binding>.destination` value, or the documentation
    # describes a topic that doesn't exist. The channel *key* is just an internal id.
    address: order.created.v1
    description: Order lifecycle events.
    messages:
      OrderCreated:
        $ref: '#/components/messages/OrderCreated'
operations:
  sendOrderCreated:
    # In AsyncAPI 3 the action is from THIS service's point of view:
    # `send` = this service produces it, `receive` = this service consumes it.
    action: send
    summary: Publish an order-created event.
    channel:
      $ref: '#/channels/orderCreated'
    messages:
      - $ref: '#/channels/orderCreated/messages/OrderCreated'
components:
  messages:
    OrderCreated:
      name: OrderCreated
      title: Order created
      contentType: application/avro
      payload:
        # Reference the .avsc, never restate it. One source of truth for the payload.
        schemaFormat: application/vnd.apache.avro;version=1.9.0
        schema:
          $ref: './order-created-v1.avsc'
```

Three things to get right, each of which silently produces wrong documentation rather than an error:

- **`action` is written from this service's point of view.** `send` is what the service produces, `receive` is what
  it consumes. This is the one thing AsyncAPI 3 deliberately fixed from 2.x, where `publish`/`subscribe` meant the
  opposite of what nearly everyone assumed. If you're adapting an older 2.x document, its `subscribe` becomes
  `send` and its `publish` becomes `receive` — not the other way round.
- **Payloads are `$ref`s to the `.avsc` files**, under a `schemaFormat` + `schema` pair, resolved relative to the
  AsyncAPI file — which is exactly why the two live in the same directory. Restating the fields inline creates a
  second copy that drifts from the one the code is generated from.
- **`address` is the topic name from the binding configuration**, not the channel key. Cross-check against
  `spring.cloud.stream.bindings.*.destination` and say in the report if the bindings don't exist yet.

Cover both directions in this one file if the service both produces and consumes; it documents the service's whole
messaging interface, not one module's half of it. Give `info.description`, every channel, every operation
(`summary`), and every message a real description — as with the OpenAPI spec, the generated pages are only as good
as the text in the contract. Add a `v2` file alongside when a breaking change arrives, following the same rule as
the other contracts.

## Step 6b — AsyncAPI documentation generation, wired into the build

The official AsyncAPI HTML generator runs over the AsyncAPI file on every build, producing a static documentation
site published to GitHub Pages next to the REST and gRPC pages. It is driven by `@asyncapi/cli`, which bundles the
generator; `@asyncapi/html-template` is the template it renders with.

```json
{
  "name": "<service>-asyncapi-docs",
  "private": true,
  "version": "1.0.0",
  "description": "AsyncAPI documentation toolchain. Not published; not a Maven module.",
  "dependencies": {
    "@asyncapi/cli": "<latest 6.x>",
    "@asyncapi/html-template": "<latest 3.x>"
  },
  "overrides": {
    "_comment": "@asyncapi/generator 3.4.0 depends on @asyncapi/generator-hooks 0.1.1, which was never published to npm; without this override `npm install` fails with ETARGET. Drop it once the upstream dependency resolves.",
    "@asyncapi/generator-hooks": "0.1.0"
  }
}
```

Check whether the `overrides` block is still needed before adding it — run `npm install` without it first and keep
it only if npm actually fails with `ETARGET`. A stale override silently pins a transitive dependency below what the
CLI expects.

Write it to `apis/messaging/docs/package.json`, run `npm install` there once, and **commit the resulting
`package-lock.json`** — it is what makes the documentation build reproducible and what lets CI use `npm ci`. Add
`apis/messaging/docs/node_modules/` to `.gitignore`. Verify the install actually succeeds before moving on; if npm
reports `ETARGET` on a package other than `generator-hooks`, pin that one in `overrides` too and record it in the
report rather than leaving a build that only works on your machine.

Then wire the generator into Maven with `frontend-maven-plugin`, so `mvn generate-resources` produces the
documentation with no globally installed Node — the same guarantee the REST and gRPC documentation already have.
Put the executions in **one** module only: `infrastructure/producers` if it exists, otherwise `api/consumers`. The
AsyncAPI document describes the whole service, so generating it in both modules would just publish the same pages
twice under two paths.

```xml
<plugin>
  <groupId>com.github.eirslett</groupId>
  <artifactId>frontend-maven-plugin</artifactId>
  <configuration>
    <!-- package.json lives with the contract it documents, not in the module. -->
    <workingDirectory>${maven.multiModuleProjectDirectory}/apis/messaging/docs</workingDirectory>
    <!-- Node itself is a build artifact: keep it under target/, out of the source tree. -->
    <installDirectory>${project.build.directory}</installDirectory>
    <skip>${asyncapi.docs.skip}</skip>
    <environmentVariables>
      <!-- The CLI otherwise writes ~/.asyncapi-analytics and reports usage to New Relic on
           every build. Pointing at a committed file with analyticsEnabled=false makes the
           build silent and offline. (CI=true also suppresses it, but local builds aren't CI.) -->
      <ASYNCAPI_METRICS_CONFIG_PATH>${maven.multiModuleProjectDirectory}/apis/messaging/docs/.asyncapi-analytics</ASYNCAPI_METRICS_CONFIG_PATH>
    </environmentVariables>
  </configuration>
  <executions>
    <execution>
      <id>asyncapi-install-node</id>
      <phase>generate-resources</phase>
      <goals>
        <goal>install-node-and-npm</goal>
      </goals>
      <configuration>
        <nodeVersion>v<node-major>.<x>.<y></nodeVersion>
      </configuration>
    </execution>
    <execution>
      <id>asyncapi-install-toolchain</id>
      <phase>generate-resources</phase>
      <goals>
        <goal>npm</goal>
      </goals>
      <configuration>
        <!-- `ci`, not `install`: the committed lock file is the contract. -->
        <arguments>ci --no-audit --no-fund</arguments>
      </configuration>
    </execution>
    <execution>
      <id>asyncapi-validate</id>
      <phase>generate-resources</phase>
      <goals>
        <goal>npx</goal>
      </goals>
      <configuration>
        <!-- Ahead of generation, so a malformed contract fails with a validation error
             instead of an obscure generator crash. This resolves and type-checks the
             referenced .avsc files too, so a missing or invalid AVRO schema fails here
             (exit 1) rather than producing a page with empty payload tables. A merely
             out-of-date `asyncapi:` version is reported as information and exits 0, so a
             new specification release can never break the build on its own. -->
        <arguments>asyncapi validate ${maven.multiModuleProjectDirectory}/apis/messaging/<service>-async-api-v1.yaml</arguments>
      </configuration>
    </execution>
    <execution>
      <id>asyncapi-docs-html</id>
      <phase>generate-resources</phase>
      <goals>
        <goal>npx</goal>
      </goals>
      <configuration>
        <!-- singleFile=true emits one self-contained index.html, so publishing is a single
             file copy and the page works from any path on GitHub Pages. -->
        <arguments>asyncapi generate fromTemplate ${maven.multiModuleProjectDirectory}/apis/messaging/<service>-async-api-v1.yaml @asyncapi/html-template -o ${project.build.directory}/generated-docs/messaging/v1 --force-write -p singleFile=true</arguments>
      </configuration>
    </execution>
  </executions>
</plugin>
```

Declare `<asyncapi.docs.skip>false</asyncapi.docs.skip>` as a root-pom property so a developer with no network can
build with `-Dasyncapi.docs.skip=true`, and say so in the report — a build step that downloads Node and ~1600 npm
packages must have an escape hatch, and CI must **not** use it or the published documentation goes stale silently.

The executions bind to `generate-resources`, the same phase as the REST `html2` and gRPC `protoc-gen-doc`
executions, so every API documentation artifact in the repository is produced by one predictable command. Add one
validate/generate pair per AsyncAPI version file, writing into `.../messaging/v2/` when `v2` arrives.

Version numbers: `frontend-maven-plugin` goes in the root pom's `<pluginManagement>` like every other plugin —
add it there (see `iru-setup-java-springboot-pom`) rather than inline here. The npm package versions are the one
exception to this repository's no-inline-versions rule, because they live in `package.json`, not in a pom; pin them
to exact currently-published versions and let the committed lock file do the rest.

## Step 7 — Verify generation actually works

Delegate to `iru-gate-runner`:

```bash
mvn -q clean generate-sources    # every generator runs
mvn -q clean compile             # hand-written code compiles against the generated code
```

Then confirm by inspection:

- generated sources exist under each expected module's `target/generated-sources/`;
- the packages match what Steps 2–6 configured, so the hand-written mappers and controllers resolve;
- if GraphQL is enabled, `target/classes/graphql/` actually contains the SDL, and no generated file imports
  `javax.validation` — both are silent-at-generation, loud-later failures (a service that won't start, and a
  module that won't compile);
- if GraphQL is enabled, `generated-docs/graphql-server/v1/index.html` exists and shows the SDL descriptions, and
  (when Voyager was wanted) `generated-docs/graphql-server/v1/voyager/index.html` exists with
  `voyager.standalone.js` and `voyager.css` beside it;
- generated HTML documentation exists under `target/generated-docs/`, including
  `generated-docs/messaging/v1/index.html` if messaging is enabled;
- that page actually lists the AVRO field names and their `doc` text. The `asyncapi-validate` execution catches the
  usual causes before generation — a `$ref` to a missing `.avsc` and an `.avsc` that isn't valid AVRO both fail it
  with `asyncapi-document-resolved` errors, since the validator resolves and type-checks the referenced schema —
  so an empty payload table here means something subtler, most often a message declared in `components` but never
  wired to a channel or an operation.

A generator that runs but produces nothing usually means a wrong `inputSpec`/`sourceDirectory` path — remember
these are absolute via `${maven.multiModuleProjectDirectory}`, because the contract lives outside the module.

Fix whatever fails. This is the step most likely to need real iteration, particularly the protobuf plugin's
configuration schema and the OpenAPI generator's `library` option names — check the plugin's actual documentation
rather than trying variations.

## Step 8 — Keep generated output out of git and out of analysis

- Confirm `.gitignore` covers `target/` (which already covers `generated-sources` and `generated-docs`). If any
  generator was configured to write outside `target/`, either change it to write inside `target/` — strongly
  preferred — or add an explicit ignore entry and explain why in the report.
- Add `apis/graphql-server/docs/node_modules/` to `.gitignore` if the GraphQL documentation toolchain was set up;
  its `package.json`, `package-lock.json`, `spectaql-*.yml` and `build-voyager.mjs` are committed.
- Add `apis/messaging/docs/node_modules/` to `.gitignore` if the AsyncAPI toolchain was set up — it's the one
  generated directory that can't live under `target/`, because npm requires it beside its `package.json`. Its
  `package.json`, `package-lock.json`, and `.asyncapi-analytics` **are** committed; nothing else in that directory
  is.
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
- **GraphQL** — the generator parses the SDL, so a syntactically broken schema already fails the build. For style
  and breaking-change detection, note GraphQL Inspector (`@graphql-inspector/cli`, with a `diff` command that
  classifies each change as breaking, dangerous, or safe) and `graphql-schema-linter` as the tools to add to CI;
  wire one in if the user wants it now. Breaking-change detection matters more here than for REST, because GraphQL
  has no URL version to hide behind — removing a field breaks every client that selected it.
- **AsyncAPI** — already wired in Step 6b as the `asyncapi-validate` execution, so a malformed contract fails with
  a validation error rather than an obscure generator crash. `asyncapi validate` also reports
  `asyncapi-latest-version` informationally when a newer specification version is available, which is the cheapest
  way to notice the document has fallen behind. Note `asyncapi diff` as the breaking-change check to add to CI
  later.

## Step 10 — Report

Summarize:

- Every contract file created, with its path and what it describes.
- Every generator execution added, to which module pom, generating into which package, and where its HTML
  documentation lands.
- The `reactive`/`library` settings resolved from `stack.concurrency`, and the protobuf plugin flavour detected.
- For GraphQL: where the SDL is copied to for runtime loading, that `modelValidationAnnotation` was set to the
  jakarta annotation, whether `graphql-java-codegen` was added to each GraphQL client module, the query
  depth/complexity limits chosen, whether Voyager was generated alongside SpectaQL, and the published paths of
  both.
- For messaging: the AsyncAPI version declared, which module hosts the documentation executions, the exact npm
  package versions pinned in `apis/messaging/docs/package.json`, whether `npm install` needed any `overrides` at
  all, and the Node version pinned in `frontend-maven-plugin`.
- The result of Step 7, including whether HTML documentation was actually produced for REST, gRPC, GraphQL and
  messaging, whether the messaging page's payload tables came out populated rather than empty, and whether the
  GraphQL pages showed real descriptions and a drawn schema graph.
- Whether the analysis exclusions in Step 8 already covered the generated packages or had to be corrected.
- What linting was wired versus merely recommended.

Warn explicitly:

- **The client specs under `apis/rest-client/` and `apis/grpc-client/` are placeholders for someone else's
  contract** — they must be replaced with the downstream service's real published spec before the client adapters
  mean anything, and kept in sync afterwards. They have two consumers, not one: the client generator *and* the
  compose stack's API mock (`stack.apiMock`), so a spec left as a placeholder produces both a useless client and a
  mock that agrees with it. Report which client specs still carry no `examples`, since those mock nothing.
- **Generated code must never be committed or edited.** Fix the spec or the generator configuration instead.
- **A GraphQL endpoint has no depth or complexity limit by default**, and the numbers set here are a guess until
  someone tunes them against the real schema. Until then a single nested query can exhaust the service. This is the
  one item in this skill's output that is a live availability risk rather than a tidiness concern.
- **The GraphQL SDL is read twice — at build time and at startup.** Changing where it lives, or the
  `spring.graphql.schema.locations` property, breaks the second one only, and only at runtime.
- The AVRO registry compatibility mode and the gRPC field-numbering discipline are enforced outside this
  repository (in the registry, and by review or `buf breaking`) — the scaffold can't enforce either.
- **Adding an event means editing the `.avsc` *and* the AsyncAPI file.** An `.avsc` added alone compiles to a Java
  class that appears in no documentation at all, and nothing in the build will say so. State which direction
  (`send`/`receive`) each operation documents, since an inverted action is the one error here that produces a
  confident, wrong diagram rather than a failure.
- Every contract's descriptions and constraints feed the documentation pages, so an under-described spec produces
  under-described documentation. `/iru-update-java-springboot-documentation` reads these files.
