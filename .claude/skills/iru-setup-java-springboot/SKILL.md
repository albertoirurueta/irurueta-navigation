---
name: iru-setup-java-springboot
description: End-to-end bootstrap for a brand-new Spring Boot service repository built on DDD + hexagonal architecture as a Maven multi-module reactor (`boot`, `application`, `domain`, `infrastructure/*`, `api/*`). Collects the project's identity (groupId, artifactId, base Java package), the target cloud (AWS or Google Cloud), the concurrency model (reactive/blocking/both), and every technology choice (databases + index/migration tooling, caches, security, Kafka via Spring Cloud Stream + AVRO, REST/gRPC/GraphQL servers and clients, the API mocking tool backing those clients in tests — Microcks or WireMock — logging, metrics, tracing, Spring AI, dynamic configuration), resolving the latest Spring Boot version and validating every dependency id against the live Spring Initializr metadata API (`https://start.spring.io/metadata/client`) rather than a hardcoded list. Records all of it in a `springboot-stack.yml` manifest, then orchestrates `iru-setup-java-springboot-pom` (root reactor pom with Javadoc/Checkstyle/PMD/SpotBugs/Surefire/Failsafe/JaCoCo/JXR/site/groovy build-info/Sonar plugins), `iru-setup-java-springboot-modules` (module tree, ports/adapters skeleton, configuration), `iru-setup-java-springboot-apis` (the `apis/` spec tree plus OpenAPI/protobuf/GraphQL-SDL/AVRO code generation and AsyncAPI documentation generation), `iru-setup-java-springboot-testcontainers` (docker compose + Testcontainers integration-test harness), `iru-setup-java-springboot-platform` (OpenTofu deployment), `iru-setup-java-springboot-github-workflows` (test/publish, deploy and undeploy workflows), and `iru-update-java-springboot-documentation` (Antora site with Mermaid + Kroki and the full template page set). Enforces Java 21 as a minimum. Invoke as `/iru-setup-java-springboot`. Use when starting a new Spring Boot microservice from nothing and you want the whole hexagonal reactor, CI/CD, infrastructure-as-code, and documentation scaffold in one pass, instead of running the sub-skills separately and re-answering the same questions each time.
model: sonnet
---

# Setup Java Spring Boot Service

Bootstrap a brand-new Spring Boot service repository in one pass. This skill is the **interviewer and
orchestrator**: it asks every question exactly once, resolves the Spring Boot version and dependency ids against
the live Spring Initializr API, writes the resolved answers to a `springboot-stack.yml` manifest at the repository
root, and then delegates the actual file generation to seven sub-skills that each read that manifest back.

The architecture is not negotiable — it is what this skill exists to impose:

- **Domain-Driven Design + hexagonal (ports & adapters)**, realised as a **Maven multi-module reactor**.
- The **root `pom.xml` is the single place any dependency or plugin version is declared**. Module poms list
  artifacts without `<version>`, inheriting from the root's `<dependencyManagement>`/`<pluginManagement>`.
- **Java 21 is the enforced minimum** (`maven.compiler.release`, `java.version`, and a
  `maven-enforcer-plugin` `requireJavaVersion` rule). Never generate a project below 21, even if Spring
  Initializr's own default is lower — as of this writing its default `javaVersion` is `17`, which this skill
  overrides.

## Module layout this skill imposes

`<a>` below is the project `artifactId`; directory names are the short forms.

| Directory | artifactId | Packaging | May depend on |
|---|---|---|---|
| `.` | `<a>-parent` | `pom` | — (declares all versions) |
| `domain/` | `<a>-domain` | `jar` | nothing in this project |
| `application/` | `<a>-application` | `jar` | `domain` |
| `infrastructure/` | `<a>-infrastructure` | `pom` | — (aggregator) |
| `infrastructure/database/` | `<a>-infrastructure-database` | `pom` | — (aggregator) |
| `infrastructure/database/<engine>/` | `<a>-infrastructure-database-<engine>` | `jar` | `domain` |
| `infrastructure/configuration/` | `<a>-infrastructure-configuration` | `jar` | `domain` |
| `infrastructure/clients/` | `<a>-infrastructure-clients` | `pom` | — (aggregator) |
| `infrastructure/clients/<name>/` | `<a>-infrastructure-clients-<name>` | `jar` | `domain` |
| `infrastructure/producers/` | `<a>-infrastructure-producers` | `jar` | `domain` |
| `infrastructure/metrics/` | `<a>-infrastructure-metrics` | `jar` | `domain` |
| `api/` | `<a>-api` | `pom` | — (aggregator) |
| `api/rest-server/` | `<a>-api-rest-server` | `jar` | `application`, `domain` |
| `api/grpc-server/` | `<a>-api-grpc-server` | `jar` | `application`, `domain` |
| `api/graphql-server/` | `<a>-api-graphql-server` | `jar` | `application`, `domain` |
| `api/consumers/` | `<a>-api-consumers` | `jar` | `application`, `domain` |
| `boot/` | `<a>-boot` | `jar` | every other module |
| `coverage/` | `<a>-coverage` | `pom` | every other module (test-scope, aggregation only) |

Only the modules the user's technology choices actually call for are created — a project with no gRPC gets no
`api/grpc-server/`, no GraphQL gets no `api/graphql-server/`, and no Kafka gets no `infrastructure/producers/` or
`api/consumers/`. `domain`, `application`, `boot`, and `coverage` are always created.

`apis/` at the repository root is **not** a Maven module — it holds the API contracts
(`apis/rest-server/`, `apis/grpc-server/`, `apis/graphql-server/`, `apis/rest-client/<name>/`,
`apis/grpc-client/<name>/`, `apis/graphql-client/<name>/`, `apis/messaging/` — AVRO schemas *and* the AsyncAPI
specification describing the topics that carry them) that the code-generation and documentation generators read.
See `iru-setup-java-springboot-apis`.

## Step 0 — Check the repository is a safe starting point

- If `pom.xml` already exists at the repository root, this repository has already been bootstrapped (by this skill
  or otherwise). Use `AskUserQuestion` to ask whether to **stop** (recommended — re-running the whole bootstrap
  over an existing reactor risks clobbering real code) or **continue anyway**, in which case every sub-skill is
  told to treat existing files as gaps-to-fill and never to overwrite a file containing hand-written code.
- If `springboot-stack.yml` already exists, read it and offer its values as the defaults for every question below,
  so a re-run is a cheap "add one more database/client" pass rather than a full re-interview.
- Verify prerequisites and report any that are missing before asking anything (a missing one is not a hard stop —
  the generated files are still correct, they just can't be verified in Step 12):
  `java -version` (must be 21+), `mvn -v`, `docker info` (needed by Testcontainers), `curl --version`,
  `git rev-parse --show-toplevel`.

## Step 1 — Resolve Spring Boot versions and the dependency catalogue from the live API

**Never rely on remembered Spring Boot version numbers, starter artifact ids, or Initializr dependency ids.**
Boot renames artifacts across major versions (in Boot 4.x, `spring-boot-starter-web` became
`spring-boot-starter-webmvc`, the Testcontainers modules became `testcontainers-<engine>`, and every starter
gained a matching `spring-boot-starter-<x>-test` module), and Initializr's dependency catalogue changes with it.
Fetch the truth instead:

```bash
curl -s https://start.spring.io/metadata/client -H 'Accept: application/json' -o /tmp/initializr.json
```

From that JSON, extract and keep for the rest of this run:

- `bootVersion.default` — the current GA release; this is what you offer the user.
- `bootVersion.values[]` — the full list. Entries ending in `.BUILD-SNAPSHOT` or containing `-M`/`-RC` are
  pre-release; never offer one as the default, and if the user picks one, warn that it isn't production-ready.
- `javaVersion.values[]` — the language levels this Initializr accepts. Filter to `>= 21`.
- `dependencies.values[].values[]` — every valid dependency, each with `id`, `name`, `description`, and a
  compatibility range. **This is the authoritative id list.** Step 4's mapping table gives the ids observed at the
  time of writing; for every id you intend to send, confirm it appears in this list, and if it doesn't, find the
  replacement by matching on the human-readable `name` instead of guessing a new id. Report any id you couldn't
  resolve rather than silently dropping the capability.

If the API is unreachable (offline, proxy, firewall), stop and tell the user — do not fall back to hardcoded
version numbers and artifact names. This skill's correctness depends on that call.

Then use `AskUserQuestion` to confirm:

- **Spring Boot version** — offer `bootVersion.default` as the recommended choice, the previous GA line as the
  conservative alternative, and "other (specify)".
- **Java version** — offer `21` as the default and the highest available LTS (e.g. `25`) as the alternative. If
  the user asks for anything below 21, refuse that value, explain that this catalog enforces 21 as a minimum, and
  re-ask. Note that Java 25+ may not yet be supported by every third-party plugin in the reactor, so `21` stays
  the safe recommendation.

## Step 2 — Collect the project identity

Ask in plain conversation (these are free-text fields, not bounded choices):

- **groupId** — e.g. `com.example`.
- **artifactId** — e.g. `orders-service`. Used as the `<a>` prefix for every module artifactId.
- **version** — default `0.0.1-SNAPSHOT`.
- **Base Java package** — e.g. `com.example.orders`. Need not equal `groupId`. Every module's package is derived
  from it (`<base>.domain`, `<base>.application`, `<base>.infrastructure.database.mongodb`, `<base>.api.rest`, …),
  and `boot`'s `@SpringBootApplication` class lives directly in `<base>`.
- **Service description** — one line; feeds the poms, the README, and the documentation overview page.
- **Developer name / email / organizationUrl** — for the root pom's `<developers>` block.

Derive, don't ask:

- **Owner/repo/host** from `git remote get-url origin` (handles both `git@host:owner/repo.git` and
  `https://host/owner/repo.git`). Feeds `<url>`/`<scm>` in the root pom and the Sonar project key. If there's no
  `origin` remote yet, ask for the intended repository URL.
- **Inception year** from `git log --reverse --format=%ad --date=format:%Y | head -1`, falling back to the current
  year.

## Step 3 — Ask the two decisions that shape everything else

Use `AskUserQuestion` for both.

**Deployment platform** — this drives `iru-setup-java-springboot-platform`'s whole OpenTofu layout, the dynamic
configuration mechanism in Step 4, and the deploy workflows:

- **Amazon Web Services** — OpenTofu targets ECS Fargate or EKS, with AWS managed data services.
- **Google Cloud** — OpenTofu targets Cloud Run or GKE Autopilot, with Google managed data services.

**Concurrency model** — this decides the starter for every single technology chosen in Step 4, so it must be
settled first:

- **Non-reactive / blocking (Spring MVC on Tomcat)** — recommended default unless there's a concrete reason
  otherwise. Simpler to write, debug, and profile; virtual threads (`spring.threads.virtual.enabled=true` on Java
  21+) already remove most of the thread-per-request scaling argument. Uses the blocking `spring-data-*` starters.
- **Reactive (Spring WebFlux on Netty, Project Reactor)** — choose when the service is predominantly I/O-bound
  fan-out with very high concurrency and every downstream dependency has a reactive driver. Uses the
  `spring-data-*-reactive` starters.
- **Both** — a blocking web tier plus reactive clients (or vice versa). Pulls in Tomcat *and* Netty and both
  driver families. Warn the user this is the hardest to reason about: two runtime models coexist, blocking calls
  on a Netty event-loop thread will stall the whole server, and both dependency sets must be kept in sync. Only
  worth it for a genuine migration or a genuinely mixed workload.

Record the answer as `stack.concurrency: blocking | reactive | both`. For every technology below, "the matching
starter" means: blocking → the plain starter; reactive → the `-reactive`/`r2dbc` starter; both → both.

## Step 4 — Choose the technologies

Work through the groups below **in order**. For each group, present the options as a formatted list including the
"when this is the right choice" guidance verbatim — that guidance is the point of this step, not decoration —
then ask in plain conversation which ones the user wants (multi-select where noted). Use `AskUserQuestion` only
for the genuinely bounded single-choice sub-questions (migration tooling, SQL engine, dynamic configuration),
since it caps out at four options.

For every accepted choice, add the corresponding Initializr dependency id(s) to a running list, and validate each
id against Step 1's catalogue before it goes in.

### 4a — Databases (multi-select, may be none)

- **MongoDB** — good when data fits in single independent documents, the structure changes freely over time, and
  many indexes are needed to search that data. Not for workloads that need real transactional guarantees.
  - ids: `data-mongodb` (blocking) / `data-mongodb-reactive` (reactive).
  - Index and schema-change management is **mandatory**; ask with `AskUserQuestion`:
    - **Mongock** (recommended) — purpose-built for MongoDB change units, integrates cleanly with Spring Data.
      Add `io.mongock:mongock-springboot` + `io.mongock:mongodb-springdata-v4-driver` (version in the root pom).
    - **Liquibase for MongoDB** — only worth it when Liquibase is already used for another database in the same
      service and one migration tool for everything is worth more than Mongock's better MongoDB ergonomics. Add
      `org.liquibase.ext:liquibase-mongodb`.
  - Testcontainers image: `mongo:<pinned-tag>`.
- **Couchbase** — good when data fits in single independent documents, the structure changes freely, and no
  indexes beyond lookup-by-id are needed. Not for workloads needing transactional guarantees. In that specific
  shape it usually outperforms MongoDB.
  - ids: `data-couchbase` / `data-couchbase-reactive`.
  - Index management is mandatory: **Liquibase for Couchbase** (`org.liquibase.ext:liquibase-couchbase`).
  - Testcontainers image: `couchbase/server:<pinned-tag>`.
- **Relational / SQL** — good when data must be structured in a very specific way, aggregation queries run over
  it, or transactional guarantees are mandatory. Generally slower than MongoDB or Couchbase for
  document-shaped access.
  - Engine (`AskUserQuestion`, exactly one): **PostgreSQL** (recommended) or **MariaDB**. If PostgreSQL is
    chosen, note that the `pgvector` extension gives it vector storage, so it can also back Spring AI embeddings
    (see 4h) — and pin the Testcontainers image to `pgvector/pgvector:<pg-tag>` rather than plain `postgres` so
    that stays available.
  - ids: blocking → `data-jpa` + (`postgresql` | `mariadb`); reactive → `data-r2dbc` + (`postgresql` |
    `mariadb`) — Initializr adds the R2DBC driver alongside the JDBC one automatically.
  - Data-model versioning and index creation is mandatory; ask with `AskUserQuestion`:
    - **Liquibase** (recommended default) — id `liquibase`.
    - **Flyway** — id `flyway`.
  - Testcontainers image: `pgvector/pgvector:<tag>` (or `postgres:<tag>` if vectors are definitely not needed) /
    `mariadb:<tag>`.
- **Elasticsearch** — good for full-text search, faceted search (e-commerce), and log aggregation. Almost always
  used *alongside* another database in a CQRS arrangement, not as the system of record — say so explicitly, and
  if it's the only database selected, ask the user to confirm that's intentional.
  - ids: `data-elasticsearch`.
  - Testcontainers image: `docker.elastic.co/elasticsearch/elasticsearch:<tag>`.
- **Neo4j** — good when the relationships between entities matter as much as or more than the entities: fraud
  detection, recommendation engines, social graphs, knowledge graphs, dependency graphs. Typically used alongside
  another database in a CQRS arrangement.
  - ids: `data-neo4j`.
  - Migration management is mandatory: **Liquibase for Neo4j** (`org.liquibase.ext:liquibase-neo4j`).
  - Testcontainers image: `neo4j:<tag>`.
- **Qdrant** — the preferred choice over PostgreSQL + `pgvector` when a large volume of vectors is stored: a
  dedicated vector engine outperforms sharing one PostgreSQL instance between application data and embeddings.
  - ids: `spring-ai-vectordb-qdrant` (Qdrant reaches the service through Spring AI's vector store abstraction).
  - Testcontainers image: `qdrant/qdrant:<tag>`.

### 4b — Caches (multi-select, may be none)

- **Caffeine** — best for a local, per-node cache. Each replica keeps its own copy, so it's ideal for small or
  static, infrequently-changing data. Fastest option, no network hop.
  - ids: `cache` (Spring's cache abstraction). Caffeine itself is not an Initializr dependency — add
    `com.github.ben-manes.caffeine:caffeine` explicitly, with its version in the root pom.
- **Redis** — best for a cache shared across all replicas: large cached datasets, or data that changes often
  enough that every node must see the change at once. Slightly slower than a local cache because of the network
  hop.
  - ids: `data-redis` / `data-redis-reactive`, plus `cache` if `@Cacheable` is wanted on top.
  - Testcontainers image: `redis:<tag>`.

If both are selected, note that the conventional arrangement is a two-level cache (Caffeine in front of Redis)
and that it needs deliberate invalidation design — flag it for the architecture documentation page.

### 4c — Spring Security (yes/no)

Needed when OAuth login flows must be implemented, or endpoints must be properly authorized rather than open.

- ids: `security`, plus `oauth2-resource-server` when the service validates incoming JWTs (the usual case for a
  microservice behind a gateway) and/or `oauth2-client` when it initiates login flows itself. Ask which of the
  two applies; both is valid.

### 4d — Spring Cloud Stream / Kafka (yes/no)

Needed for Kafka messaging — publishing or consuming public or private events. This is the standard tool for
eventual consistency in non-transactional architectures (microservices) and for saga orchestration over public
events.

- ids: `cloud-stream`, `kafka`. Add `kafka-streams` only if the user says they need stateful stream processing.
- AVRO is the message format. `avro-maven-plugin` generates Java classes from the schemas in `apis/messaging/` —
  configured by `iru-setup-java-springboot-apis`.
- The same directory also holds an **AsyncAPI specification** describing the topics, their direction, and which
  message each carries — the messaging equivalent of the OpenAPI spec. It is documentation only (no Java is
  generated from it), rendered to HTML on every build by `@asyncapi/html-template` and published to GitHub Pages
  alongside the REST and gRPC pages.
- Creates the `infrastructure/producers/` module (outbound) and/or the `api/consumers/` module (inbound) — ask
  which directions this service needs; at least one must be true.
- Testcontainers needs both a **Kafka broker** image and a **Confluent Schema Registry** image, since AVRO
  serialization resolves schemas through the registry at runtime.

### 4e — REST server (yes/no)

Needed to serve requests over a REST API.

- ids: `web` (blocking) / `webflux` (reactive) / both. Always add `validation`.
- Contract-first: OpenAPI specs live in `apis/rest-server/`, and `openapi-generator-maven-plugin` generates the
  controller interfaces and DTOs into `api/rest-server/`, plus HTML documentation.

### 4f — gRPC server (yes/no)

Needed to serve requests over gRPC.

- ids: `spring-grpc-server` (Spring Boot 4 ships first-party gRPC support; in older lines this was the separate
  `spring-grpc`/`net.devh` starter — trust Step 1's catalogue). Reactive and blocking service implementations
  use the same starter; the difference is in the generated stub flavour.
- Contract-first: protobuf definitions live in `apis/grpc-server/`, and `protobuf-maven-plugin` generates the
  service base classes and messages into `api/grpc-server/`, plus HTML documentation via `protoc-gen-doc`.

### 4g — GraphQL server (yes/no)

Needed to serve requests over a GraphQL API — one endpoint whose clients choose the fields they want, instead of
one endpoint per resource shape.

- ids: `graphql`, **plus a transport starter**: `web` (blocking) or `webflux` (reactive), matching
  `stack.concurrency`. This pairing is not optional and is the easiest thing to get wrong here:
  `spring-boot-starter-graphql` brings the GraphQL engine and `spring-graphql`, but **no HTTP transport at all** —
  Initializr's `graphql` id alone resolves to `spring-boot-starter-graphql` and `spring-boot-starter-graphql-test`
  and nothing else. Without `web` or `webflux` the application starts and serves no `/graphql` endpoint, which
  looks like a routing bug rather than a missing dependency. If REST (4e) is also enabled, the transport starter is
  already there and isn't added twice.
- Also add `validation`, as for REST — the generated input types carry Bean Validation annotations.
- Creates the `api/graphql-server/` module.
- Contract-first: SDL schema files live in `apis/graphql-server/`, and `graphql-codegen-maven-plugin` generates the
  resolver interfaces and model types into `api/graphql-server/`. As with REST, the generator owns the interfaces
  and the hand-written `@Controller` implements them.
- **The SDL is needed twice**: once at build time by the generator, and once at *runtime* by Spring for GraphQL,
  which loads the schema from `classpath:graphql/**/` (`.graphqls`/`.gqls`) to wire and validate the resolvers. The
  contract lives outside the module, so the build copies it onto the classpath —
  `iru-setup-java-springboot-apis` sets that up. A service whose generated code compiles but whose SDL never
  reached the classpath fails at startup complaining the schema is empty.

### 4h — Clients (multi-select, may be none)

- **REST clients** — needed to call other services' REST APIs. Ask for the **name of each** client (one per
  downstream service, e.g. `payments`, `inventory`); each becomes an
  `infrastructure/clients/<name>/` module and an `apis/rest-client/<name>/` spec directory, with
  `openapi-generator-maven-plugin` generating the client and DTOs.
  - ids: `spring-restclient` (blocking) / `spring-webclient` (reactive).
- **gRPC clients** — needed to call other services' gRPC APIs. Ask for the **name of each**; each becomes an
  `infrastructure/clients/<name>/` module and an `apis/grpc-client/<name>/` spec directory, with
  `protobuf-maven-plugin` generating the stubs.
  - ids: `spring-grpc-client`.
- **GraphQL clients** — needed to call other services' GraphQL APIs. Ask for the **name of each**; each becomes an
  `infrastructure/clients/<name>/` module and an `apis/graphql-client/<name>/` SDL directory, with
  `graphql-codegen-maven-plugin` generating the request/response and projection classes.
  - ids: `graphql` (the same starter as 4g — it carries `HttpGraphQlClient`/`HttpSyncGraphQlClient`, so a service
    that only *consumes* GraphQL still needs it), plus the transport starter for `stack.concurrency` if 4e or 4g
    hasn't already added one.
  - Unlike the REST and gRPC client generators, this one needs a **runtime dependency** alongside the generated
    code — `io.github.kobylynskyi:graphql-java-codegen`, matching the plugin version, which supplies the
    `GraphQLRequest`/`GraphQLResponseProjection` base classes the generated classes extend. Without it the module
    generates fine and then fails to compile.

A single downstream service reached over more than one protocol keeps **one** `infrastructure/clients/<name>/`
module with a generator execution per protocol — the module is named after the service, not the transport.

**If — and only if — at least one client of any kind was named**, ask which API mocking tool the docker compose
stack should run, using `AskUserQuestion`. Integration tests must never call a real downstream service: that
couples this repository's CI to someone else's uptime and makes error-path coverage impossible. Both tools are
driven by the contracts in `apis/rest-client/<name>/`, `apis/grpc-client/<name>/` and
`apis/graphql-client/<name>/`, so neither introduces a second description of the downstream API. Preselect the
default by what was chosen above:

- **Microcks** — the default whenever **any gRPC or GraphQL client** exists, with or without REST clients. It mocks
  REST, gRPC and GraphQL from the same contracts in one container, consuming the OpenAPI spec, the `.proto` and the
  SDL directly, and can later verify the real downstream service against those same contracts.
- **WireMock** — the default when the service has **REST clients only**. Lighter, faster to start, with a
  finer-grained stub DSL for delays and faults. Mention when offering it that a `wiremock-grpc-extension` exists,
  so it isn't a dead end if gRPC arrives later — it just adds a descriptor-set build step at that point. For
  GraphQL it is a weaker fit: every operation is a `POST /graphql`, so stubs have to match on the request body
  rather than on the path, and the SDL isn't consumed at all — the mock and the contract stop being connected,
  which is the property this whole arrangement exists to preserve.

Record the answer as `stack.apiMock`. `iru-setup-java-springboot-testcontainers` adds the container and points
every client's base URL at it; if no client was named, set `stack.apiMock: none` and no mock is added.

### 4i — Observability (ask each; recommend all three)

- **Logging** — needed to get log traces into a centralized server; by default they only go to the local console.
  Add the logging starter and configure structured (JSON) output for deployed profiles while keeping
  human-readable console output for the `local`/`dev` profile. The user can always turn logging off via
  configuration. Recommend it.
- **Metrics** — needed to report technical and functional metrics and get real insight into service behaviour.
  - ids: `actuator`, `prometheus` (Micrometer's Prometheus registry is the backend), optionally `otlp-metrics` if
    metrics should also go out over OTLP.
  - Creates the `infrastructure/metrics/` module: a domain-defined metrics port, implemented here over
    Micrometer, including the tags each metric carries and the mapping from domain entities to tag values.
  - Local execution gets **Prometheus and Grafana** containers in the docker compose stack so metrics are
    actually visible while developing.
- **Tracing** — needed to see how each received request flowed and what response was produced.
  - ids: `distributed-tracing`, `opentelemetry`. (In Boot 4 these resolve to
    `spring-boot-starter-opentelemetry`.)

### 4j — Spring AI (yes/no)

Used to integrate with LLMs — building "agentic" services or MCP servers/clients.

If yes:

1. Warn first that this **requires a paid subscription with an LLM provider, billed separately from any cloud
   spend** — token cost is the user's, not this scaffold's.
2. Offer the providers actually present in Step 1's catalogue. As observed at the time of writing these include:
   Anthropic Claude (`spring-ai-anthropic`), OpenAI (`spring-ai-openai`), Google GenAI
   (`spring-ai-google-genai`), Amazon Bedrock (`spring-ai-bedrock-converse`), Mistral AI (`spring-ai-mistral`),
   DeepSeek (`spring-ai-deepseek`), and Ollama (`spring-ai-ollama`, local models — no subscription needed).
   Present the live list, not this one. If the deployment platform from Step 3 is AWS, mention Bedrock, and if
   it's Google Cloud mention Google GenAI, as the options that authenticate via the platform's own IAM instead of
   a separate API key.
3. If embeddings will be stored, ask for the vector store and keep it consistent with 4a:
   `spring-ai-vectordb-qdrant` (preferred at volume), `spring-ai-vectordb-pgvector` (fine when PostgreSQL is
   already in play and vector volume is modest), or none.
4. Also offer `spring-ai-mcp-server` / `spring-ai-mcp-client` if the service is meant to expose or consume MCP.
5. **Never write an API token into the repository in clear text.** State this explicitly to the user, and record
   it as a required manual step in Step 11's report:
   - The generated configuration references the token only as `${<PROVIDER>_API_KEY}` — an environment variable
     resolved at runtime, with **no default value and no example key committed anywhere**.
   - For local development: export it in the shell, or put it in a `.env` file that is **git-ignored** (this
     skill adds `.env` to `.gitignore`). Never in `application.yml`.
   - For CI: a GitHub Actions secret, exposed to the job as an environment variable.
   - For deployed environments: the platform's secret manager — AWS Secrets Manager or Google Secret Manager —
     injected into the container by the OpenTofu configuration, never baked into the image.
   - Tell the user that if a token has ever been committed, rotating it at the provider is the only real fix, and
     `/iru-check-security` can scan the repository for accidentally committed secrets.

### 4k — Dynamic configuration (`AskUserQuestion`, exactly one)

Needed to change service behaviour while it is running, with no restart and no redeployment. Default the
recommendation from Step 3's platform choice:

- **Kubernetes ConfigMaps and Secrets** — recommended when deploying to GKE, or to Cloud Run/EKS backed by
  Kubernetes. Add `org.springframework.cloud:spring-cloud-starter-kubernetes-client-config` (it has no
  Initializr id in the current catalogue, so declare it explicitly with its version in the root pom, and verify
  the Spring Cloud release train in use actually ships a build compatible with the chosen Boot version — report
  it as an open gap if it doesn't).
- **AWS Secrets Manager / Parameter Store** — recommended when deploying to AWS. Add
  `io.awspring.cloud:spring-cloud-aws-starter-secrets-manager` (and
  `spring-cloud-aws-starter-parameter-store` for non-secret dynamic properties), with the
  `io.awspring.cloud:spring-cloud-aws-dependencies` BOM imported in the root pom.
- **Spring Cloud Config Server** — the platform-independent option: add `cloud-config-client`, and `cloud-bus`
  as well if a config change must be broadcast to every replica at once rather than polled.
- **None** — configuration changes require a redeploy. Acceptable for a service with no operationally-tunable
  behaviour, but note in Step 11's report that this removes the fastest mitigation lever during an incident.

Whichever is chosen, every dynamically reloadable property must be bound in the
`infrastructure/configuration/` module on `@ConfigurationProperties` beans annotated `@RefreshScope`, so a
refresh is picked up without a restart — and every one of them belongs in the documentation's configuration table
with its "dynamically changeable" column set accordingly.

### 4l — Anything else from Initializr

Show the user that the remaining groups in Step 1's catalogue are available (Developer Tools, Template Engines,
Spring Cloud Discovery/Routing/Circuit Breaker, Ops, Testing, …) and offer to add any by id or name. Warn that
anything outside the groups above is typically **less relevant to this architecture**, and that some entries
(template engines, `data-rest`, `cloud-gateway`) actively conflict with the contract-first, hexagonal shape this
scaffold imposes — flag such a conflict rather than silently accepting it.

Always include, regardless of the user's answers: `lombok` (boilerplate generation),
`configuration-processor` (configuration metadata), `validation`, `testcontainers`, and `docker-compose`.
MapStruct has no Initializr id — it's added explicitly in the root pom together with the
`lombok-mapstruct-binding` needed to make the two annotation processors coexist.

## Step 5 — Confirm the whole plan before writing anything

Present a single compact summary: project identity, Boot and Java versions, platform, concurrency model, every
selected technology with the dependency ids it resolved to, the migration tool per database, the client names and
the mocking tool chosen for them,
the exact list of Maven modules that will be created, and the docker compose services the integration-test stack
will run. Ask for explicit confirmation, or for corrections, and loop back to the relevant part of Step 4 if the
user changes something. **No file is written before this confirmation.**

## Step 6 — Write `springboot-stack.yml`

Write the confirmed answers to `springboot-stack.yml` at the repository root. Every sub-skill reads this file
instead of re-asking, and `iru-update-java-springboot-documentation` reads it later to know what the service is
built from. It is meant to be committed.

```yaml
# Generated by /iru-setup-java-springboot. Records the technology choices this service was scaffolded with.
# Sub-skills read this file; keep it in sync by re-running /iru-setup-java-springboot when the stack changes.
project:
  groupId: com.example
  artifactId: orders-service
  version: 0.0.1-SNAPSHOT
  basePackage: com.example.orders
  description: Handles the order lifecycle.
  javaVersion: "21"
  springBootVersion: "4.1.0"
  repository:
    host: github.com
    owner: example
    name: orders-service
  inceptionYear: "2026"
  developer:
    name: Jane Doe
    email: jane@example.com
    organizationUrl: https://github.com/jane
stack:
  concurrency: blocking          # blocking | reactive | both
  cloud: aws                     # aws | gcp
  databases:
    - engine: mongodb            # mongodb | couchbase | postgresql | mariadb | elasticsearch | neo4j | qdrant
      migrations: mongock        # mongock | liquibase-mongodb | liquibase-couchbase | liquibase-neo4j | liquibase | flyway | none
      image: mongo:8.0
  caches: [caffeine, redis]      # caffeine | redis
  redisImage: redis:8.2
  security:
    enabled: true
    modes: [oauth2-resource-server]
  messaging:
    enabled: true
    directions: [produce, consume]
    kafkaImage: apache/kafka:4.1.0
    schemaRegistryImage: confluentinc/cp-schema-registry:8.0.0
  restServer: true
  grpcServer: false
  graphqlServer: false
  restClients: [payments]
  grpcClients: []
  graphqlClients: []
  apiMock: wiremock              # microcks | wiremock | none — required whenever any client is listed
  apiMockImage: wiremock/wiremock:3.13.1
  logging: true
  metrics: true
  tracing: true
  ai:
    enabled: false
    providers: []
    vectorStore: none
    mcp: []
  dynamicConfig: aws-secrets-manager   # kubernetes | aws-secrets-manager | spring-cloud-config | none
  extras: []
initializr:
  dependencies: [webmvc-or-web-id, data-mongodb, cloud-stream, kafka, actuator, prometheus, ...]
modules:
  - domain
  - application
  - infrastructure/database/mongodb
  - infrastructure/configuration
  - infrastructure/clients/payments
  - infrastructure/producers
  - infrastructure/metrics
  - api/rest-server
  - api/consumers
  - boot
  - coverage
```

Pin every container image to a concrete tag — never `latest` — so integration tests are reproducible. Look up the
current stable tag for each image rather than inventing one, and record it in this manifest so the compose file,
the Testcontainers harness, and the OpenTofu engine versions all agree.

## Step 7 — Fetch the reference pom from Spring Initializr

With the confirmed answers, fetch the single-module pom Initializr would have produced. This is the **source of
truth for artifact names, BOM imports, and version properties** — not a template to keep:

```bash
curl -sG https://start.spring.io/pom.xml \
  --data-urlencode 'bootVersion=<boot-version>' \
  -d type=maven-build \
  -d javaVersion=<java-version> \
  -d groupId=<group-id> \
  -d artifactId=<artifact-id> \
  -d name=<artifact-id> \
  -d packageName=<base-package> \
  -d 'dependencies=<comma-separated ids from Step 4>' \
  -o /tmp/initializr-pom.xml
```

Save it to the scratchpad (not the repository) and read from it:

- the exact `<groupId>/<artifactId>` of every starter and driver, including test-scope ones;
- every `<dependencyManagement>` BOM import and its version property (e.g. `spring-cloud.version`,
  `spring-ai.version`) — these belong in the root pom verbatim;
- any build plugin Initializr added (e.g. `io.github.ascopes:protobuf-maven-plugin` for the gRPC starters), and
  whether the Boot parent already manages its version;
- the `annotationProcessorPaths` configuration for Lombok and the configuration processor.

Verify the response is actually XML — Initializr returns a plain-text error body for an invalid dependency id or
an incompatible version combination. If it errors, read the message, fix the offending id (usually one that
didn't validate against Step 1's catalogue, or one incompatible with the chosen Boot version), and retry. If a
dependency turns out to be incompatible with the chosen Boot version, tell the user and let them choose between
dropping it and changing the Boot version — don't decide silently.

## Step 8 — Delegate the file generation

Run the seven sub-skills **in this order**, each via the `iru-isolated-skill-executor` agent so its own file
reads, template expansion, and build output stay out of this orchestrator's context. Every sub-skill reads
`springboot-stack.yml` itself, so each prompt only needs to point it at that file and at the saved Initializr pom,
and to name what should come back.

Order matters: poms before modules (a module needs its parent to exist), modules before APIs (generated sources
need a module to land in), APIs before Testcontainers (the compose stack must know whether a schema registry is
needed), everything before the workflows and documentation (both survey what's actually on disk).

```
Agent({
  description: "Run springboot pom setup",
  subagent_type: "iru-isolated-skill-executor",
  prompt: "Invoke Skill({skill: \"iru-setup-java-springboot-pom\", args: \"stack-file: springboot-stack.yml\\n
    initializr-pom: <scratchpad-path-to-initializr-pom.xml>\"}). Report back: the modules declared in the
    reactor, the dependency and plugin versions pinned in the root pom, and any dependency it could not place.",
  run_in_background: false
})
```

Repeat that shape for, in order:

1. **`iru-setup-java-springboot-pom`** — root reactor pom (all versions, all plugins) and every module pom.
   Report back: reactor module list, pinned versions, anything unplaced.
2. **`iru-setup-java-springboot-modules`** — directory tree, package layout, ports in `domain`, adapter skeletons
   in each `infrastructure`/`api` module, the `@SpringBootApplication` class, `application.yml` per profile, the
   `@RefreshScope` configuration beans, and the ArchUnit test that enforces the dependency direction. Report
   back: modules created, ports and adapters scaffolded, profiles written.
3. **`iru-setup-java-springboot-apis`** — the `apis/` tree with a starter spec per API, and the
   `openapi-generator-maven-plugin` / `protobuf-maven-plugin` / `avro-maven-plugin` executions wired into the
   right module poms, HTML documentation generation included. Report back: spec files created, generator
   executions added and to which module.
4. **`iru-setup-java-springboot-testcontainers`** — `docker-compose.yml` (or `compose.yaml`) with one service per
   chosen technology plus Prometheus/Grafana, the Microcks or WireMock mock backing every REST/gRPC client from
   the contracts in `apis/rest-client/`/`apis/grpc-client/`, the `ComposeContainer`-based integration-test base
   class, and the Failsafe wiring. Report back: compose services and their pinned images, base test class
   location, the base URL each client resolves to against the mock, whether `mvn verify` ran and its result.
5. **`iru-setup-java-springboot-platform`** — the OpenTofu configuration for the chosen cloud. Report back:
   resources it will manage, what it deliberately left to the user, and every secret/variable that must be
   provided out-of-band.
6. **`iru-setup-java-springboot-github-workflows`** — the test/analyse/publish workflow, the deploy workflow, and
   the undeploy workflow. Report back: workflow files written, required repository secrets and environments, and
   any gap.
7. **`iru-update-java-springboot-documentation`** — the Antora site (Mermaid **and** Kroki) with the full
   template page set. Report back: pages created versus already present, and whether the site built.

If any sub-skill reports that it stopped (e.g. because a file it owns already exists and the user chose not to
overwrite it), don't treat that as a failure of this skill — carry on with the remaining sub-skills and record it
in Step 11's report. The one exception is `iru-setup-java-springboot-pom`: if the root pom wasn't written there is
no reactor for anything else to attach to, so stop the run there and report why.

## Step 9 — Supporting repository files

After the sub-skills, fill in the repository-level files they don't own:

- **`.gitignore`** — invoke `iru-setup-java-gitignore` if available. Regardless, make sure `target/`,
  `docs/build/`, `.env`, `*.tfstate`, `*.tfstate.*`, `.terraform/`, `*.tfvars` (except `*.tfvars.example`), and
  IDE directories are ignored. The tfstate and tfvars entries matter: both routinely contain credentials.
- **`README.md`** — invoke `iru-setup-readme` if available.
- **`CHANGELOG.md`** — invoke `iru-setup-changelog` if available.
- **`checkstyle.xml`** — the root pom's `maven-checkstyle-plugin` points at it; if no ruleset exists, write a
  starting one (Sun or Google checks with the line-length and Javadoc rules relaxed to what a Spring Boot service
  can realistically satisfy) rather than leaving the build referencing a missing file.
- **`LICENSE`** — ask whether the service needs one. Services often don't, unlike libraries. If one is added,
  `/iru-check-license` can backfill the file headers.

## Step 10 — Verify the reactor builds

Run, from the repository root, and fix what breaks before reporting success:

```bash
mvn -q clean verify -DskipTests   # compiles every module and runs all code generation
mvn -q test                       # unit tests (Surefire)
mvn -q verify                     # integration tests (Failsafe + Testcontainers) — needs Docker running
```

Delegate each of these to the `iru-gate-runner` agent rather than running them inline, so a multi-module Maven log
doesn't flood this orchestrator's context — ask it back for a compact pass/fail plus the first real error.

Expect the generated skeleton to build and its example tests to pass. If `mvn verify` fails only because Docker
isn't available, say so plainly and don't call it a code problem. If code generation fails (a malformed starter
spec, an OpenAPI/protobuf generator misconfiguration), fix it — a scaffold that doesn't compile isn't done.

## Step 11 — Report and warn

Summarize:

- The resolved identity, Boot version, Java version, platform, and concurrency model.
- Every technology selected, with the starter it resolved to and the migration tool per database.
- The full module tree created, and the `apis/` spec directories.
- Which sub-skills completed, which stopped and why.
- The build result from Step 10 for each of the three commands.
- **Every secret and variable that must be provided out-of-band**, consolidated into one table from what the
  workflow and platform sub-skills reported — GitHub Actions secrets, the cloud OIDC role/workload-identity
  binding, the Sonar token, and any LLM API key. State again that none of them belong in the repository, and
  that the OpenTofu state backend must be remote and encrypted since state files contain resource attributes
  that are effectively secrets.
- Any open gap: a dependency Step 1's catalogue couldn't resolve, a Spring Cloud component with no build for the
  chosen Boot version, an image tag that couldn't be pinned, a manual platform step the OpenTofu config can't do.

Finish with an explicit warning: **review every generated file before committing, building against real
infrastructure, or running a deploy.** In particular — the OpenTofu configuration will create billable cloud
resources, so run `tofu plan` and read it in full before any `apply`; the undeploy workflow destroys
infrastructure, so confirm its guards match the intended blast radius; and the generated architecture, use-case,
and API documentation pages are templates describing an empty service — they must be filled in as real code
lands, which is what `/iru-update-java-springboot-documentation` is for.
