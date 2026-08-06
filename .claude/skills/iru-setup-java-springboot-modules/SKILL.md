---
name: iru-setup-java-springboot-modules
description: Scaffold the source tree of a DDD/hexagonal Spring Boot service whose Maven reactor already exists — the package layout for every module, the ports (interfaces) that `domain` defines, a worked adapter skeleton per `infrastructure`/`api` module (repository adapter with MapStruct mapping, refreshable `@ConfigurationProperties` beans, REST/gRPC/Kafka entry points, Kafka producer, metrics publisher), the `@SpringBootApplication` class and component-scan wiring in `boot`, `application.yml` plus per-profile overrides (including structured logging, actuator/Prometheus exposure, and the dynamic-configuration binding the stack selected), the database migration/changelog directory for each engine's tooling (Mongock, Liquibase and its Couchbase/MongoDB/Neo4j extensions, or Flyway), and an ArchUnit test that fails the build when the hexagonal dependency direction is violated. Reads `springboot-stack.yml` for every decision and writes no version numbers. Invoke as `/iru-setup-java-springboot-modules`, or with `args` (`stack-file:` line) when called from `iru-setup-java-springboot`. Use after `iru-setup-java-springboot-pom` has created the poms, whenever a Spring Boot service needs its hexagonal source skeleton and runtime configuration generated instead of hand-writing the ports, adapters, and profile files module by module.
model: sonnet
---

# Setup Java Spring Boot Modules

Turn a reactor of empty modules into a service skeleton that compiles, starts, and demonstrates the hexagonal
pattern well enough that the first real feature has an obvious shape to follow.

What "demonstrates" means here matters: this skill writes **one worked example per adapter kind**, not a set of
empty files. A single `Order` aggregate, its `OrderRepository` port, one adapter implementing that port against
the chosen database, one REST endpoint, one use case. A developer's first task is to rename it or delete it — both
of which are easier than inventing the layering from an empty tree. Name the example after the service's actual
domain if the manifest's `project.description` makes that obvious; otherwise use `Example` and say so in the
report.

## Step 0 — Resolve inputs and check the reactor exists

Parse `args` for `stack-file: <path>`, defaulting to `springboot-stack.yml` at the repository root, and read it in
full. If it's missing, stop and point the user at `/iru-setup-java-springboot`.

Verify `pom.xml` exists at the root and that every module directory the manifest names has a `pom.xml`. If not,
stop and tell the user to run `/iru-setup-java-springboot-pom` first — this skill writes sources into modules, it
doesn't create modules.

Derive from the manifest:

- `<base>` — `project.basePackage`; `<base-path>` — the same with `.` replaced by `/`.
- `<Name>` — the service name in PascalCase, from `project.artifactId` (e.g. `orders-service` → `Orders`), used
  for the application class `<Name>Application`.
- Whether each module exists, from `modules:`.
- `stack.concurrency` — decides every return type below: blocking → plain values and `List<T>`; reactive →
  `Mono<T>`/`Flux<T>`; both → blocking on the inbound side and reactive on the outbound side unless the manifest
  says otherwise, and say in the report which way you resolved it.

Never overwrite a `.java` file that already contains code you didn't generate. Check first, skip and report
instead.

## Step 1 — Package layout

Create `src/main/java`, `src/main/resources`, and `src/test/java` under each module, with these packages:

| Module | Main package | Contains |
|---|---|---|
| `domain` | `<base>.domain` | `model/` (entities, value objects, aggregates), `port/` (the interfaces outer modules implement), `service/` (pure domain services), `exception/` (domain exceptions) |
| `application` | `<base>.application` | `usecase/` (one class per use case), `exception/` |
| `infrastructure/database/<engine>` | `<base>.infrastructure.database.<engine>` | `entity/` (persistence models), `repository/` (Spring Data repositories), `adapter/` (port implementations), `mapper/` (MapStruct), `config/` |
| `infrastructure/configuration` | `<base>.infrastructure.configuration` | `properties/` (`@ConfigurationProperties` beans), `adapter/` (port implementations that expose configuration as domain types) |
| `infrastructure/clients/<name>` | `<base>.infrastructure.clients.<name>` | `adapter/`, `mapper/`, `config/` — generated client code lands under `target/generated-sources` |
| `infrastructure/producers` | `<base>.infrastructure.producers` | `adapter/`, `mapper/` (domain → AVRO), `config/` |
| `infrastructure/metrics` | `<base>.infrastructure.metrics` | `adapter/`, `config/` |
| `api/rest-server` | `<base>.api.rest` | `controller/` (implementing the generated interfaces), `mapper/` (DTO ↔ domain), `error/` (`@RestControllerAdvice`) |
| `api/grpc-server` | `<base>.api.grpc` | `service/` (extending the generated base classes), `mapper/`, `error/` |
| `api/consumers` | `<base>.api.consumers` | `consumer/` (the `Consumer<Message<...>>` beans), `mapper/` (AVRO → domain) |
| `boot` | `<base>` | `<Name>Application`, `config/` (composition-root `@Configuration`) |

`domain`'s package holds no Spring annotations at all — the enforcer rule
`iru-setup-java-springboot-pom` added makes that a build failure, not a guideline.

## Step 2 — `domain`: the ports

This is the module that makes the architecture work, so write it first and write it properly. The rule the user's
architecture requires: **`domain` is shared by every other module, so every capability an outer module provides is
declared here as an interface**, and outer modules only ever depend inward on this one. Nothing outside `domain`
is visible to `application`, and no two adapters can see each other.

Write, adapted to the actual stack:

- **Model** — one aggregate root with a couple of value objects, using records or Lombok `@Value` for
  immutability. No persistence annotations, no JSON annotations, no `@Component`.
- **Ports** in `<base>.domain.port`, one interface per outbound capability the manifest calls for:
  - `<Aggregate>Repository` — one per database engine role. Method signatures follow `stack.concurrency`.
    For a CQRS setup (Elasticsearch or Neo4j alongside a system of record) define **two** ports —
    e.g. `OrderRepository` and `OrderSearchPort` — so it's clear which adapter serves which, rather than one
    interface with two implementations racing to be the primary bean.
  - `<Aggregate>EventPublisher` — if `stack.messaging.directions` includes `produce`. Takes domain events, knows
    nothing about Kafka, topics, or AVRO.
  - `<Name>Client` — one per entry in `stack.restClients`/`stack.grpcClients`, expressed in domain types only.
  - `MetricsPublisher` — if `stack.metrics`. Methods named for what is being measured
    (`recordOrderAccepted(OrderType)`), not for Micrometer primitives, so the domain never mentions counters or
    gauges.
  - `<Name>ConfigurationProvider` — if the domain needs to read dynamically-changeable configuration as domain
    types.
- **Domain exceptions** — a small hierarchy (`NotFoundException`, `ConflictException`, `ValidationException`)
  that the API modules map to HTTP status codes or gRPC status codes. Getting this right here is what stops
  `HttpStatus` leaking inward.

Add a Javadoc comment on every port explaining the contract and what an implementation must guarantee — these
interfaces are the architecture's documentation, and the Javadoc report the reactor generates will surface them.

## Step 3 — `application`: the use cases

One class per use case, named for the action (`PlaceOrderUseCase`), each with a single public method. It
depends only on `domain` ports via constructor injection, orchestrates them, and contains the application logic
that isn't domain-invariant. Write one worked example that touches at least a repository port and, if messaging is
enabled, an event publisher port — so the pattern of "use case composes ports" is visible.

If the manifest's concurrency model is reactive, the use case returns `Mono`/`Flux` and must not call anything
blocking; add a comment saying so, since this is the single most common way a reactive service quietly becomes a
broken one.

Write a unit test for the example use case that mocks every port with Mockito. It's the template for how use cases
get tested — no Spring context, no database.

## Step 4 — `infrastructure/database/<engine>`: the persistence adapters

Per engine directory in the manifest, write:

- **Entity** — the persistence model, separate from the domain model, carrying that engine's annotations
  (`@Document` for MongoDB/Elasticsearch, `@Entity`/`@Table` for JPA, `@Node` for Neo4j, `@Document` for
  Couchbase). Keeping it separate is the whole point: the database's shape may drift from the domain's.
- **Spring Data repository** — the interface extending the engine's repository type (or its reactive variant).
- **MapStruct mapper** — entity ↔ domain, `@Mapper(componentModel = "spring")`. The root pom sets
  `unmappedTargetPolicy=ERROR`, so an unmapped field fails compilation instead of silently returning null.
- **Adapter** — a `@Component` implementing the `domain` port, delegating to the Spring Data repository and the
  mapper. This is the only class in the module the rest of the service can reach through the port.
- **Migrations/index management**, per this engine's tooling from the manifest:
  - **Mongock** — `src/main/java/<base-path>/infrastructure/database/mongodb/migration/` with one
    `@ChangeUnit` creating the example collection's indexes, plus the `mongock.migration-scan-package`
    property in the module's config.
  - **Liquibase (SQL)** — `src/main/resources/db/changelog/db.changelog-master.yaml` including a first changeset
    that creates the example table and its indexes. If PostgreSQL was chosen and vectors may be stored, add a
    changeset with `CREATE EXTENSION IF NOT EXISTS vector;` and note it needs the `pgvector` image or a
    superuser grant.
  - **Flyway** — `src/main/resources/db/migration/V1__create_<table>.sql`.
  - **Liquibase for MongoDB / Couchbase / Neo4j** — the extension's own changelog format under
    `src/main/resources/db/changelog/`, with a first changeset creating the example indexes.
- **An integration test** (`*IT`) that exercises the adapter against the engine's Testcontainer. Write the test
  class here but let `iru-setup-java-springboot-testcontainers` own the base class it extends and the container
  wiring — this skill only writes the test that uses it, so the two skills don't both define the container.

Set `spring.data.<engine>.*` connection properties in the module's own `src/main/resources/application-<engine>.yml`
and import it from `boot`'s `application.yml`, so each adapter owns its own configuration surface.

## Step 5 — `infrastructure/configuration`: refreshable configuration

This module exists so that "what can be changed at runtime" is one auditable list.

- One `@ConfigurationProperties(prefix = "<service-prefix>.<area>")` record or class per area of tunable
  behaviour, with `jakarta.validation` constraints so a bad value fails fast at startup rather than at first use.
- Annotate each with `@RefreshScope` (or `@ConfigurationProperties` + `@RefreshScope` on a `@Bean` method) so a
  configuration refresh is picked up without a restart. Without this, dynamic configuration silently isn't dynamic
  — the single most common defect in a Spring Cloud Config setup.
- An adapter implementing the domain's configuration port, mapping properties to domain types so `domain` never
  reads `@Value` or knows a property name.
- Wire the mechanism from `stack.dynamicConfig`:
  - **kubernetes** — enable `spring.cloud.kubernetes.config`/`.secrets` with the ConfigMap and Secret names, set
    `spring.cloud.kubernetes.reload.enabled=true` with `mode: polling` or `event`, and note that watching
    requires RBAC to `get`/`list`/`watch` ConfigMaps — which the OpenTofu configuration must grant.
  - **aws-secrets-manager** — configure `spring.config.import: aws-secretsmanager:<name>` (and
    `aws-parameterstore:` for non-secret properties). Refresh comes from
    `management.endpoint.refresh` being invoked or the Spring Cloud AWS reload support; state which one you wired.
  - **spring-cloud-config** — `spring.config.import: configserver:<url>`, plus Spring Cloud Bus if the manifest
    asked for broadcast refresh.
  - **none** — write the `@ConfigurationProperties` beans anyway, without `@RefreshScope`, and note in the report
    that changes need a redeploy.
- Expose the `refresh` actuator endpoint (`management.endpoints.web.exposure.include`) only where it's actually
  needed, and never on a publicly-reachable port. Say so in the report — an open `refresh` endpoint is a real
  vulnerability, not a convenience.

**Every property defined here must end up in the documentation's configuration table** with its default, whether
it's dynamically changeable, and whether it's a critical performance/incident lever. Emit the list of properties
you created as part of Step 11's report so `iru-update-java-springboot-documentation` can seed that table.

## Step 6 — `infrastructure/clients/<name>`: outbound integrations

Per client, write an adapter implementing the domain's client port that:

- calls the **generated** client (OpenAPI-generated API class, or gRPC stub) — never a hand-rolled HTTP call;
- maps domain types ↔ generated DTOs with MapStruct, so generated types never escape the module;
- translates transport failures into domain exceptions, so no caller ever catches a
  `WebClientResponseException` or a `StatusRuntimeException`;
- carries timeout and retry configuration bound from `@ConfigurationProperties`, not hardcoded. A client with no
  timeout is an outage waiting for its trigger — set connect and read timeouts explicitly and mention the values
  chosen in the report;
- **takes its base URL (or gRPC target) from that same `@ConfigurationProperties` binding**, conventionally
  `clients.<name>.base-url`, with **no default value in the deployed profiles**. This is what lets the
  integration harness redirect the client to the API mock in the compose stack (`stack.apiMock` — Microcks or
  WireMock, wired by `iru-setup-java-springboot-testcontainers`); an adapter that hardcodes a host cannot be
  tested against anything and will reach a real downstream service from CI. Set the `local` profile's value to
  the mock's fixed compose port, and leave every deployed profile's value to be supplied by configuration.

`iru-setup-java-springboot-apis` owns the generator configuration and the spec files; this skill writes the adapter
that consumes the generated code. If the generated classes don't exist yet, write the adapter against the API
shape the starter spec declares and note that it compiles only after `mvn generate-sources`.

## Step 7 — `infrastructure/producers` and `api/consumers`: Kafka

Only if `stack.messaging.enabled`.

**Producers** (`directions` includes `produce`):

- An adapter implementing the domain's event-publisher port, using Spring Cloud Stream's `StreamBridge` (or a
  declared output binding) to send to the topic.
- A MapStruct mapper from domain event → the AVRO-generated class.
- Bindings in configuration: `spring.cloud.stream.bindings.<name>-out-0.destination`, the Kafka binder's broker
  list, and the Confluent serializer settings — `value.serializer` set to the AVRO serializer and
  `schema.registry.url` pointing at the registry.

**Consumers** (`directions` includes `consume`):

- A `@Bean Consumer<Message<<AvroType>>>` (or `Function<...>` when the consumer replies), named to match
  `spring.cloud.function.definition`.
- A MapStruct mapper AVRO → domain, then a call into the `application` use case.
- Explicit error handling: configure a dead-letter topic
  (`spring.cloud.stream.kafka.bindings.<name>-in-0.consumer.enableDlq=true`) and a bounded retry policy. A
  consumer with infinite retries on a poison message stalls the partition — write the DLQ configuration now, not
  after the first incident.
- Set `spring.cloud.stream.bindings.<name>-in-0.group` so the consumer group is stable across restarts, and say
  in the report what group name was chosen.

Note explicitly in the report that AVRO schema evolution rules (backward/forward compatibility) are enforced by
the schema registry's compatibility setting, and that this must be configured in the registry itself — the
scaffold can't do it.

## Step 8 — `infrastructure/metrics`

Only if `stack.metrics`. Implement the domain's `MetricsPublisher` port over Micrometer's `MeterRegistry`:

- One method per measurement, each translating domain arguments into the metric name plus its tags.
- Keep the **tag cardinality bounded** — tag values must come from enums or small closed sets, never from an id,
  an email, a free-text field, or anything user-supplied. Unbounded tag cardinality is the standard way to take
  down a Prometheus instance; state this in the generated class's Javadoc and in the report.
- A constants class listing every metric name and every tag key, so the documentation's metrics table and the code
  can't drift.
- Emit the list of metrics created (name, type, tags, meaning) in Step 11's report so
  `iru-update-java-springboot-documentation` can seed the metrics table.

## Step 9 — `api/rest-server`, `api/grpc-server`

- **REST**: controllers implementing the **generated** interfaces from `apis/rest-server/`, a MapStruct mapper
  between generated DTOs and domain types, and a `@RestControllerAdvice` mapping each domain exception to a
  status code and a consistent error body (use RFC 9457 `ProblemDetail`, which Spring supports natively). Every
  error the advice can produce must appear in the API documentation page — emit the mapping in Step 11's report.
- **gRPC**: services extending the generated base classes from `apis/grpc-server/`, mappers, and an exception
  handler translating domain exceptions to `StatusRuntimeException` with the right `Status.Code` plus error
  details.
- If `stack.security.enabled`, add the security configuration here (not in `boot`): a `SecurityFilterChain` (or
  `SecurityWebFilterChain` for WebFlux) with the JWT resource-server configuration and per-endpoint
  authorization rules. Deny by default and permit explicitly — including for the actuator endpoints, where
  `/actuator/health` is typically open and everything else is not.

## Step 10 — `boot`: the composition root and configuration

- **`<Name>Application`** in `<base>` with `@SpringBootApplication`. Because the other modules' packages are all
  under `<base>`, the default component scan already finds them — no `scanBasePackages` list to maintain. Verify
  that assumption holds for the actual packages written and add an explicit `scanBasePackages` only if it
  doesn't.
- **`src/main/resources/application.yml`** — the shared baseline:
  - `spring.application.name` from the artifactId, and `spring.profiles.group` / `spring.config.import` pulling in
    each adapter module's own configuration file.
  - `spring.threads.virtual.enabled: true` if the concurrency model is blocking and Java is 21+ — it's the reason
    blocking is a reasonable default at this Java level. Do **not** set it for reactive.
  - `management.endpoints.web.exposure.include` limited to what's needed (`health`, `info`, `prometheus`, and
    `refresh` only if dynamic configuration needs it), `management.endpoint.health.probes.enabled: true` for
    Kubernetes liveness/readiness, and `management.server.port` on a separate port so actuator isn't exposed
    through the public ingress.
  - `server.shutdown: graceful` plus a `spring.lifecycle.timeout-per-shutdown-phase`, so a rolling deploy drains
    in-flight requests instead of dropping them.
- **Per-profile files** — `application-local.yml`, `application-dev.yml`, `application-prod.yml`:
  - `local` — points at the docker compose stack (`localhost` ports), human-readable console logging at `DEBUG`
    for `<base>`, and `spring.docker.compose.enabled: true` so the stack starts with the app.
  - `dev`/`prod` — structured JSON logging (`logging.structured.format.console: ecs` — Boot 3.4+ supports this
    natively, so no Logback XML is needed; verify the property name against the Boot version in use), `INFO`
    level, and every endpoint, host, and credential resolved from the environment or the secret manager. **No
    literal credential, host, or token in any committed profile file** — placeholders only
    (`${DATABASE_URI}`), with no defaults that would accidentally work against something real.
- **`config/`** — `@Configuration` classes that wire beans crossing module boundaries (e.g. selecting the primary
  repository adapter when CQRS gives two). This is the only place such a decision belongs.
- **A smoke test** `<Name>ApplicationIT` that starts the context against the Testcontainers stack. It's the test
  that catches a broken bean graph, which unit tests never will.

## Step 11 — The ArchUnit test

Write `<base>.architecture.HexagonalArchitectureTest` in `boot`'s test sources (it's the only module that sees
every other one, so it's the only place the whole graph can be checked). The Maven enforcer rules from
`iru-setup-java-springboot-pom` catch violations at module granularity; ArchUnit catches them at **package**
granularity, which is where they actually creep in — a domain class importing `org.springframework.data`, an
adapter imported directly instead of through its port.

Assert at least:

- `<base>.domain..` depends on no other `<base>` package, and on no `org.springframework..` or `jakarta.persistence..` class.
- `<base>.application..` may access `<base>.domain..` only.
- `<base>.infrastructure..` may access `<base>.domain..` only (never `application`, never another
  `infrastructure` subpackage).
- `<base>.api..` may access `<base>.application..` and `<base>.domain..` only.
- Classes named `*Controller`/`*Consumer` reside only in `<base>.api..`; `*Adapter` only in
  `<base>.infrastructure..`; `*UseCase` only in `<base>.application..`.
- The layered-architecture rule (`Architectures.layeredArchitecture()`) expressing the same thing as one
  declaration, so a new module can't be added outside the layering by accident.

Exclude generated sources from the ArchUnit import (`ImportOption.DoNotIncludeTests` where appropriate, plus a
location filter for `generated-sources`), or generated DTOs will fail the naming rules.

## Step 12 — Verify

Delegate to the `iru-gate-runner` agent, asking for a compact result:

```bash
mvn -q clean test          # everything compiles, unit tests and the ArchUnit test pass
```

The build must compile and the ArchUnit test must pass. If ArchUnit fails, **fix the code, not the rule** — a
relaxed rule on day one is a rule that never applies. The one exception is a rule that's wrong for this project
(e.g. the domain legitimately uses `spring-context` annotations by the team's choice); if so, change it
deliberately and report it as a deviation.

Don't run `mvn verify` here: the `*IT` tests need the Testcontainers harness that
`iru-setup-java-springboot-testcontainers` sets up in the next step, so integration tests failing at this point is
expected rather than a defect.

## Step 13 — Report

Summarize:

- Which modules got sources, and which were skipped because they already contained hand-written code.
- Every **port** defined in `domain` and which adapter implements it.
- The example aggregate/use case/endpoint written, and what it's called, so the user knows exactly what to rename
  or delete first.
- The **configuration properties** created, each with its prefix, default, whether it's `@RefreshScope`d, and
  whether it's a critical tuning lever — formatted so it can be pasted into the documentation's configuration
  table.
- The **metrics** created, each with name, type, and tag keys — same, for the metrics table.
- The **domain-exception → HTTP status / gRPC status** mapping, for the API documentation pages.
- Kafka bindings, consumer group names, and DLQ configuration, if any.
- The result of Step 12, and any ArchUnit rule deliberately relaxed.

Warn explicitly:

- **Every generated profile file uses placeholders, not real endpoints or credentials.** The service will not
  start against real infrastructure until those are supplied from the environment or the secret manager, and that
  is intentional — do not "fix" it by committing a value.
- The `refresh` actuator endpoint, if exposed, must not be reachable from outside the cluster.
- Metric tag cardinality must stay bounded; the generated code shows the pattern but can't enforce it.
- The example domain/use case/endpoint is scaffolding, not a feature — it exists to be replaced.
