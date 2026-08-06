---
name: iru-update-java-springboot-documentation
description: Set up and keep current the Antora documentation site for a DDD/hexagonal Spring Boot service — bootstraps the site via `iru-setup-antora` if absent, adds the Kroki extension alongside Mermaid, and creates or refreshes the service's full page set from the codebase as it actually is: service overview, service architecture (with dependency diagrams and the reactive-versus-blocking rationale), one section per use case (flow diagram, dependencies touched, exceptions raised), the database model (entities, fields, types, indexes, mandatory fields, constraints, relationships, plus generated diagrams), REST/gRPC/messaging API pages derived from the contracts in `apis/` (versions, entities and fields with descriptions and constraints, deprecations, endpoints with methods/paths/headers/authorization and every documented error, links to the generated OpenAPI and protobuf HTML), a configuration property table (default, dynamically-changeable, criticality flags), and a metrics table (type, tags, meaning). Reads `springboot-stack.yml`, the module sources, the `apis/` contracts, and the `@ConfigurationProperties`/Micrometer code rather than inventing content, and marks anything it cannot derive as an explicit TODO instead of filling it with plausible prose. Idempotent — safe to re-run after any change, and intended to be re-run as part of the normal change cycle. Invoke as `/iru-update-java-springboot-documentation` to cover the whole service, or `/iru-update-java-springboot-documentation <page-or-area>` to refresh one page. Use whenever a Spring Boot service's documentation needs creating or bringing back in line with its code, instead of hand-maintaining nine interdependent pages.
model: sonnet
---

# Update Java Spring Boot Documentation

Create or refresh the Antora documentation for a Spring Boot service. This skill does double duty on purpose: the
first run scaffolds the page set, and every later run updates it from the code — which is the only way
documentation of this depth survives contact with an evolving service.

The governing rule: **everything on these pages is derived from something in the repository.** The contracts in
`apis/`, the `@ConfigurationProperties` classes, the Micrometer calls, the ports in `domain`, the use cases in
`application`, the persistence entities in `infrastructure/database/*`. Where something genuinely cannot be
derived — why a technology was chosen, what a use case is *for*, a business constraint — write an explicit
`TODO:` marker naming what's needed and from whom. **Never write plausible-sounding prose in place of a fact you
don't have**: a confidently wrong architecture page is worse than an obviously incomplete one, because nobody
re-checks it.

## Step 0 — Resolve scope and read the ground truth

Parse the argument: absent means every page; a page name or area (`architecture`, `use-cases`,
`database-model`, `apis/rest`, `apis/grpc`, `apis/messaging`, `configuration`, `metrics`, `overview`) means just
that one.

Read `springboot-stack.yml` for the declared stack. Treat it as **intent**, not truth: the code is truth. Where
they disagree (a database in the manifest with no adapter module, an adapter for something the manifest doesn't
list), say so in the report — that's a real finding about the repository, not a documentation problem.

Then gather the facts, using the `Explore` agent for the broad structural sweeps rather than reading every file
into this context, and reading directly only the specific files it identifies:

| Page | Source of truth |
|---|---|
| Overview | `README.md`, root pom `<description>`, the use cases in `application` |
| Architecture | the module tree, each module's pom dependencies, `stack.concurrency`, the adapters present |
| Use cases | `<base>.application.usecase.*` — the ports each one calls, the exceptions each one declares/throws |
| Database model | `infrastructure/database/*/entity/*`, the Spring Data repositories, the migration files (Mongock change units, Liquibase changelogs, Flyway scripts) — the migrations are where indexes and constraints actually live |
| REST API | `apis/rest-server/*.yaml` and the `@RestControllerAdvice` exception mapping |
| gRPC API | `apis/grpc-server/*.proto` and the gRPC exception handler |
| Messaging API | `apis/messaging/*.avsc` plus the `spring.cloud.stream.bindings.*` configuration (topic names live there, not in the schemas) |
| Configuration | every `@ConfigurationProperties` class, every `application*.yml`, and which of them carry `@RefreshScope` |
| Metrics | `infrastructure/metrics/*` — every meter name, its type, and its tag keys |

## Step 1 — Ensure the site exists, with Mermaid and Kroki

If `docs/antora.yml` is absent, invoke `iru-setup-antora` first (via the `Agent` tool with `general-purpose`, or
`Skill` directly) to get the toolchain, playbook, and ROOT module in place.

Then make sure **Kroki is wired in addition to Mermaid** — `iru-setup-antora` installs Mermaid but not Kroki, and
this page set needs both. Mermaid is right for flows, sequences, and state; Kroki adds PlantUML, C4, Graphviz, and
ERD, which is what the database-model and architecture pages need.

```bash
cd docs && npm i asciidoctor-kroki
```

In `docs/antora-playbook.yml`, add Kroki to the `asciidoc.extensions` list, keeping the existing entries:

```yaml
asciidoc:
  attributes:
    mathjax-tex-tags: ams
    mathjax-tex-packages: ams
    # Fetch diagrams at build time and commit nothing generated; images end up in the built site.
    kroki-fetch-diagram: true
    # Default is the public https://kroki.io. Point at a self-hosted instance if the source must not
    # leave the network — diagram source is sent to whichever server this names.
    kroki-server-url: https://kroki.io
  extensions:
    - '@djencks/asciidoctor-mathjax'
    - asciidoctor-kroki
```

Flag in the report that with the public Kroki server, **every diagram's source text is sent to a third-party
service at build time**. For a service whose architecture or database model is sensitive, self-host Kroki (a single
container) and change `kroki-server-url`. This is a genuine consideration, not boilerplate — the database model
page describes the data layout of a real system.

Verify both work by building the site in Step 12; a Kroki block with the extension missing silently renders as a
literal code block rather than failing, which is easy to miss.

## Step 2 — Page set and navigation

Create under `docs/modules/ROOT/pages/`, and add each to `nav.adoc` preserving existing entries:

```
index.adoc                    (service overview)
architecture.adoc
use-cases.adoc
database-model.adoc
apis/rest.adoc
apis/grpc.adoc
apis/messaging.adoc
configuration.adoc
metrics.adoc
```

Omit a page whose subject doesn't exist (no gRPC server → no `apis/grpc.adoc`); note the omission in the report so
its absence is a recorded decision.

Every page carries this immediately after its `= Title` heading, matching the rest of this catalog:

```
NOTE: This documentation was generated with the assistance of AI. Please report any inaccuracies.
```

On a re-run, **update pages rather than rewriting them**: preserve hand-written prose (especially the "why"
sections that only a human can supply) and refresh the derived parts — tables, diagrams, endpoint lists, property
lists. If a hand-written statement now contradicts the code, don't silently overwrite it: update it and list the
contradiction in the report, since it may mean the code changed without the intent changing.

## Step 3 — `index.adoc` — service overview

What the service is for and what it offers. Short — a page someone reads in a minute:

- One paragraph on purpose, in domain terms.
- What it offers: the capabilities, as a list, derived from the use cases and the API contracts.
- What it depends on, at one line each.
- Where to go next: links to the other pages, plus the generated API documentation on GitHub Pages.

Mark `TODO:` for the business purpose if `README.md` and the pom description don't state it — that's a question
for the service's owner, not something to infer from class names.

## Step 4 — `architecture.adoc` — technical architecture

The deepest page, and the one with the highest chance of drifting into invention. Discipline: state what the code
does, and mark the rationale `TODO:` unless it's recorded somewhere.

- **Hexagonal structure** — the module table, what each module may depend on, and the two mechanisms that enforce
  it (the `banned-dependencies` enforcer rules and the ArchUnit test). Name the test class so a reader can go look.
- **Concurrency model** — reactive, blocking, or both, taken from the actual starters in the poms. Explain the
  consequence (blocking with virtual threads enabled, or a Netty event loop that must never be blocked). The
  *reason* for the choice is a `TODO:` unless recorded.
- **Per-technology sections** — one each for the databases, cache, Kafka, and clients present. For each: what it
  is used for in this service, which module adapts it, which ports it implements. Include the "why this
  technology" guidance the stack manifest recorded if it's there; otherwise `TODO:`. For each downstream client,
  also name the contract it is generated from (`apis/rest-client/<name>/` or `apis/grpc-client/<name>/`) and the
  fact that the same contract drives the API mock (`stack.apiMock` — Microcks or WireMock) that stands in for that
  service in integration tests and local runs, so a reader knows where to add a response rather than inventing a
  fixture.
- **Dependency diagram** — a Mermaid `flowchart` of the service and its external dependencies (each database,
  cache, Kafka topic direction, and downstream service via REST/gRPC), generated from the modules and bindings
  actually present:

  ````
  [mermaid]
  ....
  flowchart LR
    subgraph service["<service-name>"]
      api["api/*"] --> app["application"]
      app --> domain["domain"]
      infra["infrastructure/*"] -.implements ports.-> domain
    end
    infra --> mongo[(MongoDB)]
    infra --> redis[(Redis)]
    infra -- produces --> topicOut{{orders.events.v1}}
    topicIn{{payments.events.v1}} -- consumed by --> api
    infra --> payments["payments service (REST)"]
  ....
  ````

- **Module dependency diagram** — a Kroki C4 or PlantUML component diagram of the internal module graph, which
  reads better than Mermaid for the ports-and-adapters shape:

  ````
  [plantuml,module-dependencies,svg]
  ....
  @startuml
  ...
  @enduml
  ....
  ````

- **Cross-cutting concerns** — the dynamic-configuration mechanism, how secrets reach the service, the graceful
  shutdown behaviour, and the health probes. Each is a fact readable from the configuration files.

## Step 5 — `use-cases.adoc`

One section per class in `<base>.application.usecase`. For each, derived from the code:

- **What it does**, from the class/method Javadoc. If there is none, `TODO:` — and note in the report that the use
  case is undocumented in code, which is worth fixing at the source.
- **Trigger** — which REST endpoint, gRPC method, or Kafka topic reaches it (trace inbound from the
  controllers/services/consumers).
- **Steps** — the ordered calls it makes, as a numbered list.
- **Dependencies** — every port it uses, and which adapter (hence which database, client, or topic) backs each.
- **Exceptions** — every exception it can raise, why, and what the caller sees (the HTTP status or gRPC code from
  the exception-mapping advice).
- **A Mermaid sequence diagram** of the flow, including the failure branch — a happy-path-only diagram hides the
  half of the behaviour that matters during an incident:

  ````
  [mermaid]
  ....
  sequenceDiagram
    participant C as Client
    participant R as RestController
    participant U as PlaceOrderUseCase
    participant P as OrderRepository (port)
    participant A as MongoOrderAdapter
    C->>R: POST /v1/orders
    R->>U: placeOrder(command)
    U->>P: save(order)
    P->>A: save(entity)
    A-->>U: Order
    U-->>R: OrderId
    R-->>C: 201 Created
    Note over U,P: On conflict, ConflictException → 409
  ....
  ````

## Step 6 — `database-model.adoc`

One section per database engine. Read the entity classes **and** the migration files — indexes and constraints are
usually declared in the migration, not the entity, so an entity-only read produces a model page that's missing
exactly the information an operator needs.

Per collection/table/node label:

- What it holds and why it exists.
- A field table: **name, type, mandatory, indexed, constraints (length, pattern, range, enum values), default,
  description**. Every column filled or explicitly `TODO:`.
- Which indexes exist, on which fields, whether unique, and — where derivable from the repository query methods —
  which query each index serves. An index nobody can attribute to a query is worth flagging.
- For relational models: the relationships (cardinality, FK constraints, cascade behaviour).
- For Neo4j: node labels, relationship types and directions, and their properties.

Then a diagram per engine — Kroki's ERD/PlantUML for relational, a Mermaid `erDiagram` for document stores
(showing references between documents even where the database doesn't enforce them, since the *application* does),
and a graph diagram for Neo4j.

Note the migration tool in use and where its changelogs live, so a reader knows how a model change is made.

## Step 7 — `apis/rest.adoc`

Derived from every spec in `apis/rest-server/`, one section per version:

- **Version summary** — what functionality each version provides, and the status of each (current, deprecated,
  planned removal). This is the table someone consults before integrating.
- **Entities** — per schema: what it represents, then a field table with **description, type, mandatory,
  deprecated (with reason), constraints** (`maxLength`, `pattern`, `minimum`, `format`).
- **Endpoints** — per operation:
  - HTTP method and full path, path parameters marked;
  - required request headers;
  - whether authorization is required and of what kind (Basic, Bearer/JWT, or a named OAuth2 flow with the
    required scopes) — read this from the spec's `securitySchemes` **and** cross-check it against the
    `SecurityFilterChain` rules in `api/rest-server`. If the two disagree, that's a security finding: report it
    rather than documenting one of them;
  - request and response bodies with their schema references;
  - **every error response**: HTTP status, the error body schema, and what causes it. Cross-check against the
    `@RestControllerAdvice` mapping so an exception the code can throw but the spec doesn't document shows up as a
    gap.
- A link to the generated OpenAPI HTML on GitHub Pages (`<pages-url>/api/rest-server/v1/`), and a link to the spec
  file in the repository. Don't duplicate the whole reference here — this page explains, the generated docs
  enumerate.

## Step 8 — `apis/grpc.adoc`

Same shape, from `apis/grpc-server/*.proto`:

- Version summary per `package <service>.vN`.
- **Messages and enums** — per type, and per field: description, type, mandatory, deprecated (with reason),
  constraints. Note that protobuf has no native optionality for `proto3` scalars beyond `optional`, so state which
  fields are semantically required and enforced in code.
- **RPCs** — per method: whether it's unary, server-streaming, client-streaming, or bidirectional **and why**
  (streaming is a design decision worth a sentence); the full path (`/<package>.<Service>/<Method>`); required
  metadata/headers; whether authorization is required and how (typically a JWT in the `authorization` metadata
  key); and every error: gRPC `Status.Code`, the error detail payload, and its cause — cross-checked against the
  gRPC exception handler.
- A link to the generated protobuf HTML on GitHub Pages and to the `.proto` file.
- The field-numbering and `reserved` discipline, so a reader knows how to evolve the contract safely.

## Step 9 — `apis/messaging.adoc`

From `apis/messaging/*.avsc` plus the Spring Cloud Stream bindings:

- **Topic summary** — every topic the service produces to or consumes from, its purpose, direction, the consumer
  group used, and the DLQ topic if configured. Topic names come from the bindings configuration, not the schemas.
- **Version summary per topic** — what each schema version provides, and which are still in flight.
- **Messages** — per AVRO record: what event it represents and when it's emitted; then per field: description,
  type, mandatory (a field with no default is mandatory), deprecated (with reason), constraints, and its default.
- The registry's compatibility mode and the subject naming strategy in use, since those determine what schema
  change is legal.
- Which use case produces or consumes each message, linking to `use-cases.adoc`.

## Step 10 — `configuration.adoc`

The table whose purpose is to answer "what can we change about this service's behaviour without a code change?" —
so it must be complete for the service's own properties.

Columns: **Property | Description | Type | Default | Dynamically changeable | Critical | Notes**

- **Only the service's own properties** (the `@ConfigurationProperties` prefixes from
  `infrastructure/configuration`, and any other project-specific key in `application*.yml`). A Spring Boot
  property goes in only when it's genuinely important to operating *this* service (e.g.
  `spring.threads.virtual.enabled`, a connection-pool size that has been tuned, a Kafka consumer concurrency) —
  mark such rows so it's clear they're framework properties, not the service's own.
- **Default** — the actual default from the code or the YAML, not a guess. If a property has no default and must
  be supplied, say "required" rather than leaving it blank.
- **Dynamically changeable** — Yes only where the bean is genuinely `@RefreshScope`d *and* the configured dynamic
  mechanism can deliver the change. "Restart required" and "Redeploy required" are the other two values.
  Getting this column wrong is actively harmful during an incident, so derive it from the annotations, never
  from intent.
- **Critical** — flag (e.g. `⚠`) the properties that are performance or availability levers: timeouts, retry
  counts, pool sizes, batch sizes, circuit-breaker thresholds, feature toggles that shed load, cache TTLs. The
  purpose of the flag is that during a degradation someone can scan this column for the fastest temporary
  mitigation. Add a short sentence per flagged row saying which symptom it addresses and in which direction to
  move it.
- **A legend** below the table explaining the flag, the "dynamically changeable" values, and the framework-property
  marker.
- Never put a real credential, endpoint, or token in an example value — reference the environment variable or
  secret name instead.

## Step 11 — `metrics.adoc`

Columns: **Metric | Description | Type | Tags (and possible values) | Notes**

- Every meter the service publishes from `infrastructure/metrics`, plus any notable framework meter the team
  actually watches (marked as framework-provided).
- **Type** — counter, gauge, timer, distribution summary, long task timer — read from the Micrometer call.
- **Tags** — every tag key and its possible values. Where a tag's values come from an enum, list them; where they
  can't be enumerated, say so and flag the cardinality risk, because that's the field that takes down a metrics
  backend.
- **Notes** — what a change in this metric indicates, and whether it backs an alert or a dashboard panel.
- Add a short section on where metrics are scraped from (`/actuator/prometheus` on the management port) and how to
  view them locally (the compose `observability` profile → Prometheus on 9090, Grafana on 3000).

## Step 12 — Build and verify

```bash
cd docs && npx antora antora-playbook.yml
```

Fix every error and every warning that indicates a real problem: a broken `xref:`, a malformed AsciiDoc table (the
most common failure with tables this wide), invalid Mermaid or PlantUML syntax, a missing nav entry.

Then verify what the build won't catch:

- **Every Mermaid and Kroki diagram actually rendered** — open the built HTML and check. A Kroki block with a
  syntax error, or with the extension unregistered, renders as a literal code block without failing the build.
- Links to the generated OpenAPI/protobuf HTML use the real GitHub Pages path the build workflow publishes to.
- No page contains an invented fact where a `TODO:` belongs. Re-read the architecture and use-case pages
  specifically with that question in mind — they're where invention creeps in.

## Step 13 — Report

Summarize:

- Pages created, pages updated, pages omitted (and why).
- Whether Kroki had to be installed and the playbook amended.
- **Every `TODO:` written, grouped by page**, so the user has a concrete list of what only a human can supply. This
  is the most useful part of the report — present it as a list, not a count.
- **Every discrepancy found between the code and the manifest, the spec and the security configuration, or the
  spec and the exception mapping.** These are findings about the service, not about the documentation: an
  undocumented error response, an endpoint whose spec security doesn't match its filter chain, a property that
  looks dynamic but isn't `@RefreshScope`d, a metric with unbounded tag cardinality, an index no query uses.
- Whether hand-written prose was preserved or contradicted on a re-run.
- The build result and the path to `docs/build/site/index.html`.

Warn explicitly:

- **Review the derived content before trusting it**, particularly the architecture rationale and the use-case
  descriptions — those are the pages where a plausible sentence can be wrong.
- **The configuration table's "dynamically changeable" and "critical" columns get used under pressure.** If either
  is wrong, someone will make an incident worse by acting on it. Verify them.
- With the public Kroki server, diagram source leaves the network at build time; self-host if the model is
  sensitive.
- Re-run this skill after any change to a contract, an entity, a configuration property, a metric, or a use case —
  the pages are derived, so they go stale the moment the code moves.
