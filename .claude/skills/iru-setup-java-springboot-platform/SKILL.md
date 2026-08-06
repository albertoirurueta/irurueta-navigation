---
name: iru-setup-java-springboot-platform
description: Generate the OpenTofu (Terraform-compatible) infrastructure-as-code that deploys a Spring Boot service to AWS or Google Cloud, derived from the technologies the service actually uses — a remote encrypted state backend, a keyless GitHub OIDC deployment role, a container registry, the compute runtime (ECS Fargate/EKS on AWS, Cloud Run/GKE Autopilot on Google Cloud) with region and replica/CPU/memory sizing as variables, and a managed instance of each database, cache, Kafka cluster, and secret store the stack needs, always preferring the cloud provider's managed offering over a self-packaged container. Also wires load balancing, DNS, TLS, secret injection from AWS Secrets Manager or Google Secret Manager, and the IAM/RBAC the service's dynamic configuration requires. Reads `springboot-stack.yml` for the technology list and the target cloud, asks only the questions IaC genuinely cannot infer (region, environments, sizing, existing versus new infrastructure, domain names), scaffolds `infra/` with reusable modules and one directory per environment, and writes explicit manual instructions for any step OpenTofu cannot perform. Never writes a credential, token, or secret value into any file. Invoke as `/iru-setup-java-springboot-platform`, or with `args` (`stack-file:` line) when called from `iru-setup-java-springboot`. Use whenever a Spring Boot service needs deployable, reviewable infrastructure-as-code instead of hand-clicked cloud resources.
model: sonnet
---

# Setup Java Spring Boot Platform

Generate the OpenTofu configuration that deploys this service. Five rules govern everything:

1. **Managed services over self-packaged containers.** The compose stack from
   `iru-setup-java-springboot-testcontainers` exists for local development and integration tests. Deployed
   environments use the provider's managed offering — RDS not a Postgres container, MSK not a Kafka container —
   because backups, failover, patching, and encryption-at-rest are the actual work, and running them yourself is
   a permanent cost.
2. **No secret in any file.** Not in a `.tf`, not in a `.tfvars`, not in a default. Secrets are created empty (or
   out-of-band) in the provider's secret manager and referenced by ARN/name; the value is populated by a human or
   a separate process. State this in the report.
3. **Remote, encrypted, locked state.** `tofu` state contains resource attributes that are effectively secrets
   (connection strings, generated passwords, certificate metadata). Local state in a repository is a credential
   leak waiting to happen.
4. **Keyless CI authentication.** GitHub Actions authenticates via OIDC to a role/service account with a trust
   policy scoped to this repository. No long-lived access key is ever created.
5. **Nothing is applied by this skill.** It writes configuration and runs `tofu validate`/`plan` at most. Every
   `apply` is the user's explicit decision, because every `apply` costs money.

## Step 0 — Resolve inputs

Parse `args` for `stack-file: <path>`, defaulting to `springboot-stack.yml`. Read it for `stack.cloud`, the
database engines, caches, messaging, metrics/tracing, `stack.dynamicConfig`, and whether the service exposes REST
and/or gRPC. That tells you what infrastructure is needed. It does **not** tell you the operational parameters —
Step 1 asks those.

Check whether `tofu` (or `terraform`) is on the PATH; if not, note it, write the files anyway, and skip Step 6's
validation with an explicit note. Check for an existing `infra/` directory: if one exists, survey it and treat
this run as gap-filling — never overwrite a `.tf` file that manages live resources, and never touch a `.tfstate`.

## Step 1 — Ask what cannot be inferred

Ask these directly; each genuinely changes the generated configuration. Use `AskUserQuestion` for the bounded ones.

- **Environments** — which to generate (`dev`, `staging`, `prod` is the common set; a single `prod` is valid for a
  small service). Each gets its own directory and its own state, so a `dev` mistake can't reach `prod`.
- **Region** (per environment, defaulting to the same for all) — and for `prod`, whether multi-AZ/multi-zone
  redundancy is required. Multi-AZ roughly doubles managed-database cost; it's the user's call, not a default to
  assume.
- **Sizing per environment** — replica count (min/max for autoscaling), CPU, and memory per instance. Offer
  starting points and say they're starting points: a small Spring Boot service typically wants 1 vCPU / 1–2 GiB and
  2 replicas in `prod`, 0.5 vCPU / 1 GiB and 1 replica in `dev`. Ask for the JVM heap intent too, since container
  memory must exceed heap plus metaspace plus native overhead — a container sized exactly to `-Xmx` gets
  OOM-killed.
- **Compute runtime** (`AskUserQuestion`):
  - AWS: **ECS Fargate** (recommended — no cluster to operate, and enough for a single service) or **EKS**
    (choose when Kubernetes is already the org standard, or `stack.dynamicConfig` is `kubernetes`, which requires
    it).
  - Google Cloud: **Cloud Run** (recommended — scales to zero, simplest) or **GKE Autopilot** (choose when
    Kubernetes is the org standard or `stack.dynamicConfig` is `kubernetes`).
  - If `stack.dynamicConfig` is `kubernetes` and the user picks a non-Kubernetes runtime, flag the contradiction
    and make them resolve it rather than generating a configuration that can't work.
- **Existing versus new infrastructure** — for **each** database, cache, and Kafka cluster: create it here, or
  reference one that already exists? Referencing an existing resource means a `data` source and a variable for its
  identifier, which is very different configuration from creating it. Ask per resource; teams commonly share one
  database cluster across services.
- **Public exposure** — is the service internet-facing or internal only? If internet-facing: the domain name, and
  whether the DNS zone already exists in Route 53 / Cloud DNS or must be created. TLS certificates come from ACM
  or Google-managed certificates; note that DNS validation requires the zone to be authoritative, which is a
  manual step if the registrar is elsewhere.
- **State backend** — the S3 bucket + DynamoDB lock table, or the GCS bucket, to use. If none exists, generate a
  separate tiny `infra/bootstrap/` configuration that creates just the backend (applied once, with local state,
  then migrated) and say clearly that this is the one chicken-and-egg step.
- **Cost acknowledgement** — state plainly that applying this will create billable resources, name the ones that
  bill continuously regardless of traffic (managed databases, NAT gateways, load balancers, MSK/Managed Kafka
  clusters, provisioned OpenSearch), and confirm the user wants them generated. NAT gateways and idle Kafka
  clusters are the two line items that most often surprise people.

## Step 2 — Directory layout

```
infra/
  README.md                    # how to plan/apply, what is manual, what each variable means
  bootstrap/                   # only if the state backend must be created
  modules/
    network/                   # VPC/subnets/security groups, or the GCP equivalent
    service/                   # the compute runtime + its IAM + secret wiring
    database-<engine>/         # one per engine in the stack
    cache-redis/
    messaging-kafka/
    observability/
    dns/
  envs/
    dev/
      main.tf  variables.tf  outputs.tf  backend.tf  terraform.tfvars
    prod/
      ...
```

- **One state file per environment**, configured in that environment's `backend.tf`. Never a shared state.
- Modules take inputs and return outputs; they contain no environment-specific literals. The environment
  directories are thin — provider config, backend config, module calls, and values.
- Write `.tfvars.example` files with every variable documented, and git-ignore the real `.tfvars` (which may
  contain account ids and hostnames that aren't secrets but aren't worth publishing either). Confirm `.gitignore`
  covers `.terraform/`, `*.tfstate`, `*.tfstate.*`, `.terraform.lock.hcl` is **committed** (it pins provider
  versions — that's a feature), `*.tfvars`, and `crash.log`.
- Pin the OpenTofu version (`required_version`) and every provider version (`required_providers` with `~>`
  constraints). An unpinned provider will eventually change a resource's default and produce a destructive plan.

## Step 3 — What to generate, per cloud

Look up each resource type's current arguments in the provider documentation as you write — provider schemas
change, and a plausible-looking argument that doesn't exist fails `validate` with a confusing message. Generate
only what the stack actually needs.

### Common to both clouds

- **State backend** — remote, versioned, encrypted, with locking.
- **OIDC deployment identity** — a role (AWS) or service account + workload identity pool (Google Cloud) whose
  trust policy is scoped to `repo:<owner>/<repo>:ref:refs/heads/<branch>` or
  `repo:<owner>/<repo>:environment:<env>`. **Scope it to the environment**, not to `repo:<owner>/<repo>:*` — a
  wildcard subject lets any branch or PR in the repository deploy to production. Grant only the permissions the
  deploy needs (push image, update service, read secrets), not administrator.
- **Container registry** — ECR or Artifact Registry, with an immutable-tag policy and a lifecycle rule that expires
  untagged images.
- **The service itself** — with the container image reference as a variable (the workflow passes the built tag),
  the environment variables and secret references from Step 4, health-check paths matching the actuator probes
  (`/actuator/health/liveness`, `/actuator/health/readiness` on the management port), a graceful-shutdown period
  longer than the app's shutdown timeout, and autoscaling on CPU/request concurrency.
- **Secrets** — one secret per credential in Secrets Manager / Secret Manager, created with **no value** (or a
  placeholder that cannot work), with `lifecycle { ignore_changes = [...] }` on the value so a later `apply`
  never reverts a rotated secret. This pattern is what lets IaC manage the secret's existence and IAM without ever
  seeing its content.
- **Logging and metrics sinks** — CloudWatch log group with a retention period (the default of "never expire" is a
  cost trap), or Cloud Logging with a retention policy; and the Prometheus scrape path. If `stack.metrics` is
  true, generate the managed metrics target: Amazon Managed Service for Prometheus + Managed Grafana, or Google
  Cloud Managed Service for Prometheus. Note that the local compose Prometheus/Grafana are for development only.

### AWS specifics

| Need | Managed service | Notes to put in the generated comments |
|---|---|---|
| Compute | ECS Fargate service + ALB target group, or EKS deployment | Fargate needs no capacity management |
| MongoDB | **MongoDB Atlas** (via the `mongodbatlas` provider), not DocumentDB | DocumentDB emulates an older MongoDB wire protocol and lacks features Spring Data and Mongock use; if the user insists on staying in-account, flag the compatibility risk explicitly and test the migration tooling against it early |
| PostgreSQL | RDS or Aurora PostgreSQL | `pgvector` is available as an extension — must be enabled per database; confirm the engine version supports the needed version |
| MariaDB | RDS MariaDB | — |
| Couchbase | **Couchbase Capella** (its own provider) | no first-party AWS equivalent |
| Elasticsearch | Amazon OpenSearch Service, or Elastic Cloud | OpenSearch has diverged from Elasticsearch; the Spring Data Elasticsearch client may need the Elastic-compatible endpoint or a version pin — flag this |
| Neo4j | **Neo4j Aura**, not Neptune | Neptune is not Neo4j: no Cypher-over-Bolt compatibility that Spring Data Neo4j expects |
| Qdrant | **Qdrant Cloud** | — |
| Redis | ElastiCache for Valkey/Redis, or MemoryDB if durability is required | ElastiCache is a cache; MemoryDB is a durable data store — pick per intent |
| Kafka | Amazon MSK (or MSK Serverless) | MSK has **no managed schema registry** — AVRO needs AWS Glue Schema Registry (a different client configuration than Confluent) or a self-hosted/Confluent Cloud registry. This is a real decision, not a detail: raise it explicitly and record which was chosen |
| Secrets | AWS Secrets Manager (+ Parameter Store for non-secrets) | required by `dynamicConfig: aws-secrets-manager` |
| Load balancer / DNS / TLS | ALB (REST) — gRPC needs the target group protocol set to `HTTP2`/`gRPC`; Route 53; ACM | an ALB with the wrong protocol version silently fails gRPC |
| Network | VPC, private subnets, NAT gateway, security groups | one NAT gateway per AZ is the cost/HA tradeoff — call it out |

### Google Cloud specifics

| Need | Managed service | Notes |
|---|---|---|
| Compute | Cloud Run service, or GKE Autopilot deployment | Cloud Run scales to zero — excellent for `dev`; note cold starts, and that `min-instances: 1` avoids them at a cost |
| MongoDB | MongoDB Atlas on GCP | Firestore is not MongoDB-compatible enough for Spring Data MongoDB + Mongock |
| PostgreSQL | Cloud SQL for PostgreSQL, or AlloyDB | `pgvector` supported; AlloyDB for heavier analytical load |
| MariaDB | Cloud SQL for MySQL (MariaDB-compatible) | not a true MariaDB — flag the driver/compatibility implication |
| Couchbase | Couchbase Capella | — |
| Elasticsearch | Elastic Cloud on GCP | — |
| Neo4j | Neo4j Aura on GCP | — |
| Qdrant | Qdrant Cloud | — |
| Redis | Memorystore for Redis / Valkey | — |
| Kafka | Google Cloud Managed Service for Apache Kafka, or Confluent Cloud | same schema-registry gap as MSK — Confluent Cloud includes one, the managed service may not; check and record |
| Secrets | Secret Manager | required by `dynamicConfig: kubernetes` (mounted) or read directly |
| Load balancer / DNS / TLS | Global external Application Load Balancer, Cloud DNS, Google-managed certificates | Cloud Run + gRPC needs HTTP/2 end-to-end |
| Network | VPC, Serverless VPC Access connector (Cloud Run → private database), firewall rules | a Cloud Run service **cannot** reach a private Cloud SQL/Memorystore instance without the connector — this is the most common omission |

### Dynamic configuration wiring

- **`kubernetes`** — create the ConfigMap and Secret the service reads, plus a Role/RoleBinding granting the
  service account `get`/`list`/`watch` on ConfigMaps and Secrets in its namespace. Without the `watch`
  permission, `spring.cloud.kubernetes.reload` silently never fires, which looks like an application bug.
- **`aws-secrets-manager`** — the secrets, plus IAM allowing `secretsmanager:GetSecretValue` on exactly those ARNs
  (never `*`).
- **`spring-cloud-config`** — note that a Config Server is a separate deployable this configuration does not
  create; generate the client's environment variables and say the server's URL is an input.

## Step 4 — Connect the service to its dependencies

For each managed dependency, generate:

- the connection endpoint as an environment variable on the service (matching the placeholder names the `dev`/`prod`
  Spring profiles from `iru-setup-java-springboot-modules` already reference — read those files so the names
  actually line up, rather than inventing new ones);
- the credential as a **secret reference**, injected by the runtime, never as a plain environment variable value;
- the network path — security group rule / firewall rule / VPC connector — allowing the service to reach it, scoped
  to the service's own identity rather than to a CIDR where possible;
- IAM for the service's own workload identity to read its secrets and write its logs and metrics.

Generate an `outputs.tf` per environment exposing the endpoints and secret names, so the workflow and a human
operator can find them without opening the console.

## Step 5 — Document the manual steps

Write `infra/README.md` covering, concretely rather than generically:

- The bootstrap order: create the state backend, then `tofu init -migrate-state`, then per environment
  `tofu init && tofu plan && tofu apply`.
- **Every step OpenTofu cannot do**, each with instructions:
  - registering a domain, or delegating a subdomain when the registrar is outside the cloud provider;
  - populating each secret's value (the exact CLI command per secret, with the value redacted);
  - third-party accounts and their API keys — MongoDB Atlas, Couchbase Capella, Neo4j Aura, Qdrant Cloud,
    Confluent Cloud, Elastic Cloud — each of which needs an organization, a payment method, and a provider
    credential before its OpenTofu provider can do anything;
  - the schema registry's compatibility mode;
  - any account-level quota increase or service enablement (GCP API enablement can be generated; some AWS
    service quotas cannot);
  - SSO/OAuth provider configuration if `stack.security.enabled`.
- What each variable means and its blast radius.
- How to destroy an environment safely, and what has deletion protection on purpose (production databases should;
  say so).

## Step 6 — Validate

Never `apply`. Run, per environment:

```bash
tofu -chdir=infra/envs/<env> init -backend=false
tofu -chdir=infra/envs/<env> validate
tofu fmt -recursive infra/
```

`-backend=false` lets validation run before the backend exists. Fix every error — an argument that doesn't exist
in the provider schema, a missing required argument, a type mismatch.

Offer to run `tofu plan` against real credentials **only if the user has them configured and asks for it**, and be
explicit that `plan` is read-only but does authenticate to the account. Report the resource counts (`to add`) and
call out anything the plan says it would destroy — in a fresh environment nothing should be destroyed, so a
destroy in the plan means the configuration is fighting something that already exists.

## Step 7 — Report

Summarize:

- The cloud, environments, regions, and sizing generated.
- The compute runtime chosen, and every managed service the configuration will create versus reference as
  existing.
- Where the service's **third-party** dependencies land (Atlas, Capella, Aura, Qdrant Cloud, Confluent/Elastic
  Cloud) and that each needs its own account, billing, and provider credential.
- The OIDC identity created and exactly what it's scoped to.
- The secret names created empty, and the command to populate each.
- Every compatibility caveat you flagged — DocumentDB versus MongoDB, OpenSearch versus Elasticsearch, Neptune
  versus Neo4j, Cloud SQL MySQL versus MariaDB, MSK's missing schema registry, gRPC's HTTP/2 load-balancer
  requirement, Cloud Run's VPC connector requirement.
- The result of Step 6.
- The consolidated list of manual steps from Step 5.

Warn explicitly:

- **`tofu apply` creates billable resources.** Read the full plan first. Name the always-on cost items again here.
- **Nothing was applied and no secret value was written anywhere.** The service will not start until each secret
  is populated out-of-band — that is the design.
- **State must be remote and encrypted before the first real apply**, and the state file must be treated as
  sensitive.
- **The undeploy workflow (`iru-setup-java-springboot-github-workflows`) can destroy these resources.** Confirm
  deletion protection is enabled on anything holding data before that workflow exists in a state where someone
  can run it.
- Provider versions are pinned; upgrading one is a reviewed change, because a provider major version can produce a
  destructive plan for unchanged configuration.
