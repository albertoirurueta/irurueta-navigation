# Code style

This catalog's stated preferences. Where a preference here disagrees with what most Spring Boot write-ups
assume — notably on MapStruct — this file wins, and says why.

Above all: **match the surrounding code.** If the module you are editing already does something a specific way,
follow it and note the divergence in the report rather than converting the file to this document's preference as
a side effect of an unrelated task.

## Records for immutable types

Use a `record` for anything immutable: value objects, DTOs, commands, queries, events, and configuration
property holders.

```java
/**
 * A monetary amount in a specific currency.
 *
 * @param amount   the amount; never {@code null}, scale is preserved as given.
 * @param currency the currency; never {@code null}.
 */
public record Money(BigDecimal amount, Currency currency) {

    /**
     * Creates a monetary amount.
     *
     * @throws IllegalArgumentException if {@code amount} or {@code currency} is {@code null}.
     */
    public Money {
        if (amount == null || currency == null) {
            throw new IllegalArgumentException("amount and currency are required");
        }
    }
}
```

Notes that matter in practice:

- **Validate in the compact constructor.** That is what makes "a value object is always valid" true, which in
  turn means callers never have to null-check one. See `ddd-tactical-patterns.md`.
- **A record is shallowly immutable.** A `record Order(List<Item> items)` hands out the caller's mutable list.
  Copy defensively in the compact constructor (`List.copyOf(items)`) for any collection or mutable field.
- **Don't use a record for a JPA `@Entity`** — JPA needs a no-arg constructor and non-final fields. Persistence
  entities in `infrastructure/database/*` are ordinary classes; the *domain* type they map to is the record.
  Document store and R2DBC mappings vary — check what the module already does.

## Lombok where it earns its place

Use Lombok wherever it removes real boilerplate:

- `@RequiredArgsConstructor` on Spring components for constructor injection — the standard shape for an adapter
  or use case with `private final` dependencies.
- `@Builder` where a type has enough optional fields that positional construction is unreadable.
- `@Getter`/`@Setter`, `@EqualsAndHashCode`, `@ToString` on mutable classes such as persistence entities.
- `@Slf4j` for a logger.

Don't use Lombok where it adds nothing or actively hurts:

- **Not on records** — they already generate accessors, `equals`, `hashCode`, and `toString`.
- **Not `@Data` on a persistence entity.** Its generated `equals`/`hashCode` covers every field, which breaks
  for entities with generated ids and lazy associations, and its `@ToString` will happily trigger lazy loading.
  Prefer explicit `@Getter`/`@Setter` plus an `equals`/`hashCode` on the identifier only.
- **Not `@AllArgsConstructor` on a Spring bean** — it silently changes the injected constructor's signature when
  a field is added or reordered.

If Lombok and MapStruct are both in play, `lombok-mapstruct-binding` must be on the annotation processor path,
in the order lombok → binding → mapstruct-processor. Without it MapStruct runs before Lombok has generated the
accessors and emits mappers that silently map nothing.

## MapStruct only for simple mappers — otherwise map by hand

This is the preference that runs against most published advice, so the reasoning matters.

**Use MapStruct when** the mapping is field-for-field with matching names and types, or needs only a trivial
rename. That is the case it was built for, and hand-writing it is pure boilerplate. The reactor sets
`unmappedTargetPolicy=ERROR`, so a forgotten field fails compilation rather than silently producing a null.

**Write the mapper by hand when** the mapping involves any of:

- conditional logic, or a branch on the source's state;
- building a value object through a factory or validating constructor (a generated mapper reaching for a
  constructor it doesn't understand tends to produce either a compile error or a bypassed invariant);
- flattening or restructuring — several sources into one target, or one source into a nested graph;
- a lookup, a default, or a derived field that isn't a pure function of one source field;
- more than a couple of `@Mapping(expression = "java(...)")` entries. Java-in-a-string annotation is unreadable,
  untestable in isolation, and invisible to refactoring tools. Once you are reaching for the second one, the
  hand-written mapper is shorter *and* clearer.

A hand-written mapper is a plain `@Component` (or a static factory) with a method per direction and full
Javadoc. It is ordinary code: it can be read, debugged, and unit-tested directly. That is the point.

## Type inference

Prefer `var` for locals where the type is obvious from the right-hand side:

```java
final var order = orderRepository.findById(orderId);
final var items = new ArrayList<OrderItem>();
```

Keep the explicit type when `var` would genuinely hide something the reader needs — a non-obvious factory
return, a numeric type where width matters, or a `var` that would resolve to an unintended supertype. Never use
`var` for a field, a parameter, or a return type; it isn't legal there anyway.

**Follow the surrounding file.** If a module doesn't use `var` anywhere, don't introduce it in a one-line
change.

## `final` wherever possible

- **Fields**: `private final` for every injected dependency and every field set once.
- **Locals**: `final var` / `final Type` by default. It documents that the reference doesn't change and makes an
  accidental reassignment a compile error.
- **Parameters**: mark `final` if the surrounding code already does. Don't retrofit it across a file for an
  unrelated task.
- **Classes**: `final` for value objects that aren't records, mappers, and utility classes. Not on Spring
  `@Configuration` classes or anything needing a proxy (CGLIB cannot subclass a final class, and the failure
  message is unhelpful).

## Javadoc on everything

**All code must be fully documented.** On every type and every public or protected member:

- what it does, and any contract a caller must satisfy;
- `@param` for every parameter — including every record component, on the record's own Javadoc;
- `@return`, unless `void`;
- `@throws` for every exception the caller can reasonably be expected to handle, with the condition that
  triggers it. This is not optional decoration: the `@throws` list is what the tests are written against.

Document private members whenever the *why* isn't obvious from the name. A comment explaining why an invariant
exists is worth far more than one restating what the next line does.

For a port interface, the Javadoc is the contract every adapter must honour — state what an implementation
must guarantee (ordering, idempotency, whether it may return `null`, what it does when nothing is found). These
interfaces are the architecture's real documentation.

## Other conventions

- **`IllegalArgumentException` for invalid arguments**, with a message naming the offending parameter. Domain
  rule violations throw a domain exception instead (see `ddd-tactical-patterns.md`).
- **Constructor injection only.** No `@Autowired` on fields — it defeats `final` and makes the class
  untestable without a container.
- **No new runtime dependency** unless the task explicitly calls for it. If one seems unavoidable, that is a
  blocker to report, not a decision to make silently — and remember the version would have to go in the root
  pom.
- **Don't catch and swallow.** Either handle it meaningfully or let it propagate; a bare
  `catch (Exception e) { log.error(...); }` in an adapter turns a failure into silent data loss.
- **No `System.out`** — use the injected logger, and don't log a secret, a token, or a full request body
  containing personal data.
