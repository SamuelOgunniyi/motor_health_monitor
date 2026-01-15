# Design Decisions

This document explains the key architectural decisions and trade-offs made in the Motor Health Monitor package.

## Factory Pattern for Motor Controllers

**Decision**: Use a singleton factory pattern with static auto-registration for motor controllers.

**Rationale**:
- **Extensibility**: Different robot platforms use different motor controller interfaces (PWM topics, joint states, custom messages). The factory pattern allows users to plug in their specific controller without modifying core monitoring code.
- **Zero-configuration registration**: Static initialization via `REGISTER_MOTOR_CONTROLLER` macro enables controllers to register themselves before `main()` runs, eliminating the need for explicit initialization code.
- **Runtime discovery**: The factory can enumerate registered controller types, enabling better error messages and dynamic configuration validation.
- **Loose coupling via interface**: Users implement the `MotorController` interface (dependency inversion principle). The monitoring system depends on the abstraction, not concrete implementations. Users don't modify core monitoring code.

**Coupling Analysis**:
- **Interface coupling**: Users must implement `MotorController` interface. This is intentional and minimal—only 4 virtual methods.
- **Header dependency**: Users include `motor_controller.hpp` and `motor_controller_factory.hpp`. These are stable, well-defined interfaces.
- **No runtime coupling**: Controllers are instantiated via factory, not direct dependencies. The monitoring system doesn't know about specific controller implementations.
- **Alternative approaches**: Plugin systems (dynamic loading) would reduce compile-time coupling but add complexity. For ROS 2 packages, compile-time coupling is standard and acceptable.

**Trade-offs**:
- Static initialization order is implementation-defined, but acceptable here since registration order doesn't matter.
- If controller instances need to persist across lifecycle transitions or maintain state history, explicit registration in `on_configure()` would be more appropriate.
- Some coupling exists, but it's through a stable interface, not implementation details.

**Alternatives Considered**:
- Direct instantiation: Would require modifying core code for each new controller type (tight coupling).
- Plugin system: More complex, requires dynamic loading infrastructure. Reduces compile-time coupling but adds runtime complexity.
- Dependency injection: Would require more complex configuration. Current approach balances simplicity with extensibility.

## Generic Fault Detection

**Decision**: Make fault detection generic with configurable threshold types (absolute, upper limit, lower limit) rather than PWM-specific.

**Rationale**:
- **Real-world requirements**: Field testing revealed needs beyond PWM saturation:
  - Current overload detection (upper limit)
  - Temperature monitoring (upper limit)
  - Battery voltage monitoring (lower limit)
  - PWM saturation (absolute value)
- **Code reuse**: One detector implementation handles all numeric feedback types, reducing maintenance burden.
- **Flexibility**: Users can monitor any motor feedback signal without code changes.

**Trade-offs**:
- Slightly more complex configuration (threshold type must be specified), but provides necessary flexibility.

**Alternatives Considered**:
- Separate detectors for each feedback type: Would lead to code duplication and maintenance overhead.
- Template-based approach: Overkill for this use case, adds complexity without significant benefit.

## Auto-Subscription with Topic Introspection

**Decision**: Controllers automatically detect message types via ROS 2 topic introspection rather than requiring explicit configuration.

**Rationale**:
- **Ease of use**: Most users don't need to specify message types—the system discovers them automatically.
- **Reduced configuration errors**: Eliminates mismatches between configured and actual message types.
- **Flexibility**: Still allows manual override via `motor_message_type` parameter when needed.

**Trade-offs**:
- Introspection adds a small runtime cost, but only during configuration phase.
- Requires topics to exist at configuration time (reasonable expectation for motor feedback topics).

**Alternatives Considered**:
- Require explicit message type configuration: More error-prone, less user-friendly.
- Hardcode message types: Too restrictive, prevents custom message support.

## Single-Threaded Executor

**Decision**: Use ROS 2 single-threaded executor (default for `component_container`).

**Rationale**:
- **Simplicity**: Eliminates need for mutexes and lock guards, reducing code complexity and potential deadlock risks.
- **Deterministic execution**: Single-threaded execution provides predictable timing, important for real-time control systems.
- **Sufficient performance**: The monitoring workload (message callbacks and periodic updates) doesn't require parallel processing.

**Trade-offs**:
- Cannot process multiple callbacks simultaneously, but this is acceptable given the lightweight processing requirements.

**Alternatives Considered**:
- Multi-threaded executor with mutexes: Adds complexity without performance benefit for this use case.

## Queue Depth of 1 (Bounded Queue with Drop-Oldest)

**Decision**: Default subscription queue depth of 1 with explicit QoS settings (Volatile, BestEffort).

**Rationale**:
- **Real-time control priority**: For control systems, fresh data is more valuable than historical data. Old messages are immediately dropped if processing lags.
- **Deterministic behavior**: Prevents stale data from affecting control decisions or fault detection accuracy.
- **Configurable**: Users can increase queue depth if they need message history or have bursty publishers, but default prioritizes freshness.

**Trade-offs**:
- Message loss if processing cannot keep up, but this is preferable to using stale data in control loops.

**Alternatives Considered**:
- Larger default queue: Would risk stale data affecting real-time control decisions.
- Reliable QoS: Overkill for control data where latest value matters most.

## Monotonic Clock for Stale Detection

**Decision**: Use `std::chrono::steady_clock` (monotonic) for stale data detection, ROS time for diagnostics timestamps.

**Rationale**:
- **Clock stability**: Monotonic clocks are immune to system clock adjustments (NTP updates, manual changes), ensuring accurate age calculations for stale detection.
- **ROS ecosystem compatibility**: ROS time (`now()`) is used for diagnostic message timestamps to match ROS ecosystem conventions and enable proper time synchronization in distributed systems.

**Trade-offs**:
- Two time sources add slight complexity, but each serves a distinct purpose.

**Alternatives Considered**:
- ROS time for everything: Vulnerable to clock adjustments affecting stale detection accuracy.
- Monotonic time for everything: Would break ROS time synchronization in diagnostics.

## Lifecycle Node Architecture

**Decision**: Implement as a ROS 2 lifecycle node rather than a standard node.

**Rationale**:
- **Controlled startup/shutdown**: Lifecycle states enable graceful initialization and cleanup, important for production systems.
- **Resource management**: Can cleanly release subscriptions and resources during deactivation/cleanup phases.
- **Integration**: Fits naturally with ROS 2 system lifecycle management patterns.

**Trade-offs**:
- Slightly more complex than standard node, but provides necessary production-ready behavior.

**Alternatives Considered**:
- Standard node: Simpler but lacks controlled startup/shutdown capabilities needed for production deployments.

## Component Architecture

**Decision**: Build as a composable component using `rclcpp_components`.

**Rationale**:
- **Modularity**: Can be loaded into a component container alongside other nodes, enabling efficient resource sharing.
- **Flexibility**: Supports both standalone and composed execution modes.
- **ROS 2 best practices**: Aligns with recommended ROS 2 architecture patterns.

**Trade-offs**:
- Requires component registration in CMakeLists.txt, but this is standard ROS 2 practice.

## Separation of Concerns

**Decision**: Separate command/odometry synchronization detection from motor fault detection.

**Rationale**:
- **Independent monitoring**: These detect different types of issues:
  - Sync detection: Control system or communication problems
  - Fault detection: Hardware-level motor issues
- **Clearer diagnostics**: Separate status reporting makes it easier to identify root causes.
- **Maintainability**: Each system can evolve independently.

**Trade-offs**:
- Two separate systems to maintain, but the separation provides clarity and flexibility.
