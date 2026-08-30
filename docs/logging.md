# Robot telemetry

Artemis uses one typed `Telemetry` utility backed by WPILib's Epilogue NetworkTables backend.
`DataLogManager` mirrors the resulting `/Telemetry` topics into the managed `.wpilog`.

## Subsystem pattern

Each subsystem owns a scope and logs from an `outputTelemetry()` method:

```java
private static final Scope LOG = Telemetry.scope("Mechanisms/Shooter");

private void outputTelemetry() {
    LOG.critical.log("VelocityRps", velocityRps);
    LOG.critical.log("SetpointRps", setpointRps);
    LOG.info.log("AppliedDutyCycle", appliedDutyCycle);
}
```

Call `outputTelemetry()` after refreshing hardware and completing control calculations. Use:

- `critical` for state, measurements, setpoints, poses, acceptance decisions, and timing.
- `info` for electrical, fault, and health data.
- `debug` for temperatures, module details, tag diagnostics, and visualizations.

Primitive values, enums, arrays, common WPILib structs, and explicit `Struct` values are supported.
Create child scopes for dynamic hierarchy such as cameras or BLine. Do not access
`NetworkTableInstance` or `EpilogueBackend` outside `Telemetry.java`.

Logging is synchronous with the subsystem periodic that owns it. There is no separate Epilogue
callback, aggregate snapshot tree, logging offset, generated logger, or type-specific telemetry
file. Consequently logging time is included in scheduler and full-cycle timing.

## Citrus Circuits influence

Team 1678 uses the same ownership convention: subsystem `periodic()` methods refresh their IO and
call an `outputTelemetry()` method, while shared utilities handle publisher details. Artemis keeps
that readable call-site pattern but uses a single typed Epilogue backend instead of scattered
`SmartDashboard.put*` calls.

AdvantageKit's IO/Inputs pattern remains useful if deterministic replay is desired later, but that
larger architecture is not required for this telemetry-only design.

- [Citrus Circuits 2026 public robot](https://github.com/frc1678/C2026-Public)
- [AdvantageKit IO interfaces](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/)
