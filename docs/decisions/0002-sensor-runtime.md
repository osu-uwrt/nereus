# ADR 0002: Composed sensor models and synchronous delivery

Status: accepted for the first nonvisual sensor increment.

## Context

The plant must remain usable without sensors or middleware. New sensor families
must not require edits to physics or a scheduler switch. Future training callers
need explicit time/reset control; the independent viewer will also consume live
or recorded measurements without constructing simulated devices.

## Decision

Add a `RoboticsPlatform::sensors` library depending on the plant. Concrete IMU,
FOG, and DVL value models compose mount/noise state and narrow providers. The
runtime owns a plant and model instances, acquires after each committed tick, and
exposes typed streams with separate acquisition/delivery times and bounded queues.
Data-only headers separate measurements from models and runtime ownership.

A private abstract scheduling interface provides type erasure over heterogeneous
models. This limited use of inheritance is justified by runtime ownership of an
open set of model types; it shares no model implementation or family-specific
methods. Users supply ordinary models without deriving from a sensor hierarchy.

Keep scheduling synchronous and based on integer simulated time. Use device-owned
random engines, stable seed derivation, and complete stream/model reset. Runtime
failures invalidate all stream handles until reset rather than expose a partially
successful set of acquisitions as a healthy step. Completed plant ticks remain
observable; rollback across arbitrary extension code is not promised.

Use endpoint physics derivatives for inertial sensing. Explicitly mark contact
acceleration unavailable until a force/contact model supports it. Start DVL with
an ideal single-ray bottom-lock model and a finite pool-floor query, independent
of an acoustic engine or renderer.

## Consequences

The delivered API is directly usable from standalone C++ and supports repeatable
stepping. It does not yet provide native sensor profile loading, Python bindings,
multiple-body simulation, serialized payload schemas, or asynchronous render jobs.
A future camera/stereo model can use typed payloads and separate rendering providers;
immutable shared buffers avoid copying full images. Its rendering completion and
pairing contracts must be validated when implemented, not inferred from these tests.

Direct model providers must own/reset any mutable state they introduce. Queue
capacity is explicit; callers advancing long batches must drain, allocate sufficient
capacity, or deliberately choose the dropping policy. Consumer polling never drives
acquisition. Replay guarantees apply within a supported build, not across numerical
backends or standard library implementations.

See [runtime contracts](../SENSOR_RUNTIME.md) and [remaining sensor scope](../SENSORS.md).
