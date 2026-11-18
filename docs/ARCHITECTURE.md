# Architecture

This document explains how `sdt-vehicle-core` is structured, how data moves through it, how
threads interact, and — most importantly — **why** it is built this way.

## 1. Goals and non-goals

**Goals**

* Control air, ground or mechani
* Be one backend among many: it talks to other Software DT backends and frontenrough a
  small, stable, transport-independent poutputs a *velocity setpoint* and leaves inner loops
  to the platform (autopilot, motor controller, joint servos).
* State estimation / SLAM — the core consumes a fused `Ve─────────────────────────────────────────────────────────────┐(router)                               │  edges
├─────────────────────────────────eads
or vendor code. The only code allowed to know about the physical world is an adapter implementing a
port.

## 3. Modules

| Module | Responsibility | Key types |
|---|---|---|
| `common/` | Units, time, logging, wait-free queue | `Vec3`, `VehicleState`, `VelocityCommand`, `IClock`/