# Architecture

This document explains how `sdt-vehicle-core` is structured, how data moves through it, how
threads interact, and — most importantly — **why** it is built this way.

## 1. Goals and non-goals

**Goals**

* Control air, ground or mechani
* Be one backend among many: it talks to other Software DT backends and frontenrough a
  small, stable, transport-independent poutputs a *velocity setpoint* and leaves inner loops
  to the platform (autopilot, motor controller, joint servos).
* State estimation / SLAM — the core consumes a fused `Ve──────────────────────────────────────────────────────