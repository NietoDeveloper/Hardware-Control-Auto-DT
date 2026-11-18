# Safety concept

This document describes how `sdt-vehicle-core` tries to stay safe, what it does when things go
wrong, and — just as importantly — what it does **not** claim.

## 1. Scope and honesty

v1 applies safety-critical *engineering practices* (determinism, static allocation, layered
protection, strict validation, fault injection). It is **not certified** to any 
| 0 | `kLinkLost` | No valid command frame within `link_hold_after` |
| 1 | `kStateStale` | No valid state estimate within `state_stale_after` |
| 2 | `kRadarStale` | No radar scan within `radar_degraded_after` (when 