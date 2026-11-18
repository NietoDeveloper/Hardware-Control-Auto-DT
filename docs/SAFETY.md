# Safety concept

This document describes how `sdt-vehicle-core` tries to stay safe, what it does when things go
wrong, and — just as importantly — what it does **not** claim.

## 1. Scope and honesty

v1 applies safety-critical *engineering practices* (determinism, static allocation, layered
prot