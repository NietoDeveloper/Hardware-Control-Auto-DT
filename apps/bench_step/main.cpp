// Measures the wall-clock cost of one VehicleController::step() under load
// (20 radar-tracked obstacles, mission running, avoidance active). Reports p50/p99/p99.9/max.
#include <algorithm>
#include <chrono>
#include <cstdio>
