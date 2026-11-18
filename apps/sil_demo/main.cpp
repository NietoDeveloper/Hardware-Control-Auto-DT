// Software-in-the-loop demo. A simulated dashboard talks to the vehicle core over the real wire
// protocol (loopback transport), uploads a mission, arms, and watches the vehicle fly it while
// avoiding a static and a moving obstacle. Exit code 0 only if the mission completed with no
// collision, so the demo doubles as a CI smoke test.
//
//   sil_demo [nominal|link-loss]
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
