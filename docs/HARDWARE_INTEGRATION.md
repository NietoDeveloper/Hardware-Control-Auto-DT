# Integrating real hardware

The core never talks to hardware directly. You write **adapters** that implement the ports in
`include/sdt/hal/ports.hpp` and `include/sdt/comms/transport.hpp`. The simulator in `sim/` is the
reference implementation o