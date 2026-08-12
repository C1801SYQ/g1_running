# G1 locomotion audit bridge

This first-stage bridge is intentionally incapable of commanding the robot. It
binds only to `127.0.0.1:15003`, validates and limits `vx vy wz` packets, forces
`vy=0`, and replaces the command with zero after 200 ms without a packet.

The only optional SDK calls are read-only `GetFsmId`, `GetFsmMode`, and
`GetBalanceMode` queries. There are no velocity, stop, stand, start, or FSM
mutation calls in this executable.

Build on the G1 NX against its existing SDK:

```bash
cmake -S . -B build -DUNITREE_SDK_ROOT=/home/unitree/unitree_sdk2-main
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

Run in audit mode:

```bash
./build/g1_loco_audit_bridge --mode=audit --query-status
```
