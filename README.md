# FPGA PID Controller

PID feedback controller in Verilog with Q8.8 fixed-point gains, saturating
arithmetic and an anti-windup integrator. Simulated with Icarus Verilog,
synthesized with Yosys and placed and routed with nextpnr for a Lattice ECP5.

## Design

| Parameter | Value |
|---|---|
| Data width | 16-bit signed, Q8.8 gains |
| Gains | Kp, Ki, Kd are runtime inputs |
| Integral gain | Ki / 256 per clock cycle, where Ki is the Q8.8 gain value. In raw input units that is ki_raw / 65536. The integrator keeps 16 fractional bits |
| Latency, input to output | 5 clock cycles (P path), 6 (D path); I path: 6 cycles, plus the time for the integral to reach one count |
| Derivative | Low-pass filtered (keeps 3/4 of the previous value each cycle) |
| Anti-windup | Integrator clamped to its 32-bit range, and held while the output is saturated in the direction of the error |
| Saturation | Error, each term, the sum and the output all saturate instead of wrapping |

Timing is met at 100 MHz on a Lattice ECP5-25k (nextpnr reports 126.86 MHz,
one seed, no pin constraints), so the worst-case latency is 60 ns.

## Verification

`tb/checks_tb.v` runs self-checking directed tests. Run against the first version
of this design, four of the seven checks fail (A, both B checks and D):

| Test | First version | Current |
|---|---|---|
| A. Kp = 1.0, error = 1000 | Output -24 (16-bit multiply truncation) | Output 1000 |
| B. Ki = 1.0, error +2000 held 5000 cycles | Integral term wrapped; output -26505 | Integrator clamps; output holds at +32767 |
| B. Sign flips in 20000 cycles after error reverses | 5 | 1 |
| D. All gains near maximum, error +30000 | not tested | Output pins at +32767, never wraps |

Latency is 5 / 6 / 6 cycles (P / I / D), up from 4 / 5 / 5 before the
multiplier outputs were registered.

The fixes: 32-bit product wires for each multiply, saturation at every stage,
an 18-bit sum, the integrator clamp with conditional integration, and an
integrator that accumulates the full product so small errors are not rounded
away.

`tb/pid_controller_tb.v` closes the loop around a first-order plant model
(`src/plant.v`) with noise injection. With its gains (Kp = 0.5, Ki = 0.1,
Kd = 0.25) the loop converges on the target but slowly: with each target held
for 10,000 cycles the measured value is still 4 to 6 counts off (about 122,
188 and 70 for targets of 128, 192 and 64). Held for 100,000 cycles it
averages 127.9, 192.1 and 64.1, with about ±4 counts of ripple from the
injected noise. The testbench as written holds each target for 1,000 cycles,
which is too short to see this.

## Timing and resources

Place-and-route with nextpnr for a Lattice ECP5-25k (LFE5U-25F, CABGA381),
100 MHz target, before and after registering the multiplier outputs:

| | Before | After |
|---|---|---|
| Maximum clock frequency | 73.38 MHz | 126.86 MHz |
| 100 MHz | Fails | Passes |
| Critical path delay | 13.63 ns | 7.88 ns |
| Critical path start | `error_change` register | `i_product_r` register |
| Critical path end | `derivative_term` register | `integrator` register |

Registering each multiplier output took the multiplier off the critical path,
at the cost of one cycle of latency.

| ECP5 resource (after) | Used | Available |
|---|---|---|
| LUT4s | 458 | 24,288 |
| Flip-flops | 240 | 24,288 |
| DSP multipliers (MULT18X18D) | 3 | 28 |
| I/O pins | 98 | 197 |

Generic Yosys synthesis: 6536 cells, 240 flip-flops (`docs/yosys_report.txt`).

## Files

- `src/pid_controller.v`: the controller
- `src/plant.v`: first-order plant model for closed-loop simulation
- `tb/checks_tb.v`: directed overflow, windup, latency and saturation tests
- `tb/pid_controller_tb.v`: closed-loop testbench with noise injection
- `docs/yosys_report.txt`: synthesis report

## Run

    brew install icarus-verilog yosys
    iverilog -o sim.out src/pid_controller.v tb/checks_tb.v && vvp sim.out
    iverilog -o sim.out src/pid_controller.v src/plant.v tb/pid_controller_tb.v && vvp sim.out
    yosys -p "read_verilog src/pid_controller.v; synth -top pid_controller; stat"

ECP5 timing (nextpnr-ecp5 is not in Homebrew; it ships with OSS CAD Suite):

    yosys -p "read_verilog src/pid_controller.v; synth_ecp5 -top pid_controller -json pid.json"
    nextpnr-ecp5 --25k --package CABGA381 --json pid.json --freq 100 --timing-allow-fail --seed 1
