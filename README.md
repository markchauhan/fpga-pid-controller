# FPGA PID Controller

PID feedback controller in Verilog with Q8.8 fixed-point gains, saturating
arithmetic and an anti-windup integrator. Simulated with Icarus Verilog and
synthesized with Yosys.

## Design

| Parameter | Value |
|---|---|
| Data width | 16-bit signed, Q8.8 gains |
| Gains | Kp, Ki, Kd are runtime inputs |
| Integral gain | ki / 256 per clock cycle (the integrator keeps 16 fractional bits) |
| Latency, input to output | 4 clock cycles (P path), 5 (D path); I path: 5 cycles, plus the time for the integral to reach one count |
| Derivative | Low-pass filtered (keeps 3/4 of the previous value each cycle) |
| Anti-windup | Integrator clamped to its 32-bit range, and held while the output is saturated in the direction of the error |
| Saturation | Error, each term, the sum and the output all saturate instead of wrapping |

At a 100 MHz clock the worst-case latency is 50 ns. Timing closure at that
frequency has not been checked on a target device.

## Verification

`tb/checks_tb.v` runs self-checking directed tests. The first version of this design failed
two of them, which is how these bugs were found:

| Test | Before fix | After fix |
|---|---|---|
| A. Kp = 1.0, error = 1000 | Output -24 (16-bit multiply truncation) | Output 1000 |
| B. Ki = 1.0, error +2000 held 5000 cycles | Integral term wrapped; output -26505 | Integrator clamps; output holds at +32767 |
| B. Sign flips in 20000 cycles after error reverses | 5 | 1 |
| C. Latency | 4 / 5 / 5 cycles (P / I / D) | Unchanged |
| D. All gains near maximum, error +30000 | not tested | Output pins at +32767, never wraps |

The fixes: 32-bit product wires for each multiply, saturation at every stage,
an 18-bit sum, the integrator clamp with conditional integration, and an
integrator that accumulates the full product so small errors are not rounded
away.

`tb/pid_controller_tb.v` closes the loop around a first-order plant model
(`src/plant.v`) with noise injection. With its gains (Kp = 0.5, Ki = 0.1,
Kd = 0.25) the loop converges on the target but slowly: with each target held
for 10,000 cycles the measured value is still 4 to 6 counts off (about 122,
188 and 70 for targets of 128, 192 and 64). Held for 100,000 cycles it
averages 128.0, 192.0 and 64.0, with about ±4 counts of ripple from the
injected noise. The testbench as written holds each target for 1,000 cycles,
which is too short to see this.

## Synthesis

Yosys generic synthesis: 6509 cells, 162 flip-flops (`docs/yosys_report.txt`).
Most of the cells are three 16x16 multipliers expanded into gates; on an FPGA
these would map to DSP blocks.

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
