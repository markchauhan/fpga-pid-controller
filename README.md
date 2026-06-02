# FPGA PID Controller

Real-time PID feedback control loop in Verilog. 60ns latency at 100MHz. Built for laser stabilization, motor drives, and robotic actuators.

## Specs

| Parameter | Value |
|---|---|
| Clock | 100 MHz |
| Loop latency | 6 cycles / 60ns |
| Word width | 16-bit Q8.8 fixed-point |
| Gains | Runtime-tunable inputs |
| Anti-windup | Yes |
| Derivative filter | Exponential moving average |
| Plant model | First-order inertia simulation |

## What makes this non-trivial

- Full PID with filtered derivative — raw D amplifies noise, the filter keeps it stable
- Anti-windup integrator — prevents runaway accumulation at output saturation
- Runtime-tunable gains — Kp, Ki, Kd are module inputs, not hardcoded parameters
- Closed-loop simulation — plant model with inertia closes the feedback loop in sim
- Noise injection — random perturbation validates filter performance under disturbance
- Synthesized with Yosys: 590 cells, 97 flip-flops, 0 problems

## File Structure

- src/pi_controller.v: PID controller, 6-stage pipeline
- src/plant.v: First-order plant model with inertia
- tb/pi_controller_tb.v: Closed-loop testbench with noise injection
- docs/writeup.md: Full technical write-up
- docs/yosys_report.txt: Synthesis report

## Reproduce

Install on Mac:
brew install icarus-verilog surfer yosys

Simulate:
iverilog -o sim.out src/pi_controller.v src/plant.v tb/pi_controller_tb.v
vvp sim.out
surfer waveform.vcd

Synthesize:
yosys -p "read_verilog src/pi_controller.v; synth -top pi_controller; stat"
