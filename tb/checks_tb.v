`timescale 1ns/1ps

// Directed checks against pid_controller (open loop, no plant):
//   A. proportional overflow
//   B. integrator windup
//   C. pipeline latency per term
//   D. max gains with a large positive error
// Each test prints its measurements, then a PASS/FAIL line.
// Inputs change on the falling edge, so every rising edge after a change
// counts as one full clock edge of latency.
module checks_tb;

reg         clk, rst_n;
reg  [15:0] measured, target;
reg  [15:0] kp, ki, kd;
wire [15:0] control_out;

pid_controller dut (
    .clk(clk),
    .rst_n(rst_n),
    .measured(measured),
    .target(target),
    .kp(kp),
    .ki(ki),
    .kd(kd),
    .control_out(control_out)
);

// 100 MHz clock
initial clk = 0;
always #5 clk = ~clk;

wire signed [15:0] out_s = control_out;

integer i;
integer edges;
integer first_neg_out;        // cycle control_out first went negative
integer first_neg_iterm;      // cycle integral_term first went negative
integer out_sign_flips;
integer integ_wrapped;        // 32-bit integrator changed sign while error held
integer cycles_out_neg;
integer cycles_integ_neg;
integer min_out, max_out;     // control_out extremes during test D
integer failures;
reg                prev_out_sign;
reg signed [31:0]  integ_at_switch;
reg signed [15:0]  iterm_at_switch;
reg signed [15:0]  out_at_switch;
reg        [15:0]  out_before;

// Hold reset with all inputs at zero, then release on a falling edge.
task reset_dut;
    begin
        rst_n    = 0;
        target   = 0;
        measured = 0;
        kp = 0; ki = 0; kd = 0;
        repeat (3) @(negedge clk);
        rst_n = 1;
        repeat (3) @(negedge clk);
    end
endtask

// Check a condition and display PASS/FAIL with a label.
task check;
    input cond;
    input [8*40-1:0] label;
    begin
        if (cond) $display("PASS  %0s", label);
        else begin
            $display("FAIL  %0s", label);
            failures = failures + 1;
        end
    end
endtask

// Step target 0 -> step and count rising edges until control_out changes.
task latency;
    input [15:0] p, i_gain, d;
    input [15:0] step;
    input [8*8-1:0] name;
    begin
        reset_dut;
        kp = p; ki = i_gain; kd = d;
        repeat (4) @(negedge clk);
        out_before = control_out;
        target = step;
        edges = 0;
        while (control_out === out_before && edges < 1000) begin
            @(posedge clk);
            #1;
            edges = edges + 1;
        end
        if (control_out === out_before)
            $display("C  %0s step=%0d: no change within 1000 edges", name, step);
        else
            $display("C  %0s step=%0d: control_out changed after %0d clock edges (%0d ns), first value %0d",
                     name, step, edges, edges * 10, out_s);
    end
endtask

initial begin
    failures = 0;

    // ---------------- A. Overflow ----------------
    reset_dut;
    kp = 16'h0100; ki = 0; kd = 0;
    target = 16'd1000; measured = 0;
    repeat (50) @(posedge clk);
    #1;
    $display("A  kp=1.0 target=1000 measured=0: control_out = %0d (0x%04h), proportional_term = %0d, expected 1000",
             out_s, control_out, dut.proportional_term);
    check(out_s == 1000, "A proportional output");

    // ---------------- B. Windup ----------------
    reset_dut;
    kp = 0; ki = 16'h0100; kd = 0;
    target = 16'd2000; measured = 0;      // error = +2000
    first_neg_out   = -1;
    first_neg_iterm = -1;
    out_sign_flips  = 0;
    integ_wrapped   = 0;
    prev_out_sign   = 0;
    for (i = 1; i <= 5000; i = i + 1) begin
        @(posedge clk);
        #1;
        if (dut.integrator < 0) integ_wrapped = 1;
        if (dut.integral_term < 0 && first_neg_iterm < 0) first_neg_iterm = i;
        if (out_s < 0 && first_neg_out < 0) first_neg_out = i;
        if (control_out[15] !== prev_out_sign) out_sign_flips = out_sign_flips + 1;
        prev_out_sign = control_out[15];
    end
    integ_at_switch = dut.integrator;
    iterm_at_switch = dut.integral_term;
    out_at_switch   = out_s;
    $display("B  after 5000 cycles at error=+2000: integrator = %0d (0x%08h), integral_term = %0d, control_out = %0d",
             integ_at_switch, integ_at_switch, iterm_at_switch, out_at_switch);
    $display("B  32-bit integrator wrapped sign during hold: %0s", integ_wrapped ? "yes" : "no");
    $display("B  integral_term first negative at cycle %0d, control_out first negative at cycle %0d, control_out sign flips during hold: %0d",
             first_neg_iterm, first_neg_out, out_sign_flips);
    check(out_at_switch == 32767 && first_neg_out < 0, "B holds at max during windup");

    @(negedge clk);
    target = 0; measured = 16'd2000;      // error = -2000
    cycles_out_neg   = -1;
    cycles_integ_neg = -1;
    out_sign_flips   = 0;
    for (i = 1; i <= 20000; i = i + 1) begin
        @(posedge clk);
        #1;
        if (out_s < 0 && cycles_out_neg < 0) cycles_out_neg = i;
        if (dut.integrator < 0 && cycles_integ_neg < 0) cycles_integ_neg = i;
        if (control_out[15] !== prev_out_sign) begin
            out_sign_flips = out_sign_flips + 1;
            if (out_sign_flips <= 6)
                $display("B  after switch, cycle %0d: control_out sign -> %0s (control_out = %0d, integrator = %0d)",
                         i, control_out[15] ? "negative" : "positive", out_s, dut.integrator);
        end
        prev_out_sign = control_out[15];
    end
    $display("B  cycles after switch until control_out negative: %0d (control_out at switch was %0d)",
             cycles_out_neg, out_at_switch);
    $display("B  cycles after switch until integrator negative: %0d", cycles_integ_neg);
    $display("B  control_out sign flips in 20000 cycles after switch: %0d", out_sign_flips);
    check(out_sign_flips == 1, "B one sign flip after reversal");

    // ---------------- C. Latency ----------------
    latency(16'h0100, 16'h0000, 16'h0000, 16'd100,  "kp-only");
    latency(16'h0000, 16'h0100, 16'h0000, 16'd100,  "ki-only");
    latency(16'h0000, 16'h0000, 16'h0100, 16'd100,  "kd-only");
    latency(16'h0100, 16'h0000, 16'h0000, 16'd1000, "kp-only");
    check(edges == 4, "C kp latency is 4 cycles");
    latency(16'h0000, 16'h0100, 16'h0000, 16'd1000, "ki-only");
    check(edges == 5, "C ki latency is 5 cycles");
    latency(16'h0000, 16'h0000, 16'h0100, 16'd1000, "kd-only");
    check(edges == 5, "C kd latency is 5 cycles");

    // ---------------- D. Max gains, large positive error ----------------
    reset_dut;
    kp = 16'h7F00; ki = 16'h7F00; kd = 16'h7F00;
    target = 16'd30000; measured = 0;     // error = +30000
    first_neg_out = -1;
    min_out       = 0;
    max_out       = 0;
    for (i = 1; i <= 100; i = i + 1) begin
        @(posedge clk);
        #1;
        if (out_s < 0 && first_neg_out < 0) first_neg_out = i;
        if (out_s < min_out) min_out = out_s;
        if (out_s > max_out) max_out = out_s;
    end
    $display("D  kp=ki=kd=0x7F00 error=+30000 held 100 cycles: control_out min = %0d, max = %0d, final = %0d",
             min_out, max_out, out_s);
    $display("D  final terms: proportional_term = %0d, integral_term = %0d, derivative_term = %0d, integrator = %0d",
             dut.proportional_term, dut.integral_term, dut.derivative_term, dut.integrator);
    if (first_neg_out < 0)
        $display("D  control_out ever negative: no");
    else
        $display("D  control_out ever negative: yes, first at cycle %0d", first_neg_out);
    check(first_neg_out < 0 && out_s == 32767, "D saturates without wrapping");

    // ---------------- Summary ----------------
    if (failures == 0) $display("ALL CHECKS PASSED");
    else               $display("%0d CHECK(S) FAILED", failures);

    $finish;
end

endmodule