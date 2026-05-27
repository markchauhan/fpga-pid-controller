`timescale 1ns/1ps

module pi_controller_tb; 

reg         clk, rst_n; 
wire [15:0] measured;
reg  [15:0] target;
wire [15:0] control_out; 
reg [15:0] kp, ki, kd;

pi_controller dut (
    .clk(clk),
    .rst_n(rst_n),
    .measured(measured),
    .target(target),
    .kp(kp),
    .ki(ki),
    .kd(kd),
    .control_out(control_out)
);

plant plant_model (
    .clk(clk),
    .rst_n(rst_n),
    .control_in(control_out),
    .measured(measured)
);

// 100 MHz clock
initial clk = 0; 
always #5 clk = ~clk; 

initial begin 
    $dumpfile("waveform.vcd");
    $dumpvars(0, pi_controller_tb);
end

initial begin
    // 1. Reset and set gains
    rst_n  = 0;
    target = 16'h0080;   // target = 0.5 in Q8.8
    kp     = 16'h0080;
    ki     = 16'h001A;
    kd     = 16'h0040;
    #20;

    // 2. Release reset 
    rst_n = 1;
    #10000;              // run longer so we can see the plant converge

    // 3. Step the target up and plant will follow
    target = 16'h00C0;   // new target = 0.75
    #10000;

    // 4. Step target back down
    target = 16'h0040;   // new target = 0.25
    #10000;

    $finish;
end

endmodule 