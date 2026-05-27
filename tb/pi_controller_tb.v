`timescale 1ns/1ps

module pi_controller_tb; 

reg         clk, rst_n; 
reg [15:0]  measured, target; 
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



// 100 MHz clock
initial clk = 0; 
always #5 clk = ~clk; 

initial begin 
    $dumpfile("waveform.vcd");
    $dumpvars(0, pi_controller_tb);

    // 1. Hold in reset
    rst_n   = 0;
    measured = 16'h0000;
    target   = 16'h0080;  // target = 0.5 in Q8.8
    kp = 16'h0080;   
    ki = 16'h001A;  
    kd = 16'h0040;  
    #20;

    // 2. Release reset 
    rst_n = 1;
    #2000;

    // 3. Sensor slowly moves toward target
    measured = 16'h0040;  // 0.25
    #2000;
    measured = 16'h0070;  // 0.44
    #2000;
    measured = 16'h0080;  // 0.50 at the target 
    #2000;

    // 4. Disturbance push above target
    measured = 16'h0090;  // 0.56 over target, error goes negative
    #2000;

    // 5. Recover
    measured = 16'h0080;
    #2000;

    $finish;
end
endmodule 