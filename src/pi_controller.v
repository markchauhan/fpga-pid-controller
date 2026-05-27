module pi_controller (
    input wire          clk, 
    input wire          rst_n,   
    input wire [15:0]   measured, 
    input wire [15:0]   target, 
    output reg [15:0]   control_out
);

parameter signed [15:0] Kp = 16'sh0080;
parameter signed [15:0] Ki = 16'sh001A;
parameter signed [15:0] Kd = 16'sh0040;

reg signed [15:0] error;
reg signed [15:0] previous_error; 
reg signed [15:0] error_change; 
reg signed [15:0] derivative_term;
reg signed [15:0] proportional_term;
reg signed [31:0] integrator;
reg signed [15:0] integral_term;
reg signed [16:0] pi_sum;


localparam signed [15:0] OUT_MAX = 16'sh7FFF;
localparam signed [15:0] OUT_MIN = 16'sh8000;

//Compute error every clock cycle 
always @(posedge clk or negedge rst_n) begin
  if (!rst_n) error<= 0; 
  else  error<= $signed(target) - $signed(measured);
end

//Proportional calculation
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) proportional_term <= 0;
    else proportional_term <= ($signed(Kp) * $signed(error)) >>> 8;
end

//Integral Calculation
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        integrator <= 0; 
        integral_term <= 0; 
    end else begin
        integrator <= integrator + (($signed(Ki) * $signed(error)) >>> 8);
        integral_term <= integrator[23:8];
    end
end

//Derivative Calculation
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        previous_error  <= 0;
        error_change    <= 0;
        derivative_term <= 0;
    end else begin
        previous_error  <= error;
        error_change    <= error - previous_error;
        derivative_term <= (derivative_term - (derivative_term >>> 2))
                         + (($signed(Kd) * error_change) >>> 10);
    end
end

// sum P + I 
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) pi_sum <= 0; 
    else pi_sum <= {proportional_term[15], proportional_term}
                + {integral_term[15], integral_term}; 
end

// Filter output if too high or low
always @(posedge clk or negedge rst_n) begin
    if      (!rst_n)         control_out <= 0;
    else if (pi_sum > $signed({1'b0, OUT_MAX})) control_out <= OUT_MAX;
    else if (pi_sum < $signed({1'b1, OUT_MIN})) control_out <= OUT_MIN;
    else                     control_out <= pi_sum[15:0];
end

endmodule