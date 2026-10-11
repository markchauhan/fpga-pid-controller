module pid_controller (
    input wire          clk, 
    input wire          rst_n,   
    input wire [15:0]   measured, 
    input wire [15:0]   target,
    input wire [15:0]   kp,
    input wire [15:0]   ki,
    input wire [15:0]   kd,
    output reg [15:0]   control_out
);

reg signed [15:0] error;
reg signed [15:0] previous_error; 
reg signed [15:0] error_change; 
reg signed [15:0] derivative_term;
reg signed [15:0] proportional_term;
reg signed [31:0] integrator;
reg signed [15:0] integral_term;
reg signed [17:0] pid_sum;

reg signed [31:0] p_product_r; 
reg signed [31:0] i_product_r; 
reg signed [31:0] d_product_r; 


localparam signed [15:0] OUT_MAX = 16'sh7FFF;
localparam signed [15:0] OUT_MIN = 16'sh8000;

//Compute error every clock cycle 
wire signed [16:0] err_diff = $signed(target) - $signed(measured);

always @(posedge clk or negedge rst_n) begin
  if (!rst_n)                      error <= 0; 
  else if (err_diff >  17'sd32767) error <= OUT_MAX;
  else if (err_diff < -17'sd32768) error <= OUT_MIN;
  else                             error <= err_diff[15:0];
end

//Multiply stage: one register after each multiplier
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin 
        p_product_r <= 0;
        i_product_r <= 0;
        d_product_r <= 0;
    end else begin
        p_product_r <= $signed(kp) * $signed(error);
        i_product_r <= $signed(ki) * $signed(error);
        d_product_r <= $signed(kd) * error_change;
    end
end

//Proportional calculation
wire signed [31:0] p_product = $signed(kp) * $signed(error);
wire signed [31:0] p_scaled = p_product_r >>> 8; 

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)                        proportional_term <= 0;
    else if (p_scaled >  32'sd32767)   proportional_term <= OUT_MAX;
    else if (p_scaled < -32'sd32768)   proportional_term <= OUT_MIN;
    else                               proportional_term <= p_scaled[15:0];
end

//Integral Calculation
localparam signed [31:0] INT_MAX = 32'sh7FFFFFFF;
localparam signed [31:0] INT_MIN = 32'sh80000000;

wire signed [31:0] i_product = $signed(ki) * $signed(error);
wire signed [32:0] int_next = integrator + i_product_r;

wire sat_hi = (control_out == OUT_MAX) && (i_product_r > 0);
wire sat_lo = (control_out == OUT_MIN) && (i_product_r < 0);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        integrator <= 0; 
        integral_term <= 0; 
    end else begin
        if (sat_hi || sat_lo)          integrator <= integrator;
        else if (int_next > INT_MAX)   integrator <= INT_MAX;
        else if (int_next < INT_MIN)   integrator <= INT_MIN;
        else                           integrator <= int_next[31:0];
        integral_term <= integrator[31:16];
    end
end

//Derivative Calculation
wire signed [16:0] chg_diff  = error - previous_error;
wire signed [31:0] d_product = $signed(kd) * error_change;
wire signed [31:0] d_scaled  = d_product_r >>> 10;
wire signed [31:0] d_next    = (derivative_term - (derivative_term >>> 2)) + d_scaled;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        previous_error  <= 0;
        error_change    <= 0;
        derivative_term <= 0;
    end else begin
        previous_error  <= error;

        if (chg_diff >  17'sd32767)      error_change <= OUT_MAX;
        else if (chg_diff < -17'sd32768) error_change <= OUT_MIN;
        else                             error_change <= chg_diff[15:0];

        if (d_next >  32'sd32767)        derivative_term <= OUT_MAX;
        else if (d_next < -32'sd32768)   derivative_term <= OUT_MIN;
        else                             derivative_term <= d_next[15:0];
    end
end

// sum P + I + D
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) pid_sum <= 0; 
    else pid_sum <= {{2{proportional_term[15]}}, proportional_term}
                  + {{2{integral_term[15]}},     integral_term}
                  + {{2{derivative_term[15]}},   derivative_term}; 
end

// Saturate the output
always @(posedge clk or negedge rst_n) begin
    if      (!rst_n)                 control_out <= 0;
    else if (pid_sum >  18'sd32767)  control_out <= OUT_MAX;
    else if (pid_sum < -18'sd32768)  control_out <= OUT_MIN;
    else                             control_out <= pid_sum[15:0];
end

endmodule