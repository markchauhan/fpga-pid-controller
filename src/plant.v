module plant (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] control_in,   // from PID output
    output reg  [15:0] measured      // simulated sensor reading
);

// First-order system: measured[n+1] = measured[n] + (control_in - measured[n]) / 8
// This simulates a real physical plant like a motor or laser with lag
reg signed [15:0] state;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state    <= 16'sd0;
        measured <= 16'h0000;
    end else begin
        // Move state toward control_in with inertia
        state    <= state + (($signed(control_in) - state) >>> 3);
        measured <= state;
    end
end

endmodule