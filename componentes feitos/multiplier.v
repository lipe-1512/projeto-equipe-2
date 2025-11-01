module multiplier (
    input clk,
    input start,
    input [31:0] a, b,
    output reg [31:0] hi, lo,
    output reg ready
);

always @(posedge clk) begin
    if (start) begin
        {hi, lo} = a * b;
        ready = 1'b1;
    end else begin
        ready = 1'b0;
    end
end

endmodule
