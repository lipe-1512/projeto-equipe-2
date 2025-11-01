module divider (
    input clk,
    input start,
    input [31:0] a, b,
    output reg [31:0] hi, lo,
    output reg ready,
    output reg div_zero
);

always @(posedge clk) begin
    if (start) begin
        if (b == 0) begin
            div_zero = 1'b1;
            ready = 1'b1;
        end else begin
            lo = a / b;
            hi = a % b;
            div_zero = 1'b0;
            ready = 1'b1;
        end
    end else begin
        ready = 1'b0;
        div_zero = 1'b0;
    end
end

endmodule
