module divider (
    input clk,
    input start,
    input [31:0] a, b,
    output reg [31:0] hi, lo,
    output reg ready,
    output reg div_zero
);

reg [31:0] dividend, divisor;
reg [63:0] remainder;
reg [5:0] count;
reg active;

always @(posedge clk) begin
    if (start && !active) begin
        if (b == 0) begin
            div_zero <= 1'b1;
            ready <= 1'b1;
            active <= 1'b0;
        end else begin
            dividend <= a;
            divisor <= b;
            remainder <= {32'b0, a};
            count <= 6'b000000;
            active <= 1'b1;
            ready <= 1'b0;
            div_zero <= 1'b0;
        end
    end else if (active) begin
        if (count < 32) begin
            // Shift remainder left
            remainder = remainder << 1;
            // Subtract divisor
            remainder[63:32] = remainder[63:32] - divisor;
            if (remainder[63]) begin
                // Restore
                remainder[63:32] = remainder[63:32] + divisor;
                remainder[0] = 1'b0;
            end else begin
                remainder[0] = 1'b1;
            end
            count <= count + 1;
        end else begin
            active <= 1'b0;
            ready <= 1'b1;
            lo <= remainder[31:0]; // quotient
            hi <= remainder[63:32]; // remainder
        end
    end else begin
        ready <= 1'b0;
        div_zero <= 1'b0;
    end
end

endmodule
