module multiplier (
    input clk,
    input start,
    input [31:0] a, b,
    output reg [31:0] hi, lo,
    output reg ready
);

reg [31:0] multiplicand, multiplier;
reg [63:0] product;
reg [5:0] count; // up to 32 cycles
reg active;

always @(posedge clk) begin
    if (start && !active) begin
        multiplicand <= a;
        multiplier <= b;
        product <= 64'b0;
        count <= 6'b000000;
        active <= 1'b1;
        ready <= 1'b0;
    end else if (active) begin
        // Booth's algorithm implementation
        if (count < 32) begin
            // Simplified Booth: check LSB and next bit
            case ({multiplier[1:0]})
                2'b01: product = product + {multiplicand, 32'b0};
                2'b10: product = product - {multiplicand, 32'b0};
                default: ; // 00 or 11: do nothing
            endcase
            // Arithmetic shift right
            {product, multiplier} = {product[63], product, multiplier} >>> 1;
            count <= count + 1;
        end else begin
            active <= 1'b0;
            ready <= 1'b1;
            hi <= product[63:32];
            lo <= product[31:0];
        end
    end else begin
        ready <= 1'b0;
    end
end

endmodule
