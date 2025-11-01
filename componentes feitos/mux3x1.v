module mux3x1 (
    input [31:0] in0,in1,in2,
    input sel,
    output [31:0] out
);
    assign out = (sel == 1'b0) ? in0:
                (sel == 1'b1) ? in1:
                in0;


endmodule
