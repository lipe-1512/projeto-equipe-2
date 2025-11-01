module mux2x1 (
    input [4:0] in0, in1,
    input sel,
    output [4:0] out
);

    assign out = (sel == 1'b0) ? in0 : in1;

endmodule
