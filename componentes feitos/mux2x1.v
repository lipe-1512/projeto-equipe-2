module mux2x1 #(parameter WIDTH = 32) (
    input [WIDTH-1:0] in0, in1,
    input sel,
    output [WIDTH-1:0] out
);

    assign out = (sel == 1'b0) ? in0 : in1;

endmodule
