module mux6x1 #(parameter WIDTH = 32) (
    input [WIDTH-1:0] in0, in1, in2, in3, in4, in5,
    input [2:0] sel,
    output [WIDTH-1:0] out
);
    assign out = (sel == 3'b000) ? in0 :
                 (sel == 3'b001) ? in1 :
                 (sel == 3'b010) ? in2 :
                 (sel == 3'b011) ? in3 :
                 (sel == 3'b100) ? in4 :
                 (sel == 3'b101) ? in5 :
                 in0;

endmodule
