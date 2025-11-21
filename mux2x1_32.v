module mux2x1_32 (
    input [31:0] in0, in1,
    input sel,
    output [31:0] out
);

    assign out = (sel == 1'b1) ? in1 : 
                (sel == 1'b0) ? in0 :
                32'b0;
    
endmodule
