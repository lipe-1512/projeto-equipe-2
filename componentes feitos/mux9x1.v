module mux9x1 (
    input [4:0] in0, in1, in2, in3, in4, in5, in6, in7, in8,
    input [3:0] sel,
    output [4:0] out
);

    assign out = (sel == 4'b0000) ? in0 :
                 (sel == 4'b0001) ? in1 :
                 (sel == 4'b0010) ? in2 :
                 (sel == 4'b0011) ? in3 :
                 (sel == 4'b0100) ? in4 :
                 (sel == 4'b0101) ? in5 :
                 (sel == 4'b0110) ? in6 :
                 (sel == 4'b0111) ? in7 :
                 in8;
    
endmodule