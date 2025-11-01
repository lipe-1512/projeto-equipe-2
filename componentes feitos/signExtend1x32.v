module signExtend1x32 (
    input in,
    output [31:0] out
);

    assign out = { {32{in}}};
    
endmodule