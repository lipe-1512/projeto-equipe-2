module portaOR (
    input in0, in1,
    output out
);

    assign out = in0 || in1;
    
endmodule