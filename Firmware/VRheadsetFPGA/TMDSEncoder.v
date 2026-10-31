module TMDSEncoder #(

)(
    input               CLK,
    input               disEna,
    input       [1:0]   control,
    input       [7:0]   d_in,
    output reg  [9:0]   q_out
);



always @ (*)begin
    //4 bits to represent d_in
    assign ones = 0;
    for (int i=0; i<7; ++i) begin
        if (d_in(i) = '1) begin
            assign ones = ones + 1;
            pass
        end
    end
    assign d_in_ones = ones;
end






endmodule