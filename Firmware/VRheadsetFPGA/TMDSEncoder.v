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
            
        end
    end
    assign d_in_ones = ones;
end

always @ (*) begin
    if(ones_d_in > 4 || (ones_d_in == 4 && d_in == '0')) begin
        q_m[0] = d_in[0];
        q_m[1] = q_m[0] ~^ d_in[1];
        q_m[2] = q_m[1] ~^ d_in[2];
        q_m[3] = q_m[2] ~^ d_in[3];
        q_m[4] = q_m[3] ~^ d_in[4];
        q_m[5] = q_m[4] ~^ d_in[5];
        q_m[6] = q_m[5] ~^ d_in[6];
        q_m[7] = q_m[6] ~^ d_in[7];
        q_m <= '0';
    end else begin
        q_m[0] = d_in[0];
        q_m[1] = q_m[0] ^ d_in[1];
        q_m[2] = q_m[1] ^ d_in[2];
        q_m[3] = q_m[2] ^ d_in[3];
        q_m[4] = q_m[3] ^ d_in[4];
        q_m[5] = q_m[4] ^ d_in[5];
        q_m[6] = q_m[5] ^ d_in[6];
        q_m[7] = q_m[6] ^ d_in[7];
        q_m = '1';
    end
end

always @ (*)begin
    //4 bits to represent q_m
    assign ones = 0;
    for (int i=0; i<7; ++i) begin
        if (q_m(i) = '1) begin
            assign ones = ones + 1;
            
        end
    end
    assign q_m_ones = ones;
end



/*
  PROCESS(d_in, q_m, ones_d_in)
  BEGIN
    IF(ones_d_in > 4 OR (ones_d_in = 4 AND d_in(0) = '0')) THEN
      q_m(0) <= d_in(0);
      q_m(1) <= q_m(0) XNOR d_in(1);
      q_m(2) <= q_m(1) XNOR d_in(2);
      q_m(3) <= q_m(2) XNOR d_in(3);
      q_m(4) <= q_m(3) XNOR d_in(4);
      q_m(5) <= q_m(4) XNOR d_in(5);
      q_m(6) <= q_m(5) XNOR d_in(6);
      q_m(7) <= q_m(6) XNOR d_in(7);
      q_m(8) <= '0';
    ELSE
      q_m(0) <= d_in(0);
      q_m(1) <= q_m(0) XOR d_in(1);
      q_m(2) <= q_m(1) XOR d_in(2);
      q_m(3) <= q_m(2) XOR d_in(3);
      q_m(4) <= q_m(3) XOR d_in(4);
      q_m(5) <= q_m(4) XOR d_in(5);
      q_m(6) <= q_m(5) XOR d_in(6);
      q_m(7) <= q_m(6) XOR d_in(7);
      q_m(8) <= '1';
    END IF;
  END PROCESS;
  */




endmodule