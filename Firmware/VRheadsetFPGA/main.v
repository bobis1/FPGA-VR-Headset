// This Verilog file define the Main module of this blinky example.

module main #(
    // Get the default blink divider from a macro defined in apio.ini.
    // In testing, the testbench override it with a very low value.
    parameter integer N = pN
		//parameter double X = `X
		//parameter double Y = `Y
) (
    input  CLK,   // 12MHz clock
		input X,
		input Y
);

  reg [31:0] total = 0;

  reg led = 0;

  assign LED1 = led;
  assign LED2 = !led;

  always @(posedge CLK) begin
   /* if (counter >= N - 1) begin
      counter <= 0;
      led <= !led;
    end else begin
      counter <= counter + 1;
    end*/
    assign SUM = X + Y
    
    
  end

endmodule


