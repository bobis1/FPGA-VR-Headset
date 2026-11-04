// This Verilog file define the Main module of this blinky example.

module main #(
    // Get the default blink divider from a macro defined in apio.ini.
    // In testing, the testbench override it with a very low value.
    parameter integer N = pN
    
    parameter integer initialRAMCountEnd = 12375
 //Since we have a 3 frame buffer, 400x400 each with 12 bit color it will cycle through 0-2.

		//parameter double X = `X
		//parameter double Y = `Y
) (
    input  CLK,   // 12MHz clock
		input X,
		input Y,
    input TMDSD0S,
    input TMDSD1S,
    input TMDSD2S,
    output CE0,
    output CE1,
    output CE2,
    output RAMCLK,
    inout Si00,
    inout SI01,
    inout SI02,
    inout SI03
);

  wire FCLK;
  reg isInitialized = 0;
  reg currentFrame = 0; 
  reg initialRAMCount = 0;

  SB_PLL40_CORE #(
    .FEEDBACK_PATH("SIMPLE"),
    .DIVR(4'b0000),         
    .DIVF(7'b0110111),    
    .DIVQ(3'b011),          
    .FILTER_RANGE(3'b001)
) my_pll (
    .REFERENCECLK(CLK),     
    .PLLOUTCORE(FCLK),  
    .LOCK(),                
    .RESETB(1'b1),          
    .BYPASS(1'b0)           
);

  assign RAMCLK = FCLK;

  always @(posedge FCLK) begin
   if (initialRAMCount <= initialRAMCountEnd) begin
       assign initialRAMCount = initialRAMCount + 1;
   end
    if (initialRAMCount >= initialRAMCountEnd) begin
      assign isInitialized = 1;
      if (currentFrame == 0) begin
        assign CE0 = 0'b1;
        assign CE1 = 1'b1;
        assign CE2 = 1'b1;
      end
      if (currentFrame == 1) begin
        assign CE1 = 0'b1;
        assign CE0 = 1'b1;
        assign CE2 = 1'b1;
      end
      if (currentFrame == 2) begin
        assign CE2 = 0'b1;
        assign CE0 = 1'b1;
        assign CE1 = 1'b1;
      end
    end
  end


endmodule


