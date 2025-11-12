`include "define.vh" 

module FU_STAGE(
  input wire                              clk,
  input wire                              reset,
  input wire [`from_DE_to_FU_WIDTH-1:0]    from_DE_to_FU,
  output wire [`from_FU_to_DE_WIDTH-1:0]   from_FU_to_DE
);
  /////////////////////////////////////////////////////////////////
  // Unpack signals from DE stage
  wire [`ALUDATABITS-1:0] OP1;
  wire [`ALUDATABITS-1:0] OP2;
  wire [`ALUOPBITS-1:0] ALUOP;
  wire [`ALUCSRINBITS-1:0] CSR_ALU_IN;

  assign {CSR_ALU_IN, ALUOP, OP2, OP1} = from_DE_to_FU;

  // Signals from ALU
  wire [`ALUDATABITS-1:0] OP3;
  wire [`ALUCSROUTBITS-1:0] CSR_ALU_OUT;

  // Pack signals to DE stage
  assign from_FU_to_DE = {CSR_ALU_OUT, OP3};

  // Instantiate external ALU
  external_alu ext_alu (
    .clk(clk),
    .rst(reset),
    .OP1(OP1),
    .OP2(OP2),
    .OP3(OP3),
    .ALUOP(ALUOP),
    .CSR_ALU_OUT(CSR_ALU_OUT),
    .CSR_ALU_IN(CSR_ALU_IN)
  );

endmodule