`include "define.vh" 

module FU_STAGE(
  input wire                              clk,
  input wire                              reset,
  input wire [`from_DE_to_FU_WIDTH-1:0]    from_DE_to_FU,   
  output wire [`from_FU_to_DE_WIDTH-1:0]   from_FU_to_DE
);
  /////////////////////////////////////////////////////////////////
  //TODO: add your code here to instantiate the external_alu module

  wire [`ALUOPBITS - 1: 0] ext_alu_op;
  wire [`ALUDATABITS - 1: 0] ext_alu_op1;
  wire [`ALUDATABITS - 1: 0] ext_alu_op2;
  wire [`ALUDATABITS - 1: 0] ext_alu_result;
  wire [`ALUCSROUTBITS - 1: 0] csr_alu_out;
  wire [`ALUCSRINBITS - 1: 0] csr_alu_in;

  assign {
    ext_alu_op1,
    ext_alu_op2,
    ext_alu_op,
    csr_alu_in
  } = from_DE_to_FU;

  assign from_FU_to_DE = {
    ext_alu_result,
    csr_alu_out
  };

  external_alu alu (
    .clk(clk),
    .rst(reset),
    .OP1(ext_alu_op1),
    .OP2(ext_alu_op2),
    .OP3(ext_alu_result),
    .ALUOP(ext_alu_op),
    .CSR_ALU_OUT(csr_alu_out),
    .CSR_ALU_IN(csr_alu_in)
  );

endmodule