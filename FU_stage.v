`include "define.vh" 

module FU_STAGE(
  input wire clk,
  input wire reset,
  input wire [`from_DE_to_FU_WIDTH-1:0] from_DE_to_FU,   
  output wire [`from_FU_to_DE_WIDTH-1:0] from_FU_to_DE
);

  // Decode inputs from DE stage
  wire [`ALUDATABITS-1:0] operand_a;
  wire [`ALUDATABITS-1:0] operand_b;
  wire [`ALUOPBITS-1:0] operation;
  wire [`ALUCSRINBITS-1:0] control_signals_in;
  
  assign {operand_a, operand_b, operation, control_signals_in} = from_DE_to_FU;
  
  // Outputs from ALU
  wire [`ALUDATABITS-1:0] result;
  wire [`ALUCSROUTBITS-1:0] status_signals;
  
  assign from_FU_to_DE = {result, status_signals};
  
  // External ALU instantiation
  external_alu alu_instance (
    .clk(clk),
    .rst(reset),
    .OP1(operand_a),
    .OP2(operand_b),
    .OP3(result),
    .ALUOP(operation),
    .CSR_ALU_OUT(status_signals),
    .CSR_ALU_IN(control_signals_in)
  );

endmodule