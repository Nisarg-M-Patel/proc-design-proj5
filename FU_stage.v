`include "define.vh" 

module FU_STAGE(
  input wire                              clk,
  input wire                              reset,
  input wire [`from_DE_to_FU_WIDTH-1:0]    from_DE_to_FU,   
  output wire [`from_FU_to_DE_WIDTH-1:0]   from_FU_to_DE
);
  /////////////////////////////////////////////////////////////////
  //TODO: add your code here to instantiate the external_alu module

  wire [`ALUOPBITS - 1: 0]    alu_operation_code;      
  wire [`ALUDATABITS - 1: 0]  alu_operand_a;           
  wire [`ALUDATABITS - 1: 0]  alu_operand_b;          
  
  wire [`ALUDATABITS - 1: 0]  alu_computation_result;  
  
  wire [`ALUCSROUTBITS - 1: 0] alu_status_output;      
  wire [`ALUCSRINBITS - 1: 0]  alu_control_input;      
  
  assign {
    alu_operand_a,
    alu_operand_b,
    alu_operation_code,
    alu_control_input
  } = from_DE_to_FU;
  
  assign from_FU_to_DE = {
    alu_computation_result,
    alu_status_output
  };
  
  external_alu external_alu_instance (
    .clk(clk),
    .rst(reset),
    .OP1(alu_operand_a),
    .OP2(alu_operand_b),
    .OP3(alu_computation_result),
    .ALUOP(alu_operation_code),
    .CSR_ALU_OUT(alu_status_output),
    .CSR_ALU_IN(alu_control_input)
  );


endmodule