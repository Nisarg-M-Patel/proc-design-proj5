`include "define.vh" 

module FU_STAGE(
  input wire clk,
  input wire reset,
  input wire [`from_DE_to_FU_WIDTH-1:0] from_DE_to_FU,   
  output wire [`from_FU_to_DE_WIDTH-1:0] from_FU_to_DE
);

  // Unpack inputs from DE
  wire wr_aluop;
  wire wr_op1;
  wire wr_op2;
  wire [`DBITS-1:0] wr_data;
  wire rd_op3;
  
  assign wr_aluop = from_DE_to_FU[0];
  assign wr_op1 = from_DE_to_FU[1];
  assign wr_op2 = from_DE_to_FU[2];
  assign wr_data = from_DE_to_FU[34:3];
  assign rd_op3 = from_DE_to_FU[35];

  // ALU register storage
  reg [`ALUOPBITS-1:0] ALUOP_reg;
  reg [`ALUDATABITS-1:0] OP1_reg;
  reg [`ALUDATABITS-1:0] OP2_reg;
  
  // Track if operands have been written
  reg op1_written;
  reg op2_written;
  
  // ALU interface signals
  wire [`ALUDATABITS-1:0] OP3;
  wire [`ALUCSROUTBITS-1:0] CSR_ALU_OUT;
  reg [`ALUCSRINBITS-1:0] CSR_ALU_IN;
  
  // State machine
  reg [2:0] state;
  localparam IDLE          = 3'd0;
  localparam WAIT_OP1      = 3'd1;
  localparam LOAD_OP1      = 3'd2;
  localparam WAIT_OP2      = 3'd3;
  localparam LOAD_OP2      = 3'd4;
  localparam COMPUTING     = 3'd5;
  localparam RESULT_READY  = 3'd6;
  
  // Debug output
  always @(posedge clk) begin
    if (!reset && (wr_aluop || wr_op1 || wr_op2 || rd_op3 || state != IDLE)) begin
      $display("[%0t] FU: state=%0d, CSR_IN=%b, CSR_OUT=%b, ALUOP=%h, OP1=%h, OP2=%h, OP3=%h, wr_aluop=%b, wr_op1=%b, wr_op2=%b, rd_op3=%b, op1_wr=%b, op2_wr=%b", 
               $time, state, CSR_ALU_IN, CSR_ALU_OUT, ALUOP_reg, OP1_reg, OP2_reg, OP3, 
               wr_aluop, wr_op1, wr_op2, rd_op3, op1_written, op2_written);
    end
  end
  
  always @(posedge clk) begin
    if (reset) begin
      state <= IDLE;
      CSR_ALU_IN <= 3'b000;
      ALUOP_reg <= 4'b0;
      OP1_reg <= 32'b0;
      OP2_reg <= 32'b0;
      op1_written <= 1'b0;
      op2_written <= 1'b0;
    end else begin
      // Capture ALUOP anytime it's written
      if (wr_aluop) begin
        ALUOP_reg <= wr_data[`ALUOPBITS-1:0];
      end
      
      // Capture OP1 anytime it's written
      if (wr_op1) begin
        OP1_reg <= wr_data;
        op1_written <= 1'b1;
      end
      
      // Capture OP2 anytime it's written
      if (wr_op2) begin
        OP2_reg <= wr_data;
        op2_written <= 1'b1;
      end
      
      case (state)
        IDLE: begin
          CSR_ALU_IN <= 3'b000;
          op1_written <= 1'b0;
          op2_written <= 1'b0;
          
          // Start when OP1 has been written
          if (op1_written) begin
            state <= WAIT_OP1;
          end
        end
        
        WAIT_OP1: begin
          if (CSR_ALU_OUT[0]) begin  // OP1 port ready
            CSR_ALU_IN[1] <= 1'b1;   // Signal OP1 is stable
            state <= LOAD_OP1;
          end
        end
        
        LOAD_OP1: begin
          CSR_ALU_IN[1] <= 1'b0;
          state <= WAIT_OP2;
        end
        
        WAIT_OP2: begin
          // Wait for BOTH OP2 to be written AND ALU to be ready
          if (op2_written && CSR_ALU_OUT[1]) begin
            CSR_ALU_IN[2] <= 1'b1;   // Signal OP2 is stable
            state <= LOAD_OP2;
          end
        end
        
        LOAD_OP2: begin
          CSR_ALU_IN[2] <= 1'b0;
          state <= COMPUTING;
        end
        
        COMPUTING: begin
          // Keep checking for result
          if (CSR_ALU_OUT[2]) begin  // Result valid
            CSR_ALU_IN[0] <= 1'b1;   // Protect result
            state <= RESULT_READY;
          end
        end
        
        RESULT_READY: begin
          if (rd_op3) begin
            CSR_ALU_IN[0] <= 1'b0;
            state <= IDLE;
          end
        end
        
        default: begin
          state <= IDLE;
          CSR_ALU_IN <= 3'b000;
        end
      endcase
    end
  end
  
  // Pack outputs to DE
  assign from_FU_to_DE = {
    CSR_ALU_OUT,    // bits [34:32]
    OP3             // bits [31:0]
  };
  
  // Instantiate external ALU
  external_alu ext_alu (
    .clk(clk),
    .rst(reset),
    .OP1(OP1_reg),
    .OP2(OP2_reg),
    .OP3(OP3),
    .ALUOP(ALUOP_reg),
    .CSR_ALU_OUT(CSR_ALU_OUT),
    .CSR_ALU_IN(CSR_ALU_IN)
  );

endmodule