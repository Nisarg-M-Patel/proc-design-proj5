`include "define.vh" 

module FU_STAGE(
  input wire clk,
  input wire reset,
  input wire [`from_DE_to_FU_WIDTH-1:0] from_DE_to_FU,   
  output wire [`from_FU_to_DE_WIDTH-1:0] from_FU_to_DE
);

  wire wr_aluop, wr_op1, wr_op2;
  wire [`DBITS-1:0] wr_data;
  wire rd_op3;
  
  assign wr_aluop = from_DE_to_FU[0];
  assign wr_op1 = from_DE_to_FU[1];
  assign wr_op2 = from_DE_to_FU[2];
  assign wr_data = from_DE_to_FU[34:3];
  assign rd_op3 = from_DE_to_FU[35];

  reg [`ALUOPBITS-1:0] ALUOP_reg;
  reg [`ALUDATABITS-1:0] OP1_reg;
  reg [`ALUDATABITS-1:0] OP2_reg;
  
  wire [`ALUDATABITS-1:0] OP3;
  wire [`ALUCSROUTBITS-1:0] CSR_ALU_OUT;
  reg [`ALUCSRINBITS-1:0] CSR_ALU_IN;
  
  reg [2:0] state;
  localparam IDLE          = 3'd0;
  localparam LOAD_ALUOP    = 3'd1;  // Waiting for ALUOP write
  localparam LOAD_OP1      = 3'd2;  // Waiting for OP1 write & CSR ready
  localparam LOAD_OP2      = 3'd3;  // Waiting for OP2 write & CSR ready
  localparam COMPUTING     = 3'd4;
  localparam RESULT_READY  = 3'd5;
  
  always @(posedge clk) begin
    if (reset) begin
      state <= IDLE;
      CSR_ALU_IN <= 3'b000;
      ALUOP_reg <= 4'b0;
      OP1_reg <= 32'b0;
      OP2_reg <= 32'b0;
    end else begin
      case (state)
        IDLE: begin
          CSR_ALU_IN <= 3'b000;
          
          // Wait for ALUOP write to start
          if (wr_aluop) begin
            ALUOP_reg <= wr_data[`ALUOPBITS-1:0];
            state <= LOAD_ALUOP;
          end
        end
        
        LOAD_ALUOP: begin
          // Wait for OP1 write
          if (wr_op1) begin
            OP1_reg <= wr_data;
            state <= LOAD_OP1;
          end
        end
        
        LOAD_OP1: begin
          // Wait for ALU to be ready for OP1, then signal it's stable
          if (CSR_ALU_OUT[0]) begin
            CSR_ALU_IN[1] <= 1'b1;  // Signal OP1 valid
            
            // If OP2 also written, move on
            if (wr_op2) begin
              OP2_reg <= wr_data;
              state <= LOAD_OP2;
            end
          end
          // Capture OP2 if it arrives while waiting
          else if (wr_op2) begin
            OP2_reg <= wr_data;
          end
        end
        
        LOAD_OP2: begin
          CSR_ALU_IN[1] <= 1'b0;  // Clear OP1 strobe
          
          // Wait for ALU ready for OP2, then signal
          if (CSR_ALU_OUT[1]) begin
            CSR_ALU_IN[2] <= 1'b1;  // Signal OP2 valid
            state <= COMPUTING;
          end
        end
        
        COMPUTING: begin
          CSR_ALU_IN[2] <= 1'b0;  // Clear OP2 strobe
          
          // Wait for result valid
          if (CSR_ALU_OUT[2]) begin
            CSR_ALU_IN[0] <= 1'b1;  // Protect result
            state <= RESULT_READY;
          end
        end
        
        RESULT_READY: begin
          if (rd_op3) begin
            CSR_ALU_IN[0] <= 1'b0;
            state <= IDLE;
          end
        end
      endcase
    end
  end
  
  assign from_FU_to_DE = {
    CSR_ALU_OUT,
    OP3
  };
  
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