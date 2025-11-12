`include "define.vh" 

module FU_STAGE(
  input wire clk,
  input wire reset,
  input wire [`from_DE_to_FU_WIDTH-1:0] from_DE_to_FU,   
  output wire [`from_FU_to_DE_WIDTH-1:0] from_FU_to_DE
);

  // Unpack inputs from DE (use only what we need from the 71-bit bus)
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
  // Bits [70:36] unused

  // ALU register storage
  reg [`ALUOPBITS-1:0] ALUOP_reg;
  reg [`ALUDATABITS-1:0] OP1_reg;
  reg [`ALUDATABITS-1:0] OP2_reg;
  
  // ALU interface signals
  wire [`ALUDATABITS-1:0] OP3;
  wire [`ALUCSROUTBITS-1:0] CSR_ALU_OUT;
  reg [`ALUCSRINBITS-1:0] CSR_ALU_IN;
  
  // State machine for ALU handshake protocol
  reg [2:0] state;
  localparam IDLE          = 3'd0;
  localparam WAIT_OP1      = 3'd1;
  localparam LOAD_OP1      = 3'd2;
  localparam WAIT_OP2      = 3'd3;
  localparam LOAD_OP2      = 3'd4;
  localparam COMPUTING     = 3'd5;
  localparam RESULT_READY  = 3'd6;
  
  // State machine
  always @(posedge clk) begin
    if (reset) begin
      state <= IDLE;
      CSR_ALU_IN <= 3'b001;  // CSR_ALU_IN[0]=1 protects result initially
      ALUOP_reg <= 4'b0;
      OP1_reg <= 32'b0;
      OP2_reg <= 32'b0;
    end else begin
      case (state)
        IDLE: begin
          CSR_ALU_IN <= 3'b001;
          
          // Capture ALUOP write
          if (wr_aluop) begin
            ALUOP_reg <= wr_data[`ALUOPBITS-1:0];
          end
          
          // Start sequence when OP1 is written
          if (wr_op1) begin
            OP1_reg <= wr_data;
            state <= WAIT_OP1;
          end
        end
        
        WAIT_OP1: begin
          // Wait for ALU to be ready for OP1
          if (CSR_ALU_OUT[0]) begin  // OP1 port ready
            CSR_ALU_IN[1] <= 1'b1;   // Signal OP1 is stable
            state <= LOAD_OP1;
          end
        end
        
        LOAD_OP1: begin
          // Keep CSR_ALU_IN[1] high for one more cycle
          CSR_ALU_IN[1] <= 1'b0;     // Clear OP1 stable signal
          state <= WAIT_OP2;
        end
        
        WAIT_OP2: begin
          // Capture OP2 write
          if (wr_op2) begin
            OP2_reg <= wr_data;
          end
          
          // Wait for ALU to be ready for OP2
          if (CSR_ALU_OUT[1]) begin  // OP2 port ready
            CSR_ALU_IN[2] <= 1'b1;   // Signal OP2 is stable
            state <= LOAD_OP2;
          end
        end
        
        LOAD_OP2: begin
          // Keep CSR_ALU_IN[2] high for one more cycle
          CSR_ALU_IN[2] <= 1'b0;     // Clear OP2 stable signal
          state <= COMPUTING;
        end
        
        COMPUTING: begin
          CSR_ALU_IN[0] <= 1'b0;     // Allow ALU to write result
          
          // Wait for result to be valid
          if (CSR_ALU_OUT[2]) begin  // Result valid
            CSR_ALU_IN[0] <= 1'b1;   // Protect result from being overwritten
            state <= RESULT_READY;
          end
        end
        
        RESULT_READY: begin
          // Wait for CPU to read result via SW instruction
          if (rd_op3) begin
            CSR_ALU_IN[0] <= 1'b0;   // Allow ALU to overwrite result
            state <= IDLE;
          end
        end
        
        default: begin
          state <= IDLE;
          CSR_ALU_IN <= 3'b001;
        end
      endcase
    end
  end
  
  // ALU is busy when not in IDLE state
  wire alu_busy;
  assign alu_busy = (state != IDLE);
  
  // Pack outputs to DE (fit into 35-bit bus)
  // Format: alu_busy(1) + CSR_ALU_OUT(3) + OP3(31 bits, truncated)
  assign from_FU_to_DE = {
    alu_busy,              // bit [34]
    CSR_ALU_OUT,          // bits [33:31]
    OP3[30:0]             // bits [30:0]
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