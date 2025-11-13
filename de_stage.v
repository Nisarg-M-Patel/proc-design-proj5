`include "define.vh" 

//TODO: part2/bonus modify as necessary; may need to add more inputs/outputs?
module DE_STAGE(
  input wire                              clk,
  input wire                              reset,
  input wire [`FE_latch_WIDTH-1:0]        from_FE_latch,
  input wire [`from_AGEX_to_DE_WIDTH-1:0] from_AGEX_to_DE,  
  input wire [`from_MEM_to_DE_WIDTH-1:0]  from_MEM_to_DE,     
  input wire [`from_WB_to_DE_WIDTH-1:0]   from_WB_to_DE,  
  output wire [`from_DE_to_FE_WIDTH-1:0]  from_DE_to_FE,
  input wire [`from_FU_to_DE_WIDTH-1:0]   from_FU_to_DE,
  output wire [`from_DE_to_FU_WIDTH-1:0]  from_DE_to_FU,   
  output wire [`DE_latch_WIDTH-1:0]       DE_latch_out
);

  `UNUSED_VAR (from_MEM_to_DE)

  /* pipeline latch*/   
  reg [`DE_latch_WIDTH-1:0] DE_latch;
  wire valid_DE;

  /* architecture register file */ 
  /* verilator lint_off MULTIDRIVEN */
  reg [`DBITS-1:0] reg_file [`REGWORDS-1:0];
  
  /* decode signals */
  wire [`INSTBITS-1:0] inst_DE; 
  wire [`DBITS-1:0] PC_DE;
  wire [`DBITS-1:0] pcplus_DE;
  wire [`DBITS-1:0] pcnext_DE;
  wire [7:0] pc_xor_bhr_DE;
  wire[`DE_latch_WIDTH-1:0] DE_latch_contents;

  // extracting a part of opcode 
  wire [2:0] F3_DE; 
  wire [6:0] F7_DE; 
  wire [6:0] op_DE; 

  assign op_DE = inst_DE[6:0];  
  assign F3_DE = inst_DE[14:12];
  assign F7_DE = inst_DE[31:25];  
 
  /* opcode decoding logic */ 
  reg [`IOPBITS-1:0 ] op_I_DE; //  internal opcode enumerator for easy programming.  
  reg [`TYPENOBITS-1:0] type_I_DE;  // instruction format type information for decoding purpose 
  reg [`IMMTYPENOBITS-1:0] type_immediate_DE;  // immediate type information for decodding purpose 

  always @(*) begin 
    if ((op_DE == `ADD_OPCODE) && (F3_DE == `ADD_FUNCT3) && (F7_DE == `ADD_FUNCT7))
      op_I_DE = `ADD_I; 
    else if ((op_DE == `SUB_OPCODE) && (F3_DE == `SUB_FUNCT3) && (F7_DE == `SUB_FUNCT7))
      op_I_DE = `SUB_I; 
    else if ((op_DE == `AND_OPCODE) && (F3_DE == `AND_FUNCT3) && (F7_DE == `AND_FUNCT7))
      op_I_DE = `AND_I; 
    else if ((op_DE == `OR_OPCODE) && (F3_DE == `OR_FUNCT3) && (F7_DE == `OR_FUNCT7))
      op_I_DE = `OR_I; 
    else if ((op_DE == `XOR_OPCODE) && (F3_DE == `XOR_FUNCT3) && (F7_DE == `XOR_FUNCT7))
      op_I_DE = `XOR_I; 
    else if ((op_DE == `SLT_OPCODE) && (F3_DE == `SLT_FUNCT3) && (F7_DE == `SLT_FUNCT7))
      op_I_DE = `SLT_I; 
    else if ((op_DE == `SLTU_OPCODE) && (F3_DE == `SLTU_FUNCT3) && (F7_DE == `SLTU_FUNCT7))
      op_I_DE = `SLTU_I; 
    else if ((op_DE == `SRA_OPCODE) && (F3_DE == `SRA_FUNCT3) && (F7_DE == `SRA_FUNCT7))
      op_I_DE = `SRA_I; 
    else if ((op_DE == `SRL_OPCODE) && (F3_DE == `SRL_FUNCT3) && (F7_DE == `SRL_FUNCT7))
      op_I_DE = `SRL_I; 
    else if ((op_DE == `SLL_OPCODE) && (F3_DE == `SLL_FUNCT3) && (F7_DE == `SLL_FUNCT7))
      op_I_DE = `SLL_I; 
    else if ((op_DE == `MUL_OPCODE) && (F3_DE == `MUL_FUNCT3) && (F7_DE == `MUL_FUNCT7))
      op_I_DE = `MUL_I; 
    else if ((op_DE == `ADDI_OPCODE) && (F3_DE == `ADDI_FUNCT3))
      op_I_DE = `ADDI_I; 
    else if ((op_DE == `ANDI_OPCODE) && (F3_DE == `ANDI_FUNCT3))
      op_I_DE = `ANDI_I; 
    else if ((op_DE == `ORI_OPCODE) && (F3_DE == `ORI_FUNCT3))
      op_I_DE = `ORI_I; 
    else if ((op_DE == `XORI_OPCODE) && (F3_DE == `XORI_FUNCT3))
      op_I_DE = `XORI_I; 
    else if ((op_DE == `SLTI_OPCODE) && (F3_DE == `SLTI_FUNCT3))
      op_I_DE = `SLTI_I; 
    else if ((op_DE == `SLTIU_OPCODE) && (F3_DE == `SLTIU_FUNCT3))
      op_I_DE = `SLTIU_I; 
    else if ((op_DE == `SRAI_OPCODE) && (F3_DE == `SRAI_FUNCT3) && (F7_DE == `SRAI_FUNCT7))
      op_I_DE = `SRAI_I; 
    else if ((op_DE == `SRLI_OPCODE) && (F3_DE == `SRLI_FUNCT3) && (F7_DE == `SRLI_FUNCT7))
      op_I_DE = `SRLI_I; 
    else if ((op_DE == `SLLI_OPCODE) && (F3_DE == `SLLI_FUNCT3) && (F7_DE == `SLLI_FUNCT7))
      op_I_DE = `SLLI_I; 
    else if ((op_DE == `LUI_OPCODE))
      op_I_DE = `LUI_I; 
    else if ((op_DE == `AUIPC_OPCODE))
      op_I_DE = `AUIPC_I; 
    else if ((op_DE == `LW_OPCODE) && (F3_DE == `LW_FUNCT3))
      op_I_DE = `LW_I; 
    else if ((op_DE == `SW_OPCODE) && (F3_DE == `SW_FUNCT3))
      op_I_DE = `SW_I; 
    else if ((op_DE == `JAL_OPCODE))
      op_I_DE = `JAL_I;   
    else if ((op_DE == `JALR_OPCODE) && (F3_DE == `JALR_FUNCT3))
      op_I_DE = `JALR_I; 
    else if ((op_DE == `BEQ_OPCODE) && (F3_DE == `BEQ_FUNCT3))
      op_I_DE = `BEQ_I; 
    else if ((op_DE == `BNE_OPCODE) && (F3_DE == `BNE_FUNCT3))
      op_I_DE = `BNE_I; 
    else if ((op_DE == `BLT_OPCODE) && (F3_DE == `BLT_FUNCT3))
      op_I_DE = `BLT_I; 
    else if ((op_DE == `BGE_OPCODE) && (F3_DE == `BGE_FUNCT3))
      op_I_DE = `BGE_I; 
    else if ((op_DE == `BLTU_OPCODE) && (F3_DE == `BLTU_FUNCT3))
      op_I_DE = `BLTU_I; 
    else if ((op_DE == `BGEU_OPCODE) && (F3_DE == `BGEU_FUNCT3))
      op_I_DE = `BGEU_I; 
    else if ((op_DE == `CSRR_OPCODE) && (F3_DE == `CSRR_FUNCT3))
      op_I_DE = `CSRR_I; 
    else if ((op_DE == `CSRW_OPCODE) && (F3_DE == `CSRW_FUNCT3))
      op_I_DE = `CSRW_I; 
    else 
      op_I_DE = `INVALID_I;
  end 

  always @(*) begin
    type_I_DE = '0;
    type_immediate_DE = '0;
    if ((op_I_DE == `ADD_I) || 
        (op_I_DE == `SUB_I ) || 
        (op_I_DE ==  `AND_I) || 
        (op_I_DE == `OR_I) || 
        (op_I_DE == `XOR_I) || 
        (op_I_DE == `SLT_I) || 
        (op_I_DE ==  `SLTU_I) || 
        (op_I_DE ==  `SRA_I) || 
        (op_I_DE == `SRL_I ) || 
        (op_I_DE == `SLL_I) || 
        (op_I_DE ==  `MUL_I)) begin
      type_I_DE = `R_Type;
    end else 
    if ((op_I_DE == `CSRR_I) || 
        (op_I_DE == `CSRW_I) || 
        (op_I_DE == `ADDI_I ) || 
        (op_I_DE == `ANDI_I) || 
        (op_I_DE == `ORI_I) || 
        (op_I_DE == `XORI_I) || 
        (op_I_DE == `SLTI_I) ||  
        (op_I_DE == `SLTIU_I ) || 
        (op_I_DE == `LW_I ) || 
        (op_I_DE == `JR_I) || 
        (op_I_DE == `JALR_I) ) begin
      type_I_DE = `I_Type; 
      type_immediate_DE = `I_immediate;
    end else 
    if ((op_I_DE ==  `SRAI_I ) || 
        (op_I_DE == `SRLI_I) || 
        (op_I_DE == `SLLI_I)) begin
      type_I_DE = `I_Type;
      type_immediate_DE = `I_immediate;
    end else 
    if ((op_I_DE ==  `LUI_I) || 
        (op_I_DE == `AUIPC_I )) begin 
      type_I_DE = `I_Type; 
      type_immediate_DE = `U_immediate; 
    end else 
    if (op_I_DE == `SW_I) begin
      type_I_DE = `S_Type;
      type_immediate_DE = `S_immediate;  
    end else 
    if (op_I_DE ==  `JAL_I) begin 
      type_I_DE = `U_Type;
      type_immediate_DE = `J_immediate; 
    end else 
    if ((op_I_DE ==  `BEQ_I ) || 
        (op_I_DE == `BNE_I) || 
        (op_I_DE == `BLT_I) || 
        (op_I_DE == `BGE_I) || 
        (op_I_DE == `BLTU_I) || 
        (op_I_DE == `BGEU_I)) begin 
      type_I_DE = `S_Type;
      type_immediate_DE = `B_immediate;
    end
  end

  // Decode immediate value
  reg [`DBITS-1:0] sxt_imm_DE;
  always @(*) begin 
    case (type_immediate_DE)  
    `I_immediate: sxt_imm_DE = {{21{inst_DE[31]}}, inst_DE[30:25], inst_DE[24:21], inst_DE[20]}; 
    `S_immediate: sxt_imm_DE = {{21{inst_DE[31]}}, inst_DE[30:25], inst_DE[11:8], inst_DE[7]}; 
    `B_immediate: sxt_imm_DE = {{20{inst_DE[31]}}, inst_DE[7], inst_DE[30:25], inst_DE[11:8], 1'b0};
    `U_immediate: sxt_imm_DE = {inst_DE[31], inst_DE[30:20], inst_DE[19:12], 12'b0}; 
    `J_immediate: sxt_imm_DE = {{12{inst_DE[31]}}, inst_DE[19:12], inst_DE[20], inst_DE[30:25], inst_DE[24:21], 1'b0}; 
    default:      sxt_imm_DE = '0; 
    endcase  
  end
 
  /* this signal is passed from WB stage */ 
  wire wr_reg_WB; // is this instruction writing into a register file? 
  wire [`REGNOBITS-1:0] wregno_WB; // destination register ID 
  wire [`DBITS-1:0] regval_WB;  // the contents to be written in the register file (or CSR )

  // signals come from WB stage for register WB 
  assign {
    wr_reg_WB, 
    wregno_WB, 
    regval_WB
  } = from_WB_to_DE;  

  wire pipeline_stall_DE; 
  
  // decoding the contents of FE latch out. the order should be matched with the fe_stage.v 
  assign {
    valid_DE,
    inst_DE,
    PC_DE, 
    pcplus_DE,
    pcnext_DE,
    pc_xor_bhr_DE
  } = from_FE_latch;  // based on the contents of the latch, you can decode the conten
  


  wire [`REGNOBITS-1:0] rs1_DE;
  wire [`REGNOBITS-1:0] rs2_DE;
  wire [`REGNOBITS-1:0] rd_DE;
  
  wire [`DBITS-1:0] rs1_val_DE;
  wire [`DBITS-1:0] rs2_val_DE;

  wire is_br_DE;    // is conditional branch instr
  wire is_jmp_DE;   // is jump instr
  wire rd_mem_DE;   // is LD instr
  wire wr_mem_DE;   // is ST instr
  wire wr_reg_DE;   // is writing back to register file
  reg  use_rs1_DE;  // instruction uses rs1
  reg  use_rs2_DE;  // instruction uses rs2

  // Decode instruction registers
  assign rs1_DE = inst_DE[19:15];
  assign rs2_DE = inst_DE[24:20];
  assign rd_DE  = inst_DE[11:7]; 

  // Read register file
  assign rs1_val_DE = reg_file[rs1_DE];
  assign rs2_val_DE = reg_file[rs2_DE];

  // decode instruction info
  assign is_br_DE  = ((op_I_DE == `BEQ_I) || (op_I_DE == `BNE_I) || (op_I_DE == `BLT_I) || (op_I_DE == `BGE_I) || (op_I_DE == `BLTU_I) || (op_I_DE == `BGEU_I)) ? 1 : 0;
  assign is_jmp_DE = ((op_I_DE == `JAL_I) || (op_I_DE == `JR_I) || (op_I_DE == `JALR_I)) ? 1 : 0;  
  assign rd_mem_DE = (op_I_DE == `LW_I) ? 1 :0 ;
  assign wr_mem_DE = (op_I_DE == `SW_I) ? 1 : 0 ; 
  assign wr_reg_DE = ((op_I_DE == `CSRR_I) || 
                      (op_I_DE == `ADD_I) || 
                      (op_I_DE == `SUB_I) || 
                      (op_I_DE == `AND_I) || 
                      (op_I_DE == `OR_I) || 
                      (op_I_DE == `XOR_I) || 
                      (op_I_DE == `SLT_I) || 
                      (op_I_DE == `SLTU_I) || 
                      (op_I_DE == `SRA_I) || 
                      (op_I_DE == `SRL_I) || 
                      (op_I_DE == `SLL_I) || 
                      (op_I_DE == `MUL_I) || 
                      (op_I_DE == `ADDI_I) || 
                      (op_I_DE == `ANDI_I) || 
                      (op_I_DE == `ORI_I) || 
                      (op_I_DE == `XORI_I) || 
                      (op_I_DE == `SLTI_I) || 
                      (op_I_DE == `SLTIU_I) || 
                      (op_I_DE == `SRAI_I) || 
                      (op_I_DE == `SRLI_I) || 
                      (op_I_DE == `SLLI_I) || 
                      (op_I_DE == `LUI_I) || 
                      (op_I_DE == `AUIPC_I) || 
                      (op_I_DE == `LW_I) || 
                      (op_I_DE == `JAL_I) || 
                      (op_I_DE == `JALR_I)) ? ((rd_DE != 0) ? 1 : 0): 0; 

  always @(*) begin 
    case (type_I_DE) 
    `I_Type: begin
      use_rs1_DE = 1;
      use_rs2_DE = 0; 
    end
    `R_Type: begin
      use_rs1_DE = 1;
      use_rs2_DE = 1; 
    end
    `S_Type: begin 
      use_rs1_DE = 1; 
      use_rs2_DE = 1; 
    end
    `U_Type: begin 
      use_rs1_DE = 0; 
      use_rs2_DE = 0;
    end
    default: begin
      use_rs1_DE = 0; 
      use_rs2_DE = 0;
    end
    endcase
  end
  
  // Handle data dependency

  reg [31:0] in_use_regs;
  wire has_data_hazards;
  wire br_mispred_AGEX;

  // process AGEX forwarding
  assign { 
    br_mispred_AGEX          
  } = from_AGEX_to_DE;
 
  assign has_data_hazards = (use_rs1_DE && in_use_regs[rs1_DE]) 
                         || (use_rs2_DE && in_use_regs[rs2_DE]);

  assign pipeline_stall_DE = has_data_hazards || br_mispred_AGEX;

  always @(posedge clk) begin
    if (reset) begin
      in_use_regs <= '0;
    end else begin
      if (~pipeline_stall_DE && wr_reg_DE) begin
        in_use_regs[rd_DE] <= 1;
      end 
      if (wr_reg_WB) begin
        in_use_regs[wregno_WB] <= 0;
      end
    end
  end

  /////////////////////////////////////////////////////////////////////////////

  // register file initialization
  initial begin
    for (integer i = 0; i < 32; ++i) 
      reg_file[i] = '0;
  end

  // register file update - exclude ALU registers
  always @ (negedge clk) begin 
    if (wr_reg_WB) begin
      // Don't write to ALU special registers (x26, x27, x29, x30, x31)
      if ((wregno_WB != 5'd26) && (wregno_WB != 5'd27) && 
          (wregno_WB != 5'd29) && (wregno_WB != 5'd30) && (wregno_WB != 5'd31)) begin
        reg_file[wregno_WB] <= regval_WB; 
      end
    end
  end

  // Forward declaration for ALU bypass (defined below in TODO section)
  wire [`DBITS-1:0] rs2_val_final_DE;

  // the order of latch contents should be matched in the AGEX stage when we extract the contents.
  assign DE_latch_contents = {
    valid_DE,
    inst_DE,
    PC_DE,
    pcplus_DE,
    pcnext_DE,
    op_I_DE,
    rs1_val_DE,
    rs2_val_final_DE,    // Use bypassed value for ALU operations
    sxt_imm_DE,
    is_br_DE,
    is_jmp_DE,
    rd_mem_DE,
    wr_mem_DE,
    wr_reg_DE,
    rd_DE,
    pc_xor_bhr_DE
  };

  // Update DE latch
  always @ (posedge clk) begin
    if (reset) begin
      DE_latch <= '0;
    end else begin  
      if (pipeline_stall_DE) 
        DE_latch <= '0;
      else
        DE_latch <= DE_latch_contents;
     end 
  end

  // forward signals to FE stage
  assign from_DE_to_FE = {
    pipeline_stall_DE
  };

  // send DE latch contents to next pipeline stage
  assign DE_latch_out = DE_latch;

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // External ALU Integration
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  // Storage for ALU operation and operands
  reg [`ALUOPBITS-1:0] alu_operation;
  reg [`ALUDATABITS-1:0] alu_operand_a;
  reg [`ALUDATABITS-1:0] alu_operand_b;
  
  // Handshake protocol state machine
  reg [2:0] alu_protocol_state;
  reg [2:0] alu_protocol_next;
  
  localparam STATE_WAIT_OP      = 3'd0;
  localparam STATE_WAIT_FIRST   = 3'd1;
  localparam STATE_WAIT_SECOND  = 3'd2;
  localparam STATE_EXECUTE      = 3'd3;
  localparam STATE_COMPLETE     = 3'd4;
  
  // Control signals for handshake
  reg [2:0] handshake_control;
  wire [2:0] handshake_status;
  wire [`ALUDATABITS-1:0] alu_output;
  
  // Status bits from ALU
  wire first_operand_ready;
  wire second_operand_ready;
  wire computation_done;
  
  assign first_operand_ready = handshake_status[0];
  assign second_operand_ready = handshake_status[1];
  assign computation_done = handshake_status[2];
  
  // Connect to FU stage
  assign from_DE_to_FU = {
    alu_operand_a,
    alu_operand_b,
    alu_operation,
    handshake_control
  };
  
  assign {alu_output, handshake_status} = from_FU_to_DE;
  
  // Determine if pipeline needs to stall for ALU
  wire alu_needs_stall;
  assign alu_needs_stall = 
    (alu_protocol_state == STATE_WAIT_FIRST && !first_operand_ready) ||
    (alu_protocol_state == STATE_WAIT_SECOND && !second_operand_ready);
  
  // Update state on clock edge
  always @(posedge clk) begin
    if (reset)
      alu_protocol_state <= STATE_WAIT_OP;
    else
      alu_protocol_state <= alu_protocol_next;
  end
  
  // Capture operands and manage computation
  always @(posedge clk) begin
    if (reset) begin
      alu_operation <= 4'd0;
      alu_operand_a <= 32'd0;
      alu_operand_b <= 32'd0;
    end else if (!alu_needs_stall) begin
      // Store result when computation completes
      if (alu_protocol_state == STATE_EXECUTE && computation_done) begin
        reg_file[5'd27] <= alu_output;  // Write to x27 (OP3)
      end
      // Clear operands at start of new operation
      else if (alu_protocol_state == STATE_WAIT_OP) begin
        alu_operand_a <= 32'd0;
        alu_operand_b <= 32'd0;
      end
      // Capture values during writeback phase
      else if (wr_reg_WB) begin
        case (wregno_WB)
          5'd29: alu_operation <= regval_WB[`ALUOPBITS-1:0];  // ALUOP
          5'd30: alu_operand_a <= regval_WB;                  // OP1
          5'd31: alu_operand_b <= regval_WB;                  // OP2
          default: begin
            // Do nothing for other register writes
          end
        endcase
      end 
    end
  end
  
  // State machine and handshake control generation
  always @(*) begin
    // Default: stay in current state
    alu_protocol_next = alu_protocol_state;
    handshake_control = 3'b000;
    
    case (alu_protocol_state)
      STATE_WAIT_OP: begin
        // Waiting for operation to begin
        handshake_control = 3'b000;
        alu_protocol_next = STATE_WAIT_FIRST;
      end
      
      STATE_WAIT_FIRST: begin
        // Waiting for first operand acknowledgment
        handshake_control = 3'b000;
        if (first_operand_ready && (alu_operand_a != 32'd0)) begin
          handshake_control = 3'b010;  // Signal first operand ready
          alu_protocol_next = STATE_WAIT_SECOND;
        end
      end
      
      STATE_WAIT_SECOND: begin
        // Waiting for second operand acknowledgment
        handshake_control = 3'b010;
        if (second_operand_ready && (alu_operand_b != 32'd0) && (alu_operation != 4'd0)) begin
          handshake_control = 3'b110;  // Signal second operand ready
          alu_protocol_next = STATE_EXECUTE;
        end
      end
      
      STATE_EXECUTE: begin
        // Computing result
        handshake_control = 3'b110;
        if (computation_done) begin
          handshake_control = 3'b111;  // Lock result
          alu_protocol_next = STATE_COMPLETE;
        end
      end
      
      STATE_COMPLETE: begin
        // Result stored, ready for next operation
        handshake_control = 3'b111;
        alu_protocol_next = STATE_WAIT_OP;
      end
      
      default: begin
        handshake_control = 3'b000;
        alu_protocol_next = STATE_WAIT_OP;
      end
    endcase
  end

endmodule