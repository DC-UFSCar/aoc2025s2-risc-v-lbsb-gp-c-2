module riscvmulti (
    input         clk,
    input         reset,
    output [31:0] Address,
    output [31:0] WriteData,
    output        MemWrite,
    input  [31:0] ReadData,
    output [3:0]  WriteMask,
    output reg    halt
);

    // instruction, PC and state
    reg [31:0] instr;
    reg [31:0] PC;
    reg [2:0]  state;

    // instruction decoders
    wire isALUreg  =  (instr[6:0] == 7'b0110011);
    wire isALUimm  =  (instr[6:0] == 7'b0010011);
    wire isBranch  =  (instr[6:0] == 7'b1100011);
    wire isJALR    =  (instr[6:0] == 7'b1100111);
    wire isJAL     =  (instr[6:0] == 7'b1101111);
    wire isAUIPC   =  (instr[6:0] == 7'b0010111);
    wire isLUI     =  (instr[6:0] == 7'b0110111);
    wire isLoad    =  (instr[6:0] == 7'b0000011);
    wire isStore   =  (instr[6:0] == 7'b0100011);
    wire isSYSTEM  =  (instr[6:0] == 7'b1110011);
    wire isEBREAK  =  (isSYSTEM && (instr[14:12] == 3'b000));

    // immediates
    wire [31:0] Uimm = { instr[31], instr[30:12], {12{1'b0}} };
    wire [31:0] Iimm = {{21{instr[31]}}, instr[30:20]};
    wire [31:0] Simm = {{21{instr[31]}}, instr[30:25], instr[11:7]};
    wire [31:0] Bimm = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
    wire [31:0] Jimm = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};

    // register indices & funct fields
    wire [4:0] rs1Id_A1 = instr[19:15];
    wire [4:0] rs2Id_A2 = instr[24:20];
    wire [4:0] rdId_A3  = instr[11:7];
    wire [2:0] funct3   = instr[14:12];
    wire [6:0] funct7   = instr[31:25];

    // register file and sources
    reg [31:0] RegisterBank [0:31];
    reg [31:0] rs1;
    reg [31:0] rs2;

    // ALU
    wire [31:0] SrcA = rs1;
    wire [31:0] SrcB = (isALUreg || isBranch) ? rs2 : Iimm;
    wire [31:0] aluPlus = SrcA + SrcB;
    wire [32:0] aluMinus = {1'b1, ~SrcB} + {1'b0, SrcA} + 33'b1;
    wire        LT  = (SrcA[31] ^ SrcB[31]) ? SrcA[31] : aluMinus[32];
    wire        LTU = aluMinus[32];
    wire        EQ  = (aluMinus[31:0] == 0);

    function [31:0] flip32;
        input [31:0] x;
        begin
            flip32 = {x[0],x[1],x[2],x[3],x[4],x[5],x[6],x[7],
                      x[8],x[9],x[10],x[11],x[12],x[13],x[14],x[15],
                      x[16],x[17],x[18],x[19],x[20],x[21],x[22],x[23],
                      x[24],x[25],x[26],x[27],x[28],x[29],x[30],x[31]};
        end
    endfunction

    wire [31:0] shifter_in = (funct3 == 3'b001) ? flip32(SrcA) : SrcA;
    wire [31:0] shifter = $signed({instr[30] & SrcA[31], shifter_in}) >>> SrcB[4:0];
    wire [31:0] leftshift = flip32(shifter);

    reg [31:0] ALUResult;
    always @(*) begin
        case (funct3)
            3'b000: ALUResult = (funct7[5] & instr[5]) ? aluMinus[31:0] : aluPlus;
            3'b001: ALUResult = leftshift;
            3'b010: ALUResult = {31'b0, LT};
            3'b011: ALUResult = {31'b0, LTU};
            3'b100: ALUResult = (SrcA ^ SrcB);
            3'b101: ALUResult = shifter;
            3'b110: ALUResult = (SrcA | SrcB);
            3'b111: ALUResult = (SrcA & SrcB);
            default: ALUResult = 32'b0;
        endcase
    end

    // branch predicate
    reg takeBranch;
    always @(*) begin
        case (funct3)
            3'b000: takeBranch = EQ;
            3'b001: takeBranch = !EQ;
            3'b100: takeBranch = LT;
            3'b101: takeBranch = !LT;
            3'b110: takeBranch = LTU;
            3'b111: takeBranch = !LTU;
            default: takeBranch = 1'b0;
        endcase
    end

    // PC helpers
    wire [31:0] PCplus4  = PC + 4;
    wire [31:0] PCTarget = PC + (isJAL ? Jimm : isAUIPC ? Uimm : Bimm);
    wire [31:0] PCNext = ((isBranch && takeBranch) || isJAL) ? PCTarget :
                         isJALR ? {aluPlus[31:1],1'b0} : PCplus4;

    // states
    localparam FETCH_INSTR = 3'd0;
    localparam WAIT_INSTR  = 3'd1;
    localparam FETCH_REGS  = 3'd2;
    localparam EXECUTE     = 3'd3;
    localparam LOAD        = 3'd4;
    localparam WAIT_DATA   = 3'd5;
    localparam STORE       = 3'd6;

    initial begin
        PC    = 32'b0;
        state = FETCH_INSTR;
        instr = 32'b0;
        halt  = 1'b0;
    end

    // --- Load/Store address and offset ---
    wire [31:0] LoadStoreAddress = rs1 + (isStore ? Simm : Iimm);
    wire [1:0]  byte_offset      = LoadStoreAddress[1:0];

    // --- write-back enable & data ---
    wire writeBackEn =
           (state == EXECUTE && (isALUreg || isALUimm || isJAL || isJALR || isLUI || isAUIPC))
        || (state == WAIT_DATA && isLoad);

    reg [31:0] load_data;
    always @(*) begin
        case (funct3)
            3'b000: begin // LB
                load_data = {{24{ (ReadData >> (8*byte_offset) & 8'h80) ? 1'b1 : 1'b0 }},
                              (ReadData >> (8*byte_offset)) & 8'hFF};
            end
            3'b001: begin // LH
                load_data = {{16{ (ReadData >> (8*(byte_offset & 2)) & 16'h8000) ? 1'b1 : 1'b0 }},
                              (ReadData >> (8*(byte_offset & 2))) & 16'hFFFF};
            end
            3'b010: load_data = ReadData; // LW
            3'b100: load_data = {24'h0, (ReadData >> (8*byte_offset)) & 8'hFF}; // LBU
            3'b101: load_data = {16'h0, (ReadData >> (8*(byte_offset & 2))) & 16'hFFFF}; // LHU
            default: load_data = ReadData;
        endcase
    end

    wire [31:0] writeBackData =
          isLoad ? load_data :
          (isJAL || isJALR) ? PCplus4 :
          isLUI ? Uimm :
          isAUIPC ? (PC + Uimm) :
          (isALUreg || isALUimm) ? ALUResult :
          32'b0;

    // --- Address selects: PC for instruction fetch, LoadStoreAddress for data ---
    assign Address = (state == FETCH_INSTR || state == WAIT_INSTR) ? PC : LoadStoreAddress;

    // --- Memory write enable when STORE state ---
    assign MemWrite = (state == STORE);

    // --- Prepare WriteData and WriteMask (little-endian bytes in a word) ---
    reg [31:0] write_data_r;
    reg [3:0]  write_mask_r;
    reg [7:0]  tmp_b;
    reg [15:0] tmp_h;

    always @(*) begin
        write_data_r = 32'b0;
        write_mask_r = 4'b0000;

        if (state == STORE) begin
            case (funct3)
                3'b000: begin // SB
                    tmp_b = rs2[7:0];
                    case (byte_offset)
                        2'b00: begin write_data_r = {24'b0, tmp_b};               write_mask_r = 4'b0001; end
                        2'b01: begin write_data_r = {16'b0, tmp_b, 8'b0};        write_mask_r = 4'b0010; end
                        2'b10: begin write_data_r = {8'b0, tmp_b, 16'b0};        write_mask_r = 4'b0100; end
                        2'b11: begin write_data_r = {tmp_b, 24'b0};              write_mask_r = 4'b1000; end
                    endcase
                end
                3'b001: begin // SH
                    tmp_h = rs2[15:0];
                    case (byte_offset)
                        2'b00: begin write_data_r = {16'b0, tmp_h};              write_mask_r = 4'b0011; end
                        2'b01: begin write_data_r = {8'b0, tmp_h, 8'b0};         write_mask_r = 4'b0110; end
                        2'b10: begin write_data_r = {tmp_h, 16'b0};              write_mask_r = 4'b1100; end
                        2'b11: begin write_data_r = {tmp_h[7:0], tmp_h[15:8], 16'b0}; write_mask_r = 4'b1001; end
                    endcase
                end
                3'b010: begin // SW
                    write_data_r = rs2;
                    write_mask_r = 4'b1111;
                end
                default: begin
                    write_data_r = 32'b0;
                    write_mask_r = 4'b0000;
                end
            endcase
        end
    end

    assign WriteData = write_data_r;
    assign WriteMask = write_mask_r;

    // sequential control
    integer i;
    always @(posedge clk) begin
        if (reset) begin
            PC <= 32'b0;
            state <= FETCH_INSTR;
            instr <= 32'b0;
            halt <= 1'b0;
            for (i = 0; i < 32; i = i + 1) RegisterBank[i] <= 32'b0;
        end else begin
            // write-back (x0 hardwired to zero)
            if (writeBackEn) begin
                if (rdId_A3 != 5'b0)
                    RegisterBank[rdId_A3] <= writeBackData;
            end

            case (state)
                FETCH_INSTR: state <= WAIT_INSTR;
                WAIT_INSTR: begin instr <= ReadData; state <= FETCH_REGS; end
                FETCH_REGS: begin
                    rs1 <= rs1Id_A1 ? RegisterBank[rs1Id_A1] : 32'b0;
                    rs2 <= rs2Id_A2 ? RegisterBank[rs2Id_A2] : 32'b0;
                    state <= EXECUTE;
                end
                EXECUTE: begin
                    if (!isSYSTEM)
                        PC <= PCNext;
                    else if (isEBREAK) begin
                        PC <= PC;
                        halt <= 1'b1;
                    end
                    state <= isLoad  ? LOAD :
                             isStore ? STORE :
                             FETCH_INSTR;
                end
                LOAD: state <= WAIT_DATA;
                WAIT_DATA: state <= FETCH_INSTR;
                STORE: state <= FETCH_INSTR;
                default: state <= FETCH_INSTR;
            endcase
        end
    end

    // on halt dump regs and finish (optional)
    always @(posedge clk) begin
        if (halt) begin
            $writememh("regs.out", RegisterBank);
            #10 $finish();
        end
    end

endmodule
