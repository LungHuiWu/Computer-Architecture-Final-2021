module RISCV(clk,
            rst_n,
            // for mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // for mem_I
            mem_addr_I,
            mem_rdata_I);

    input         clk, rst_n ;
    // For mem_D
    output  mem_wen_D  ;
    output  [31:2]  mem_addr_D ;
    output  [63:0]  mem_wdata_D;
    input   [63:0]  mem_rdata_D;
    // For mem_I
    output  [31:2]  mem_addr_I ;
    input   [31:0]  mem_rdata_I;

    // wire & reg

    wire    [31:0]  instruction;
    wire    [63:0]  data_read;

    reg     [11:0]  ctrl_signal;
    reg     [31:0]  immediate;
    wire    [11:0]  ctrl_signal_nxt;
    wire    [31:0]  immediate_nxt;
    reg     [63:0]  WriteData;
    reg     [63:0]  ReadData1;
    reg     [63:0]  ReadData2;
    reg     [31:0]  PC;
    reg     [31:0]  PC_nxt;

    wire    [63:0]  alu_input2;
    wire    [63:0]  ALUresult;
    wire    zero;
    wire    [63:0]  pre_write;
    wire    [31:0]  PC_4;
    wire    [31:0]  sum1;
    wire    w1;
    wire    branch_or_not;
    wire    [31:0]  branch_address;
    reg     [63:0]  mem [0:31];
    wire    [63:0]  rdata_inv;
    integer     i;
    wire    gan;
    wire    gan_eq_zero;

    //reg     [31:0]  inst;
    
    invert32    inv_inst(mem_rdata_I, instruction);
    invert64    inv_data(mem_rdata_D, data_read);
    
    CHIP        chip(clk, rst_n, instruction, ctrl_signal_nxt, immediate_nxt,gan);

    wire [4:0] ReadReg1, ReadReg2, WriteReg;

    assign ReadReg1 = instruction[19:15];
    assign ReadReg2 = instruction[24:20];
    assign WriteReg = instruction[11:7];
    //RegWrite = ctrl_signal[5];

    ALU  alu(ReadData1, alu_input2, ctrl_signal[3:0], zero, ALUresult[63:0]);

    assign  PC_4    =   PC + 4;
    assign  alu_input2  =   (ctrl_signal[4]) ? $signed({immediate}) : ReadData2;
    assign  pre_write = (ctrl_signal[6]) ? data_read : ALUresult;

    assign  sum1 = immediate + PC;
    assign  gan_eq_zero = gan == zero;
    assign  w1 = ctrl_signal[9] && gan_eq_zero;
    assign  branch_or_not = w1 || ctrl_signal[11];
    assign  branch_address = (branch_or_not) ? sum1 : PC_4;

    // output
    assign  mem_wen_D   =   ctrl_signal[7];
    assign  mem_addr_D  =   ALUresult[31:2];
    assign  mem_wdata_D =   rdata_inv;
    assign  mem_addr_I  =   PC[31:2];

    invert64    inv_rdata(ReadData2,rdata_inv);

    // todo
    always @(*) begin

        PC_nxt          = 0;

        ctrl_signal = ctrl_signal_nxt;
        immediate   = immediate_nxt;
        
        //  handle register memory
        ReadData1 = mem[ReadReg1];
        ReadData2 = mem[ReadReg2];
    
        WriteData = (ctrl_signal[10] || ctrl_signal[11]) ? {32'b0 , PC_4} : pre_write;
    
        PC_nxt = (ctrl_signal[10]) ? (immediate + ReadData1[31:0]) : branch_address;
    end

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin  
            for(i=0;i<32;i=i+1) begin
                mem[i] <= 0;
            end

            PC          <= 0;
        end
        else begin
            PC          <=  PC_nxt;
            if(ctrl_signal[5] == 1'b1) begin
                mem[WriteReg] <= WriteData;
            end
            mem[0] <= 0;
        end
    end

endmodule

module invert32(litt_ins, usable_ins);
    input [31:0] litt_ins;
    output [31:0] usable_ins;
    assign usable_ins[7:0] = litt_ins[31:24];
    assign usable_ins[15:8] = litt_ins[23:16];
    assign usable_ins[23:16] = litt_ins[15:8];
    assign usable_ins[31:24] = litt_ins[7:0];
endmodule

module invert64(litt_ins, usable_ins);
    input [63:0] litt_ins;
    output [63:0] usable_ins;
    assign usable_ins[7:0] = litt_ins[63:56];
    assign usable_ins[15:8] = litt_ins[55:48];
    assign usable_ins[23:16] = litt_ins[47:40];
    assign usable_ins[31:24] = litt_ins[39:32];
    assign usable_ins[39:32] = litt_ins[31:24];
    assign usable_ins[47:40] = litt_ins[23:16];
    assign usable_ins[55:48] = litt_ins[15:8];
    assign usable_ins[63:56] = litt_ins[7:0];
endmodule

module ALU(rd1, rd2, ALUctrl, zero, ALUresult);

    input   [63:0]  rd1,rd2;
    input   [3:0]   ALUctrl;
    output  zero;
    output  [63:0]  ALUresult;
    
    reg     [63:0]  ALUresultnice;
    reg     zero_w;

    always @(*) begin
        case(ALUctrl[3:0])
            4'b0000: begin
                ALUresultnice = rd1 + rd2;
                zero_w = ALUresultnice != 0;
            end//add
            4'b1000: begin
                ALUresultnice = rd1 - rd2;
                zero_w = ALUresultnice != 0;
            end//sub
            4'b0001: begin
                ALUresultnice = rd1 << rd2;
                zero_w = ALUresultnice != 0;
            end//sll
            4'b0010: begin
                ALUresultnice = (rd1 < rd2)?1:0;
                zero_w = ALUresultnice != 0;
            end//slt
            4'b0100: begin
                ALUresultnice = rd1 ^ rd2;
                zero_w = ALUresultnice != 0;
            end//xor
            4'b0101: begin
                ALUresultnice = rd1 >> rd2;
                zero_w = ALUresultnice != 0;
            end//srl
            4'b1101: begin
                ALUresultnice = rd1 >>> (rd2%64);
                zero_w = ALUresultnice != 0;
            end//sra
            4'b0110: begin
                ALUresultnice = rd1 | rd2;
                zero_w = ALUresultnice != 0;
            end//or
            4'b0111: begin
                ALUresultnice = rd1 & rd2;
                zero_w = ALUresultnice != 0;
            end//and
            default: begin
                ALUresultnice = 64'b0;
                zero_w = 0;
            end
        endcase
    end
    assign  ALUresult = ALUresultnice;
    assign  zero = zero_w;

endmodule

module CHIP(clk,rst_n,mem_rdata_I,ctrl_signal,immediate,gan);
    input         clk, rst_n        ;
    input  [31:0] mem_rdata_I       ;
	output [11:0] ctrl_signal  ;
	output [31:0] immediate;
    output  gan;
    
	// wire/reg 

    reg    [31:0] immediate_w;
    reg    [11:0] ctrl_signal_w;
    reg     gan_w;
    

    always @(*) begin
        gan_w = 0;
        case(mem_rdata_I[6:0])
            7'b1101111: begin     //jal UJ
                //immediate_w = {24'b0, 8'b01011100};
                immediate_w = {(mem_rdata_I[31]) ? 11'b11111111111 : 11'b0, mem_rdata_I[31], mem_rdata_I[19:12], mem_rdata_I[20], mem_rdata_I[30:21], 1'b0};
                ctrl_signal_w = 12'b100001111111;
            end
            7'b1100111: begin     //jalr I
                //immediate_w = {24'b111111111111111111111111, 8'b10011100};
                immediate_w = {(mem_rdata_I[31]) ? 20'b11111111111111111111 : 20'b0, mem_rdata_I[31:20]};
                ctrl_signal_w = 12'b010001111111;
            end
            7'b1100011: begin     
                case(mem_rdata_I[14:12])
                    3'b000: begin    //beq SB
                        //immediate_w = {24'b111111111111111111111111, 8'b11111000};
                        immediate_w = {(mem_rdata_I[31]) ? 19'b1111111111111111111 : 19'b0, mem_rdata_I[31], mem_rdata_I[7], mem_rdata_I[30:25], mem_rdata_I[11:8], 1'b0};
                        ctrl_signal_w = 12'b001001001000;
                        gan_w = 0;
                    end
                    3'b001: begin    //bne SB
                        //immediate_w = {24'b0, 8'b01010000};
                        immediate_w = {(mem_rdata_I[31]) ? 19'b1111111111111111111 : 19'b0, mem_rdata_I[31], mem_rdata_I[7], mem_rdata_I[30:25], mem_rdata_I[11:8], 1'b0};
                        ctrl_signal_w = 12'b001001001000;
                        gan_w = 1;
                    end
                    default: begin
                        immediate_w = 0;
                        ctrl_signal_w = 12'b000000000000;
                    end
                endcase
            end
            7'b0000011: begin     //ld I
                //immediate_w = {24'b0, 8'b0};
                immediate_w = {(mem_rdata_I[31]) ? 20'b11111111111111111111 : 20'b0, mem_rdata_I[31:20]};
                ctrl_signal_w = 12'b000101110000;
            end
            7'b0100011: begin     //sd S
                //immediate_w = {24'b0, 8'b00001000};
                immediate_w = {(mem_rdata_I[31]) ? 19'b1111111111111111111 : 19'b0, mem_rdata_I[31:25], mem_rdata_I[11:7]};
                ctrl_signal_w = 12'b000011010000;
            end
            7'b0010011: begin     
                case(mem_rdata_I[14:12])
                    3'b000: begin    //addi I
                        //immediate_w = {24'b111111111111111111111111, 8'b10011100};
                        immediate_w = {(mem_rdata_I[31]) ? 20'b11111111111111111111 : 20'b0, mem_rdata_I[31:20]};
                        ctrl_signal_w = 12'b000000110000;
                    end
                    3'b010: begin    //slti I
                        //immediate_w = {24'b0, 8'b00000011};
                        immediate_w = {(mem_rdata_I[31]) ? 20'b11111111111111111111 : 20'b0, mem_rdata_I[31:20]};
                        ctrl_signal_w = 12'b000000110010;
                    end
                    3'b100: begin    //xori I
                        //immediate_w = {24'b111111111111111111111111, 8'b10011100};
                        immediate_w = {(mem_rdata_I[31]) ? 20'b11111111111111111111 : 20'b0, mem_rdata_I[31:20]};
                        ctrl_signal_w = 12'b000000110100;
                    end
                    3'b110: begin    //ori I
                        //immediate_w = {24'b111111111111111111111111, 8'b10011100};
                        immediate_w = {(mem_rdata_I[31]) ? 20'b11111111111111111111 : 20'b0, mem_rdata_I[31:20]};
                        ctrl_signal_w = 12'b000000110110;
                    end
                    3'b111: begin    //andi I
                        //immediate_w = {24'b0, 8'b00001010};
                        immediate_w = {(mem_rdata_I[31]) ? 20'b11111111111111111111 : 20'b0, mem_rdata_I[31:20]};
                        ctrl_signal_w = 12'b000000110111;
                    end
                    3'b001: begin    //slli I
                        //immediate_w = {24'b0, 8'b00000111};
                        immediate_w = {(mem_rdata_I[31]) ? 20'b11111111111111111111 : 20'b0, mem_rdata_I[31:20]};
                        ctrl_signal_w = 12'b000000110001;
                    end
                    3'b101: begin    
                        case(mem_rdata_I[31:25])
                            7'b0000000: begin   //srli I
                                //immediate_w = {24'b0, 8'b00001010};
                                immediate_w = {(mem_rdata_I[31]) ? 20'b11111111111111111111 : 20'b0, mem_rdata_I[31:20]};
                                ctrl_signal_w = 12'b000000110101;
                            end
                            7'b0100000: begin   //srai I
                                //immediate_w = {20'b0, 12'b010000010001};
                                immediate_w = {(mem_rdata_I[31]) ? 20'b11111111111111111111 : 20'b0, mem_rdata_I[31:20]};
                                ctrl_signal_w = 12'b000000111101;
                            end
                            default: begin
                                immediate_w = 0;
                                ctrl_signal_w = 12'b000000000000;
                            end
                        endcase
                    end
                    default: begin
                        immediate_w = 0;
                        ctrl_signal_w = 12'b000000000000;
                    end
                endcase 
            end
            7'b0110011: begin 
                case(mem_rdata_I[14:12])
                    3'b000: begin       
                        case(mem_rdata_I[31:25])
                            7'b0000000: begin   //add R
                                immediate_w = {24'b0, 8'b0};
                                
                                ctrl_signal_w = 12'b000000100000;
                            end
                            7'b0100000: begin   //sub R
                                immediate_w = {24'b0, 8'b0};
                                
                                ctrl_signal_w = 12'b000000101000;
                            end
                            default: begin
                                immediate_w = 0;
                                ctrl_signal_w = 12'b000000000000;
                            end
                        endcase
                    end
                    3'b001: begin     //sll  R
                        immediate_w = {24'b0, 8'b0};
                        
                        ctrl_signal_w = 12'b000000100001;
                    end
                    3'b010: begin     //slt R
                        immediate_w = {24'b0, 8'b0};
                        
                        ctrl_signal_w = 12'b000000100010;
                    end
                    3'b100: begin     //xor R
                        immediate_w = {24'b0, 8'b0};
                        
                        ctrl_signal_w = 12'b000000100100;
                    end
                    3'b101: begin     
                        case(mem_rdata_I[31:25])
                            7'b0000000: begin   //srl R
                                immediate_w = {24'b0, 8'b0};
                                
                                ctrl_signal_w = 12'b000000100101;
                            end
                            7'b0100000: begin   //sra
                                immediate_w = {24'b0, 8'b0};
                                
                                ctrl_signal_w = 12'b000000101101;
                            end
                            default: begin
                                immediate_w = 0;
                                ctrl_signal_w = 12'b000000000000;
                            end
                        endcase
                    end
                    3'b110: begin     //or
                        immediate_w = {24'b0, 8'b0};
                        
                        ctrl_signal_w = 12'b000000100110;
                    end
                    3'b111: begin   //and
                        immediate_w = {24'b0, 8'b0};
                        
                        ctrl_signal_w = 12'b000000100111;
                    end
                    default: begin
                        immediate_w = 0;
                        ctrl_signal_w = 12'b000000000000;
                    end
                endcase
            end
            default: begin
                immediate_w = 0;
                ctrl_signal_w = 12'b000000000000;
            end
        endcase
    end
    assign ctrl_signal = ctrl_signal_w;
	assign immediate = immediate_w;
    assign gan = gan_w;
	

endmodule

