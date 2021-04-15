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
    output        mem_wen_D  ;
    output [31:0] mem_addr_D ;
    output [63:0] mem_wdata_D;
    input  [63:0] mem_rdata_D;
    // For mem_I
    output [31:0] mem_addr_I ;
    input  [31:0] mem_rdata_I;
    wire   [31:0] nice_mem_rdata_I;
    wire   [63:0] nice_mem_rdata_D, nice_mem_wdata_D;
    invert32 inv32(mem_rdata_I, nice_mem_rdata_I);
    invert64 inv3(mem_rdata_D, nice_mem_rdata_D);

    reg    [31:0] sum1        ;
    reg    [31:0] PC, PC_4    ;            
    wire   [31:0] PC_nxt      ;           
    wire          regWrite    ;             
    reg    [ 4:0] rd1, rd2    ;             
    wire   [63:0] rs1_data    ;
    wire   [63:0] rs2_data    ;
    wire   [63:0] WriteData   ;
    wire   [31:0] immediate   ;
    wire   [11:0] ctrl_signal ;
    wire   [63:0] alu_input2  ;
    wire   [63:0] pre_write   ;
    wire   [31:0] ALUresult   ;
    wire          w1, branch_or_not, zero;
    assign PC_4 = PC + 32'd4;
    assign mem_wen_D   = ctrl_signal[7];
    assign mem_addr_D  = ALUresult; 
    assign nice_mem_wdata_D = rs2_data;
    invert64 inv2(nice_mem_wdata_D, mem_wdata_D);
    assign mem_addr_I  = PC;
    
    // complete modules here
    
    CHIP chip(clk,rst_n,nice_mem_rdata_I,ctrl_signal,immediate); 
    register register(nice_mem_rdata_I,ctrl_signal[5],WriteData,rs1_data,rs2_data);
    assign alu_input2 = (ctrl_signal[4]) ? $signed({immediate}) : rs2_data;
    ALU  alu(rs1_data, alu_input2, ctrl_signal[3:0], zero, ALUresult);
    assign pre_write = (ctrl_signal[6]) ? nice_mem_rdata_D : ALUresult;
    assign WriteData = (ctrl_signal[10] | ctrl_signal[11]) ? {32'b0 , PC_4} : pre_write;
    
    // complete!
    
    assign sum1 = immediate + PC;
    and an1(w1, ctrl_signal[9], zero);
    or  or1(branch_or_not, w1, ctrl_signal[11]);
    assign branch_address = (branch_or_not) ? sum1 : PC_4;

    assign PC_nxt = (ctrl_signal[10]) ? (immediate + rs1_data[31:0]) : branch_address;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PC <= 32'h00010000; 
        end
        else begin
            PC <= PC_nxt; 
        end
    end

endmodule

module invert32(litt_ins, usable_ins);
    input [31:0] litt_ins;
    output [31:0] usable_ins;
    assign usable_ins[3:0] = litt_ins[31:28];
    assign usable_ins[7:4] = litt_ins[27:24];
    assign usable_ins[11:8] = litt_ins[23:20];
    assign usable_ins[15:12] = litt_ins[19:16];
    assign usable_ins[19:16] = litt_ins[15:12];
    assign usable_ins[23:20] = litt_ins[11:8];
    assign usable_ins[27:24] = litt_ins[7:4];
    assign usable_ins[31:28] = litt_ins[3:0];
endmodule

module invert64(litt_ins, usable_ins);
    input [63:0] litt_ins;
    output [63:0] usable_ins;
    assign usable_ins[3:0] = litt_ins[63:60];
    assign usable_ins[7:4] = litt_ins[59:56];
    assign usable_ins[11:8] = litt_ins[55:52];
    assign usable_ins[15:12] = litt_ins[51:48];
    assign usable_ins[19:16] = litt_ins[47:44];
    assign usable_ins[23:20] = litt_ins[43:40];
    assign usable_ins[27:24] = litt_ins[39:36];
    assign usable_ins[31:28] = litt_ins[35:32];
    assign usable_ins[35:32] = litt_ins[31:28];
    assign usable_ins[39:36] = litt_ins[27:24];
    assign usable_ins[43:40] = litt_ins[23:20];
    assign usable_ins[47:44] = litt_ins[19:16];
    assign usable_ins[51:48] = litt_ins[15:12];
    assign usable_ins[55:52] = litt_ins[11:8];
    assign usable_ins[59:56] = litt_ins[7:4];
    assign usable_ins[63:60] = litt_ins[3:0];
endmodule

module ALU(rd1, rd2, ALUctrl, zero, ALUresult);

    input   [63:0]  rd1,rd2;
    input   [3:0]   ALUctrl;
    output  zero;
    output  [31:0]  ALUresult;
    
    wire    [63:0]  ALUresultnice;

    case(ALUctrl)
        0000: begin
            ALUresultnice = rd1 + rd2;
            zero = 0;
        end//add
        1000: begin
            ALUresultnice = rd1 - rd2;
            zero = 0;
        end//sub
        0001: begin
            ALUresultnice = rd1 << rd2;
            zero = 0;
        end//sll
        0010: begin
            ALUresultnice = (rd1 < rd2)?1:0;
            zero = 0;
        end//slt
        0100: begin
            ALUresultnice = rd1 ^ rd2;
            zero = 0;
        end//xor
        0101: begin
            ALUresultnice = rd1 >> rd2;
            zero = 0;
        end//srl
        1101: begin
            ALUresultnice = rd1 >>> rd2;
            zero = 0;
        end//sra
        0110: begin
            ALUresultnice = rd1 || rd2;
            zero = 0;
        end//or
        0111: begin
            ALUresultnice = rd1 && rd2;
            zero = 0;
        end//and
        default: begin
            ALUresultnice = 64'b0;
            zero = 1;
        end
    endcase
    
    assign ALUresult = ALUresultnice[31:0];

endmodule

module register(mem_rdata_I,
            RegWrite,
            WriteData,
            ReadData1,
            ReadData2);
    
    input  [31:0] mem_rdata_I;
    input [0] RegWrite;
    input [63:0] WriteData;
    reg [4:0] ReadReg1, ReadReg2, WriteReg;
    output reg [63:0] ReadData1, ReadData2;
    reg [63:0] ReadData1_w, ReadData2_w;
    reg [63:0] mem [0:31];
    reg [63:0] mem_w [0:31];

    ReadReg1 = mem_rdata_I[19:15];
    ReadReg2 = mem_rdata_I[24:20];
    WriteReg = mem_rdata_I[11:7];
    //RegWrite = ctrl_signal[5];
    

    integer i;

    always @(*) begin
        for (i=0; i<32; i=i+1)
            mem_w[i] = mem[i];
        ReadData1_w = ReadData1;
        ReadData2_w = ReadData2;
        if(RegWrite == 1'b1) mem_w[WriteReg] = WriteData; 
        ReadData1_w = mem_w[ReadReg1];
        ReadData2_w = mem_w[ReadReg2];
    
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i=0; i<32; i=i+1) mem[i] <= 32'h0;
            ReadData1 <= 0;
            ReadData2 <= 0;
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<32; i=i+1)
                mem[i] <= mem_w[i];
            ReadData1 <= ReadData1_w;
            ReadData2 <= ReadData2_w;
        end       
    end
endmodule

module CHIP(clk,rst_n,mem_rdata_I,ctrl_signal,immediate);
    input         clk, rst_n        ;
    input  [31:0] mem_rdata_I       ;
	output reg [11:0] ctrl_signal  ;
	output reg [31:0] immediate;
    
	// wire/reg 

    reg [11:0]  ctrl_signal_w;
    reg [31:0]  immediate_w;
    

    always @(*) begin
        immediate_w = immediate;
        ctrl_signal_w = ctrl_signal;
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
                        ctrl_signal_w = 12'b001001001111;
                    end
                    3'b001: begin    //bne SB
                        //immediate_w = {24'b0, 8'b01010000};
                        immediate_w = {(mem_rdata_I[31]) ? 19'b1111111111111111111 : 19'b0, mem_rdata_I[31], mem_rdata_I[7], mem_rdata_I[30:25], mem_rdata_I[11:8], 1'b0};
                        ctrl_signal_w = 12'b001001001111;
                    end
                endcase
            end
            7'b0000011: begin     //ld I
                //immediate_w = {24'b0, 8'b0};
                immediate_w = {(mem_rdata_I[31]) ? 20'b11111111111111111111 : 20'b0, mem_rdata_I[31:20]};
                ctrl_signal_w = 12'b000101111111;
            end
            7'b0100011: begin     //sd S
                //immediate_w = {24'b0, 8'b00001000};
                immediate_w = {(mem_rdata_I[31]) ? 19'b1111111111111111111 : 19'b0, mem_rdata_I[31:25], mem_rdata_I[11:7]};
                ctrl_signal_w = 12'b000011011111;
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
                        endcase
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
                endcase
            end
        endcase
    end



	always @(posedge clk or negedge rst_n) begin
    	if (!rst_n) begin
        	// reset
        	immediate <= 32'b0;
        	ctrl_signal <= 12'b0;
    	end

    	else begin
        	ctrl_signal <= ctrl_signal_w;
        	immediate <= immediate_w;
    	end
	end
	

	// Connect to your HW3 module
	
	

endmodule



