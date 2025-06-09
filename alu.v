module alu (
    input  [15:0] a, b,
    input  [3:0]  alu_op,
    output reg [15:0] result
);

    always @(*) begin
        case (alu_op)
            4'b0000: result = a + b;     // ADD
            4'b0001: result = a - b;     // SUB
            4'b0010: result = a * b;     // MUL
            4'b0011: result = a / b;     // DIV
            4'b0100: result = a & b;     // AND
            4'b0101: result = a | b;     // OR
            4'b0110: result = a ^ b;     // XOR
            4'b0111: result = ~a;        // NOT
            default: result = 16'h0000;
        endcase
    end
endmodule

module register_file (
    input clk,
    input reg_write,
    input [3:0] read_reg1, read_reg2, write_reg,
    input [15:0] write_data,
    output [15:0] read_data1, read_data2
);
    reg [15:0] regs [15:0];

    assign read_data1 = regs[read_reg1];
    assign read_data2 = regs[read_reg2];

    always @(posedge clk) begin
        if (reg_write)
            regs[write_reg] <= write_data;
    end
endmodule

module control_unit (
    input [18:0] instr,
    output [4:0] opcode,
    output [3:0] rd, rs1, rs2,
    output [9:0] imm
);
    assign opcode = instr[18:14];
    assign rd     = instr[13:10];
    assign rs1    = instr[9:6];
    assign rs2    = instr[5:2];
    assign imm    = instr[9:0];
endmodule

module memory (
    input clk,
    input mem_write,
    input mem_read,
    input [15:0] addr,
    input [15:0] write_data,
    output reg [15:0] read_data
);
    reg [15:0] mem [0:65535];

    always @(posedge clk) begin
        if (mem_write)
            mem[addr] <= write_data;
        if (mem_read)
            read_data <= mem[addr];
    end
endmodule

`include "alu.v"
`include "register_file.v"
`include "control_unit.v"
`include "memory.v"

module cpu (
    input clk
);
    reg [15:0] pc = 0;
    wire [18:0] instr;
    reg [18:0] instr_mem [0:255];

    initial begin
        // Example instructions: ADD r1, r2, r3
        instr_mem[0] = 19'b00000_0001_0010_0011_00; // ADD r1, r2, r3
        instr_mem[1] = 19'b00001_0100_0001_0010_00; // SUB r4, r1, r2
        instr_mem[2] = 19'b00010_0101_0001_0011_00; // MUL r5, r1, r3
    end

    wire [4:0] opcode;
    wire [3:0] rd, rs1, rs2;
    wire [9:0] imm;
    wire [15:0] read_data1, read_data2, alu_result;

    control_unit cu (
        .instr(instr_mem[pc]),
        .opcode(opcode),
        .rd(rd), .rs1(rs1), .rs2(rs2),
        .imm(imm)
    );

    register_file rf (
        .clk(clk),
        .reg_write(1'b1),
        .read_reg1(rs1),
        .read_reg2(rs2),
        .write_reg(rd),
        .write_data(alu_result),
        .read_data1(read_data1),
        .read_data2(read_data2)
    );

    alu a1 (
        .a(read_data1),
        .b(read_data2),
        .alu_op(opcode[3:0]),
        .result(alu_result)
    );

    always @(posedge clk) begin
        pc <= pc + 1;
    end
endmodule
