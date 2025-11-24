/*
 * Copyright (c) 2025 Tholin
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_as_scrapcpu (
	input  wire [7:0] ui_in,    // Dedicated inputs
	output wire [7:0] uo_out,   // Dedicated outputs
	input  wire [7:0] uio_in,   // IOs: Input path
	output wire [7:0] uio_out,  // IOs: Output path
	output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
	input  wire       ena,      // always 1 when the design is powered, so you can ignore it
	input  wire       clk,      // clock
	input  wire       rst_n     // reset_n - low to reset
);

assign uo_out[7] = SCLK_ROM;
assign uo_out[6] = ROM_DO;
wire ROM_DI = ui_in[7];
assign uio_out[7] = CS_ROM;
assign uio_oe[7] = 1'b1;
wire double_speed = ui_in[6];
assign uio_out[6] = 1'b0;

reg carry;
reg zero;
reg [5:0] A;
reg [5:0] B;
reg [5:0] MAR;
reg [5:0] RAM [63:0];
wire [5:0] RAMval = MAR == 63 ? uio_in[5:0] : (MAR == 60 ? ui_in[5:0] : RAM[MAR]);
reg [5:0] P;
reg [11:0] PC;
reg [5:0] insin;
reg [5:0] imm_buff;
reg compat;

reg [11:0] last_PC;
reg [1:0] last_flags;
reg [5:0] last_A;
reg [5:0] last_B;
reg [5:0] last_MAR;
reg [5:0] last_P;

wire [5:0] out_port1 = RAM[63];
wire [5:0] out_port2 = RAM[60];
wire [5:0] port_dirs = RAM[59];
assign uo_out[5:0] = out_port2;
assign uio_out[5:0] = out_port1;
assign uio_oe[5:0] = port_dirs;

wire HCF = RAM[58] == 6'h2A && double_speed;

assign uio_oe[6] = 1'b0;
wire inter = uio_in[6];
reg needs_irupt;
reg last_inter;
reg in_irupt;

reg [2:0] instr_cycle;

/*
 * Instruction decode
 */
wire needs_address = !insin[5] && insin != 6'o17 && insin != 6'o20 && !is_jump;
wire needs_immediate = insin == 6'o17 || insin == 6'o20 || insin == 6'o77 || (is_jump && !insin[5]);
wire needs_argument = needs_address || needs_immediate;
wire to_MAR = needs_address || insin == 6'o17;
wire to_P = insin == 6'o20;
wire quick = insin[4];
wire is_ALU_op = insin[2] ? !insin[3] : insin[3];
wire is_jump = (insin[2] && insin[4:3] == 2'b01) || insin[4:0] == 5'o36;

wire [5:0] instr_arg = needs_immediate ? imm_buff : RAMval;

/*
 * Combinatorials
 */

reg [6:0] ALU_out;
always @(*) begin
	case(insin[2:0])
		default: ALU_out = {carry, A};
		0: ALU_out = {carry, instr_arg == A ? 6'h01 : 6'h00};
		1: ALU_out = {carry, A > instr_arg ? 6'h01 : 6'h00};
		2: ALU_out = {carry, A ^ instr_arg};
		3: ALU_out = {carry, A & instr_arg};
		4: ALU_out = {1'b0, A} + {1'b0, instr_arg};
		5: ALU_out = {1'b0, A} + {1'b0, instr_arg} + carry;
		6: ALU_out = {1'b0, A} + {1'b0, ~instr_arg} + 7'h01;
		7: ALU_out = {1'b0, A} + {1'b0, ~instr_arg} + carry;
	endcase
end

reg should_jump;
always @(*) begin
	case(insin[1:0])
		default: should_jump = 1'b1;
		1: should_jump = zero;
		2: should_jump = insin[4] ? carry : !zero;
	endcase
end

wire [5:0] rsh_result = {insin == 6'o23 ? carry : 1'b0, A[5:1]};

/*
 * Regs & IO for spiflash memory interface
 */
reg [4:0] ROM_spi_cycle;
reg [2:0] mem_cycle;
reg CS_ROM;
reg SCLK_ROM;
reg ROM_DO;
reg [7:0] ROM_spi_dat_out;
reg [11:0] ROM_addr_buff;
reg [11:0] last_addr;
reg spi_clkdiv;
reg [1:0] ROM_dest;
/*
 *
 */

always @(posedge clk) begin
	if(rst_n) begin
		if(inter && !last_inter) begin
			needs_irupt <= 1;
		end
		last_inter <= inter;
	end
	if(!rst_n) begin
		A <= 0;
		B <= 0;
		MAR <= 0;
		P <= 0;
		PC <= 0;
		insin <= 0;
		instr_cycle <= 0;
		carry <= 0;
		zero <= 0;
		compat <= 1;
		needs_irupt <= 0;
		last_inter <= 0;
		in_irupt <= 0;
		RAM[63] <= 0;
		RAM[60] <= 0;
		RAM[59] <= 6'h3F;
		RAM[58] <= 0;
	end else if(mem_cycle == 0 && ROM_spi_cycle == 0) begin
		if(instr_cycle == 0) begin
			if(needs_irupt && !in_irupt) begin
				last_PC <= PC;
				last_MAR <= MAR;
				last_A <= A;
				last_B <= B;
				last_P <= P;
				last_flags <= {zero, carry};
				PC <= 12'h004;
				ROM_addr_buff <= 12'h004;
				in_irupt <= 1;
				needs_irupt <= 0;
			end else begin
				ROM_addr_buff <= PC;
				PC <= PC + 1;
			end
			mem_cycle <= 1;
			instr_cycle <= 1;
			ROM_dest <= 0;
		end else if(instr_cycle == 1) begin
			if(insin == 6'o62) begin
				PC <= last_PC;
				MAR <= last_MAR;
				A <= last_A;
				B <= last_B;
				P <= last_P;
				zero <= last_flags[1];
				carry <= last_flags[0];
				instr_cycle <= 0;
				in_irupt <= 0;
			end else if(insin == 6'o61) begin
				compat <= !compat;
				instr_cycle <= 0;
			end else if(insin == 6'o40) begin
				carry <= 0;
				instr_cycle <= 0;
			end else if(insin == 6'o21) begin
				carry <= 1;
				instr_cycle <= 0;
			end else if(insin == 6'o22 || insin == 6'o23) begin
				B <= rsh_result;
				carry <= A[0];
				zero <= rsh_result == 0;
				instr_cycle <= 0;
			end else if(insin == 0) begin
				instr_cycle <= 0;
			end else if(needs_argument) begin
				instr_cycle <= insin == 6'o17 || to_P ? 0 : 2;
				ROM_addr_buff <= PC;
				mem_cycle <= 1;
				PC <= PC + 1;
				ROM_dest <= to_MAR ? 1 : (to_P ? 3 : 2);
			end else begin
				instr_cycle <= 2;
			end
		end else if(instr_cycle == 2) begin
			instr_cycle <= 0;
			if(insin[4:0] == 5'o01 || insin == 6'o77) begin
				A <= instr_arg;
				if(!compat && insin[4:0] == 5'o01) zero <= instr_arg == 0;
			end
			if(insin[4:0] == 5'o07) begin
				MAR <= instr_arg;
			end
			if(insin[4:0] == 5'o20) begin
				P <= instr_arg;
			end
			if(insin[4:0] == 5'o02) begin
				RAM[MAR] <= B;
			end
			if(insin[4:0] == 5'o03) begin
				RAM[MAR] <= A;
				if(compat) begin
					B <= A;
					carry <= 0;
					zero <= A == 0;
				end
			end
			if(is_ALU_op) begin
				B <= ALU_out[5:0];
				carry <= ALU_out[6];
				zero <= ALU_out[5:0] == 0;
				if(!quick) A <= ALU_out[5:0];
			end
			if(is_jump && should_jump) begin
				PC <= {P, instr_arg};
			end
		end
	end
	
	/*
	 * spiflash memory interface
	 */
	if(!rst_n) begin
		ROM_spi_cycle <= 5'h00;
		mem_cycle <= 3'h1;
		CS_ROM <= 1'b1;
		SCLK_ROM <= 1'b0;
		ROM_DO <= 0;
		ROM_spi_dat_out <= 8'h00;
		ROM_spi_cycle <= 5'h00;
		ROM_addr_buff <= 0;
		last_addr <= 0;
		spi_clkdiv <= 0;
		ROM_dest <= 0;
	end else if(ROM_spi_cycle != 0) begin
		spi_clkdiv <= !spi_clkdiv;
		if(spi_clkdiv || double_speed) begin
			ROM_spi_cycle <= ROM_spi_cycle == 17 ? 0 : ROM_spi_cycle + 1;
			if(ROM_spi_cycle[0]) begin
				SCLK_ROM <= 1'b0;
				ROM_DO <= ROM_spi_dat_out[7];
				ROM_spi_dat_out <= {ROM_spi_dat_out[6:0], ROM_DI};
			end else begin
				SCLK_ROM <= 1'b1;
			end
		end
	end else if(mem_cycle != 0) begin
		mem_cycle <= mem_cycle + 1;
		case(mem_cycle)
			1: begin
				last_addr <= ROM_addr_buff;
				if(last_addr + 1 == ROM_addr_buff) begin
					//Sequential read, just clock the flash a few times to get it
					mem_cycle <= 6;
				end else begin
					//Non-sequential read, need to restart spiflash read with a new address
					CS_ROM <= 1;
					SCLK_ROM <= 0;
				end
			end
			2: begin
				CS_ROM <= 0;
				ROM_spi_dat_out <= 8'h03;
				ROM_spi_cycle <= 1;
			end
			3: begin
				ROM_spi_dat_out <= 8'h00;
				ROM_spi_cycle <= 1;
			end
			4: begin
				ROM_spi_dat_out <= {4'h0, ROM_addr_buff[11:8]};
				ROM_spi_cycle <= 1;
			end
			5: begin
				ROM_spi_dat_out <= ROM_addr_buff[7:0];
				ROM_spi_cycle <= 1;
			end
			6: begin
				ROM_spi_dat_out <= 8'h00;
				ROM_spi_cycle <= 1;
			end
			7: begin
				if(ROM_dest == 0) insin <= ROM_spi_dat_out[5:0];
				else if(ROM_dest == 1) MAR <= ROM_spi_dat_out[5:0];
				else if(ROM_dest == 2) imm_buff <= ROM_spi_dat_out[5:0];
				else P <= ROM_spi_dat_out[5:0];
			end
		endcase
	end
end

endmodule
