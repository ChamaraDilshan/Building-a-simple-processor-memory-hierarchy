// Computer Architecture (CO224) - Verilog Delays
// Design: Sample testbench module to test verilog delays
// Author: Mahendra Bandara


module testbench;
reg [3:0] a,b;
reg ci;
wire [3:0] sum;
wire co;
reg clk;


/*
Use one adder module at a time. Comment out other 
Don't simultaneously run multiple adder modules that will give erroneous results
*/

adder_assign adder_assign(co, sum, a, b, ci);
//adder_nonb_lhs adder_non_blocking_lhs(co, sum, a, b, ci);
//adder_nonb_rhs adder_non_blocking_rhs(co, sum, a, b, ci);
//adder_b_lhs adder_blocking_lhs(co, sum, a, b, ci);
//adder_b_rhs adder_blocking_rhs(co, sum, a, b, ci);


	initial
	begin
		a = 0;
		b = 0;
		ci = 0;
	
		#15 a = 4'd10;
		#2 b = 4'd3;
		#4 a = 4'd2;
		#2 a = 4'd15;
		
		#27 $finish;
	end

	initial
	begin
		$dumpfile("wavedata.vcd");
		$dumpvars(0,testbench);
	end

endmodule



module adder_b_lhs (co, sum, a, b, ci);
	output co;
	output [3:0] sum;
	input [3:0] a, b;
	input ci;
	reg co;
	reg [3:0] sum;
 
	always @(a or b or ci)
	begin
		#12 {co, sum} = a + b + ci;
	end

endmodule

module adder_b_rhs (co, sum, a, b, ci);
	output co;
	output [3:0] sum;
	input [3:0] a, b;
	input ci;
	reg co;
	reg [3:0] sum;
 
	always @(a or b or ci)
	begin
		{co, sum} = #12 a + b + ci;
	end

endmodule

module adder_nonb_lhs (co, sum, a, b, ci);
	output co;
	output [3:0] sum;
	input [3:0] a, b;
	input ci;
	reg co;
	reg [3:0] sum;
 
	always @(a or b or ci)
	begin
		#12 {co, sum} <= a + b + ci;
	end

endmodule

module adder_nonb_rhs (co, sum, a, b, ci);
	output co;
	output [3:0] sum;
	input [3:0] a, b;
	input ci;
	reg co;
	reg [3:0] sum;
 
	always @(a or b or ci)
	begin
		{co, sum} <= #12 a + b + ci;
	end

endmodule

module adder_assign (co, sum, a, b, ci);
	output co;
	output [3:0] sum;
	input [3:0] a, b;
	input ci;
 
	assign	#12 {co, sum} = a + b + ci;
	

endmodule