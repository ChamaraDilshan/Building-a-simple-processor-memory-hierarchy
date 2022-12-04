// Define the stimulus module (no ports). This is a non-synthesizable module, only to be used for simulation purposes
module Testbench;

	// Declarations of wire, reg, and other variables
	wire q, qbar;
	reg set, reset; // Insput signals to the SR Latch

	// Instantiate lower-level modules
	// In this case, instantiate SR_latch
	// Feed inverted set and reset signals to the SR latch
	SR_latch sr1(q, qbar, ~set, ~reset);

	// Behavioural block, initial
	initial
	begin
		$monitor($time, " set = %b, reset= %b, q= %b\n",set,reset,q);
		set = 0; reset = 0;
		#5 reset = 1;
		#5 reset = 0;
		#5 set = 1;
	end

endmodule


// SR_latch module. This is a synthesizable module
module SR_latch(Q, Qbar, Sbar, Rbar);

	//Port declarations
	output Q, Qbar;
	input Sbar, Rbar;

	// Instantiate lower-level modules
	// In this case, instantiate Verilog primitive nand gates
	// Note, how the wires are connected in a cross-coupled fashion.
	nand n1(Q, Sbar, Qbar);
	nand n2(Qbar, Rbar, Q);

// endmodule statement
endmodule



