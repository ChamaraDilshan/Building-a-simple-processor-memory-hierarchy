module Testbench;

    // Declarations of wire, reg, and other variables
    wire q, qbar;
    reg d, clk;

    // Instantiate lower-level modules
    // In this case, instantiate D_flipflop
    D_flipflop Dff(q, qbar, q, qbar, d, clk);

    // behavioral block, initial
    initial
    begin
        $monitor($time, " clk = %d  d = %d  q = %d  qbar = %d", clk, d, q, qbar);
        clk = 0; d = 0;
        #5

        #5 d = 1;
        #5 clk = 1; d = 0;
        #5 d = 1;

    end

// endmodule statement
endmodule

module D_flipflop(q, qbar, qnext, qbarnext, d, clk);

    // declaration of ports
    input d, clk;
    inout q, qbar, qnext, qbarnext;
    wire x, y, dbar;

	// Instantiate lower-level modules
    // In this case, instantiate Verilog primitive nand gates
    not not1(dbar, d);
    and and1(x, dbar, clk);
    and and2(y, d, clk);
    nor nr1(qnext, x, qbar);
    nor nr2(qbarnext, y, q);

// endmodule statement    
endmodule