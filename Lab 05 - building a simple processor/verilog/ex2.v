module Testbench;
    // Declarations of wire, reg, and other variables
    reg d = 0, clk = 0;
    wire q;
   
    // Instantiate lower-level modules
    // In this case, instantiate D_flipflop
    D_flipflop Dff(q, d, clk);

    // clock running
    always begin
        clk = ~clk;
        #10;
    end

    // behavioral block, initial
    initial begin
        $monitor ($time ," clk = %d  d = %d  q = %d", clk, d, q);
        #10; d = 0;
        #10; d = 1;
        #10; d = 0;
        #10; d = 0;
        $finish;
    end

// endmodule statement
endmodule

module D_flipflop(q, d, clk);

    // declaration of ports
    input d, clk;
    output reg q;

    // behavioral logic
    always @(posedge clk)
    begin
        q = d;
    end

// endmodule statement
endmodule