module testbench;
    reg a,b,c;
    wire x,y;
    
    combinational mycircuit(a,b,c,x,y);

    initial
    begin
        a = 1'b1;
        b = 1'b1;
        c = 1'b0; 
    end

    initial
    begin
        $monitor("TIME = %g a = %b b = %b c = %b x = %b y = %b", $time,a,b,c,x,y);
        #5 a = 1'b0;
        #5 $finish;
    end
endmodule


module combinational(a,b,c,x,y);
    input a,b,c;
    output reg x,y;

    always @ (a,b,c)
    begin
        x = a & b;
        y = x | c;
    end
endmodule