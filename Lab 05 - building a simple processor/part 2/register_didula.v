/*module Testbench;


	wire signed[7:0] out1;
    wire signed[7:0] out2;
	reg signed[2:0] out1add, out2add, inadd;
    reg signed[7:0] in;
    reg signed write = 0, clk = 0, reset = 0;

    reg_file r(in, out1, out2, inadd, out1add, out2add, write, clk, reset);

	initial
	begin
    $monitor($time, " out1 =%d out2 =%d out1address =%s out2address =%s\n",out1, out2, out1add, out2add);
    #5
    clk = 0;
    out1add = "reg5";
    out2add = "reg6";
    //$monitor("out1 =%d out2 =%d out1address =%s out2address =%s\n",out1, out2, out1add, out2add);
    #5
    clk = 1;
    write = 1;
    //reset = 0;
    in = 8'd11;
    inadd = "reg4";
    #5
    clk = 0;
    out1add = "reg4";
    out2add = "reg3";
    #5
    clk = 1;
    reset = 1;
    out1add = "reg2";
    out2add = "reg3";
    


    //$monitor("out1 =%d out2 =%d out1address =%s out2address =%s\n",out1, out2, out1add, out2add);

	end


endmodule*/

module reg_file_tb;
    
    reg [7:0] WRITEDATA;
    reg [2:0] WRITEREG, READREG1, READREG2;
    reg CLK, RESET, WRITEENABLE; 
    wire [7:0] REGOUT1, REGOUT2;
    
    reg_file myregfile(WRITEDATA, REGOUT1, REGOUT2, WRITEREG, READREG1, READREG2, WRITEENABLE, CLK, RESET);
       
    initial
    begin
        CLK = 1'b1;
        
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("test.vcd");
		$dumpvars(0, reg_file_tb);
        
        // assign values with time to input signals to see output 
        RESET = 1'b0;
        WRITEENABLE = 1'b0;
        
        #5
        RESET = 1'b1;
        READREG1 = 3'd0;
        READREG2 = 3'd4;
        
        #7
        RESET = 1'b0;
        
        #3
        WRITEREG = 3'd2;
        WRITEDATA = 8'd95;
        WRITEENABLE = 1'b1;
        
        #9
        WRITEENABLE = 1'b0;
        
        #1
        READREG1 = 3'd2;
        
        #9
        WRITEREG = 3'd1;
        WRITEDATA = 8'd28;
        WRITEENABLE = 1'b1;
        READREG1 = 3'd1;
        
        #10
        WRITEENABLE = 1'b0;
        
        #10
        WRITEREG = 3'd4;
        WRITEDATA = 8'd6;
        WRITEENABLE = 1'b1;
        
        #10
        WRITEDATA = 8'd15;
        WRITEENABLE = 1'b1;
        
        #10
        WRITEENABLE = 1'b0;
        
        #6
        WRITEREG = 3'd1;
        WRITEDATA = 8'd50;
        WRITEENABLE = 1'b1;
        
        #5
        WRITEENABLE = 1'b0;
        
        #10
        $finish;
    end
    
    // clock signal generation
    always
        #5 CLK = ~CLK;
        

endmodule

module reg_file(IN, OUT1, OUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITE, CLK, RESET);
    reg [7:0] Registers[0:7];
    /*reg [7:0] Values[0:7];

    initial
    begin
    Registers[0] = "reg0";
    Registers[1] = "reg1";
    Registers[2] = "reg2";
    Registers[3] = "reg3";
    Registers[4] = "reg4";
    Registers[5] = "reg5";
    Registers[6] = "reg6";
    Registers[7] = "reg7";
    end

    initial
    begin
    Values[0] = 8'd0;
    Values[1] = 8'd1; 
    Values[2] = 8'd2; 
    Values[3] = 8'd3; 
    Values[4] = 8'd4; 
    Values[5] = 8'd5; 
    Values[6] = 8'd6;
    Values[7] = 8'd7;  
    end*/


    input [7:0] IN;
    input [2:0] OUT1ADDRESS, OUT2ADDRESS, INADDRESS;
    input CLK, RESET, WRITE;
    output [7:0] OUT1, OUT2;
    integer i;

    assign #2 OUT1 = Registers[OUT1ADDRESS];
    assign #2 OUT2 = Registers[OUT2ADDRESS];


    always@(posedge CLK)
    begin
        if(WRITE & !RESET)
           #1 Registers[INADDRESS] <= IN;

        if(RESET) begin
            for(i = 0; i < 8; i += 1) begin
                #1 Registers[i] = 8'd0;
            end
        end

    end

    /*initial begin
    #100
        for(i = 0; i < 8; i += 1) begin
            $display("%d ", Values[i]);
        end
    end*/

endmodule