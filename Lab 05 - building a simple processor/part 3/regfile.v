// tenchbench for regfile module
module regfile_tb;
    
    reg [7:0] WRITEDATA;
    reg [2:0] WRITEREG, READREG1, READREG2;
    reg CLK, RESET, WRITEENABLE; 
    wire [7:0] REGOUT1, REGOUT2;
    
    reg_file myregfile(WRITEDATA, REGOUT1, REGOUT2, WRITEREG, READREG1, READREG2, WRITEENABLE, CLK, RESET);
       
    initial
    begin
        CLK = 1'b1;
        
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("regfile.vcd");
		$dumpvars(0, regfile_tb);
        
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

// creating the register file module
module reg_file(IN, OUT1, OUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITE, CLK, RESET);
    // declare a register array to store data
    reg [7 : 0] file [0 : 7];
    
    // declaring the ports
    // inputs 
    input [7 : 0] IN;
    input [2 : 0] OUT1ADDRESS, OUT2ADDRESS, INADDRESS;
    input CLK, WRITE, RESET;

    // outputs
    output [7 : 0] OUT1, OUT2;

    integer i;

    // reading the register file
    assign #2 OUT1 = file[OUT1ADDRESS];
    assign #2 OUT2 = file[OUT2ADDRESS];

    always @(posedge CLK) 
    begin
        if (WRITE & !RESET) // writing to the register file
        begin
            #1 file[INADDRESS] <= IN;
        end

        if (RESET) // reset the register by making all the elements 0, when RESET signal is high
        begin
            for (i = 0; i < 8; i = i + 1) 
            begin
                #1 file[i] = 8'd0;
            end
        end
    end

endmodule