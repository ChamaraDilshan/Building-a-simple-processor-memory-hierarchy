module cpu_tb;

    reg CLK, RESET;
    wire [31:0] PC;
    reg [31:0] INSTRUCTION;
    
    /* 
    ------------------------
     SIMPLE INSTRUCTION MEM
    ------------------------
    */
    
    // TODO: Initialize an array of registers (8x1024) named 'instr_mem' to be used as instruction memory
    reg [7 : 0] instr_mem [1023 : 0];

    // TODO: Create combinational logic to support CPU instruction fetching, given the Program Counter(PC) value 
    //       (make sure you include the delay for instruction fetching here)
    always @(PC) 
    begin
      #2
      INSTRUCTION = {instr_mem[PC+3],instr_mem[PC+2],instr_mem[PC+1],instr_mem[PC]};
    end
    
    initial
    begin
        // Initialize instruction memory with the set of instructions you need execute on CPU
        
        // METHOD 1: manually loading instructions to instr_mem
        //{instr_mem[10'd3], instr_mem[10'd2], instr_mem[10'd1], instr_mem[10'd0]} = 32'b00000000000001000000000000000101;
        //{instr_mem[10'd7], instr_mem[10'd6], instr_mem[10'd5], instr_mem[10'd4]} = 32'b00000000000000100000000000001001;
        //{instr_mem[10'd11], instr_mem[10'd10], instr_mem[10'd9], instr_mem[10'd8]} = 32'b00000010000001100000010000000010;
        
        // METHOD 2: loading instr_mem content from instr_mem.mem file
        $readmemb("instr_mem.mem", instr_mem);
    end
    
    /* 
    -----
     CPU
    -----
    */
    cpu mycpu(PC, INSTRUCTION, CLK, RESET);

    initial
    begin
    
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("cpu_wavedata.vcd");
		$dumpvars(0, cpu_tb);
        
        CLK = 1'b0;
        RESET = 1'b0;
        
        // TODO: Reset the CPU (by giving a pulse to RESET signal) to start the program execution
        #2 RESET = 1'b1;
        #4 RESET = 1'b0;
        
        // finish simulation after some time
        #100
        $finish;
        
    end
    
    // clock signal generation
    always
        #4 CLK = ~CLK;
        

endmodule

module cpu(PC, INSTRUCTION, CLK, RESET);
    // declare the ports
    input [31 : 0] INSTRUCTION;
    input RESET, CLK;
    
    output reg [31 : 0] PC;

    // declare the registers,wires 
    reg WRITE, sub, immediate;
    reg [2 : 0] ALUOP, OUTADD1, OUTADD2, INADD;
    reg [7 : 0] IN, IMMEDIATE_VAL, OPCODE;
    wire [7 : 0] ALURESULT, minus_num, OUT1, OUT2, mux_out1, mux_out2;
    wire [31 : 0] next_PC;

    // instantiate pc_update module
    pc_update pc(PC, next_PC);

    always @(posedge CLK) 
    begin
        if(RESET)
        begin
            PC = 0; // write zero to PC when RESET is high and at a positive clock edge
        end

        else
        begin
            #1
            PC = next_PC; // write next PC value to the PC at a positive clock edge
        end
    end

    // decoding the opcode from the instruction
    always @(INSTRUCTION) 
    begin
        // take the opcode from the instruction
        #1
        OPCODE = INSTRUCTION [31 : 24];

        case (OPCODE) // use case statement to select relavent instruction
            8'b00000000:
                begin
                  WRITE = 1'b1;
                  ALUOP = 3'b000; // FORWARD instruction
                  sub = 1'b0;
                  immediate = 1'b1;
                end
            8'b00000001:
                begin
                  WRITE = 1'b1;
                  ALUOP = 3'b000; // FORWARD instruction
                  sub = 1'b0;
                  immediate = 1'b0;
                end
            8'b00000010:
                begin
                  WRITE = 1'b1;
                  ALUOP = 3'b001; // ADD instruction
                  sub = 1'b0;
                  immediate = 1'b0;
                end
            8'b00000011:
                begin
                  WRITE = 1'b1;
                  ALUOP = 3'b001; // SUB instruction
                  sub = 1'b1;
                  immediate = 1'b0;
                end
            8'b00000100:
                begin
                  WRITE = 1'b1;
                  ALUOP = 3'b010; // AND instruction
                  sub = 1'b0;
                  immediate = 1'b0;
                end 
            8'b00000101:
                begin
                  WRITE = 1'b1;
                  ALUOP = 3'b011; // FORWARD instruction
                  sub = 1'b0;
                  immediate = 1'b0;
                end
            default: 
                begin
                  WRITE = 1'b0;
                  ALUOP = 3'b111; // DEFAULT
                  sub = 1'b0;
                  immediate = 1'b0;
                end
        endcase
        
    end

    // instantiate the register file
    reg_file myregister(IN, OUT1, OUT2, INADD, OUTADD1, OUTADD2, WRITE, CLK, RESET);

    // get the addresses and immediate value from the instruction
    always @(INSTRUCTION) 
    begin
        INADD = INSTRUCTION [18 : 16];
        OUTADD1 = INSTRUCTION [10 : 8];
        OUTADD2 = INSTRUCTION [2 : 0];
        IMMEDIATE_VAL = INSTRUCTION [7 : 0];              
    end

    // instantiate minus_val module
    minus_val num(OUT2, minus_num);  

    // instantiate mux module
    mux mymux1(minus_num, OUT2, sub, mux_out1);

    mux mymux2(IMMEDIATE_VAL, minus_num, immediate, mux_out2);

    // instantiate alu module
    ALU alu(OUT1, mux_out2, ALURESULT, ALUOP);

    // setting the output of alu to the input value of register file
    always @(ALURESULT)
    begin
      IN = ALURESULT; 
    end

endmodule

// module to increament PC value by 4
module pc_update(pc_in, next_pc);
    // declare the ports
    input [31 : 0] pc_in;
    output reg [31 : 0] next_pc;

    always @(pc_in) 
    begin
        next_pc = pc_in + 4;
    end

endmodule

// function to get the 2s complement of a given values
module minus_val(DATA, RESULT);
    // declare the ports
    input [7 : 0] DATA;
    output reg [7 : 0] RESULT;

    always @(*) 
    begin
        #1
        RESULT = ~DATA + 1;
    end

endmodule

// module for 2 to 1 multiplexer
module mux(in1, in2, sel, out);
    // declare the ports
    input [7 : 0] in1, in2;
    input sel;
    output reg [7 : 0] out;

    always @(in1, in2, sel) 
    begin
      if (sel) 
      begin
        out = in1;
      end

      else
      begin
        out = in2;
      end

    end

endmodule

// creating the ALU module
module ALU(DATA1, DATA2, RESULT, SELECT);
    // declaring the ports
    // inputs
    input [7 : 0] DATA1, DATA2;
    input [2 : 0] SELECT;
    // outputs
    output reg [7 : 0] RESULT;
    wire [7 : 0] fwd_out, add_out, and_out, or_out; // to get outputs from the instances created by the function modules

    // instantiate function modules
    FORWARD fwd(DATA2, fwd_out);
    ADD add(DATA1, DATA2, add_out);
    AND and_(DATA1, DATA2, and_out);
    OR or_(DATA1, DATA2, or_out);

    // always block
    always @(*) // wrong : data1 or data2 or select
    begin
        case (SELECT)
            3'b000 : RESULT <= fwd_out;
            3'b001 : RESULT <= add_out;
            3'b010 : RESULT <= and_out;
            3'b011 : RESULT <= or_out;
            default: RESULT <= 0;
        endcase
        
    end
endmodule

// FORWARD module
module FORWARD(DATA2, fwd_out);
    // declaring the ports
    input [7 : 0] DATA2;
    output [7 : 0] fwd_out;

    assign #1 fwd_out = DATA2; // for loadi, mov instructions
endmodule

// ADD module
module ADD(DATA1, DATA2, add_out);
    // declaring the ports
    input [7 : 0] DATA1;
    input [7 : 0] DATA2;
    output [7 : 0] add_out;

    assign #2 add_out = DATA1 + DATA2; // for add, sub instructions
endmodule

// AND module
module AND(DATA1, DATA2, and_out);
    // declaring the ports
    input [7 : 0] DATA1;
    input [7 : 0] DATA2;
    output [7 : 0] and_out;

    assign #1 and_out = DATA1 & DATA2; // for and instruction

endmodule

// OR module
module OR(DATA1, DATA2, or_out);
    // declaring the ports
    input [7 : 0] DATA1;
    input [7 : 0] DATA2;
    output [7 : 0] or_out;

    assign #1 or_out = DATA1 | DATA2; // for or instruction

endmodule

// creating the register file module
module reg_file(IN, OUT1, OUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITE, CLK, RESET);
    
    reg [7:0] Registers[0:7]; //8 bit wide element array with depth of 8

    //declaration of inputs and ouputs
    input [7:0] IN;
    input [2:0] OUT1ADDRESS, OUT2ADDRESS, INADDRESS;
    input CLK, RESET, WRITE;
    output [7:0] OUT1, OUT2;

    integer i;

    //read asynchronously and load values into OUT1 and OUT2 
    assign #2 OUT1 = Registers[OUT1ADDRESS];
    assign #2 OUT2 = Registers[OUT2ADDRESS];

    //always block only teriggerd in the positive edge of clock
    always@(posedge CLK)
    begin
        if(WRITE & !RESET) begin// if write port is set to high and reset is low then IN port is write into register named as INADDRESS
           #1 Registers[INADDRESS] <= IN;
           
        end
        if(RESET) begin // if reset is high, all values in registers are written as 0
            #1
            for(i = 0; i < 8; i = i + 1) begin // make for loop for access each element in an array
                 Registers[i] <= 8'd0;
            end
        end

    end
     initial begin
     #100 $display($time, "%d %d %d %d %d %d %d %d\n", Registers[0], Registers[1], Registers[2], Registers[3], Registers[4], Registers[5], Registers[6], Registers[7]);
     end
endmodule


/*module reg_file(IN, OUT1, OUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITE, CLK, RESET);
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

endmodule */