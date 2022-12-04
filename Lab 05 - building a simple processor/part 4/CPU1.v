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
    reg [7:0] instr_mem[0:1023];

    // TODO: Create combinational logic to support CPU instruction fetching, given the Program Counter(PC) value 
    //       (make sure you include the delay for instruction fetching here)

    always @(PC) begin
        #2
        INSTRUCTION = {instr_mem[PC+3], instr_mem[PC+2], instr_mem[PC+1], instr_mem[PC]};
    end
    
    initial
    begin
        // Initialize instruction memory with the set of instructions you need execute on CPU
        
        // METHOD 1: manually loading instructions to instr_mem
        // {instr_mem[10'd3], instr_mem[10'd2], instr_mem[10'd1], instr_mem[10'd0]} = 32'b00000000000001000000000000000101;
        // {instr_mem[10'd7], instr_mem[10'd6], instr_mem[10'd5], instr_mem[10'd4]} = 32'b00000000000000100000000000001001;
        // {instr_mem[10'd11], instr_mem[10'd10], instr_mem[10'd9], instr_mem[10'd8]} = 32'b00000010000001100000010000000010;
        
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
        #2
        RESET = 1'b1;

        #4
        RESET= 1'b0;
        
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
    output reg [31:0] PC;
    input [31:0] INSTRUCTION;
    input CLK, RESET;

    // declare the registers and wires
    reg [2:0] SEL;
    wire [7:0] OUT1, OUT2, NEGATIVE, OUT3, OUT4;
    wire [7:0] RESULT;
    reg [2:0] DESTINATION, SOURCE1, SOURCE2;
    reg WRITE , isNEGATIVE, isIMMEDIATE, branch, isJUMP = 0;
    wire [31:0] NEXT, extended_out, offset, branched_pc, new_pc, pc_out;
    wire ZERO, and_out;



    //instantiate all modules
    pc_adder add(PC, NEXT);
    reg_file a0(RESULT, OUT1, OUT2, DESTINATION, SOURCE1, SOURCE2, WRITE, CLK, RESET);
    twosCompliment t1(OUT2, NEGATIVE); // convert positive value to negative value
    selectMUX8 m1(NEGATIVE, OUT2, OUT3, isNEGATIVE); //choose negative value for sub and positive value for add
    selectMUX8 m2(INSTRUCTION[7:0], OUT3, OUT4, isIMMEDIATE); //choose immediate value or register value 
    alu a2(OUT1, OUT4, RESULT, SEL, ZERO);
    AND_gate a(ZERO, branch, and_out);
    selectMUX32 m3(branched_pc, NEXT, pc_out, and_out);
    sign_extend se(INSTRUCTION[23 : 16], extended_out);
    shift_left2 sl2(extended_out, offset);
    branch_adder ba(NEXT, offset, branched_pc);
    selectMUX32 m4(branched_pc, pc_out, new_pc, isJUMP);

    always @(posedge CLK) 
    begin
        #1 PC = new_pc; // write next PC value to the PC at a positive clock edge

        if(RESET) PC = 32'd0; // write zero to PC when RESET is high and at a positive clock edge 
        
    end

    //check the opcode and decode
    always @(INSTRUCTION) begin
        // get input and ouput addresses to the register file
        DESTINATION = INSTRUCTION[18:16]; // INADDRESS
        SOURCE1 = INSTRUCTION[10:8];      // OUTADDRESS1
        SOURCE2 = INSTRUCTION[2:0];       // OUTADDRESS2

        #1
        case(INSTRUCTION[31:24]) 
            8'd0 : 
                begin
                    WRITE = 1;
                    isNEGATIVE = 1'b0;
                    isIMMEDIATE = 1'b1;
                    SEL = 3'b000; // FORWARD
                    branch = 0;

                end
            8'd1 : 
                begin
                    WRITE = 1;
                    isNEGATIVE = 1'b0;
                    isIMMEDIATE = 1'b0;
                    SEL = 3'b000; // MOV
                    branch = 0;

                end
            8'd2 : 
                begin
                    WRITE = 1;
                    isNEGATIVE = 1'b0;
                    isIMMEDIATE = 1'b0;
                    SEL = 3'b001; // ADD
                    branch = 0;
                    
                end
            8'd3 : 
                begin
                    WRITE = 1;
                    isNEGATIVE = 1'b1;
                    isIMMEDIATE = 1'b0;
                    SEL = 3'b001; // SUB
                    branch = 0;
                    
                end
            8'd4 : 
                begin
                    WRITE = 1;
                    isNEGATIVE = 1'b0;
                    isIMMEDIATE = 1'b0;
                    SEL = 3'b010; // AND
                    branch = 0;
                    
                end
            8'd5 : 
                begin
                    WRITE = 1;
                    isNEGATIVE = 1'b0;
                    isIMMEDIATE = 1'b0;
                    SEL = 3'b011; // OR
                    branch = 0;
                    
                end
            8'd6 :
                begin
                    isJUMP = 1;
                    WRITE = 1;
                    isNEGATIVE = 1'b0;
                    isIMMEDIATE = 1'b0;
                    SEL = 3'b100; // jump
                    branch = 1;

                end
            8'd7:
                begin
                    WRITE = 1;
                    isNEGATIVE = 1'b1;
                    isIMMEDIATE = 1'b0;
                    SEL = 3'b001; // beq 
                    branch = 1;
                end
            
        endcase
    
    end

    

endmodule


 //module for 2's compliment
module twosCompliment(DATA, OUT);
   //declare input and output
   input [7:0] DATA;
   output [7:0] OUT;

   assign #1 OUT = ~DATA + 1;
endmodule

// select mux for 8 bit inputs
module selectMUX8(IN1, IN2, OUT, SELECT);
    input [7:0] IN1, IN2;
    input SELECT;
    output reg [7:0] OUT;

    always@(*) begin
        case(SELECT)
            1'b1: OUT <= IN1;
            1'b0: OUT <= IN2;
            default: OUT <= 8'd0;
        endcase
    end
endmodule

// module to increament PC value by 4
module pc_adder(IN, OUT);
    input [31:0] IN; // input address/ current instruction
    output [31:0] OUT; // output address/ next instruction

    assign #1 OUT = IN + 32'd4;
  
endmodule

// module for and gate
module AND_gate(in1,in2,out);
    input in1, in2;
    output reg out = 0;

    always @(*)
    begin
        if(in1 & in2)
        begin
          out = 1;
        end
    end

endmodule

// new pc adder module for j and beq instructions
module branch_adder(pcNext, offset, out);
    input [31:0] pcNext, offset;
    output [31:0] out;

    assign #2 out = pcNext + offset;

endmodule

// module for sign extend from 8 bit to 32 bit
module sign_extend(in, out);
    input [7 : 0] in;
    output [31 : 0] out;

    assign out[7 : 0] = in[7 : 0];
    assign out[31 : 8] = {24{in[7]}}; 

endmodule

// module for shift a given input by 2
module shift_left2(in, out);
    input [31 : 0] in;
    output [31 : 0] out;

    assign out = in << 2;

endmodule

// select mux for 32 bit inputs
module selectMUX32(IN1, IN2, OUT, SELECT);
    input [31:0] IN1, IN2;
    input SELECT;
    output reg [31:0] OUT;

    always@(*) begin
        case(SELECT)
            1'b1: OUT <= IN1;
            1'b0: OUT <= IN2;
            default: OUT <= 8'd0;
        endcase
    end
endmodule


//reg_file module
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
    /* initial begin
     #100 $display($time, "%d %d %d %d %d %d %d %d\n", Registers[0], Registers[1], Registers[2], Registers[3], Registers[4], Registers[5], Registers[6], Registers[7]);
     end */
endmodule



module alu(DATA1, DATA2, RESULT, SELECT, ZERO);

  //declaration for inputs and output ports
  input [7:0] DATA1;
  input [7:0] DATA2;
  input [2:0] SELECT;
  output reg [7:0] RESULT;
  output reg ZERO = 0;

  //declaration of wires that hold vlues of each operation
  wire [7:0]a, b, c, d;

  //instantiate modules
  FORWARD ins1(DATA2, a);
  ADD ins2(DATA1, DATA2, b);
  AND ins5(DATA1, DATA2, c);
  OR ins6(DATA1, DATA2, d);

    always @(b) 
    begin
      if(b == 0)
      begin
        ZERO = 1;
      end
    end
    always@(DATA1, DATA2, SELECT)
    begin
      case(SELECT)
      //doing corresponding operation for each select value 
      3'b000 : assign RESULT = a; // RESULT = DATA2
      3'b001 : assign RESULT = b; //RESULT = DATA1 + DATA2
      3'b010 : assign RESULT = c; //RESULT = DATA1 & DATA2
      3'b011 : assign RESULT = d; //RESULT = DATA1 | DATA2
      default: assign RESULT = 8'd0; //in default RESULT is 0
      endcase


    end
    
endmodule

//FORWARD module
module FORWARD(DATA, OUT);

  //declare input and output
  input [7:0] DATA;
  output [7:0] OUT;

  assign #1 OUT = DATA;
endmodule


//module for addition
 module ADD(DATA1, DATA2, OUT);
//declare inputs and output
   input [7:0] DATA1;
   input [7:0] DATA2;
   output [7:0] OUT;

  assign #2 OUT = DATA1 + DATA2;

 endmodule


//module for and operation
module AND(DATA1, DATA2, OUT);
//declare inputs and output
   input [7:0] DATA1;
   input [7:0] DATA2;
   output [7:0] OUT;

  assign #1 OUT = DATA1 & DATA2;

 endmodule

//module for or operation
module OR(DATA1, DATA2, OUT);
  //declare inputs and output
   input [7:0] DATA1;
   input [7:0] DATA2;
   output [7:0] OUT;

  assign #1 OUT = DATA1 | DATA2;

 endmodule