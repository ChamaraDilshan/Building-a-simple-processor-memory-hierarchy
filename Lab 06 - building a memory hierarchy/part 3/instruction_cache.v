`timescale 1ns/100ps
module instructcache(clock,reset,pc,instcache_busywait,instruct_read,instruct_address,instruct_readdata,instruct_busywait,instruction);

input clock,reset;
input [10:0] pc;                          //input the least 11 bits from PC (fetched 11 bits so that most significant bit will decide if its a positive or negative value (check when pc = -4))
input instruct_busywait;                  //signal from instruction memory to keep instcache in read state until instruction read is complete 
input [127:0] instruct_readdata;

output reg instruct_read;                 //output read signal to the instruction memory  
output reg [5:0] instruct_address;        //output address for fetching 16 byte block from instruction memory 
output [31:0] instruction;                //output fetched instruction according to PC and offset to the cpu
output reg instcache_busywait;            //signal to stall/not stall the cpu when PC is updated(for accessing instruction memory cpu needs to stall)

reg signed [10:0] address;                //store fetched pc in a signed reg address (signed reg can decide if pc = -4 when comparing, line 34,35)
reg [127:0] inst_cache [0:7];             //instruction cache memory

reg [127:0] inst_data;                    //store data from the current address block(16 bytes)
reg [2:0] tag_array [0:7];                //arrays to store tag and valid status of each block
reg [7:0] valid_array;                    

wire valid;                               //valid,hit,tagMatch signals
wire hit;
wire [2:0] tag,index;                     // store current index,tag by address spliting
wire tagMatch;

/*
    Combinational part for indexing, tag comparison for hit deciding, etc.
    ...
*/

//when a new pc comes change busywait accordingly
always @(pc)
begin
  address = pc;
  if(address == -4)                     //to not stall the pc if pc = -4
    instcache_busywait = 0;
  else
    instcache_busywait = 1;
end

assign index = address[6:4];          //index part of current address

always @(*)
begin
  #1;
  inst_data = inst_cache[index];          //fetch the corresponding block from instcache according to index 
end

// assigning tag,valid status
assign #1 tag = tag_array[index];
assign #1 valid = valid_array[index];

//tagMatch and finding if a hit or not (check whether needed instruction is in the cache or not)
assign #0.9 tagMatch = (tag == address[9:7]) ? 1 : 0;
assign hit = valid && tagMatch;

//read the requested instruction, according to address(pc) offset and send to the CPU 
assign #1 instruction = ((address[3:2] == 2'b00) && hit)? inst_data[31:0]:
                        ((address[3:2] == 2'b01) && hit)? inst_data[63:32]:
                        ((address[3:2] == 2'b10) && hit)? inst_data[95:64]:inst_data[127:96];

/*
always @(*)
begin
  if(hit==1)
  begin
    case(address[3:2])
        2'b00: #1 instruction = inst_data[31:0];
        2'b01: #1 instruction = inst_data[63:32];
        2'b10: #1 instruction = inst_data[95:64];
        default: #1 instruction = inst_data[127:96];
    endcase
    busywait = 0;
  end
end
*/
integer i;
//resetting the instruction cache when a reset is occured
always @(posedge reset)
begin
  for(i=0; i<8; i = i+1)
  begin
    inst_cache[i] = 0;
    tag_array[i] = 0;
    valid_array[i] = 0;
  end
end
//if its an instruction hit de assert the busywait on posedge clock
always @(posedge clock)
begin
  if(hit)
    instcache_busywait = 0;
end


/* Instruction Cache Controller FSM Start */
parameter IDLE = 2'b00,INST_READ = 2'b01, UPDATE_CACHE = 2'b10;
reg [2:0] state, next_state;

//combinational next state logic
always @(*)
begin
  case(state)
    IDLE:
        if((address != -4) && !hit)
            next_state = INST_READ;       //go to instruction read state 
        else
            next_state = IDLE;

    INST_READ:
        if(!instruct_busywait)
            next_state = UPDATE_CACHE;      //when busywait for fetching instruction block from memory de-asserts go to update cache
        else
            next_state = INST_READ;

    UPDATE_CACHE:
        next_state = IDLE;
  endcase
end

//combinational output logic
always @(state)
begin
  case(state)
    IDLE:
    begin
      instruct_read = 0;
      instruct_address = 6'bx;  
      //busywait = 0;
    end

    INST_READ:
    begin                             //access instruction memory address by sending read signal and address
      instruct_read = 1;        
      instruct_address = {address[9:4]};
    end

    UPDATE_CACHE:
    begin
      instruct_read = 0;
      instruct_address = 6'bx;
      //update inst_cache with the newly fetched instruction block with a #1 time delay
      #1  
      inst_cache[index] = instruct_readdata;
      tag_array[index] = address[9:7];
      valid_array[index] = 1;
    end
  endcase
end


//sequential logic for state transitioning
always @(posedge clock,reset)
begin
  if(reset)
    state = IDLE;
  else
    state = next_state;
end

/* Instruction Cache FSM End*/

endmodule