module testbench;
    reg in0,in1,sel;
    wire out;

    mux mymux(in0,b,sel,out);

    initial
    begin
      $monitor($time," in0: %b, in1: %b, sel: %b, out: %b",in0,in1,sel,out);
      $dumpfile("wavedata.vcd");
      $dumpvars(0,mymux);
    end

    initial
    begin
      in0= 1'b1;
      in1= 1'b0;
      sel = 1'b0;

      #10
      sel = 1'b1;

      #5
      in0= 1'b0;
      in1= 1'b1;
    end
endmodule

module mux(in0,in1,sel,out);

input in0,in1,sel;
output out;

/*reg out;

always @ (in0,in1,sel)
begin
  if(sel == 1'b0)
  begin
    out = in0;
  end
  else
  begin
    out = in1;
  end
end*/

assign out = sel ? in1: in0;

endmodule