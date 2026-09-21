`timescale 1ns/1ps
module tb;
  reg clk=0, resetn=0;
  always #5 clk=~clk;
  wire mem_valid, mem_instr; wire mem_ready; wire [31:0] mem_addr, mem_wdata; wire [3:0] mem_wstrb; reg [31:0] mem_rdata;
  reg [31:0] mem [0:32767];
  integer i; reg [1023:0] fn;
  initial begin for(i=0;i<32768;i=i+1) mem[i]=0; if(!$value$plusargs("hex=%s",fn)) fn="fw.hex"; $readmemh(fn, mem); end
`ifdef WS1
  reg rdy=0; always @(posedge clk) rdy <= mem_valid && !rdy;
  assign mem_ready = rdy;
`else
  assign mem_ready = mem_valid;
`endif
  always @* mem_rdata = mem[mem_addr[16:2]];
  always @(posedge clk) if (mem_valid && mem_ready) begin
    if (mem_addr < 32'h20000) begin
      if (mem_wstrb[0]) mem[mem_addr[16:2]][ 7: 0] <= mem_wdata[ 7: 0];
      if (mem_wstrb[1]) mem[mem_addr[16:2]][15: 8] <= mem_wdata[15: 8];
      if (mem_wstrb[2]) mem[mem_addr[16:2]][23:16] <= mem_wdata[23:16];
      if (mem_wstrb[3]) mem[mem_addr[16:2]][31:24] <= mem_wdata[31:24];
    end else if (mem_addr==32'h10000000) $write("%c", mem_wdata[7:0]);
    else if (mem_addr==32'h10000004) $display("%0d", mem_wdata);
    else if (mem_addr==32'h10000008) $finish;
  end
  picorv32 #(.ENABLE_COUNTERS(1),.ENABLE_COUNTERS64(1),.ENABLE_REGS_DUALPORT(1),.BARREL_SHIFTER(1),
             .ENABLE_FAST_MUL(1),.ENABLE_DIV(1),.COMPRESSED_ISA(0),.ENABLE_IRQ(0),.PROGADDR_RESET(0),.STACKADDR(32'h20000))
   cpu(.clk(clk),.resetn(resetn),.mem_valid(mem_valid),.mem_instr(mem_instr),.mem_ready(mem_ready),
       .mem_addr(mem_addr),.mem_wdata(mem_wdata),.mem_wstrb(mem_wstrb),.mem_rdata(mem_rdata));
  initial begin #100 resetn=1; #2000000000 $display("TIMEOUT"); $finish; end
  reg [63:0] cyc=0; always @(posedge clk) cyc<=cyc+1;
  integer tf; reg tracing=0; initial tf=$fopen("trace.txt","w");
  always @(posedge clk) if (mem_valid && mem_ready) begin
    if (mem_addr==32'h1000000C) tracing <= mem_wdata[0];
    if (tracing && mem_instr) $fwrite(tf,"%0d %h\n",cyc,mem_addr);
  end
endmodule
