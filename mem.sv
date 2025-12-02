module mem (
  input        clk,
  input        we,
  input  [31:0] a,    // byte address
  input  [31:0] wd,   // write data (32-bit word, bytes positioned as requested)
  output [31:0] rd,   // read word (word aligned)
  input  [3:0]  wm    // byte write mask, wm[0] -> least-significant byte
);

  // word-addressable memory (32-bit words)
  reg [31:0] RAM [0:255];

  // initialize memory with instructions/data (riscv.hex must contain words)
  initial begin
    $readmemh("riscv.hex", RAM);
  end

  // read returns the whole word (word-aligned using address[31:2])
  assign rd = RAM[a[31:2]];

  // synchronous write with byte-enable mask
  reg [31:0] cur;
  reg [31:0] mask;
  always @(posedge clk) begin
    if (we) begin
      cur  = RAM[a[31:2]];
      mask = { {8{wm[3]}}, {8{wm[2]}}, {8{wm[1]}}, {8{wm[0]}} };
      RAM[a[31:2]] <= (cur & ~mask) | (wd & mask);
    end
  end
endmodule
