`timescale 1ns/1ps

module dmem
#(  parameter DMEM_DEPTH = 1024,    // dmem depth (default: 1024 entries = 8 KB)
              DMEM_ADDR_WIDTH = 10 )
(
    input                           clk                     ,
    input   [DMEM_ADDR_WIDTH-1:0]   addr                    ,
    input   [31:0]                  din                     ,
    input                           mem_read                ,
    input                           mem_write               ,
    output  [31:0]                  dout
);

    // the aligned memory accesses.
    logic   [31:0]  data[0:DMEM_DEPTH-1];

    // Write operation:
    always_ff @ (posedge clk) begin
        if (mem_write)
            data[addr] <= din;
    end

    // Read operation:
    // - dout = 0 if (mem_read==0)
    assign dout = (mem_read) ? data[addr]: 'b0;

// synthesis translate_off
    initial begin
        $readmemh("dmem.mem", data);
    end
// synthesis translate_on

endmodule
