`ifndef BROM_MMIO_V
`define BROM_MMIO_V

`timescale 1ns/1ps

`ifndef BROM_DEFAULT_SIZE
`define BROM_DEFAULT_SIZE 4096                              // 4KB default boot ROM size
`endif

`ifndef BROM_INIT_FILE
`define BROM_INIT_FILE "bootrom.hex"
`endif

module mrom_mmio #(
    parameter [31:0]  BROM_ADDR     = 32'h2000_0000,        // Boot ROM base address
    parameter [31:0]  BASE_ADDR     = 32'h8100_5000,        // Boot ROM csr address
    parameter [31:0]  ROM_SIZE      = `BROM_DEFAULT_SIZE,   // ROM size in bytes
    parameter         INIT_FILE     = `BROM_INIT_FILE,      // Initialization file
    parameter         READ_ONLY     = 1'b1                  // Read-only memory
)(
    input  wire                     clk,
    input  wire                     resetn,

    input  wire                     mem_valid,
    input  wire                     mem_instr,
    output reg                      mem_ready,
    input  wire [31:0]              mem_addr,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [31:0]              mem_wdata,
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire [3:0]               mem_wstrb,
    output reg  [31:0]              mem_rdata,

    output reg                      irq,
    input  wire                     eoi
);

    localparam [31:0]
      RW_BROM_STATUS    = BASE_ADDR + 32'h00,
      RW_BROM_CTRL      = BASE_ADDR + 32'h04,
      RO_BROM_SIZE_REG  = BASE_ADDR + 32'h08;

    localparam ADDR_WIDTH = $clog2(ROM_SIZE);
    localparam NUM_OF_CSR = 3;

    reg [31:0] ctrl_reg;
    reg [31:0] status_reg;
    reg [7:0]  rom_mem [0:(ROM_SIZE/4)-1];

    wire [31:0] wmask = { {8{mem_wstrb[3]}}, {8{mem_wstrb[2]}}, {8{mem_wstrb[1]}}, {8{mem_wstrb[0]}} };
    wire [31:0] wdata = mem_wdata & wmask;

    wire [31:0] rom_addr = mem_addr - BROM_ADDR;

    wire is_mem_area = (mem_addr >= BROM_ADDR && mem_addr < BASE_ADDR);
    wire is_csr_area = (mem_addr >= BASE_ADDR);
    wire is_mem_oom  = (is_mem_area && mem_addr > BROM_ADDR+(ROM_SIZE/4)-1);
    wire is_csr_oom  = (is_csr_area && mem_addr > BASE_ADDR+4*NUM_OF_CSR-1);

    wire ctrl_rom_en         = ctrl_reg[0];
    wire ctrl_error_irq_en   = ctrl_reg[1];        // Error IRQ enable
    wire ctrl_access_irq_en  = ctrl_reg[2];        // Access IRQ enable, for trace or debug

    wire status_access_error = status_reg[0];      // Access error (write to read-only)
    wire status_addr_error   = status_reg[1];      // Address out of range

    function [31:0] zext32_8;
        input [7:0] in;
        begin
            zext32_8 = {24'b0, in};
        end
    endfunction

    integer i;

    initial begin
        if (INIT_FILE != "") begin
            $readmemh(INIT_FILE, rom_mem);
        end else begin
            for (i = 0; i < ROM_SIZE/4; i = i + 1) begin
                rom_mem[i] <= 8'h00;
            end
            status_reg[2] <= 1'b1;
        end
    end

    always @(posedge clk) begin
        if (!resetn) begin
            mem_ready <= 0;
        end
        mem_ready <= mem_valid && !mem_instr;
    end

    always @(posedge clk) begin: MMIO_READ
        if (!resetn) begin
            mem_rdata <= 0;
        end else if (mem_valid && (!mem_instr) && mem_wstrb == 4'b0000) begin
            if (is_mem_area && !is_mem_oom) begin
                if ((mem_addr & 32'h3) == 0) begin
                    mem_rdata <= {rom_mem[rom_addr+3], rom_mem[rom_addr+2],
                                 rom_mem[rom_addr+1], rom_mem[rom_addr]};
                end else begin
                    mem_rdata <= 32'b0;
                    status_reg[1] <= 1'b1;  // Non-aligned access error
                end
            end else if (is_csr_area && !is_csr_oom) begin
                case (mem_addr)
                    RW_BROM_STATUS:    mem_rdata <= status_reg;
                    RW_BROM_CTRL:      mem_rdata <= ctrl_reg;
                    RO_BROM_SIZE_REG:  mem_rdata <= ROM_SIZE;
                    default:           mem_rdata <= 32'b0;
                endcase
            end else begin
                mem_rdata <= 32'b0;
                status_reg[1] <= 1'b1;
            end
        end else begin
            mem_rdata <= 32'b0;
        end
    end

    always @(posedge clk) begin: MMIO_WRITE
        if (!resetn) begin
            ctrl_reg <= 32'b1;
            status_reg[1:0] <= 2'b00;
        end else begin
            if (mem_valid && (!mem_instr) && mem_wstrb != 4'b0000) begin
                if (is_mem_area && !is_mem_oom) begin
                    if (READ_ONLY) begin
                        status_reg[0] <= 1'b1;      // Attempt to write to read-only ROM
                    end else if (ctrl_rom_en) begin
                        if ((mem_addr & 32'h3) == 0) begin
                            if (mem_wstrb[0]) rom_mem[rom_addr]   <= wdata[7:0];
                            if (mem_wstrb[1]) rom_mem[rom_addr+1] <= wdata[15:8];
                            if (mem_wstrb[2]) rom_mem[rom_addr+2] <= wdata[23:16];
                            if (mem_wstrb[3]) rom_mem[rom_addr+3] <= wdata[31:24];
                        end else begin
                            status_reg[1] <= 1'b1;  // Non-aligned access error
                        end
                    end
                end else if (is_csr_area && !is_csr_oom) begin
                    case (mem_addr)
                        RW_BROM_CTRL: begin
                            if (READ_ONLY) begin
                                ctrl_reg[2:1] <= wdata[2:1]; // Only update IRQ enables
                                status_reg[0] <= wdata[0];
                            end else begin
                                ctrl_reg <= wdata;
                            end
                        end
                        RW_BROM_STATUS: begin
                            if (wdata[0]) status_reg[0] <= 1'b0;
                            if (wdata[1]) status_reg[1] <= 1'b0;
                        end
                        default: ;
                    endcase
                end
                status_reg[1] <= (is_mem_area && is_mem_oom || is_csr_area && is_csr_oom);
            end
        end
    end

    always @(posedge clk) begin: IRQ_GEN
        if (!resetn) begin
            irq <= 1'b0;
        end else begin
            if (eoi) begin
                irq <= 1'b0;
            end else if (ctrl_rom_en && !irq) begin
                if ((ctrl_error_irq_en && (status_access_error || status_addr_error)) ||
                    (ctrl_access_irq_en && mem_valid && (is_mem_area || is_csr_area))) begin
                    irq <= 1'b1;
                end
            end
        end
    end

endmodule

`endif
