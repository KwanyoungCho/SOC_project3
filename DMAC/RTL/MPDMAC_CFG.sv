// Copyright (c) 2024 Sungkyunkwan University
//
// Authors:
// - Jungrae Kim <dale40@skku.edu>

module MPDMAC_CFG
(
    input   wire                clk,
    input   wire                rst_n,  // _n means active low

    // AMBA APB interface
    input   wire                psel_i,
    input   wire                penable_i,
    input   wire    [11:0]      paddr_i,
    input   wire                pwrite_i,
    input   wire    [31:0]      pwdata_i,
    output  reg                 pready_o,
    output  reg     [31:0]      prdata_o,
    output  reg                 pslverr_o,

    // configuration registers
    output  reg     [31:0]      src_addr_o,
    output  reg     [31:0]      dst_addr_o,
    output  reg     [5:0]       mat_width_o,
    output  wire                start_o,
    input   wire                done_i
);

    // Configuration register to read/write
    reg     [31:0]              src_addr;
    reg     [31:0]              dst_addr;
    reg     [5:0]               mat_width;

    //----------------------------------------------------------
    // Write
    //----------------------------------------------------------
    // an APB write occurs when PSEL & PENABLE & PWRITE
    // clk     : __--__--__--__--__--__--__--__--__--__--
    // psel    : ___--------_____________________________
    // penable : _______----_____________________________
    // pwrite  : ___--------_____________________________
    // wren    : _______----_____________________________
    //
    // DMA start command must be asserted when APB writes 1 to the DMA_CMD
    // register
    // clk     : __--__--__--__--__--__--__--__--__--__--
    // psel    : ___--------_____________________________
    // penable : _______----_____________________________
    // pwrite  : ___--------_____________________________
    // paddr   :    |DMA_CMD|
    // pwdata  :    |   1   |
    // start   : _______----_____________________________

    wire    wren                = psel_i & penable_i & pwrite_i;

    always @(posedge clk) begin
        if (!rst_n) begin
            src_addr            <= 32'd0;
            dst_addr            <= 32'd0;
            mat_width           <= 6'd0;
        end
        else if (wren) begin
            case (paddr_i)
                'h100: src_addr       <= pwdata_i[31:0];
                'h104: dst_addr       <= pwdata_i[31:0];
                'h108: mat_width      <= pwdata_i[31:0];
            endcase
        end
    end
    wire    start               = wren & (paddr_i=='h10C) & pwdata_i[0];

    //----------------------------------------------------------
    // READ
    //----------------------------------------------------------
    // an APB read occurs when PSEL & PENABLE & !PWRITE
    // To make read data a direct output from register,
    // this code shall buffer the muxed read data into a register
    // in the SETUP cycle (PSEL & !PENABLE)
    // clk        : __--__--__--__--__--__--__--__--__--__--
    // psel       : ___--------_____________________________
    // penable    : _______----_____________________________
    // pwrite     : ________________________________________
    // reg update : ___----_________________________________
    // prdata     :        |DATA

    reg     [31:0]              rdata;

    always @(posedge clk) begin
        if (!rst_n) begin
            rdata               <= 32'd0;
        end
        else if (psel_i & !penable_i & !pwrite_i) begin // in the setup cycle in the APB state diagram
            case (paddr_i)
                'h0:   rdata            <= 32'h0101_2025;
                'h100: rdata            <= src_addr;
                'h104: rdata            <= dst_addr;
                'h108: rdata            <= mat_width;
                //'h10C:rdata           <= start
                'h110: rdata            <= {31'd0, done_i};
                default: rdata          <= 32'd0;
            endcase
        end
    end

    // output assignments
    assign  pready_o            = 1'b1;
    assign  prdata_o            = rdata;
    assign  pslverr_o           = 1'b0;

    assign  src_addr_o          = src_addr;
    assign  dst_addr_o          = dst_addr;
    assign  mat_width_o         = mat_width;
    assign  start_o             = start;

endmodule
