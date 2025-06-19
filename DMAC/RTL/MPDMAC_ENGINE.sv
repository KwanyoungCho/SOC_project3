// Copyright (c) 2024 Sungkyunkwan University
//
// Authors:
// - Jungrae Kim <dale40@skku.edu>

module MPDMAC_ENGINE
(
    input   wire                        clk,
    input   wire                        rst_n,

    // from TRDMAC_CFG(SFRs)
    input   wire    [31:0]              src_addr_i,
    input   wire    [31:0]              dst_addr_i,
    input   wire    [5:0]               mat_width_i,
    input   wire                        start_i,
    output  wire                        done_o,

    // AMBA AXI interface (AW channel)
    output  wire    [3:0]               awid_o,
    output  wire    [31:0]              awaddr_o,
    output  wire    [3:0]               awlen_o,
    output  wire    [2:0]               awsize_o,
    output  wire    [1:0]               awburst_o,
    output  wire                        awvalid_o,
    input   wire                        awready_i,

    // AMBA AXI interface (W channel)
    output  wire    [3:0]               wid_o,
    output  wire    [31:0]              wdata_o,
    output  wire    [3:0]               wstrb_o,
    output  wire                        wlast_o,
    output  wire                        wvalid_o,
    input   wire                        wready_i,

    // AMBA AXI interface (B channel)
    input   wire    [3:0]               bid_i,
    input   wire    [1:0]               bresp_i,
    input   wire                        bvalid_i,
    output  wire                        bready_o,

    // AMBA AXI interface (AR channel)
    output  wire    [3:0]               arid_o,
    output  wire    [31:0]              araddr_o,
    output  wire    [3:0]               arlen_o,
    output  wire    [2:0]               arsize_o,
    output  wire    [1:0]               arburst_o,
    output  wire                        arvalid_o,
    input   wire                        arready_i,

    // AMBA AXI interface (R channel)
    input   wire    [3:0]               rid_i,
    input   wire    [31:0]              rdata_i,
    input   wire    [1:0]               rresp_i,
    input   wire                        rlast_i,
    input   wire                        rvalid_i,
    output  wire                        rready_o
);
    // Design here

    // ----------------------------------------------------------
    // Parameters and internal signals
    // ----------------------------------------------------------

    typedef enum logic [3:0] {
        S_IDLE      = 4'd0,
        S_RD0_AR    = 4'd1,
        S_RD0_R     = 4'd2,
        S_RD1_AR    = 4'd3,
        S_RD1_R     = 4'd4,
        S_RD2_AR    = 4'd5,
        S_RD2_R     = 4'd6,
        S_RD3_AR    = 4'd7,
        S_RD3_R     = 4'd8,
        S_AW0       = 4'd9,
        S_W0        = 4'd10,
        S_B0        = 4'd11,
        S_AW1       = 4'd12,
        S_W1        = 4'd13,
        S_B1        = 4'd14
    } state_t;

    state_t                  state, state_n;

    reg [31:0]               src_base, dst_base;
    reg [5:0]                mat_width;
    reg [5:0]                pad_width;

    reg [5:0]                row, col;
    reg [5:0]                row_n, col_n;

    reg [31:0]               buffer[1:0][1:0];
    reg [1:0]                beat_cnt;
    reg [31:0]               axi_addr;

    reg                      done, done_n;

    // ----------------------------------------------------------
    // Helper function for mirror index
    // ----------------------------------------------------------
    function automatic [5:0] mirror_idx(
        input [5:0] pidx,
        input [5:0] pad_w,
        input [5:0] mat_w
    );
        if (pidx == 0)
            mirror_idx = 6'd1;
        else if (pidx == pad_w - 1)
            mirror_idx = mat_w - 2;
        else
            mirror_idx = pidx - 1;
    endfunction

    wire        start_trig = start_i & done;

    // ----------------------------------------------------------
    // sequential logic
    // ----------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            src_base    <= 32'd0;
            dst_base    <= 32'd0;
            mat_width   <= 6'd0;
            pad_width   <= 6'd0;
            row         <= 6'd0;
            col         <= 6'd0;
            beat_cnt    <= 2'd0;
            axi_addr    <= 32'd0;
            buffer[0][0]<= 32'd0;
            buffer[0][1]<= 32'd0;
            buffer[1][0]<= 32'd0;
            buffer[1][1]<= 32'd0;
            done        <= 1'b1;
        end else begin
            state       <= state_n;
            row         <= row_n;
            col         <= col_n;
            beat_cnt    <= (state==S_W0 || state==S_W1 || state==S_RD0_R || state==S_RD1_R || state==S_RD2_R || state==S_RD3_R) ? beat_cnt + 1'b1 : 2'd0;
            axi_addr    <= (state==S_RD0_AR || state==S_RD1_AR || state==S_RD2_AR || state==S_RD3_AR || state==S_AW0 || state==S_AW1) ? axi_addr : axi_addr;
            if (state==S_RD0_R && rvalid_i)      buffer[0][0] <= rdata_i;
            if (state==S_RD1_R && rvalid_i)      buffer[0][1] <= rdata_i;
            if (state==S_RD2_R && rvalid_i)      buffer[1][0] <= rdata_i;
            if (state==S_RD3_R && rvalid_i)      buffer[1][1] <= rdata_i;
            done        <= done_n;
        end
    end

    // ----------------------------------------------------------
    // address generation
    // ----------------------------------------------------------
    wire [5:0] src_r0 = mirror_idx(row    , pad_width, mat_width);
    wire [5:0] src_r1 = mirror_idx(row+1 , pad_width, mat_width);
    wire [5:0] src_c0 = mirror_idx(col    , pad_width, mat_width);
    wire [5:0] src_c1 = mirror_idx(col+1 , pad_width, mat_width);

    wire [31:0] src_addr00 = src_base + (((src_r0 * mat_width) + src_c0) << 2);
    wire [31:0] src_addr01 = src_base + (((src_r0 * mat_width) + src_c1) << 2);
    wire [31:0] src_addr10 = src_base + (((src_r1 * mat_width) + src_c0) << 2);
    wire [31:0] src_addr11 = src_base + (((src_r1 * mat_width) + src_c1) << 2);

    wire [31:0] dst_addr0  = dst_base + (((row   * pad_width) + col) << 2);
    wire [31:0] dst_addr1  = dst_base + ((((row+1) * pad_width) + col) << 2);

    // ----------------------------------------------------------
    // combinational FSM
    // ----------------------------------------------------------
    always_comb begin
        state_n   = state;
        row_n     = row;
        col_n     = col;
        done_n    = done;

        case (state)
            S_IDLE: begin
                if (start_trig) begin
                    state_n = S_RD0_AR;
                    row_n   = 6'd0;
                    col_n   = 6'd0;
                    done_n  = 1'b0;
                end
            end

            // read four source words
            S_RD0_AR: begin
                if (arready_i) begin
                    state_n = S_RD0_R;
                end
            end
            S_RD0_R: begin
                if (rvalid_i) begin
                    state_n = S_RD1_AR;
                end
            end
            S_RD1_AR: begin
                if (arready_i) begin
                    state_n = S_RD1_R;
                end
            end
            S_RD1_R: begin
                if (rvalid_i) begin
                    state_n = S_RD2_AR;
                end
            end
            S_RD2_AR: begin
                if (arready_i) begin
                    state_n = S_RD2_R;
                end
            end
            S_RD2_R: begin
                if (rvalid_i) begin
                    state_n = S_RD3_AR;
                end
            end
            S_RD3_AR: begin
                if (arready_i) begin
                    state_n = S_RD3_R;
                end
            end
            S_RD3_R: begin
                if (rvalid_i) begin
                    state_n = S_AW0;
                end
            end

            // write first row (2-beat burst)
            S_AW0: begin
                if (awready_i) begin
                    state_n = S_W0;
                end
            end
            S_W0: begin
                if (wready_i && beat_cnt==1) begin
                    state_n = S_B0;
                end
            end
            S_B0: begin
                if (bvalid_i) begin
                    state_n = S_AW1;
                end
            end

            // write second row (2-beat burst)
            S_AW1: begin
                if (awready_i) begin
                    state_n = S_W1;
                end
            end
            S_W1: begin
                if (wready_i && beat_cnt==1) begin
                    state_n = S_B1;
                end
            end
            S_B1: begin
                if (bvalid_i) begin
                    if (row == pad_width-2 && col == pad_width-2) begin
                        state_n = S_IDLE;
                        done_n  = 1'b1;
                    end else begin
                        // move to next tile
                        if (col == pad_width-2) begin
                            col_n = 6'd0;
                            row_n = row + 6'd2;
                        end else begin
                            col_n = col + 6'd2;
                        end
                        state_n = S_RD0_AR;
                    end
                end
            end
        endcase
    end

    // ----------------------------------------------------------
    // AXI control logic
    // ----------------------------------------------------------
    assign arid_o     = 4'd0;
    assign arsize_o   = 3'b010; // 4 bytes
    assign arburst_o  = 2'b01;  // INCR
    assign arlen_o    = 4'd0;   // single beat

    assign awid_o     = 4'd0;
    assign awsize_o   = 3'b010;
    assign awburst_o  = 2'b01;
    assign awlen_o    = 4'd1;   // two beats

    assign wid_o      = 4'd0;
    assign wstrb_o    = 4'hF;

    // AR address selection per state
    assign arvalid_o  = (state==S_RD0_AR) || (state==S_RD1_AR) || (state==S_RD2_AR) || (state==S_RD3_AR);
    assign araddr_o   = (state==S_RD0_AR) ? src_addr00 :
                        (state==S_RD1_AR) ? src_addr01 :
                        (state==S_RD2_AR) ? src_addr10 :
                        src_addr11;

    // R handshake
    assign rready_o   = (state==S_RD0_R) || (state==S_RD1_R) || (state==S_RD2_R) || (state==S_RD3_R);

    // AW address per state
    assign awvalid_o  = (state==S_AW0) || (state==S_AW1);
    assign awaddr_o   = (state==S_AW0) ? dst_addr0 : dst_addr1;

    // Write data channel
    assign wvalid_o   = (state==S_W0) || (state==S_W1);
    assign wdata_o    = (state==S_W0) ? (beat_cnt==0 ? buffer[0][0] : buffer[0][1]) :
                        (beat_cnt==0 ? buffer[1][0] : buffer[1][1]);
    assign wlast_o    = (beat_cnt==1);

    // B channel
    assign bready_o   = (state==S_B0) || (state==S_B1);

    assign done_o     = done;

endmodule