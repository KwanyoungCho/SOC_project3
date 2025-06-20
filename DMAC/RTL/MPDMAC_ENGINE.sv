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

    // ----------------------------------------------------------
    // Internal registers
    // ----------------------------------------------------------
    typedef enum logic [2:0] {
        S_IDLE,
        S_AR,
        S_R,
        S_AW,
        S_W,
        S_B
    } state_t;

    state_t             state, state_n;
    reg [31:0]          src_base, dst_base;
    reg [5:0]           mat_width;
    reg [5:0]           pad_width;

    reg [5:0]           row, row_n;
    reg [5:0]           col, col_n;

    reg [31:0]          rdata_buf;
    reg                 done, done_n;

    // ----------------------------------------------------------
    // Start logic
    // ----------------------------------------------------------
    wire start_trig = start_i & done;

    // ----------------------------------------------------------
    // FSM sequential
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
            rdata_buf   <= 32'd0;
            done        <= 1'b1;
        end else begin
            state       <= state_n;
            src_base    <= (start_trig) ? src_addr_i : src_base;
            dst_base    <= (start_trig) ? dst_addr_i : dst_base;
            mat_width   <= (start_trig) ? mat_width_i : mat_width;
            pad_width   <= (start_trig) ? (mat_width_i + 6'd2) : pad_width;
            row         <= row_n;
            col         <= col_n;
            if (state==S_R && rvalid_i)
                rdata_buf <= rdata_i;
            done        <= done_n;
        end
    end

    // ----------------------------------------------------------
    // Address calculations
    // ----------------------------------------------------------
    wire [5:0] src_row = (row==0) ? 6'd1 : (row==pad_width-1) ? (mat_width-2) : (row-1);
    wire [5:0] src_col = (col==0) ? 6'd1 : (col==pad_width-1) ? (mat_width-2) : (col-1);

    wire [10:0] src_idx  = src_row * mat_width + src_col;
    wire [31:0] araddr   = src_base + (src_idx << 2);

    wire [10:0] dst_idx  = row * pad_width + col;
    wire [31:0] awaddr   = dst_base + (dst_idx << 2);

    // ----------------------------------------------------------
    // FSM combinational
    // ----------------------------------------------------------
    always_comb begin
        state_n     = state;
        row_n       = row;
        col_n       = col;
        done_n      = done;

        case (state)
            S_IDLE: begin
                if (start_trig) begin
                    state_n = S_AR;
                    row_n   = 6'd0;
                    col_n   = 6'd0;
                    done_n  = 1'b0;
                end
            end
            S_AR: begin
                if (arready_i) begin
                    state_n = S_R;
                end
            end
            S_R: begin
                if (rvalid_i) begin
                    state_n = S_AW;
                end
            end
            S_AW: begin
                if (awready_i) begin
                    state_n = S_W;
                end
            end
            S_W: begin
                if (wready_i) begin
                    state_n = S_B;
                end
            end
            S_B: begin
                if (bvalid_i) begin
                    if (row==pad_width-1 && col==pad_width-1) begin
                        state_n = S_IDLE;
                        done_n  = 1'b1;
                    end else begin
                        state_n = S_AR;
                        if (col==pad_width-1) begin
                            col_n = 0;
                            row_n = row + 6'd1;
                        end else begin
                            col_n = col + 6'd1;
                        end
                    end
                end
            end
        endcase
    end

    // ----------------------------------------------------------
    // Output assignments
    // ----------------------------------------------------------
    assign done_o       = done;

    // AW channel
    assign awid_o       = 4'd0;
    assign awlen_o      = 4'd0;
    assign awsize_o     = 3'b010;
    assign awburst_o    = 2'b01;
    assign awaddr_o     = awaddr;
    assign awvalid_o    = (state==S_AW);

    // W channel
    assign wid_o        = 4'd0;
    assign wdata_o      = rdata_buf;
    assign wstrb_o      = 4'hF;
    assign wlast_o      = 1'b1;
    assign wvalid_o     = (state==S_W);

    // B channel
    assign bready_o     = (state==S_B);

    // AR channel
    assign arid_o       = 4'd0;
    assign arlen_o      = 4'd0;
    assign arsize_o     = 3'b010;
    assign arburst_o    = 2'b01;
    assign araddr_o     = araddr;
    assign arvalid_o    = (state==S_AR);

    // R channel
    assign rready_o     = (state==S_R);

endmodule

