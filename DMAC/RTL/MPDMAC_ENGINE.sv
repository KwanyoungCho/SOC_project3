// Copyright (c) 2024 Sungkyunkwan University
//
// Authors:
// - Jungrae Kim <dale40@skku.edu>

module MPDMAC_ENGINE (
    input  wire                       clk,
    input  wire                       rst_n,

    // from MPDMAC_CFG(SFRs)
    input  wire [31:0]               src_addr_i,
    input  wire [31:0]               dst_addr_i,
    input  wire [5:0]                mat_width_i,
    input  wire                      start_i,
    output wire                      done_o,

    // AMBA AXI interface (AW channel)
    output wire [3:0]                awid_o,
    output wire [31:0]               awaddr_o,
    output wire [3:0]                awlen_o,
    output wire [2:0]                awsize_o,
    output wire [1:0]                awburst_o,
    output wire                      awvalid_o,
    input  wire                      awready_i,

    // AMBA AXI interface (W channel)
    output wire [3:0]                wid_o,
    output wire [31:0]               wdata_o,
    output wire [3:0]                wstrb_o,
    output wire                      wlast_o,
    output wire                      wvalid_o,
    input  wire                      wready_i,

    // AMBA AXI interface (B channel)
    input  wire [3:0]                bid_i,
    input  wire [1:0]                bresp_i,
    input  wire                      bvalid_i,
    output wire                      bready_o,

    // AMBA AXI interface (AR channel)
    output wire [3:0]                arid_o,
    output wire [31:0]               araddr_o,
    output wire [3:0]                arlen_o,
    output wire [2:0]                arsize_o,
    output wire [1:0]                arburst_o,
    output wire                      arvalid_o,
    input  wire                      arready_i,

    // AMBA AXI interface (R channel)
    input  wire [3:0]                rid_i,
    input  wire [31:0]               rdata_i,
    input  wire [1:0]                rresp_i,
    input  wire                      rlast_i,
    input  wire                      rvalid_i,
    output wire                      rready_o
);

    // ----------------------------------------------------------
    // Parameters and internal signals
    // ----------------------------------------------------------

    typedef enum logic [2:0] {
        S_IDLE  = 3'd0,
        S_AR    = 3'd1,
        S_R     = 3'd2,
        S_AW    = 3'd3,
        S_W     = 3'd4,
        S_B     = 3'd5
    } state_t;

    state_t                  state, state_n;

    reg [31:0]               src_base, dst_base;
    reg [5:0]                mat_width;
    reg [5:0]                pad_width;

    reg [5:0]                row, col;
    reg [5:0]                row_n, col_n;

    reg [31:0]               buffer[1:0][1:0];

    reg [1:0]                ar_cnt, ar_cnt_n;
    reg [2:0]                rd_cnt, rd_cnt_n;
    reg [1:0]                aw_cnt, aw_cnt_n;
    reg [2:0]                wr_cnt, wr_cnt_n;
    reg [1:0]                b_cnt,  b_cnt_n;

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
            ar_cnt      <= 2'd0;
            rd_cnt      <= 3'd0;
            aw_cnt      <= 2'd0;
            wr_cnt      <= 3'd0;
            b_cnt       <= 2'd0;
            buffer[0][0]<= 32'd0;
            buffer[0][1]<= 32'd0;
            buffer[1][0]<= 32'd0;
            buffer[1][1]<= 32'd0;
            done        <= 1'b1;
        end else begin
            state       <= state_n;
            row         <= row_n;
            col         <= col_n;
            ar_cnt      <= ar_cnt_n;
            rd_cnt      <= rd_cnt_n;
            aw_cnt      <= aw_cnt_n;
            wr_cnt      <= wr_cnt_n;
            b_cnt       <= b_cnt_n;
            if (state==S_R && rvalid_i)
                case (rd_cnt)
                    3'd0: buffer[0][0] <= rdata_i;
                    3'd1: buffer[0][1] <= rdata_i;
                    3'd2: buffer[1][0] <= rdata_i;
                    3'd3: buffer[1][1] <= rdata_i;
                endcase
            if (state==S_IDLE && start_trig) begin
                src_base    <= src_addr_i;
                dst_base    <= dst_addr_i;
                mat_width   <= mat_width_i;
                pad_width   <= mat_width_i + 6'd2;
            end
            done        <= done_n;
        end
    end

    // ----------------------------------------------------------
    // address generation
    // ----------------------------------------------------------
    wire [5:0] src_r0 = mirror_idx(row    , pad_width, mat_width);
    wire [5:0] src_r1 = mirror_idx(row+1 , pad_width, mat_width);
    wire [5:0] src_c0 = mirror_idx(col    , pad_width, mat_width);

    wire [31:0] src_addr0 = src_base + (((src_r0 * mat_width) + src_c0) << 2);
    wire [31:0] src_addr1 = src_base + (((src_r1 * mat_width) + src_c0) << 2);

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
        ar_cnt_n  = ar_cnt;
        rd_cnt_n  = rd_cnt;
        aw_cnt_n  = aw_cnt;
        wr_cnt_n  = wr_cnt;
        b_cnt_n   = b_cnt;

        case (state)
            S_IDLE: begin
                if (start_trig) begin
                    state_n   = S_AR;
                    row_n     = 6'd0;
                    col_n     = 6'd0;
                    ar_cnt_n  = 2'd0;
                    rd_cnt_n  = 3'd0;
                    aw_cnt_n  = 2'd0;
                    wr_cnt_n  = 3'd0;
                    b_cnt_n   = 2'd0;
                    done_n    = 1'b0;
                end
            end

            S_AR: begin
                if (arready_i) begin
                    if (ar_cnt == 0) begin
                        ar_cnt_n = 1;
                    end else begin
                        ar_cnt_n = 0;
                        state_n  = S_R;
                    end
                end
            end

            S_R: begin
                if (rvalid_i) begin
                    if (rlast_i) begin
                        rd_cnt_n = 3'd0;
                        state_n  = S_AW;
                    end else begin
                        rd_cnt_n = rd_cnt + 1;
                    end
                end
            end

            S_AW: begin
                if (awready_i) begin
                    if (aw_cnt == 0) begin
                        aw_cnt_n = 1;
                    end else begin
                        aw_cnt_n = 0;
                        state_n  = S_W;
                    end
                end
            end

            S_W: begin
                if (wready_i) begin
                    if (wlast_o) begin
                        wr_cnt_n = 3'd0;
                        state_n  = S_B;
                    end else begin
                        wr_cnt_n = wr_cnt + 1;
                    end
                end
            end

            S_B: begin
                if (bvalid_i) begin
                    if (b_cnt == 0) begin
                        b_cnt_n = 1;
                    end else begin
                        b_cnt_n = 0;
                        if (row == pad_width-2 && col == pad_width-2) begin
                            state_n = S_IDLE;
                            done_n  = 1'b1;
                        end else begin
                            if (col == pad_width-2) begin
                                col_n = 6'd0;
                                row_n = row + 6'd2;
                            end else begin
                                col_n = col + 6'd2;
                            end
                            state_n = S_AR;
                        end
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
    assign arlen_o    = 4'd1;   // two beats

    assign awid_o     = 4'd0;
    assign awsize_o   = 3'b010;
    assign awburst_o  = 2'b01;
    assign awlen_o    = 4'd1;   // two beats

    assign wid_o      = 4'd0;
    assign wstrb_o    = 4'hF;

    assign arvalid_o  = (state==S_AR);
    assign araddr_o   = (ar_cnt==0) ? src_addr0 : src_addr1;

    assign rready_o   = (state==S_R);

    assign awvalid_o  = (state==S_AW);
    assign awaddr_o   = (aw_cnt==0) ? dst_addr0 : dst_addr1;

    assign wvalid_o   = (state==S_W);
    assign wdata_o    = (wr_cnt==0) ? buffer[0][0] :
                        (wr_cnt==1) ? buffer[0][1] :
                        (wr_cnt==2) ? buffer[1][0] :
                                       buffer[1][1];
    assign wlast_o    = (wr_cnt==1) || (wr_cnt==3);

    assign bready_o   = (state==S_B);

    assign done_o     = done;

endmodule

