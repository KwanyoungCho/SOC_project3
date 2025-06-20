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

    // State Machine - DMAC_ENGINE 패턴을 따라함
    localparam S_IDLE  = 3'd0;
    localparam S_RREQ  = 3'd1;
    localparam S_RDATA = 3'd2;
    localparam S_WREQ  = 3'd3;
    localparam S_WDATA = 3'd4;

    reg [2:0]               state, state_n;
    
    // Configuration registers
    reg [31:0]              src_addr, src_addr_n;
    reg [31:0]              dst_addr, dst_addr_n;
    reg [5:0]               mat_width, mat_width_n;
    
    // Processing variables
    reg [5:0]               dst_row, dst_row_n;     // 현재 쓰고 있는 출력 행 (0~width+1)
    reg [5:0]               dst_col, dst_col_n;     // 현재 쓰고 있는 출력 열 (0~width+1)
    
    reg [31:0]              data_buf, data_buf_n;   // 읽어온 데이터 버퍼
    
    // Control signals
    reg                     arvalid;
    reg                     rready;
    reg                     awvalid;
    reg                     wvalid;
    reg                     done;
    
    // Helper signals
    wire [5:0] out_width = mat_width + 2;
    wire dst_complete = (dst_row >= mat_width + 2);
    
    // Mirror padding function - 출력 좌표에서 소스 좌표로 매핑
    function [31:0] calc_src_addr_from_dst;
        input [5:0] out_row, out_col;
        input [5:0] width;
        input [31:0] base_addr;
        reg [5:0] src_row_mapped, src_col_mapped;
        begin
            // Mirror padding mapping
            if (out_row == 0) begin
                src_row_mapped = 1;  // 첫 번째 출력 행 -> 소스 1행
            end else if (out_row == width + 1) begin
                src_row_mapped = width - 2;  // 마지막 출력 행 -> 소스 width-2행
            end else begin
                src_row_mapped = out_row - 1;  // 나머지 -> 1칸씩 이동
            end
            
            if (out_col == 0) begin
                src_col_mapped = 1;  // 첫 번째 출력 열 -> 소스 1열
            end else if (out_col == width + 1) begin
                src_col_mapped = width - 2;  // 마지막 출력 열 -> 소스 width-2열
            end else begin
                src_col_mapped = out_col - 1;  // 나머지 -> 1칸씩 이동
            end
            
            calc_src_addr_from_dst = base_addr + (src_row_mapped * width + src_col_mapped) * 4;
        end
    endfunction
    
    // Output address calculation
    function [31:0] calc_dst_addr;
        input [5:0] out_row, out_col;
        input [5:0] width;
        input [31:0] base_addr;
        begin
            calc_dst_addr = base_addr + (out_row * (width + 2) + out_col) * 4;
        end
    endfunction

    // Sequential logic - DMAC_ENGINE 패턴
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state <= S_IDLE;
            src_addr <= 32'd0;
            dst_addr <= 32'd0;
            mat_width <= 6'd0;
            dst_row <= 6'd0;
            dst_col <= 6'd0;
            data_buf <= 32'd0;
        end else begin
            state <= state_n;
            src_addr <= src_addr_n;
            dst_addr <= dst_addr_n;
            mat_width <= mat_width_n;
            dst_row <= dst_row_n;
            dst_col <= dst_col_n;
            data_buf <= data_buf_n;
        end
    end

    // Combinational logic - DMAC_ENGINE 패턴
    always_comb begin
        state_n = state;
        src_addr_n = src_addr;
        dst_addr_n = dst_addr;
        mat_width_n = mat_width;
        dst_row_n = dst_row;
        dst_col_n = dst_col;
        data_buf_n = data_buf;
        
        arvalid = 1'b0;
        rready = 1'b0;
        awvalid = 1'b0;
        wvalid = 1'b0;
        done = 1'b0;

        case (state)
            S_IDLE: begin
                done = 1'b1;
                if (start_i && mat_width_i != 0) begin
                    src_addr_n = src_addr_i;
                    dst_addr_n = dst_addr_i;
                    mat_width_n = mat_width_i;
                    dst_row_n = 6'd0;
                    dst_col_n = 6'd0;
                    
                    $display("[DEBUG] Starting DMA: src=%h, dst=%h, width=%d", 
                            src_addr_i, dst_addr_i, mat_width_i);
                    
                    state_n = S_RREQ;
                end
            end
            
            S_RREQ: begin
                arvalid = 1'b1;
                if (arready_i) begin
                    state_n = S_RDATA;
                    
                    $display("[DEBUG] Read REQ: dst_pos(%d,%d) -> src_addr=%h", 
                            dst_row, dst_col, calc_src_addr_from_dst(dst_row, dst_col, mat_width, src_addr));
                end
            end
            
            S_RDATA: begin
                rready = 1'b1;
                if (rvalid_i) begin
                    data_buf_n = rdata_i;
                    state_n = S_WREQ;
                    
                    $display("[DEBUG] Read DATA: %d", rdata_i);
                end
            end
            
            S_WREQ: begin
                awvalid = 1'b1;
                if (awready_i) begin
                    state_n = S_WDATA;
                    
                    $display("[DEBUG] Write REQ: dst_pos(%d,%d) -> dst_addr=%h", 
                            dst_row, dst_col, calc_dst_addr(dst_row, dst_col, mat_width, dst_addr));
                end
            end
            
            S_WDATA: begin
                wvalid = 1'b1;
                if (wready_i) begin
                    $display("[DEBUG] Write DATA: pos(%d,%d) = %d", 
                            dst_row, dst_col, data_buf);
                    
                    // 다음 위치로 이동
                    if (dst_col == out_width - 1) begin
                        dst_col_n = 6'd0;
                        dst_row_n = dst_row + 1;
                    end else begin
                        dst_col_n = dst_col + 1;
                    end
                    
                    // 완료 체크
                    if (dst_complete) begin
                        state_n = S_IDLE;
                        $display("[DEBUG] DMA Complete!");
                    end else begin
                        state_n = S_RREQ;
                    end
                end
            end
        endcase
    end

    // Output assignments - DMAC_ENGINE 패턴과 동일
    assign done_o = done;

    assign awid_o = 4'd0;
    assign awaddr_o = calc_dst_addr(dst_row, dst_col, mat_width, dst_addr);
    assign awlen_o = 4'd0;      // 1-burst
    assign awsize_o = 3'b010;   // 4 bytes per transfer
    assign awburst_o = 2'b01;   // incremental
    assign awvalid_o = awvalid;

    assign wid_o = 4'd0;
    assign wdata_o = data_buf;
    assign wstrb_o = 4'b1111;   // all bytes valid
    assign wlast_o = 1'b1;      // single beat
    assign wvalid_o = wvalid;

    assign bready_o = 1'b1;     // always ready for response

    assign arid_o = 4'd0;
    assign araddr_o = calc_src_addr_from_dst(dst_row, dst_col, mat_width, src_addr);
    assign arlen_o = 4'd0;      // 1-burst
    assign arsize_o = 3'b010;   // 4 bytes per transfer  
    assign arburst_o = 2'b01;   // incremental
    assign arvalid_o = arvalid;

    assign rready_o = rready;

endmodule 