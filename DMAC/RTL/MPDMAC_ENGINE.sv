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

    // State Machine
    localparam S_IDLE           = 4'd0;
    localparam S_READ_3x3       = 4'd1;
    localparam S_PREPARE_BLOCK  = 4'd2;
    localparam S_WRITE_TL       = 4'd3;
    localparam S_WRITE_TR       = 4'd4;
    localparam S_WRITE_BL       = 4'd5;
    localparam S_WRITE_BR       = 4'd6;
    localparam S_NEXT_BLOCK     = 4'd7;
    localparam S_DONE           = 4'd8;

    reg [3:0] state;
    
    // Configuration registers
    reg [31:0] src_addr;
    reg [31:0] dst_addr;
    reg [5:0]  mat_width;
    reg        done;
    
    // Current 2x2 block position in output matrix (0-based)
    reg [5:0]  block_row;  // 0, 2, 4, ... (width+2-2)
    reg [5:0]  block_col;  // 0, 2, 4, ... (width+2-2)
    
    // 3x3 buffer for current region
    reg [31:0] buffer_3x3 [0:8];  // 9 elements: [0][1][2]
                                  //             [3][4][5]
                                  //             [6][7][8]
    
    // Current center position in source matrix (1-based)
    reg signed [6:0] center_row;  // Can be negative for padding
    reg signed [6:0] center_col;  // Can be negative for padding
    
    // Read state
    reg [3:0]  read_count;
    reg [3:0]  read_needed;
    reg        reading_active;
    
    // AXI control signals
    reg        ar_valid;
    reg [31:0] ar_addr;
    reg        r_ready;
    
    reg        aw_valid;
    reg [31:0] aw_addr;
    reg        w_valid;
    reg [31:0] w_data;
    reg        w_last;
    reg        b_ready;
    
    // Output data for 2x2 block
    reg [31:0] output_block [0:3];  // TL, TR, BL, BR
    
    // Handshake signals
    wire ar_handshake = ar_valid & arready_i;
    wire r_handshake = rvalid_i & r_ready;
    wire aw_handshake = aw_valid & awready_i;
    wire w_handshake = w_valid & wready_i;
    wire b_handshake = bvalid_i & b_ready;
    
    // Output assignments
    assign done_o = done;
    
    // AXI AR channel
    assign arid_o = 4'd0;
    assign araddr_o = ar_addr;
    assign arlen_o = 4'd0;      // Single read
    assign arsize_o = 3'b010;   // 4 bytes
    assign arburst_o = 2'b01;   // INCR
    assign arvalid_o = ar_valid;
    assign rready_o = r_ready;
    
    // AXI AW channel  
    assign awid_o = 4'd0;
    assign awaddr_o = aw_addr;
    assign awlen_o = 4'd0;      // Single write transaction
    assign awsize_o = 3'b010;   // 4 bytes
    assign awburst_o = 2'b01;   // INCR
    assign awvalid_o = aw_valid;
    
    // AXI W channel
    assign wid_o = 4'd0;
    assign wdata_o = w_data;
    assign wstrb_o = 4'hF;
    assign wlast_o = w_last;
    assign wvalid_o = w_valid;
    assign bready_o = b_ready;
    
    // 미러 패딩된 주소 계산
    function [31:0] get_mirrored_addr;
        input signed [6:0] src_r;  // 1-based source row
        input signed [6:0] src_c;  // 1-based source col
        input [5:0] width;
        reg signed [6:0] mirrored_r, mirrored_c;
        begin
            // Mirror padding logic
            if (src_r < 1) begin
                mirrored_r = 1;  // Top padding: mirror from row 1
            end else if (src_r > width) begin
                mirrored_r = width;  // Bottom padding: mirror from last row
            end else begin
                mirrored_r = src_r;  // Normal region
            end
            
            if (src_c < 1) begin
                mirrored_c = 1;  // Left padding: mirror from col 1
            end else if (src_c > width) begin
                mirrored_c = width;  // Right padding: mirror from last col
            end else begin
                mirrored_c = src_c;  // Normal region
            end
            
            // Return address offset from source base
            get_mirrored_addr = ((mirrored_r - 1) * width + (mirrored_c - 1)) * 4;
        end
    endfunction
    
    // 3x3 버퍼의 특정 위치에서 값 읽기
    function [31:0] get_buffer_value;
        input signed [6:0] relative_r;  // -1, 0, 1 relative to center
        input signed [6:0] relative_c;  // -1, 0, 1 relative to center
        reg [3:0] buf_idx;
        begin
            // Convert relative position to buffer index
            // Buffer layout: [0][1][2]  <- (center_r-1)
            //               [3][4][5]  <- (center_r)
            //               [6][7][8]  <- (center_r+1)
            buf_idx = (relative_r + 1) * 3 + (relative_c + 1);
            get_buffer_value = buffer_3x3[buf_idx];
        end
    endfunction
    
    // 출력 매트릭스의 특정 위치에 대한 값 계산 (미러 패딩 적용)
    function [31:0] calc_output_value;
        input [5:0] out_r;  // 0-based output row
        input [5:0] out_c;  // 0-based output col
        input [5:0] width;
        reg signed [6:0] rel_r, rel_c;  // Relative to center
        reg [3:0] buf_idx;
        begin
            // Calculate relative position to current center
            rel_r = out_r - center_row;
            rel_c = out_c - center_col;
            
            // Convert relative position to buffer index
            // Buffer layout: [0][1][2]  <- (center_r-1)
            //               [3][4][5]  <- (center_r)
            //               [6][7][8]  <- (center_r+1)
            buf_idx = (rel_r + 1) * 3 + (rel_c + 1);
            calc_output_value = buffer_3x3[buf_idx];
        end
    endfunction
    
    // 출력 매트릭스 주소 계산
    function [31:0] calc_output_addr;
        input [5:0] out_r;  // 0-based output row
        input [5:0] out_c;  // 0-based output col
        input [5:0] width;
        begin
            // Output matrix size: (width+2) x (width+2)
            calc_output_addr = dst_addr + (out_r * (width + 2) + out_c) * 4;
        end
    endfunction
    
    // Main state machine
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_IDLE;
            src_addr <= 32'd0;
            dst_addr <= 32'd0;
            mat_width <= 6'd0;
            done <= 1'b1;
            
            block_row <= 6'd0;
            block_col <= 6'd0;
            center_row <= 7'd0;
            center_col <= 7'd0;
            
            read_count <= 4'd0;
            read_needed <= 4'd9;
            reading_active <= 1'b0;
            
            ar_valid <= 1'b0;
            ar_addr <= 32'd0;
            r_ready <= 1'b0;
            
            aw_valid <= 1'b0;
            aw_addr <= 32'd0;
            w_valid <= 1'b0;
            w_data <= 32'd0;
            w_last <= 1'b0;
            b_ready <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (start_i) begin
                        done <= 1'b0;
                        src_addr <= src_addr_i;
                        dst_addr <= dst_addr_i;
                        mat_width <= mat_width_i;
                        block_row <= 6'd0;
                        block_col <= 6'd0;
                        
                        // 첫 번째 2x2 블록 (0,0)의 중심은 (0,0) - 출력 매트릭스 기준
                        center_row <= 7'd0;  // 출력 매트릭스 기준 0-based
                        center_col <= 7'd0;  // 출력 매트릭스 기준 0-based
                        
                        state <= S_READ_3x3;
                        read_count <= 4'd0;
                        read_needed <= 4'd9;
                        reading_active <= 1'b0;
                    end else begin
                        done <= 1'b1;
                    end
                end
                
                S_READ_3x3: begin
                    if (!reading_active) begin
                        // Start reading the next element of 3x3 region
                        reg signed [6:0] target_out_r, target_out_c;  // 출력 매트릭스 좌표
                        reg signed [6:0] target_src_r, target_src_c;  // 소스 매트릭스 좌표
                        reg [31:0] read_addr_offset;
                        
                        // Calculate which element we're reading (row-major order)
                        // 출력 매트릭스 기준으로 center 주변 3x3 좌표 계산
                        target_out_r = center_row - 1 + (read_count / 3);
                        target_out_c = center_col - 1 + (read_count % 3);
                        
                        // 출력 좌표를 소스 좌표로 변환 (패딩 적용)
                        if (target_out_r <= 0) begin
                            target_src_r = 2;  // 상단 패딩: row 2에서 미러링 (1-based)
                        end else if (target_out_r >= mat_width + 1) begin
                            target_src_r = mat_width - 1;  // 하단 패딩: 마지막에서 두 번째 row에서 미러링
                        end else begin
                            target_src_r = target_out_r + 1;  // 정상 영역: out_r+1이 소스 row (1-based)
                        end
                        
                        if (target_out_c <= 0) begin
                            target_src_c = 2;  // 좌측 패딩: col 2에서 미러링 (1-based)
                        end else if (target_out_c >= mat_width + 1) begin
                            target_src_c = mat_width - 1;  // 우측 패딩: 마지막에서 두 번째 col에서 미러링
                        end else begin
                            target_src_c = target_out_c + 1;  // 정상 영역: out_c+1이 소스 col (1-based)
                        end
                        
                        // Calculate read address
                        read_addr_offset = ((target_src_r - 1) * mat_width + (target_src_c - 1)) * 4;
                        
                        ar_valid <= 1'b1;
                        ar_addr <= src_addr + read_addr_offset;
                        r_ready <= 1'b1;
                        reading_active <= 1'b1;
                    end
                    
                    if (ar_handshake) begin
                        ar_valid <= 1'b0;
                    end
                    
                    if (r_handshake) begin
                        buffer_3x3[read_count] <= rdata_i;
                        read_count <= read_count + 1;
                        reading_active <= 1'b0;
                        r_ready <= 1'b0;
                        
                        if (read_count == read_needed - 1) begin
                            // All 3x3 data read, prepare 2x2 block
                            state <= S_PREPARE_BLOCK;
                            read_count <= 4'd0;
                        end
                    end
                end
                
                S_PREPARE_BLOCK: begin
                    // Calculate 2x2 block values with mirror padding
                    output_block[0] <= calc_output_value(block_row, block_col, mat_width);         // TL
                    output_block[1] <= calc_output_value(block_row, block_col + 1, mat_width);     // TR
                    output_block[2] <= calc_output_value(block_row + 1, block_col, mat_width);     // BL
                    output_block[3] <= calc_output_value(block_row + 1, block_col + 1, mat_width); // BR
                    
                    state <= S_WRITE_TL;
                    
                    // Start write transaction for TL
                    aw_valid <= 1'b1;
                    aw_addr <= calc_output_addr(block_row, block_col, mat_width);  // TL address
                end
                
                S_WRITE_TL: begin
                    if (aw_handshake) begin
                        aw_valid <= 1'b0;
                        w_valid <= 1'b1;
                        w_data <= output_block[0];  // TL
                        w_last <= 1'b1;
                        b_ready <= 1'b1;
                    end
                    
                    if (w_handshake) begin
                        w_valid <= 1'b0;
                        w_last <= 1'b0;
                    end
                    
                    if (b_handshake) begin
                        b_ready <= 1'b0;
                        state <= S_WRITE_TR;
                        
                        // Start write transaction for TR
                        aw_valid <= 1'b1;
                        aw_addr <= calc_output_addr(block_row, block_col + 1, mat_width);  // TR address
                    end
                end
                
                S_WRITE_TR: begin
                    if (aw_handshake) begin
                        aw_valid <= 1'b0;
                        w_valid <= 1'b1;
                        w_data <= output_block[1];  // TR
                        w_last <= 1'b1;
                        b_ready <= 1'b1;
                    end
                    
                    if (w_handshake) begin
                        w_valid <= 1'b0;
                        w_last <= 1'b0;
                    end
                    
                    if (b_handshake) begin
                        b_ready <= 1'b0;
                        state <= S_WRITE_BL;
                        
                        // Start write transaction for BL
                        aw_valid <= 1'b1;
                        aw_addr <= calc_output_addr(block_row + 1, block_col, mat_width);  // BL address
                    end
                end
                
                S_WRITE_BL: begin
                    if (aw_handshake) begin
                        aw_valid <= 1'b0;
                        w_valid <= 1'b1;
                        w_data <= output_block[2];  // BL
                        w_last <= 1'b1;
                        b_ready <= 1'b1;
                    end
                    
                    if (w_handshake) begin
                        w_valid <= 1'b0;
                        w_last <= 1'b0;
                    end
                    
                    if (b_handshake) begin
                        b_ready <= 1'b0;
                        state <= S_WRITE_BR;
                        
                        // Start write transaction for BR
                        aw_valid <= 1'b1;
                        aw_addr <= calc_output_addr(block_row + 1, block_col + 1, mat_width);  // BR address
                    end
                end
                
                S_WRITE_BR: begin
                    if (aw_handshake) begin
                        aw_valid <= 1'b0;
                        w_valid <= 1'b1;
                        w_data <= output_block[3];  // BR
                        w_last <= 1'b1;
                        b_ready <= 1'b1;
                    end
                    
                    if (w_handshake) begin
                        w_valid <= 1'b0;
                        w_last <= 1'b0;
                    end
                    
                    if (b_handshake) begin
                        b_ready <= 1'b0;
                        state <= S_NEXT_BLOCK;
                    end
                end
                
                S_NEXT_BLOCK: begin
                    // Move to next 2x2 block
                    if (block_col + 2 < mat_width + 2) begin
                        block_col <= block_col + 2;
                        center_col <= center_col + 2;  // 출력 매트릭스 기준
                    end else begin
                        block_col <= 6'd0;
                        center_col <= 7'd0;  // 출력 매트릭스 기준
                        block_row <= block_row + 2;
                        center_row <= center_row + 2;  // 출력 매트릭스 기준
                    end
                    
                    // Check if done - fixed completion condition
                    if (block_row >= mat_width + 2) begin
                        state <= S_DONE;
                    end else begin
                        state <= S_READ_3x3;
                        read_count <= 4'd0;
                        read_needed <= 4'd9;
                        reading_active <= 1'b0;
                    end
                end
                
                S_DONE: begin
                    done <= 1'b1;
                    if (!start_i) begin
                        state <= S_IDLE;
                    end
                end
            endcase
        end
    end

endmodule 