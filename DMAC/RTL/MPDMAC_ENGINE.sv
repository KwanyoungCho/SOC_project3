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
    localparam S_IDLE           = 3'd0;
    localparam S_READ_REQ       = 3'd1;
    localparam S_READ_DATA      = 3'd2;
    localparam S_PREPARE_BLOCK  = 3'd3;
    localparam S_WRITE_REQ      = 3'd4;
    localparam S_WRITE_DATA     = 3'd5;
    localparam S_WRITE_RESP     = 3'd6;
    localparam S_NEXT_BLOCK     = 3'd7;

    reg [2:0] state;
    
    // Configuration registers
    reg [31:0] src_addr;
    reg [31:0] dst_addr;
    reg [5:0]  mat_width;
    reg        done;
    
    // Current 2x2 block position in output matrix (0-based)
    reg [5:0]  block_row;  
    reg [5:0]  block_col;  
    
    // 3x3 buffer for current region
    reg [31:0] buffer_3x3 [0:8];  
    
    // Current center position in source matrix (1-based)
    reg signed [6:0] center_row;  
    reg signed [6:0] center_col;  
    
    // Read state
    reg [3:0]  read_count;
    reg [3:0]  burst_count;
    reg [31:0] current_read_addr;
    
    // Write state 
    reg [1:0]  write_count;
    reg [31:0] current_write_addr;
    
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
    assign arlen_o = 4'd2;      // Burst of 3 for each row
    assign arsize_o = 3'b010;   // 4 bytes
    assign arburst_o = 2'b01;   // INCR
    assign arvalid_o = ar_valid;
    assign rready_o = r_ready;
    
    // AXI AW channel  
    assign awid_o = 4'd0;
    assign awaddr_o = aw_addr;
    assign awlen_o = 4'd0;      // Single transaction per write
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
            
            // Debug output for boundary cases
            if (src_r < 1 || src_r > width || src_c < 1 || src_c > width) begin
                $display("[DEBUG] get_mirrored_addr: src(%d,%d) -> mirrored(%d,%d) offset=%h", 
                         src_r, src_c, mirrored_r, mirrored_c, get_mirrored_addr);
            end
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
        reg signed [6:0] src_r, src_c;  // 1-based source coordinates
        reg signed [6:0] rel_r, rel_c;  // Relative to center
        begin
            // 출력 매트릭스는 (width+2) x (width+2), 소스 매트릭스는 width x width
            // 미러 패딩 매핑 (0-based 출력 좌표 -> 1-based 소스 좌표):
            // - 출력 row 0 -> 소스 row 2 (원본의 두 번째 행을 미러링)
            // - 출력 row 1 -> 소스 row 1 (원본의 첫 번째 행)
            // - 출력 row 2~(width-1) -> 소스 row 2~(width-1)
            // - 출력 row width -> 소스 row width
            // - 출력 row (width+1) -> 소스 row (width-1) (원본의 마지막에서 두 번째 행을 미러링)
            
            // 행 변환
            if (out_r == 0) begin
                // 상단 패딩: 출력 0행 -> 소스 2행 (1-based)
                src_r = 2;
            end else if (out_r == width + 1) begin
                // 하단 패딩: 출력 (width+1)행 -> 소스 (width-1)행 (1-based)
                src_r = width - 1;
            end else if (out_r >= 1 && out_r <= width) begin
                // 일반 영역: 출력 행이 그대로 소스 행이 됨 (1-based)
                src_r = out_r;
            end else begin
                src_r = out_r; // 예외 처리
            end
            
            // 열 변환
            if (out_c == 0) begin
                // 좌측 패딩: 출력 0열 -> 소스 2열 (1-based)
                src_c = 2;
            end else if (out_c == width + 1) begin
                // 우측 패딩: 출력 (width+1)열 -> 소스 (width-1)열 (1-based)
                src_c = width - 1;
            end else if (out_c >= 1 && out_c <= width) begin
                // 일반 영역: 출력 열이 그대로 소스 열이 됨 (1-based)
                src_c = out_c;
            end else begin
                src_c = out_c; // 예외 처리
            end
            
            // 현재 중심에 대한 상대 위치 계산
            rel_r = src_r - center_row;
            rel_c = src_c - center_col;
            
            // 디버깅 출력
            if (out_r >= width || out_c >= width) begin
                $display("[DEBUG] calc_output_value: out(%d,%d) -> src(%d,%d) -> rel(%d,%d) vs center(%d,%d)", 
                         out_r, out_c, src_r, src_c, rel_r, rel_c, center_row, center_col);
            end
            
            // 첫 번째 블록의 경우 상세 디버깅
            if (out_r <= 1 && out_c <= 1 && center_row == 1 && center_col == 1) begin
                $display("[DEBUG] First block calc: out(%d,%d) -> src(%d,%d) -> rel(%d,%d) vs center(%d,%d)", 
                         out_r, out_c, src_r, src_c, rel_r, rel_c, center_row, center_col);
            end
            
            // 3x3 버퍼 범위 내인지 확인
            if (rel_r >= -1 && rel_r <= 1 && rel_c >= -1 && rel_c <= 1) begin
                // 버퍼 인덱스로 변환하여 값 가져오기
                calc_output_value = buffer_3x3[(rel_r + 1) * 3 + (rel_c + 1)];
                if (out_r >= width || out_c >= width) begin
                    $display("[DEBUG] Buffer access: index=%d, value=%d", (rel_r + 1) * 3 + (rel_c + 1), calc_output_value);
                end
                // 첫 번째 블록의 경우 상세 디버깅
                if (out_r <= 1 && out_c <= 1 && center_row == 1 && center_col == 1) begin
                    $display("[DEBUG] First block buffer: index=%d, value=%d", (rel_r + 1) * 3 + (rel_c + 1), calc_output_value);
                end
            end else begin
                // 범위를 벗어나면 0 반환 (오류 상황)
                calc_output_value = 32'd0;
                $display("[DEBUG] ERROR: out_of_range out(%d,%d) rel(%d,%d) center(%d,%d)", 
                         out_r, out_c, rel_r, rel_c, center_row, center_col);
            end
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
            burst_count <= 4'd0;
            current_read_addr <= 32'd0;
            
            write_count <= 2'd0;
            current_write_addr <= 32'd0;
            
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
                        
                        center_row <= 7'd1;
                        center_col <= 7'd1;
                        
                        $display("[DEBUG] Starting DMA: src=%h, dst=%h, width=%d", src_addr_i, dst_addr_i, mat_width_i);
                        
                        state <= S_READ_REQ;
                        read_count <= 4'd0;
                        burst_count <= 4'd0;
                    end else begin
                        done <= 1'b1;
                    end
                end
                
                S_READ_REQ: begin
                    if (!ar_valid && (read_count < 9)) begin
                        // Calculate address for next 3-element burst
                        reg signed [6:0] row_r, base_c;
                        reg [31:0] read_addr_offset;
                        
                        row_r = center_row - 1 + (read_count / 3);
                        base_c = center_col - 1;
                        
                        read_addr_offset = get_mirrored_addr(row_r, base_c, mat_width);
                        
                        $display("[DEBUG] Read REQ: block(%d,%d) center(%d,%d) reading row %d from pos(%d,%d) addr=%h", 
                                 block_row, block_col, center_row, center_col, read_count/3, row_r, base_c, src_addr + read_addr_offset);
                        
                        ar_valid <= 1'b1;
                        ar_addr <= src_addr + read_addr_offset;
                        burst_count <= 4'd2; // 3 beats: 0, 1, 2
                        
                        state <= S_READ_DATA;
                    end else if (read_count >= 9) begin
                        $display("[DEBUG] All 3x3 data read, moving to PREPARE");
                        state <= S_PREPARE_BLOCK;
                    end
                end
                
                S_READ_DATA: begin
                    if (ar_handshake) begin
                        ar_valid <= 1'b0;
                        r_ready <= 1'b1;
                    end
                    
                    if (r_handshake) begin
                        buffer_3x3[read_count] <= rdata_i;
                        $display("[DEBUG] Read DATA: buffer[%d] = %d (0x%h)", read_count, rdata_i, rdata_i);
                        read_count <= read_count + 1;
                        burst_count <= burst_count - 1;
                        
                        if (rlast_i) begin
                            r_ready <= 1'b0;
                            state <= S_READ_REQ;
                        end
                    end
                end
                
                S_PREPARE_BLOCK: begin
                    // Calculate 2x2 block values
                    output_block[0] <= calc_output_value(block_row, block_col, mat_width);
                    output_block[1] <= calc_output_value(block_row, block_col + 1, mat_width);
                    output_block[2] <= calc_output_value(block_row + 1, block_col, mat_width);
                    output_block[3] <= calc_output_value(block_row + 1, block_col + 1, mat_width);
                    
                    $display("[DEBUG] PREPARE: block(%d,%d) center(%d,%d)", block_row, block_col, center_row, center_col);
                    $display("[DEBUG] Output positions: TL(%d,%d) TR(%d,%d) BL(%d,%d) BR(%d,%d)",
                             block_row, block_col, block_row, block_col+1, 
                             block_row+1, block_col, block_row+1, block_col+1);
                    
                    // Print 3x3 buffer for debugging
                    $display("[DEBUG] 3x3 Buffer:");
                    $display("  %d %d %d", buffer_3x3[0], buffer_3x3[1], buffer_3x3[2]);
                    $display("  %d %d %d", buffer_3x3[3], buffer_3x3[4], buffer_3x3[5]);
                    $display("  %d %d %d", buffer_3x3[6], buffer_3x3[7], buffer_3x3[8]);
                    
                    // 첫 번째 블록의 경우 상세 디버깅
                    if (block_row == 0 && block_col == 0) begin
                        $display("[DEBUG] First block output calculation:");
                        $display("  TL(0,0) -> value=%d", calc_output_value(block_row, block_col, mat_width));
                        $display("  TR(0,1) -> value=%d", calc_output_value(block_row, block_col + 1, mat_width));
                        $display("  BL(1,0) -> value=%d", calc_output_value(block_row + 1, block_col, mat_width));
                        $display("  BR(1,1) -> value=%d", calc_output_value(block_row + 1, block_col + 1, mat_width));
                    end
                    
                    state <= S_WRITE_REQ;
                    write_count <= 2'd0;
                    current_write_addr <= calc_output_addr(block_row, block_col, mat_width);
                end
                
                S_WRITE_REQ: begin
                    if (!aw_valid) begin
                        aw_valid <= 1'b1;
                        
                        // Calculate address for current write position (TL, TR, BL, BR)
                        case (write_count)
                            2'd0: aw_addr <= calc_output_addr(block_row, block_col, mat_width);           // TL
                            2'd1: aw_addr <= calc_output_addr(block_row, block_col + 1, mat_width);       // TR
                            2'd2: aw_addr <= calc_output_addr(block_row + 1, block_col, mat_width);       // BL
                            2'd3: aw_addr <= calc_output_addr(block_row + 1, block_col + 1, mat_width);   // BR
                        endcase
                        
                        $display("[DEBUG] Write REQ[%d]: addr=%h data=%d", 
                                 write_count, 
                                 (write_count == 0) ? calc_output_addr(block_row, block_col, mat_width) :
                                 (write_count == 1) ? calc_output_addr(block_row, block_col + 1, mat_width) :
                                 (write_count == 2) ? calc_output_addr(block_row + 1, block_col, mat_width) :
                                                      calc_output_addr(block_row + 1, block_col + 1, mat_width),
                                 output_block[write_count]);
                        
                        state <= S_WRITE_DATA;
                    end
                end
                
                S_WRITE_DATA: begin
                    if (aw_handshake) begin
                        aw_valid <= 1'b0;
                        w_valid <= 1'b1;
                        w_data <= output_block[write_count];
                        w_last <= 1'b1;  // Single beat transaction
                        b_ready <= 1'b1;
                        
                        $display("[DEBUG] Write DATA[%d]: sending data=%d", write_count, output_block[write_count]);
                    end
                    
                    if (w_handshake) begin
                        w_valid <= 1'b0;
                        w_last <= 1'b0;
                    end
                    
                    if (b_handshake) begin
                        b_ready <= 1'b0;
                        write_count <= write_count + 1;
                        
                        if (write_count == 2'd3) begin
                            // All 4 writes done
                            $display("[DEBUG] All writes done for block(%d,%d)", block_row, block_col);
                            state <= S_NEXT_BLOCK;
                        end else begin
                            // More writes needed, go back to REQ
                            state <= S_WRITE_REQ;
                        end
                    end
                end
                
                S_WRITE_RESP: begin
                    // This state is no longer needed for single transactions
                    state <= S_NEXT_BLOCK;
                end
                
                S_NEXT_BLOCK: begin
                    // Move to next 2x2 block
                    if (block_col + 2 <= mat_width) begin
                        block_col <= block_col + 2;
                        if (center_col + 2 <= mat_width) begin
                            center_col <= center_col + 2;
                        end else begin
                            center_col <= mat_width;
                        end
                        
                        $display("[DEBUG] Moving to next column: block(%d,%d)->(%d,%d) center(%d,%d)->(%d,%d)", 
                                 block_row, block_col, block_row, block_col + 2,
                                 center_row, center_col, center_row, 
                                 (center_col + 2 <= mat_width) ? center_col + 2 : mat_width);
                    end else begin
                        block_col <= 6'd0;
                        center_col <= 7'd1;
                        
                        block_row <= block_row + 2;
                        if (center_row + 2 <= mat_width) begin
                            center_row <= center_row + 2;
                        end else begin
                            center_row <= mat_width;
                        end
                        
                        $display("[DEBUG] Moving to next row: block(%d,%d)->(%d,0) center(%d,%d)->(%d,1)", 
                                 block_row, block_col, block_row + 2,
                                 center_row, center_col,
                                 (center_row + 2 <= mat_width) ? center_row + 2 : mat_width);
                    end
                    
                    if (block_row + 2 > mat_width + 2) begin
                        $display("[DEBUG] All blocks completed! Going to IDLE");
                        state <= S_IDLE;
                        done <= 1'b1;
                    end else begin
                        $display("[DEBUG] Starting next block read");
                        state <= S_READ_REQ;
                        read_count <= 4'd0;
                        burst_count <= 4'd0;
                    end
                end
            endcase
        end
    end

endmodule 