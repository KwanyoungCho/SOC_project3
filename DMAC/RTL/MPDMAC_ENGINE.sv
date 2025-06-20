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
    localparam S_APPLY_PADDING  = 3'd3;
    localparam S_WRITE_REQ      = 3'd4;
    localparam S_WRITE_DATA     = 3'd5;
    localparam S_NEXT_BLOCK     = 3'd6;

    reg [2:0] state;
    
    // Configuration registers
    reg [31:0] src_addr;
    reg [31:0] dst_addr;
    reg [5:0]  mat_width;
    reg        done;
    
    // Current 4x4 block position in source matrix (0-based)
    reg [5:0]  block_row;  
    reg [5:0]  block_col;  
    
    // 4x4 source buffer and 5x5 padded buffer
    reg [31:0] src_buffer_4x4 [0:15];  // 4x4 source data
    reg [31:0] padded_buffer_5x5 [0:24]; // 5x5 with mirror padding
    
    // Read state
    reg [4:0]  read_count;     // 0-15 for 4x4 reads
    reg [3:0]  burst_count;    // 0-3 for each row burst
    reg [1:0]  read_row;       // Current row being read (0-3)
    
    // Write state 
    reg [4:0]  write_count;    // 0-15 for 4x4 writes
    reg [3:0]  write_burst_count; // 0-3 for each row burst
    reg [1:0]  write_row;      // Current row being written (0-3)
    
    // Temporary variables for calculations
    reg [31:0] row_addr;
    reg [5:0]  current_src_row, output_row;
    reg [4:0]  padded_idx, next_padded_idx;
    reg [2:0]  write_col, next_write_col;
    reg [31:0] write_addr;
    reg [3:0]  buf_idx;
    
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
    assign arlen_o = 4'd3;      // Burst of 4 for each row
    assign arsize_o = 3'b010;   // 4 bytes
    assign arburst_o = 2'b01;   // INCR
    assign arvalid_o = ar_valid;
    assign rready_o = r_ready;
    
    // AXI AW channel  
    assign awid_o = 4'd0;
    assign awaddr_o = aw_addr;
    assign awlen_o = 4'd3;      // Burst of 4 for each row
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
    
    // 소스 매트릭스에서 안전한 값 가져오기 (mirror padding 적용)
    function [31:0] get_src_value;
        input signed [6:0] src_r;  // 0-based source row (can be negative or > width)
        input signed [6:0] src_c;  // 0-based source col (can be negative or > width)
        input [5:0] width;
        reg [5:0] safe_r, safe_c;
        reg [3:0] buf_r, buf_c;
        begin
            // Mirror padding logic for boundaries
            if (src_r < 0) begin
                safe_r = 0;  // Top boundary: use first row
            end else if (src_r >= width) begin
                safe_r = width - 1;  // Bottom boundary: use last row
            end else begin
                safe_r = src_r;  // Normal case
            end
            
            if (src_c < 0) begin
                safe_c = 0;  // Left boundary: use first col
            end else if (src_c >= width) begin
                safe_c = width - 1;  // Right boundary: use last col
            end else begin
                safe_c = src_c;  // Normal case
            end
            
            // Map to current 4x4 buffer coordinates
            // For boundary cases, clamp to buffer edges
            if (safe_r < block_row) begin
                buf_r = 0;  // Use top row of buffer
            end else if (safe_r >= block_row + 4) begin
                buf_r = 3;  // Use bottom row of buffer
            end else begin
                buf_r = safe_r - block_row;
            end
            
            if (safe_c < block_col) begin
                buf_c = 0;  // Use left col of buffer
            end else if (safe_c >= block_col + 4) begin
                buf_c = 3;  // Use right col of buffer
            end else begin
                buf_c = safe_c - block_col;
            end
            
            get_src_value = src_buffer_4x4[buf_r * 4 + buf_c];
        end
    endfunction
    
    // 5x5 패딩 버퍼 채우기
    task apply_mirror_padding;
        integer i, j;
        reg signed [6:0] src_r, src_c;
        begin
            for (i = 0; i < 5; i = i + 1) begin
                for (j = 0; j < 5; j = j + 1) begin
                    // 5x5 버퍼의 (i,j) 위치에 해당하는 소스 좌표 계산
                    // 5x5 버퍼는 4x4 블록 주변에 1픽셀씩 패딩
                    src_r = block_row + i - 1;  // -1, 0, 1, 2, 3, 4
                    src_c = block_col + j - 1;  // -1, 0, 1, 2, 3, 4
                    
                    padded_buffer_5x5[i * 5 + j] = get_src_value(src_r, src_c, mat_width);
                end
            end
        end
    endtask
    
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
            
            read_count <= 5'd0;
            burst_count <= 4'd0;
            read_row <= 2'd0;
            
            write_count <= 5'd0;
            write_burst_count <= 4'd0;
            write_row <= 2'd0;
            
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
                        
                        $display("[DEBUG] Starting DMA: src=%h, dst=%h, width=%d", src_addr_i, dst_addr_i, mat_width_i);
                        
                        state <= S_READ_REQ;
                        read_count <= 5'd0;
                        burst_count <= 4'd0;
                        read_row <= 2'd0;
                    end else begin
                        done <= 1'b1;
                    end
                end
                
                S_READ_REQ: begin
                    if (!ar_valid && (read_row < 4)) begin
                        // Read one row (4 elements) at a time
                        current_src_row = block_row + read_row;
                        
                        // Check bounds and apply mirror padding for address calculation
                        if (current_src_row >= mat_width) begin
                            current_src_row = mat_width - 1;  // Use last row
                        end
                        if (block_col >= mat_width) begin
                            // This should not happen with proper block iteration
                            row_addr = src_addr + (current_src_row * mat_width + (mat_width - 4)) * 4;
                        end else if (block_col + 4 > mat_width) begin
                            // Partial row read - start from safe position
                            row_addr = src_addr + (current_src_row * mat_width + (mat_width - 4)) * 4;
                        end else begin
                            // Normal case
                            row_addr = src_addr + (current_src_row * mat_width + block_col) * 4;
                        end
                        
                        $display("[DEBUG] Read REQ: block(%d,%d) reading row %d from addr=%h", 
                                 block_row, block_col, current_src_row, row_addr);
                        
                        ar_valid <= 1'b1;
                        ar_addr <= row_addr;
                        burst_count <= 4'd3; // 4 beats: 3, 2, 1, 0
                        
                        state <= S_READ_DATA;
                    end else if (read_row >= 4) begin
                        $display("[DEBUG] All 4x4 data read, applying padding");
                        state <= S_APPLY_PADDING;
                        read_row <= 2'd0;  // Reset for next block
                    end
                end
                
                S_READ_DATA: begin
                    if (ar_handshake) begin
                        ar_valid <= 1'b0;
                        r_ready <= 1'b1;
                    end
                    
                    if (r_handshake) begin
                        // Store data in 4x4 buffer
                        buf_idx = read_row * 4 + (3 - burst_count);  // burst_count: 3,2,1,0 → col: 0,1,2,3
                        
                        src_buffer_4x4[buf_idx] <= rdata_i;
                        $display("[DEBUG] Read DATA: buffer[%d] (row=%d,col=%d) = %d", 
                                 buf_idx, read_row, 3 - burst_count, rdata_i);
                        
                        burst_count <= burst_count - 1;
                        
                        if (rlast_i) begin
                            r_ready <= 1'b0;
                            read_row <= read_row + 1;
                            state <= S_READ_REQ;
                        end
                    end
                end
                
                S_APPLY_PADDING: begin
                    // Apply mirror padding to create 5x5 buffer
                    apply_mirror_padding();
                    
                    $display("[DEBUG] Padding applied for block(%d,%d)", block_row, block_col);
                    
                    // Debug: print 5x5 padded buffer
                    $display("[DEBUG] 5x5 Padded Buffer:");
                    $display("  %d %d %d %d %d", padded_buffer_5x5[0], padded_buffer_5x5[1], padded_buffer_5x5[2], padded_buffer_5x5[3], padded_buffer_5x5[4]);
                    $display("  %d %d %d %d %d", padded_buffer_5x5[5], padded_buffer_5x5[6], padded_buffer_5x5[7], padded_buffer_5x5[8], padded_buffer_5x5[9]);
                    $display("  %d %d %d %d %d", padded_buffer_5x5[10], padded_buffer_5x5[11], padded_buffer_5x5[12], padded_buffer_5x5[13], padded_buffer_5x5[14]);
                    $display("  %d %d %d %d %d", padded_buffer_5x5[15], padded_buffer_5x5[16], padded_buffer_5x5[17], padded_buffer_5x5[18], padded_buffer_5x5[19]);
                    $display("  %d %d %d %d %d", padded_buffer_5x5[20], padded_buffer_5x5[21], padded_buffer_5x5[22], padded_buffer_5x5[23], padded_buffer_5x5[24]);
                    
                    state <= S_WRITE_REQ;
                    write_count <= 5'd0;
                    write_burst_count <= 4'd0;
                    write_row <= 2'd0;
                end
                
                S_WRITE_REQ: begin
                    if (!aw_valid && (write_row < 4)) begin
                        // Write one row (4 elements) at a time to output matrix
                        output_row = block_row + write_row;
                        write_addr = calc_output_addr(output_row, block_col, mat_width);
                        
                        $display("[DEBUG] Write REQ: output row %d, addr=%h", output_row, write_addr);
                        
                        aw_valid <= 1'b1;
                        aw_addr <= write_addr;
                        write_burst_count <= 4'd3; // 4 beats: 3, 2, 1, 0
                        
                        state <= S_WRITE_DATA;
                    end else if (write_row >= 4) begin
                        $display("[DEBUG] All 4x4 writes done for block(%d,%d)", block_row, block_col);
                        state <= S_NEXT_BLOCK;
                        write_row <= 2'd0;  // Reset for next block
                    end
                end
                
                S_WRITE_DATA: begin
                    if (aw_handshake) begin
                        aw_valid <= 1'b0;
                        w_valid <= 1'b1;
                        b_ready <= 1'b1;
                        
                        // Set initial data and w_last for first beat
                        write_col = 3 - write_burst_count;  // First beat: col = 0
                        padded_idx = (write_row + 1) * 5 + (write_col + 1);
                        
                        w_data <= padded_buffer_5x5[padded_idx];
                        w_last <= (write_burst_count == 0);
                        
                        $display("[DEBUG] Write DATA start: row=%d col=%d data=%d (from padded[%d])", 
                                 write_row, write_col, padded_buffer_5x5[padded_idx], padded_idx);
                    end
                    
                    if (w_handshake) begin
                        write_burst_count <= write_burst_count - 1;
                        
                        if (write_burst_count > 0) begin
                            // More beats to send
                            next_write_col = 3 - (write_burst_count - 1);
                            next_padded_idx = (write_row + 1) * 5 + (next_write_col + 1);
                            
                            w_data <= padded_buffer_5x5[next_padded_idx];
                            w_last <= (write_burst_count == 1);
                            
                            $display("[DEBUG] Write DATA cont: row=%d col=%d data=%d (from padded[%d])", 
                                     write_row, next_write_col, padded_buffer_5x5[next_padded_idx], next_padded_idx);
                        end else begin
                            // Last beat sent
                            w_valid <= 1'b0;
                            w_last <= 1'b0;
                        end
                    end
                    
                    if (b_handshake) begin
                        b_ready <= 1'b0;
                        write_row <= write_row + 1;
                        state <= S_WRITE_REQ;
                    end
                end
                
                S_NEXT_BLOCK: begin
                    // Move to next 4x4 block
                    if (block_col + 4 < mat_width) begin
                        block_col <= block_col + 4;
                        $display("[DEBUG] Moving to next column: block(%d,%d)", block_row, block_col + 4);
                    end else begin
                        block_col <= 6'd0;
                        block_row <= block_row + 4;
                        $display("[DEBUG] Moving to next row: block(%d,0)", block_row + 4);
                    end
                    
                    if (block_row + 4 >= mat_width) begin
                        $display("[DEBUG] All blocks completed! Going to IDLE");
                        state <= S_IDLE;
                        done <= 1'b1;
                    end else begin
                        $display("[DEBUG] Starting next block read");
                        state <= S_READ_REQ;
                        read_count <= 5'd0;
                        burst_count <= 4'd0;
                        read_row <= 2'd0;
                    end
                end
            endcase
        end
    end

endmodule 