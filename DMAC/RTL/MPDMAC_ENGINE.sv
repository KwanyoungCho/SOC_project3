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
    localparam S_WRITE_BURST    = 4'd3;
    localparam S_NEXT_BLOCK     = 4'd4;
    localparam S_DONE           = 4'd5;

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
    
    // Write state for burst
    reg [1:0]  write_count;
    reg [31:0] write_base_addr;
    
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
    assign arlen_o = 4'd2;      // Burst of 3 for row-based reading
    assign arsize_o = 3'b010;   // 4 bytes
    assign arburst_o = 2'b01;   // INCR
    assign arvalid_o = ar_valid;
    assign rready_o = r_ready;
    
    // AXI AW channel  
    assign awid_o = 4'd0;
    assign awaddr_o = aw_addr;
    assign awlen_o = 4'd0;      // Single write per transaction (but pipelined)
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
        reg signed [6:0] src_r, src_c;  // 1-based source coordinates
        reg signed [6:0] rel_r, rel_c;  // Relative to center
        begin
            // Convert output coordinates to source coordinates with mirror padding
            // Output matrix is (width+2) x (width+2), Source matrix is width x width
            
            // Row mirroring
            if (out_r == 0) begin
                // Top padding: mirror from row 1 (0-based) -> row 2 (1-based)
                src_r = 2;
            end else if (out_r == width + 1) begin
                // Bottom padding: mirror from row (width-2) (0-based) -> row (width-1) (1-based)
                src_r = width - 1;
            end else begin
                // Normal: out_r maps to src_r (both 1-based in this context)
                src_r = out_r;
            end
            
            // Column mirroring
            if (out_c == 0) begin
                // Left padding: mirror from col 1 (0-based) -> col 2 (1-based)
                src_c = 2;
            end else if (out_c == width + 1) begin
                // Right padding: mirror from col (width-2) (0-based) -> col (width-1) (1-based)
                src_c = width - 1;
            end else begin
                // Normal: out_c maps to src_c (both 1-based in this context)
                src_c = out_c;
            end
            
            // Calculate relative position to current center
            rel_r = src_r - center_row;
            rel_c = src_c - center_col;
            
            // Check if within 3x3 buffer range
            if (rel_r >= -1 && rel_r <= 1 && rel_c >= -1 && rel_c <= 1) begin
                // Convert to buffer index and get value
                calc_output_value = buffer_3x3[(rel_r + 1) * 3 + (rel_c + 1)];
            end else begin
                // Should not happen in correct implementation
                calc_output_value = 32'd0;
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
                        
                        // Start with center at (1,1) for first 2x2 block at (0,0)
                        center_row <= 7'd1;
                        center_col <= 7'd1;
                        
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
                        // Read row by row with burst (3 elements per row)
                        reg signed [6:0] row_base_r, row_base_c;
                        reg [31:0] read_addr_offset;
                        
                        // Calculate base position for current row
                        row_base_r = center_row - 1 + (read_count / 3);
                        row_base_c = center_col - 1;
                        
                        // Calculate read address with mirroring for leftmost element
                        read_addr_offset = get_mirrored_addr(row_base_r, row_base_c, mat_width);
                        
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
                        
                        // Check if we've read all 3 elements of current row
                        if ((read_count % 3) == 2) begin
                            // End of row burst, reset for next row
                            reading_active <= 1'b0;
                            r_ready <= 1'b0;
                        end
                        
                        if (read_count == read_needed - 1) begin
                            // All 3x3 data read, prepare 2x2 block
                            state <= S_PREPARE_BLOCK;
                            read_count <= 4'd0;
                            reading_active <= 1'b0;
                            r_ready <= 1'b0;
                        end
                    end
                end
                
                S_PREPARE_BLOCK: begin
                    // Calculate 2x2 block values with mirror padding
                    output_block[0] <= calc_output_value(block_row, block_col, mat_width);         // TL
                    output_block[1] <= calc_output_value(block_row, block_col + 1, mat_width);     // TR
                    output_block[2] <= calc_output_value(block_row + 1, block_col, mat_width);     // BL
                    output_block[3] <= calc_output_value(block_row + 1, block_col + 1, mat_width); // BR
                    
                    state <= S_WRITE_BURST;
                    write_count <= 2'd0;
                    
                    // Start write transaction for TL
                    aw_valid <= 1'b1;
                    aw_addr <= calc_output_addr(block_row, block_col, mat_width);  // TL address
                end
                
                S_WRITE_BURST: begin
                    if (aw_handshake) begin
                        aw_valid <= 1'b0;
                        w_valid <= 1'b1;
                        b_ready <= 1'b1;
                        
                        case (write_count)
                            2'd0: begin
                                w_data <= output_block[0];  // TL
                                w_last <= 1'b0;
                            end
                            2'd1: begin
                                w_data <= output_block[1];  // TR
                                w_last <= 1'b0;
                            end
                            2'd2: begin
                                w_data <= output_block[2];  // BL
                                w_last <= 1'b0;
                            end
                            2'd3: begin
                                w_data <= output_block[3];  // BR
                                w_last <= 1'b1;
                            end
                        endcase
                    end
                    
                    if (w_handshake) begin
                        write_count <= write_count + 1;
                        
                        if (write_count == 2'd3) begin
                            // Last write data sent
                            w_valid <= 1'b0;
                            w_last <= 1'b0;
                        end else begin
                            // More data to write, start next address
                            case (write_count)
                                2'd0: aw_addr <= calc_output_addr(block_row, block_col + 1, mat_width);     // TR
                                2'd1: aw_addr <= calc_output_addr(block_row + 1, block_col, mat_width);     // BL
                                2'd2: aw_addr <= calc_output_addr(block_row + 1, block_col + 1, mat_width); // BR
                            endcase
                            aw_valid <= 1'b1;
                        end
                    end
                    
                    if (b_handshake && write_count == 2'd3) begin
                        b_ready <= 1'b0;
                        state <= S_NEXT_BLOCK;
                    end
                end
                
                S_NEXT_BLOCK: begin
                    // Move to next 2x2 block
                    // Output matrix is (width+2) x (width+2), so we need blocks at 0,2,4,...,width
                    // For width=8, we need blocks at columns 0,2,4,6,8
                    if (block_col + 2 <= mat_width) begin  // Continue if next block position is valid
                        block_col <= block_col + 2;
                        // Keep center within source matrix bounds (1 to width)
                        if (center_col + 2 <= mat_width) begin
                            center_col <= center_col + 2;
                        end else begin
                            center_col <= mat_width;  // Clamp to last valid column
                        end
                    end else begin
                        // End of row, move to next row
                        block_col <= 6'd0;
                        center_col <= 7'd1;
                        
                        // Move to next row
                        block_row <= block_row + 2;
                        // Keep center within source matrix bounds (1 to width)
                        if (center_row + 2 <= mat_width) begin
                            center_row <= center_row + 2;
                        end else begin
                            center_row <= mat_width;  // Clamp to last valid row
                        end
                    end
                    
                    // Check if done - we need to process all blocks including block_row = width
                    // After incrementing from block_row = width, it becomes width+2 > width
                    if (block_row > mat_width) begin  
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