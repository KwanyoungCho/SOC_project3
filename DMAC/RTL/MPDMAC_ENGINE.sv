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
    localparam S_READ_2X2       = 3'd1;
    localparam S_WRITE_BLOCK    = 3'd2;
    localparam S_WRITE_RESP     = 3'd3;
    localparam S_DONE           = 3'd4;

    reg [2:0] state;
    
    // Configuration registers
    reg [31:0] src_addr;
    reg [31:0] dst_addr;
    reg [5:0]  mat_width;
    reg        done;
    
    // 2x2 buffer - only 4 words for area optimization
    reg [31:0] buffer_2x2 [0:1][0:1];
    
    // Block position in output matrix (in 2x2 block units)
    reg [5:0]  block_row;
    reg [5:0]  block_col;
    
    // Total blocks in each dimension
    wire [5:0] total_blocks = (mat_width + 2 + 1) / 2;  // (N+2)/2 rounded up
    
    // AXI control signals
    reg        ar_valid;
    reg [31:0] ar_addr;
    reg        r_ready;
    reg [1:0]  read_cnt;
    
    reg        aw_valid;
    reg [31:0] aw_addr;
    reg        w_valid;
    reg [31:0] w_data;
    reg        w_last;
    reg        b_ready;
    reg [1:0]  write_cnt;
    
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
    assign arlen_o = 4'd3;      // Always read 4 elements for 2x2
    assign arsize_o = 3'b010;   // 4 bytes
    assign arburst_o = 2'b01;   // INCR
    assign arvalid_o = ar_valid;
    assign rready_o = r_ready;
    
    // AXI AW channel
    assign awid_o = 4'd0;
    assign awaddr_o = aw_addr;
    assign awlen_o = 4'd3;      // Always write 4 elements for 2x2
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
    
    // Calculate source address for a 2x2 block
    function [31:0] calc_src_addr;
        input [5:0] blk_row;
        input [5:0] blk_col;
        input [5:0] width;
        reg [5:0] src_row, src_col;
        begin
            // Convert block coordinates to source matrix coordinates
            src_row = blk_row * 2 - 1;  // -1 because output is padded
            src_col = blk_col * 2 - 1;
            
            // Clamp to valid source range [0, width-1]
            if (src_row >= width) src_row = width - 1;
            if (src_col >= width) src_col = width - 1;
            
            calc_src_addr = src_addr + (src_row * width + src_col) * 4;
        end
    endfunction
    
    // Calculate destination address for a 2x2 block
    function [31:0] calc_dst_addr;
        input [5:0] blk_row;
        input [5:0] blk_col;
        input [5:0] width;
        reg [5:0] dst_row, dst_col;
        begin
            dst_row = blk_row * 2;
            dst_col = blk_col * 2;
            calc_dst_addr = dst_addr + (dst_row * (width + 2) + dst_col) * 4;
        end
    endfunction
    
    // Check if we need to read from source (not in padding area)
    function need_read;
        input [5:0] blk_row;
        input [5:0] blk_col;
        input [5:0] width;
        begin
            // Need to read if any part of 2x2 block overlaps with source matrix
            need_read = (blk_row * 2 < width + 1) && (blk_col * 2 < width + 1) &&
                       (blk_row * 2 + 1 >= 1) && (blk_col * 2 + 1 >= 1);
        end
    endfunction
    
    // Get padded value for specific position in 2x2 block
    function [31:0] get_padded_value;
        input [5:0] blk_row;
        input [5:0] blk_col;
        input [5:0] width;
        input [1:0] pos_row;  // 0 or 1 (within 2x2 block)
        input [1:0] pos_col;  // 0 or 1 (within 2x2 block)
        reg [5:0] out_row, out_col;
        reg [5:0] src_row, src_col;
        reg [1:0] buf_row, buf_col;
        begin
            // Calculate absolute position in output matrix
            out_row = blk_row * 2 + pos_row;
            out_col = blk_col * 2 + pos_col;
            
            // Mirror padding logic
            if (out_row == 0) begin
                // Top padding - mirror from row 1
                src_row = 1;
            end else if (out_row == width + 1) begin
                // Bottom padding - mirror from row width
                src_row = width;
            end else begin
                // Normal area
                src_row = out_row;
            end
            
            if (out_col == 0) begin
                // Left padding - mirror from col 1
                src_col = 1;
            end else if (out_col == width + 1) begin
                // Right padding - mirror from col width
                src_col = width;
            end else begin
                // Normal area
                src_col = out_col;
            end
            
            // Convert back to buffer coordinates
            buf_row = src_row - (blk_row * 2 - 1);
            buf_col = src_col - (blk_col * 2 - 1);
            
            // Handle boundary cases for buffer indexing
            if (buf_row > 1) buf_row = 1;
            if (buf_col > 1) buf_col = 1;
            if (buf_row[1]) buf_row = 0;  // Handle negative (underflow)
            if (buf_col[1]) buf_col = 0;  // Handle negative (underflow)
            
            get_padded_value = buffer_2x2[buf_row][buf_col];
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
            
            ar_valid <= 1'b0;
            ar_addr <= 32'd0;
            r_ready <= 1'b0;
            read_cnt <= 2'd0;
            
            aw_valid <= 1'b0;
            aw_addr <= 32'd0;
            w_valid <= 1'b0;
            w_data <= 32'd0;
            w_last <= 1'b0;
            b_ready <= 1'b0;
            write_cnt <= 2'd0;
            
            buffer_2x2[0][0] <= 32'd0;
            buffer_2x2[0][1] <= 32'd0;
            buffer_2x2[1][0] <= 32'd0;
            buffer_2x2[1][1] <= 32'd0;
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
                        
                        // Start processing first 2x2 block
                        if (need_read(6'd0, 6'd0, mat_width_i)) begin
                            state <= S_READ_2X2;
                            ar_valid <= 1'b1;
                            ar_addr <= calc_src_addr(6'd0, 6'd0, mat_width_i);
                            r_ready <= 1'b1;
                            read_cnt <= 2'd0;
                        end else begin
                            // First block is all padding
                            state <= S_WRITE_BLOCK;
                            aw_valid <= 1'b1;
                            aw_addr <= calc_dst_addr(6'd0, 6'd0, mat_width_i);
                            write_cnt <= 2'd0;
                        end
                    end else begin
                        done <= 1'b1;
                    end
                end
                
                S_READ_2X2: begin
                    if (ar_handshake) begin
                        ar_valid <= 1'b0;
                    end
                    
                    if (r_handshake) begin
                        case (read_cnt)
                            2'd0: buffer_2x2[0][0] <= rdata_i;
                            2'd1: buffer_2x2[0][1] <= rdata_i;
                            2'd2: buffer_2x2[1][0] <= rdata_i;
                            2'd3: buffer_2x2[1][1] <= rdata_i;
                        endcase
                        read_cnt <= read_cnt + 1;
                        
                        if (rlast_i) begin
                            r_ready <= 1'b0;
                            read_cnt <= 2'd0;
                            state <= S_WRITE_BLOCK;
                            aw_valid <= 1'b1;
                            aw_addr <= calc_dst_addr(block_row, block_col, mat_width);
                            write_cnt <= 2'd0;
                        end
                    end
                end
                
                S_WRITE_BLOCK: begin
                    if (aw_handshake) begin
                        aw_valid <= 1'b0;
                        w_valid <= 1'b1;
                        b_ready <= 1'b1;
                        w_data <= get_padded_value(block_row, block_col, mat_width, 2'd0, 2'd0);
                        w_last <= 1'b0;
                    end
                    
                    if (w_handshake) begin
                        write_cnt <= write_cnt + 1;
                        
                        case (write_cnt)
                            2'd0: begin
                                w_data <= get_padded_value(block_row, block_col, mat_width, 2'd0, 2'd1);
                                w_last <= 1'b0;
                            end
                            2'd1: begin
                                w_data <= get_padded_value(block_row, block_col, mat_width, 2'd1, 2'd0);
                                w_last <= 1'b0;
                            end
                            2'd2: begin
                                w_data <= get_padded_value(block_row, block_col, mat_width, 2'd1, 2'd1);
                                w_last <= 1'b1;
                            end
                            2'd3: begin
                                w_valid <= 1'b0;
                                w_last <= 1'b0;
                            end
                        endcase
                        
                        if (w_last) begin
                            state <= S_WRITE_RESP;
                        end
                    end
                end
                
                S_WRITE_RESP: begin
                    if (b_handshake) begin
                        b_ready <= 1'b0;
                        write_cnt <= 2'd0;
                        
                        // Move to next block
                        if (block_col < total_blocks - 1) begin
                            block_col <= block_col + 1;
                        end else begin
                            block_col <= 6'd0;
                            block_row <= block_row + 1;
                        end
                        
                        // Check if done
                        if (block_row >= total_blocks - 1 && block_col >= total_blocks - 1) begin
                            state <= S_DONE;
                        end else begin
                            // Prepare next block
                            reg [5:0] next_row, next_col;
                            if (block_col < total_blocks - 1) begin
                                next_row = block_row;
                                next_col = block_col + 1;
                            end else begin
                                next_row = block_row + 1;
                                next_col = 6'd0;
                            end
                            
                            if (need_read(next_row, next_col, mat_width)) begin
                                state <= S_READ_2X2;
                                ar_valid <= 1'b1;
                                ar_addr <= calc_src_addr(next_row, next_col, mat_width);
                                r_ready <= 1'b1;
                                read_cnt <= 2'd0;
                            end else begin
                                state <= S_WRITE_BLOCK;
                                aw_valid <= 1'b1;
                                aw_addr <= calc_dst_addr(next_row, next_col, mat_width);
                                write_cnt <= 2'd0;
                            end
                        end
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