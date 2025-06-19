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
    
    // 2x2 buffer - exactly 4 words for area optimization
    reg [31:0] buffer_2x2 [0:3];  // Linear array: [0]=top-left, [1]=top-right, [2]=bottom-left, [3]=bottom-right
    
    // Current 2x2 block position in output matrix (padded coordinates)
    reg [5:0]  out_row;     // Current output row (0 to mat_width+1, step by 2)
    reg [5:0]  out_col;     // Current output column (0 to mat_width+1, step by 2)
    
    // AXI control signals
    reg        ar_valid;
    reg [31:0] ar_addr;
    reg [3:0]  ar_len;
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
    assign arlen_o = ar_len;
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
    
    // Determine if 2x2 block needs reading from source memory
    function need_read_from_source;
        input [5:0] out_r;
        input [5:0] out_c;
        input [5:0] width;
        begin
            // Need to read if any part of 2x2 block overlaps with original matrix
            need_read_from_source = (out_r > 0 && out_r <= width) && 
                                   (out_c > 0 && out_c <= width);
        end
    endfunction
    
    // Calculate read address for source matrix (always starts from top-left corner of valid area)
    function [31:0] calc_read_addr;
        input [5:0] out_r;
        input [5:0] out_c;
        input [5:0] width;
        reg [5:0] src_r, src_c;
        begin
            // Start reading from top-left corner of overlapping area
            src_r = (out_r > 0) ? (out_r - 1) : 0;
            src_c = (out_c > 0) ? (out_c - 1) : 0;
            
            // Clamp to valid source range
            if (src_r >= width) src_r = width - 1;
            if (src_c >= width) src_c = width - 1;
            
            calc_read_addr = src_addr + (src_r * width + src_c) * 4;
        end
    endfunction
    
    // Get padded value for specific position using mirror padding
    function [31:0] get_output_value;
        input [5:0] out_r;
        input [5:0] out_c;
        input [5:0] width;
        input [1:0] pos; // 0=top-left, 1=top-right, 2=bottom-left, 3=bottom-right
        reg [5:0] abs_row, abs_col;
        reg [5:0] src_row, src_col;
        reg [5:0] read_base_r, read_base_c;
        reg [1:0] buf_idx;
        begin
            // Calculate absolute position in output matrix
            case (pos)
                2'd0: begin
                    abs_row = out_r;
                    abs_col = out_c;
                end
                2'd1: begin
                    abs_row = out_r;
                    abs_col = out_c + 1;
                end
                2'd2: begin
                    abs_row = out_r + 1;
                    abs_col = out_c;
                end
                2'd3: begin
                    abs_row = out_r + 1;
                    abs_col = out_c + 1;
                end
            endcase
            
            // Apply mirror padding to get source coordinates
            if (abs_row == 0) begin
                // Top padding - mirror from row 1 (index 0 in source)
                src_row = 1;
            end else if (abs_row == width + 1) begin
                // Bottom padding - mirror from row width (index width-1 in source)
                src_row = width;
            end else begin
                // Normal area (1-based to 0-based conversion)
                src_row = abs_row;
            end
            
            if (abs_col == 0) begin
                // Left padding - mirror from col 1 (index 0 in source)
                src_col = 1;
            end else if (abs_col == width + 1) begin
                // Right padding - mirror from col width (index width-1 in source)
                src_col = width;
            end else begin
                // Normal area (1-based to 0-based conversion)
                src_col = abs_col;
            end
            
            // Calculate where we read the buffer from (what was the base read position)
            read_base_r = (out_r > 0) ? (out_r - 1) : 0;
            read_base_c = (out_c > 0) ? (out_c - 1) : 0;
            if (read_base_r >= width) read_base_r = width - 1;
            if (read_base_c >= width) read_base_c = width - 1;
            
            // Map source position to buffer index based on read base
            if (src_row == read_base_r + 1 && src_col == read_base_c + 1) buf_idx = 2'd0;     // top-left
            else if (src_row == read_base_r + 1 && src_col == read_base_c + 2) buf_idx = 2'd1; // top-right  
            else if (src_row == read_base_r + 2 && src_col == read_base_c + 1) buf_idx = 2'd2; // bottom-left
            else if (src_row == read_base_r + 2 && src_col == read_base_c + 2) buf_idx = 2'd3; // bottom-right
            else buf_idx = 2'd0; // fallback
            
            get_output_value = buffer_2x2[buf_idx];
        end
    endfunction
    
    // Calculate destination address for 2x2 block write
    function [31:0] calc_write_addr;
        input [5:0] out_r;
        input [5:0] out_c;
        input [5:0] width;
        begin
            calc_write_addr = dst_addr + (out_r * (width + 2) + out_c) * 4;
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
            
            out_row <= 6'd0;
            out_col <= 6'd0;
            
            ar_valid <= 1'b0;
            ar_addr <= 32'd0;
            ar_len <= 4'd0;
            r_ready <= 1'b0;
            read_cnt <= 2'd0;
            
            aw_valid <= 1'b0;
            aw_addr <= 32'd0;
            w_valid <= 1'b0;
            w_data <= 32'd0;
            w_last <= 1'b0;
            b_ready <= 1'b0;
            write_cnt <= 2'd0;
            
            buffer_2x2[0] <= 32'd0;
            buffer_2x2[1] <= 32'd0;
            buffer_2x2[2] <= 32'd0;
            buffer_2x2[3] <= 32'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (start_i) begin
                        done <= 1'b0;
                        src_addr <= src_addr_i;
                        dst_addr <= dst_addr_i;
                        mat_width <= mat_width_i;
                        out_row <= 6'd0;
                        out_col <= 6'd0;
                        
                        // Check if first block needs reading
                        if (need_read_from_source(6'd0, 6'd0, mat_width_i)) begin
                            state <= S_READ_2X2;
                            ar_valid <= 1'b1;
                            ar_addr <= calc_read_addr(6'd0, 6'd0, mat_width_i);
                            ar_len <= 4'd3; // Read 4 elements
                            r_ready <= 1'b1;
                            read_cnt <= 2'd0;
                        end else begin
                            // All padding, use default values
                            state <= S_WRITE_BLOCK;
                            aw_valid <= 1'b1;
                            aw_addr <= calc_write_addr(6'd0, 6'd0, mat_width_i);
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
                        buffer_2x2[read_cnt] <= rdata_i;
                        read_cnt <= read_cnt + 1;
                        
                        if (rlast_i) begin
                            r_ready <= 1'b0;
                            read_cnt <= 2'd0;
                            state <= S_WRITE_BLOCK;
                            aw_valid <= 1'b1;
                            aw_addr <= calc_write_addr(out_row, out_col, mat_width);
                            write_cnt <= 2'd0;
                        end
                    end
                end
                
                S_WRITE_BLOCK: begin
                    if (aw_handshake) begin
                        aw_valid <= 1'b0;
                        w_valid <= 1'b1;
                        b_ready <= 1'b1;
                        w_data <= get_output_value(out_row, out_col, mat_width, 2'd0);
                        w_last <= 1'b0;
                    end
                    
                    if (w_handshake) begin
                        write_cnt <= write_cnt + 1;
                        
                        case (write_cnt)
                            2'd0: begin
                                w_data <= get_output_value(out_row, out_col, mat_width, 2'd1);
                                w_last <= 1'b0;
                            end
                            2'd1: begin
                                w_data <= get_output_value(out_row, out_col, mat_width, 2'd2);
                                w_last <= 1'b0;
                            end
                            2'd2: begin
                                w_data <= get_output_value(out_row, out_col, mat_width, 2'd3);
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
                        
                        // Move to next 2x2 block
                        if (out_col < mat_width) begin
                            out_col <= out_col + 2;
                        end else begin
                            out_col <= 6'd0;
                            out_row <= out_row + 2;
                        end
                        
                        // Check if done
                        if (out_row >= mat_width && out_col >= mat_width) begin
                            state <= S_DONE;
                        end else begin
                            // Prepare next block coordinates
                            reg [5:0] next_row, next_col;
                            if (out_col < mat_width) begin
                                next_row = out_row;
                                next_col = out_col + 2;
                            end else begin
                                next_row = out_row + 2;
                                next_col = 6'd0;
                            end
                            
                            // Check if next block needs reading
                            if (need_read_from_source(next_row, next_col, mat_width)) begin
                                state <= S_READ_2X2;
                                ar_valid <= 1'b1;
                                ar_addr <= calc_read_addr(next_row, next_col, mat_width);
                                ar_len <= 4'd3;
                                r_ready <= 1'b1;
                                read_cnt <= 2'd0;
                            end else begin
                                // All padding for this block
                                state <= S_WRITE_BLOCK;
                                aw_valid <= 1'b1;
                                aw_addr <= calc_write_addr(next_row, next_col, mat_width);
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