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

    // State Machine for 2x2 buffer iteration
    localparam S_IDLE           = 3'd0;
    localparam S_READ_2X2       = 3'd1;
    localparam S_WRITE_BLOCK    = 3'd2;
    localparam S_WRITE_RESP     = 3'd3;
    localparam S_DONE           = 3'd4;

    reg [2:0] state;
    
    // Internal registers
    reg [31:0] src_addr;
    reg [31:0] dst_addr;
    reg [5:0]  mat_width;
    reg        done;
    
    // 2x2 buffer (as shown in the diagram)
    reg [31:0] buffer_2x2 [0:1][0:1];  // buffer[row][col]
    
    // Position counters for output matrix (including padding)
    reg [5:0]  out_row;     // Current output row (0 to mat_width+1)
    reg [5:0]  out_col;     // Current output column (0 to mat_width+1)
    reg [5:0]  block_row;   // Current 2x2 block row in output
    reg [5:0]  block_col;   // Current 2x2 block col in output
    
    // Read control
    reg        ar_valid;
    reg [31:0] ar_addr;
    reg [3:0]  ar_len;
    reg        r_ready;
    reg [1:0]  read_cnt;    // Counter for 2x2 read (0-3)
    
    // Write control
    reg        aw_valid;
    reg [31:0] aw_addr;
    reg [3:0]  aw_len;
    reg        w_valid;
    reg [31:0] w_data;
    reg        w_last;
    reg        b_ready;
    reg [1:0]  write_cnt;   // Counter for 2x2 write (0-3)
    
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
    assign arsize_o = 3'b010; // 4 bytes
    assign arburst_o = 2'b01; // INCR
    assign arvalid_o = ar_valid;
    
    // AXI R channel
    assign rready_o = r_ready;
    
    // AXI AW channel
    assign awid_o = 4'd0;
    assign awaddr_o = aw_addr;
    assign awlen_o = aw_len;
    assign awsize_o = 3'b010; // 4 bytes
    assign awburst_o = 2'b01; // INCR
    assign awvalid_o = aw_valid;
    
    // AXI W channel
    assign wid_o = 4'd0;
    assign wdata_o = w_data;
    assign wstrb_o = 4'hF;
    assign wlast_o = w_last;
    assign wvalid_o = w_valid;
    
    // AXI B channel
    assign bready_o = b_ready;
    
    // Get source address for 2x2 block
    function [31:0] get_src_addr_2x2;
        input [5:0] out_row_pos;
        input [5:0] out_col_pos;
        input [5:0] width;
        reg [5:0] src_row, src_col;
        begin
            // Convert output position to source position (remove padding offset)
            if (out_row_pos == 0) 
                src_row = 0;  // Top padding uses first row
            else if (out_row_pos == width + 1) 
                src_row = width - 1;  // Bottom padding uses last row
            else 
                src_row = out_row_pos - 1;  // Normal row
                
            if (out_col_pos == 0) 
                src_col = 0;  // Left padding uses first col
            else if (out_col_pos == width + 1) 
                src_col = width - 1;  // Right padding uses last col
            else 
                src_col = out_col_pos - 1;  // Normal col
                
            get_src_addr_2x2 = src_addr + (src_row * width + src_col) * 4;
        end
    endfunction
    
    // Get padded data value using mirror padding logic
    function [31:0] get_padded_value;
        input [5:0] out_row_pos;
        input [5:0] out_col_pos;
        input [5:0] width;
        input [1:0] buf_row;
        input [1:0] buf_col;
        reg [5:0] src_row, src_col;
        reg [1:0] actual_buf_row, actual_buf_col;
        begin
            // Calculate which buffer position to use based on mirror padding
            src_row = out_row_pos;
            src_col = out_col_pos;
            actual_buf_row = buf_row;
            actual_buf_col = buf_col;
            
            // Mirror padding for rows
            if (out_row_pos == 0) begin
                // Top padding - mirror from next row
                actual_buf_row = (buf_row == 0) ? 1 : 0;
            end else if (out_row_pos == width + 1) begin
                // Bottom padding - mirror from previous row  
                actual_buf_row = (buf_row == 1) ? 0 : 1;
            end
            
            // Mirror padding for columns
            if (out_col_pos == 0) begin
                // Left padding - mirror from next col
                actual_buf_col = (buf_col == 0) ? 1 : 0;
            end else if (out_col_pos == width + 1) begin
                // Right padding - mirror from previous col
                actual_buf_col = (buf_col == 1) ? 0 : 1;
            end
            
            get_padded_value = buffer_2x2[actual_buf_row][actual_buf_col];
        end
    endfunction
    
    // Check if we need to read from source memory
    function need_src_read;
        input [5:0] out_row_pos;
        input [5:0] out_col_pos;
        input [5:0] width;
        begin
            // Don't read if we're in padding area
            need_src_read = (out_row_pos > 0) && (out_row_pos <= width) && 
                           (out_col_pos > 0) && (out_col_pos <= width);
        end
    endfunction
    
    // Main state machine for 2x2 buffer processing
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_IDLE;
            src_addr <= 32'd0;
            dst_addr <= 32'd0;
            mat_width <= 6'd0;
            done <= 1'b1;
            
            out_row <= 6'd0;
            out_col <= 6'd0;
            block_row <= 6'd0;
            block_col <= 6'd0;
            
            ar_valid <= 1'b0;
            ar_addr <= 32'd0;
            ar_len <= 4'd0;
            r_ready <= 1'b0;
            read_cnt <= 2'd0;
            
            aw_valid <= 1'b0;
            aw_addr <= 32'd0;
            aw_len <= 4'd0;
            w_valid <= 1'b0;
            w_data <= 32'd0;
            w_last <= 1'b0;
            b_ready <= 1'b0;
            write_cnt <= 2'd0;
            
            // Initialize 2x2 buffer
            buffer_2x2[0][0] <= 32'd0;
            buffer_2x2[0][1] <= 32'd0;
            buffer_2x2[1][0] <= 32'd0;
            buffer_2x2[1][1] <= 32'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 1'b1;
                    if (start_i) begin
                        done <= 1'b0;
                        src_addr <= src_addr_i;
                        dst_addr <= dst_addr_i;
                        mat_width <= mat_width_i;
                        out_row <= 6'd0;
                        out_col <= 6'd0;
                        block_row <= 6'd0;
                        block_col <= 6'd0;
                        
                        // Start with first 2x2 block
                        if (need_src_read(6'd0, 6'd0, mat_width_i)) begin
                            state <= S_READ_2X2;
                            ar_valid <= 1'b1;
                            ar_addr <= get_src_addr_2x2(6'd0, 6'd0, mat_width_i);
                            ar_len <= 4'd3;  // Read 4 elements (2x2)
                            r_ready <= 1'b1;
                            read_cnt <= 2'd0;
                        end else begin
                            // Start with padding area
                            state <= S_WRITE_BLOCK;
                            aw_valid <= 1'b1;
                            aw_addr <= dst_addr_i + (block_row * (mat_width_i + 2) * 2 + block_col * 2) * 4;
                            aw_len <= 4'd3;  // Write 4 elements (2x2)
                            write_cnt <= 2'd0;
                        end
                    end
                end
                
                S_READ_2X2: begin
                    // AR handshake
                    if (ar_handshake) begin
                        ar_valid <= 1'b0;
                    end
                    
                    // Read 2x2 block data
                    if (r_handshake) begin
                        case (read_cnt)
                            2'd0: buffer_2x2[0][0] <= rdata_i;  // Top-left
                            2'd1: buffer_2x2[0][1] <= rdata_i;  // Top-right
                            2'd2: buffer_2x2[1][0] <= rdata_i;  // Bottom-left
                            2'd3: buffer_2x2[1][1] <= rdata_i;  // Bottom-right
                        endcase
                        read_cnt <= read_cnt + 1;
                        
                        if (rlast_i) begin
                            r_ready <= 1'b0;
                            read_cnt <= 2'd0;
                            
                            // Start writing 2x2 block with padding
                            state <= S_WRITE_BLOCK;
                            aw_valid <= 1'b1;
                            aw_addr <= dst_addr + (block_row * (mat_width + 2) * 2 + block_col * 2) * 4;
                            aw_len <= 4'd3;  // Write 4 elements (2x2)
                            write_cnt <= 2'd0;
                        end
                    end
                end
                
                S_WRITE_BLOCK: begin
                    // AW handshake
                    if (aw_handshake) begin
                        aw_valid <= 1'b0;
                        w_valid <= 1'b1;
                        b_ready <= 1'b1;
                        // Set first write data
                        w_data <= get_padded_value(out_row, out_col, mat_width, 2'd0, 2'd0);
                    end
                    
                    // Write 2x2 block data
                    if (w_handshake) begin
                        write_cnt <= write_cnt + 1;
                        
                        case (write_cnt)
                            2'd0: begin
                                // Writing top-left, prepare top-right
                                w_data <= get_padded_value(out_row, out_col + 1, mat_width, 2'd0, 2'd1);
                                w_last <= 1'b0;
                            end
                            2'd1: begin
                                // Writing top-right, prepare bottom-left
                                w_data <= get_padded_value(out_row + 1, out_col, mat_width, 2'd1, 2'd0);
                                w_last <= 1'b0;
                            end
                            2'd2: begin
                                // Writing bottom-left, prepare bottom-right
                                w_data <= get_padded_value(out_row + 1, out_col + 1, mat_width, 2'd1, 2'd1);
                                w_last <= 1'b1;
                            end
                            2'd3: begin
                                // Writing bottom-right, done
                                w_last <= 1'b0;
                                w_valid <= 1'b0;
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
                        if (block_col < (mat_width + 2) / 2 - 1) begin
                            // Move to next column block
                            block_col <= block_col + 1;
                            out_col <= out_col + 2;
                        end else begin
                            // Move to next row block
                            block_col <= 6'd0;
                            out_col <= 6'd0;
                            block_row <= block_row + 1;
                            out_row <= out_row + 2;
                        end
                        
                        // Check if all blocks are done
                        if (block_row >= (mat_width + 2) / 2 - 1 && 
                            block_col >= (mat_width + 2) / 2 - 1) begin
                            state <= S_DONE;
                        end else begin
                            // Determine next block position
                            reg [5:0] next_out_row, next_out_col;
                            if (block_col < (mat_width + 2) / 2 - 1) begin
                                next_out_row = out_row;
                                next_out_col = out_col + 2;
                            end else begin
                                next_out_row = out_row + 2;
                                next_out_col = 6'd0;
                            end
                            
                            // Check if we need to read next 2x2 block
                            if (need_src_read(next_out_row, next_out_col, mat_width)) begin
                                state <= S_READ_2X2;
                                ar_valid <= 1'b1;
                                ar_addr <= get_src_addr_2x2(next_out_row, next_out_col, mat_width);
                                ar_len <= 4'd3;
                                r_ready <= 1'b1;
                                read_cnt <= 2'd0;
                            end else begin
                                // Use padding values from current buffer
                                state <= S_WRITE_BLOCK;
                                aw_valid <= 1'b1;
                                aw_addr <= dst_addr + ((block_row + (block_col < (mat_width + 2) / 2 - 1 ? 0 : 1)) * (mat_width + 2) * 2 + 
                                                      (block_col < (mat_width + 2) / 2 - 1 ? (block_col + 1) * 2 : 0)) * 4;
                                aw_len <= 4'd3;
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