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

    // State Machine (Project2 스타일 간소화)
    localparam S_IDLE           = 2'd0;
    localparam S_READ_ROW       = 2'd1;
    localparam S_WRITE_ROW      = 2'd2;
    localparam S_DONE           = 2'd3;

    reg [1:0] state;
    
    // Internal registers
    reg [31:0] src_addr;
    reg [31:0] dst_addr;
    reg [5:0]  mat_width;
    reg        done;
    
    // Row buffer to store one row at a time
    reg [31:0] row_buffer [0:31];
    
    // Position counters
    reg [5:0]  row_idx;          // Current output row (0 to mat_width+1)
    reg [5:0]  col_idx;          // Current column being processed
    
    // Read/Write control (Project2 스타일)
    reg        ar_valid;
    reg [31:0] ar_addr;
    reg [3:0]  ar_len;
    reg        r_ready;
    
    reg        aw_valid;
    reg [31:0] aw_addr;
    reg [3:0]  aw_len;
    reg        w_valid;
    reg [31:0] w_data;
    reg        w_last;
    reg        b_ready;
    reg [3:0]  write_cnt;
    
    // Handshake signals (Project2 스타일)
    wire ar_handshake = ar_valid & arready_i;
    wire r_handshake = rvalid_i & r_ready;
    wire aw_handshake = aw_valid & awready_i;
    wire w_handshake = w_valid & wready_i;
    wire b_handshake = bvalid_i & b_ready;
    
    // Output assignments
    assign done_o = done;
    
    // AXI AR channel (Project2 스타일)
    assign arid_o = 4'd0;
    assign araddr_o = ar_addr;
    assign arlen_o = ar_len;
    assign arsize_o = 3'b010; // 4 bytes
    assign arburst_o = 2'b01; // INCR
    assign arvalid_o = ar_valid;
    
    // AXI R channel
    assign rready_o = r_ready;
    
    // AXI AW channel (Project2 스타일)
    assign awid_o = 4'd0;
    assign awaddr_o = aw_addr;
    assign awlen_o = aw_len;
    assign awsize_o = 3'b010; // 4 bytes
    assign awburst_o = 2'b01; // INCR
    assign awvalid_o = aw_valid;
    
    // AXI W channel (Project2 스타일)
    assign wid_o = 4'd0;
    assign wdata_o = w_data;
    assign wstrb_o = 4'hF;
    assign wlast_o = w_last;
    assign wvalid_o = w_valid;
    
    // AXI B channel
    assign bready_o = b_ready;
    
    // Get source row for padding
    function [5:0] get_src_row;
        input [5:0] out_row;
        input [5:0] width;
        begin
            if (out_row == 0) 
                get_src_row = 1;  // Top padding - mirror from row 1
            else if (out_row == width + 1) 
                get_src_row = width - 2;  // Bottom padding - mirror from row width-2
            else 
                get_src_row = out_row - 1;  // Normal row (0-indexed)
        end
    endfunction
    
    // Get padded data value
    function [31:0] get_padded_value;
        input [5:0] out_col;
        input [5:0] width;
        reg [5:0] src_col;
        begin
            // Mirror padding logic for columns
            if (out_col == 0) begin
                // Left padding - mirror from col 1
                src_col = 1;
            end else if (out_col == width + 1) begin
                // Right padding - mirror from col width-2
                src_col = width - 2;
            end else begin
                // Normal col (0-indexed)
                src_col = out_col - 1;
            end
            
            get_padded_value = row_buffer[src_col];
        end
    endfunction
    
    // Main state machine (Project2 스타일)
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_IDLE;
            src_addr <= 32'd0;
            dst_addr <= 32'd0;
            mat_width <= 6'd0;
            done <= 1'b1;
            
            row_idx <= 6'd0;
            col_idx <= 6'd0;
            
            ar_valid <= 1'b0;
            ar_addr <= 32'd0;
            ar_len <= 4'd0;
            r_ready <= 1'b0;
            
            aw_valid <= 1'b0;
            aw_addr <= 32'd0;
            aw_len <= 4'd0;
            w_valid <= 1'b0;
            w_data <= 32'd0;
            w_last <= 1'b0;
            b_ready <= 1'b0;
            write_cnt <= 4'd0;
            
            for (integer i = 0; i < 32; i = i + 1) begin
                row_buffer[i] <= 32'd0;
            end
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 1'b1;
                    if (start_i) begin
                        done <= 1'b0;
                        src_addr <= src_addr_i;
                        dst_addr <= dst_addr_i;
                        mat_width <= mat_width_i;
                        row_idx <= 6'd0;
                        col_idx <= 6'd0;
                        
                        // Start reading first source row
                        state <= S_READ_ROW;
                        ar_valid <= 1'b1;
                        ar_addr <= src_addr_i;
                        ar_len <= mat_width_i - 1;
                        r_ready <= 1'b1;
                    end
                end
                
                S_READ_ROW: begin
                    // AR handshake
                    if (ar_handshake) begin
                        ar_valid <= 1'b0;
                    end
                    
                    // Read data
                    if (r_handshake) begin
                        row_buffer[col_idx] <= rdata_i;
                        col_idx <= col_idx + 1;
                        
                        if (rlast_i) begin
                            col_idx <= 6'd0;
                            r_ready <= 1'b0;
                            
                            // Start writing padded row
                            state <= S_WRITE_ROW;
                            aw_valid <= 1'b1;
                            aw_addr <= dst_addr + (row_idx * (mat_width + 2) * 4);
                            aw_len <= mat_width + 1;  // N+2 elements
                            write_cnt <= 4'd0;
                        end
                    end
                end
                
                S_WRITE_ROW: begin
                    // AW handshake
                    if (aw_handshake) begin
                        aw_valid <= 1'b0;
                        w_valid <= 1'b1;
                        b_ready <= 1'b1;
                        w_data <= get_padded_value(6'd0, mat_width);  // First column
                    end
                    
                    // Write data
                    if (w_handshake) begin
                        write_cnt <= write_cnt + 1;
                        
                        if (write_cnt == mat_width + 1) begin
                            w_last <= 1'b1;
                        end else begin
                            w_last <= 1'b0;
                            w_data <= get_padded_value(write_cnt + 1, mat_width);  // Next column
                        end
                        
                        if (w_last) begin
                            w_valid <= 1'b0;
                            w_last <= 1'b0;
                        end
                    end
                    
                    // B handshake
                    if (b_handshake) begin
                        b_ready <= 1'b0;
                        row_idx <= row_idx + 1;
                        write_cnt <= 4'd0;
                        
                        if (row_idx == mat_width + 1) begin
                            // All rows written
                            state <= S_DONE;
                        end else begin
                            // Check if we need to read a new source row
                            reg [5:0] next_src_row;
                            reg [5:0] curr_src_row;
                            
                            curr_src_row = get_src_row(row_idx, mat_width);
                            next_src_row = get_src_row(row_idx + 1, mat_width);
                            
                            if (next_src_row != curr_src_row) begin
                                // Read new source row
                                state <= S_READ_ROW;
                                ar_valid <= 1'b1;
                                ar_addr <= src_addr + (next_src_row * mat_width * 4);
                                ar_len <= mat_width - 1;
                                r_ready <= 1'b1;
                                col_idx <= 6'd0;
                            end else begin
                                // Same source row, just write again
                                aw_valid <= 1'b1;
                                aw_addr <= dst_addr + ((row_idx + 1) * (mat_width + 2) * 4);
                                aw_len <= mat_width + 1;
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