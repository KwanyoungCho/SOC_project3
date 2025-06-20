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
    localparam S_READ_ADDR      = 3'd1;
    localparam S_READ_DATA      = 3'd2;
    localparam S_APPLY_PADDING  = 3'd3;
    localparam S_WRITE_ADDR     = 3'd4;
    localparam S_WRITE_DATA     = 3'd5;
    localparam S_WRITE_RESP     = 3'd6;
    localparam S_NEXT_BLOCK     = 3'd7;

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
    
    // Read state - similar to SGDMAC_READ
    reg [1:0]  read_row;       // Current row being read (0-3)
    reg [3:0]  read_burst_cnt; // Burst counter for current row
    reg [31:0] read_addr;      // Current read address
    
    // Write state - similar to SGDMAC_WRITE
    reg [1:0]  write_row;      // Current row being written (0-3)
    reg [3:0]  write_burst_cnt; // Burst counter for current row
    reg [31:0] write_addr;     // Current write address
    
    // Handshake signals
    wire ar_handshake = arvalid_o & arready_i;
    wire r_handshake = rvalid_i & rready_o;
    wire aw_handshake = awvalid_o & awready_i;
    wire w_handshake = wvalid_o & wready_i;
    wire b_handshake = bvalid_i & bready_o;
    
    // Control signals
    wire burst_complete = r_handshake & rlast_i;
    wire write_burst_done = w_handshake & wlast_o;
    wire is_last_write_beat = (write_burst_cnt == 4'd0);
    
    // Output assignments
    assign done_o = done;
    
    // AXI AR channel - similar to SGDMAC_READ
    assign arid_o = 4'd0;
    assign araddr_o = read_addr;
    assign arlen_o = 4'd3;      // Always 4 beats for 4x4 read
    assign arsize_o = 3'b010;   // 4 bytes
    assign arburst_o = 2'b01;   // INCR
    assign arvalid_o = (state == S_READ_ADDR);
    assign rready_o = (state == S_READ_DATA);
    
    // AXI AW channel - similar to SGDMAC_WRITE
    assign awid_o = 4'd0;
    assign awaddr_o = write_addr;
    assign awlen_o = 4'd3;      // Always 4 beats for 4x4 write
    assign awsize_o = 3'b010;   // 4 bytes
    assign awburst_o = 2'b01;   // INCR
    assign awvalid_o = (state == S_WRITE_ADDR);
    
    // AXI W channel - similar to SGDMAC_WRITE
    assign wid_o = 4'd0;
    assign wdata_o = padded_buffer_5x5[(write_row + 1) * 5 + (3 - write_burst_cnt + 1)];
    assign wstrb_o = 4'hF;
    assign wlast_o = (state == S_WRITE_DATA) & is_last_write_beat;
    assign wvalid_o = (state == S_WRITE_DATA);
    assign bready_o = (state == S_WRITE_DATA) | (state == S_WRITE_RESP);
    
    // Mirror padding function
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
    
    // Apply mirror padding to create 5x5 buffer
    task apply_mirror_padding;
        integer i, j;
        reg signed [6:0] src_r, src_c;
        begin
            for (i = 0; i < 5; i = i + 1) begin
                for (j = 0; j < 5; j = j + 1) begin
                    // 5x5 버퍼의 (i,j) 위치에 해당하는 소스 좌표 계산
                    src_r = block_row + i - 1;  // -1, 0, 1, 2, 3, 4
                    src_c = block_col + j - 1;  // -1, 0, 1, 2, 3, 4
                    
                    padded_buffer_5x5[i * 5 + j] = get_src_value(src_r, src_c, mat_width);
                end
            end
        end
    endtask
    
    // Calculate output address
    function [31:0] calc_output_addr;
        input [5:0] out_r;  // 0-based output row
        input [5:0] out_c;  // 0-based output col
        input [5:0] width;
        begin
            // Output matrix size: (width+2) x (width+2)
            calc_output_addr = dst_addr + (out_r * (width + 2) + out_c) * 4;
        end
    endfunction
    
    // Main state machine - based on SGDMAC pattern
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_IDLE;
            src_addr <= 32'd0;
            dst_addr <= 32'd0;
            mat_width <= 6'd0;
            done <= 1'b1;
            
            block_row <= 6'd0;
            block_col <= 6'd0;
            
            read_row <= 2'd0;
            read_burst_cnt <= 4'd0;
            read_addr <= 32'd0;
            
            write_row <= 2'd0;
            write_burst_cnt <= 4'd0;
            write_addr <= 32'd0;
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
                        read_row <= 2'd0;
                        
                        // Calculate first read address
                        read_addr <= src_addr_i + (block_row * mat_width_i + block_col) * 4;
                        
                        $display("[DEBUG] Starting DMA: src=%h, dst=%h, width=%d", src_addr_i, dst_addr_i, mat_width_i);
                        state <= S_READ_ADDR;
                    end else begin
                        done <= 1'b1;
                    end
                end
                
                S_READ_ADDR: begin
                    if (ar_handshake) begin
                        state <= S_READ_DATA;
                        read_burst_cnt <= 4'd3; // 4 beats: 3, 2, 1, 0
                        
                        $display("[DEBUG] Read ADDR: block(%d,%d) row %d addr=%h", 
                                 block_row, block_col, read_row, read_addr);
                    end
                end
                
                S_READ_DATA: begin
                    if (r_handshake) begin
                        // Store data in 4x4 buffer
                        src_buffer_4x4[read_row * 4 + (3 - read_burst_cnt)] <= rdata_i;
                        read_burst_cnt <= read_burst_cnt - 1;
                        
                        $display("[DEBUG] Read DATA: buffer[%d] = %d", 
                                 read_row * 4 + (3 - read_burst_cnt), rdata_i);
                        
                        if (burst_complete) begin
                            read_row <= read_row + 1;
                            
                            if (read_row == 2'd3) begin
                                // All 4 rows read, apply padding
                                $display("[DEBUG] All 4x4 data read, applying padding");
                                state <= S_APPLY_PADDING;
                            end else begin
                                // Read next row
                                read_addr <= src_addr + ((block_row + read_row + 1) * mat_width + block_col) * 4;
                                state <= S_READ_ADDR;
                            end
                        end
                    end
                end
                
                S_APPLY_PADDING: begin
                    // Apply mirror padding to create 5x5 buffer
                    apply_mirror_padding();
                    
                    $display("[DEBUG] Padding applied for block(%d,%d)", block_row, block_col);
                    
                    // Initialize write state
                    write_row <= 2'd0;
                    write_addr <= calc_output_addr(block_row, block_col, mat_width);
                    
                    state <= S_WRITE_ADDR;
                end
                
                S_WRITE_ADDR: begin
                    if (aw_handshake) begin
                        state <= S_WRITE_DATA;
                        write_burst_cnt <= 4'd3; // 4 beats: 3, 2, 1, 0
                        
                        $display("[DEBUG] Write ADDR: row %d addr=%h", write_row, write_addr);
                    end
                end
                
                S_WRITE_DATA: begin
                    if (w_handshake) begin
                        write_burst_cnt <= write_burst_cnt - 1;
                        
                        $display("[DEBUG] Write DATA: data=%d", wdata_o);
                        
                        if (write_burst_done) begin
                            if (b_handshake) begin
                                write_row <= write_row + 1;
                                
                                if (write_row == 2'd3) begin
                                    // All 4 rows written
                                    state <= S_NEXT_BLOCK;
                                end else begin
                                    // Write next row
                                    write_addr <= calc_output_addr(block_row + write_row + 1, block_col, mat_width);
                                    state <= S_WRITE_ADDR;
                                end
                            end else begin
                                state <= S_WRITE_RESP;
                            end
                        end
                    end
                end
                
                S_WRITE_RESP: begin
                    if (b_handshake) begin
                        write_row <= write_row + 1;
                        
                        if (write_row == 2'd3) begin
                            // All 4 rows written
                            state <= S_NEXT_BLOCK;
                        end else begin
                            // Write next row
                            write_addr <= calc_output_addr(block_row + write_row + 1, block_col, mat_width);
                            state <= S_WRITE_ADDR;
                        end
                    end
                end
                
                S_NEXT_BLOCK: begin
                    // Move to next 4x4 block
                    if (block_col + 4 < mat_width) begin
                        block_col <= block_col + 4;
                        read_addr <= src_addr + (block_row * mat_width + block_col + 4) * 4;
                        $display("[DEBUG] Moving to next column: block(%d,%d)", block_row, block_col + 4);
                    end else begin
                        block_col <= 6'd0;
                        block_row <= block_row + 4;
                        read_addr <= src_addr + ((block_row + 4) * mat_width + 6'd0) * 4;
                        $display("[DEBUG] Moving to next row: block(%d,0)", block_row + 4);
                    end
                    
                    if (block_row + 4 >= mat_width) begin
                        $display("[DEBUG] All blocks completed! Going to IDLE");
                        state <= S_IDLE;
                        done <= 1'b1;
                    end else begin
                        $display("[DEBUG] Starting next block read");
                        read_row <= 2'd0;
                        state <= S_READ_ADDR;
                    end
                end
            endcase
        end
    end

endmodule 