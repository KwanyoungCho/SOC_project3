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

    // State Machine - 행별로 처리
    localparam S_IDLE           = 2'd0;
    localparam S_READ_ROW       = 2'd1;
    localparam S_WRITE_ROW      = 2'd2;
    localparam S_NEXT_ROW       = 2'd3;

    reg [1:0] state;
    
    // Configuration registers
    reg [31:0] src_addr;
    reg [31:0] dst_addr;
    reg [5:0]  mat_width;
    reg        done;
    
    // Current processing row (0 to mat_width+1)
    reg [5:0]  current_row;
    
    // Source row buffer (entire row)
    reg [31:0] src_row_buffer [0:63]; // 최대 64개 원소
    
    // Read state
    reg [5:0]  read_cnt;
    reg [31:0] read_addr;
    
    // Write state 
    reg [5:0]  write_cnt;
    reg [31:0] write_addr;
    reg [31:0] write_data;
    
    // Handshake signals
    wire ar_handshake = arvalid_o & arready_i;
    wire r_handshake = rvalid_i & rready_o;
    wire aw_handshake = awvalid_o & awready_i;
    wire w_handshake = wvalid_o & wready_i;
    wire b_handshake = bvalid_i & bready_o;
    
    // Control signals
    wire read_complete = r_handshake & rlast_i;
    wire write_complete = w_handshake & wlast_o;
    wire is_last_read = (read_cnt == mat_width - 1);
    wire is_last_write = (write_cnt == mat_width + 1);
    
    // Output assignments
    assign done_o = done;
    
    // AXI AR channel
    assign arid_o = 4'd0;
    assign araddr_o = read_addr;
    assign arlen_o = mat_width - 1;  // Read entire source row
    assign arsize_o = 3'b010;        // 4 bytes
    assign arburst_o = 2'b01;        // INCR
    assign arvalid_o = (state == S_READ_ROW) && (current_row < mat_width);
    assign rready_o = (state == S_READ_ROW);
    
    // AXI AW channel
    assign awid_o = 4'd0;
    assign awaddr_o = write_addr;
    assign awlen_o = mat_width + 1;  // Write entire output row (width+2)
    assign awsize_o = 3'b010;        // 4 bytes
    assign awburst_o = 2'b01;        // INCR
    assign awvalid_o = (state == S_WRITE_ROW);
    
    // AXI W channel
    assign wid_o = 4'd0;
    assign wdata_o = write_data;
    assign wstrb_o = 4'hF;
    assign wlast_o = (state == S_WRITE_ROW) & is_last_write;
    assign wvalid_o = (state == S_WRITE_ROW);
    assign bready_o = (state == S_WRITE_ROW);
    
    // Mirror padding function - 출력 위치에서 소스 값 계산
    function [31:0] get_padded_value;
        input [5:0] out_row;  // 0-based output row (0 to width+1)
        input [5:0] out_col;  // 0-based output col (0 to width+1)  
        input [5:0] width;
        reg [5:0] src_row, src_col;
        begin
            // Mirror padding mapping
            if (out_row == 0) begin
                src_row = 1;  // out[0] -> src[1]
            end else if (out_row == width + 1) begin
                src_row = width - 2;  // out[width+1] -> src[width-2]
            end else begin
                src_row = out_row - 1;  // out[1..width] -> src[0..width-1]
            end
            
            if (out_col == 0) begin
                src_col = 1;  // out[0] -> src[1]  
            end else if (out_col == width + 1) begin
                src_col = width - 2;  // out[width+1] -> src[width-2]
            end else begin
                src_col = out_col - 1;  // out[1..width] -> src[0..width-1]
            end
            
            // Get value from appropriate source row buffer
            if (src_row == current_row && current_row < width) begin
                get_padded_value = src_row_buffer[src_col];
            end else begin
                // For boundary rows, use previously read data or repeat logic
                if (current_row == 0) begin
                    // First output row (all mirror from src row 1)
                    get_padded_value = src_row_buffer[src_col];
                end else if (current_row == width + 1) begin
                    // Last output row (all mirror from src row width-2) 
                    get_padded_value = src_row_buffer[src_col];
                end else begin
                    get_padded_value = src_row_buffer[src_col];
                end
            end
        end
    endfunction
    
    // Calculate addresses
    function [31:0] calc_src_addr;
        input [5:0] row;
        input [5:0] width;
        begin
            calc_src_addr = src_addr + (row * width) * 4;
        end
    endfunction
    
    function [31:0] calc_dst_addr;
        input [5:0] row;
        input [5:0] width;
        begin
            calc_dst_addr = dst_addr + (row * (width + 2)) * 4;
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
            current_row <= 6'd0;
            read_cnt <= 6'd0;
            write_cnt <= 6'd0;
            read_addr <= 32'd0;
            write_addr <= 32'd0;
            write_data <= 32'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (start_i) begin
                        done <= 1'b0;
                        src_addr <= src_addr_i;
                        dst_addr <= dst_addr_i;
                        mat_width <= mat_width_i;
                        current_row <= 6'd0;
                        
                        $display("[DEBUG] Starting DMA: src=%h, dst=%h, width=%d", src_addr_i, dst_addr_i, mat_width_i);
                        
                        // Start with first row processing
                        read_addr <= calc_src_addr(6'd1, mat_width_i); // Read src row 1 for output row 0
                        state <= S_READ_ROW;
                        read_cnt <= 6'd0;
                    end else begin
                        done <= 1'b1;
                    end
                end
                
                S_READ_ROW: begin
                    if (current_row < mat_width) begin
                        // Read source row
                        if (ar_handshake) begin
                            $display("[DEBUG] Read ROW: output_row=%d, src_row=%d, addr=%h", 
                                     current_row, 
                                     (current_row == 0) ? 1 : 
                                     (current_row == mat_width + 1) ? mat_width - 2 : current_row - 1, 
                                     read_addr);
                        end
                        
                        if (r_handshake) begin
                            src_row_buffer[read_cnt] <= rdata_i;
                            read_cnt <= read_cnt + 1;
                            
                            $display("[DEBUG] Read DATA: buffer[%d] = %d", read_cnt, rdata_i);
                            
                            if (read_complete) begin
                                state <= S_WRITE_ROW;
                                write_cnt <= 6'd0;
                                write_addr <= calc_dst_addr(current_row, mat_width);
                                write_data <= get_padded_value(current_row, 6'd0, mat_width);
                            end
                        end
                    end else begin
                        // For boundary rows, no read needed, directly write
                        state <= S_WRITE_ROW;
                        write_cnt <= 6'd0;
                        write_addr <= calc_dst_addr(current_row, mat_width);
                        write_data <= get_padded_value(current_row, 6'd0, mat_width);
                    end
                end
                
                S_WRITE_ROW: begin
                    if (aw_handshake) begin
                        $display("[DEBUG] Write ROW: output_row=%d, addr=%h", current_row, write_addr);
                    end
                    
                    if (w_handshake) begin
                        $display("[DEBUG] Write DATA: col=%d, data=%d", write_cnt, write_data);
                        
                        write_cnt <= write_cnt + 1;
                        
                        if (!is_last_write) begin
                            write_data <= get_padded_value(current_row, write_cnt + 1, mat_width);
                        end
                        
                        if (write_complete && b_handshake) begin
                            state <= S_NEXT_ROW;
                        end
                    end
                end
                
                S_NEXT_ROW: begin
                    current_row <= current_row + 1;
                    
                    if (current_row >= mat_width + 1) begin
                        // All rows processed
                        $display("[DEBUG] All rows completed! Going to IDLE");
                        state <= S_IDLE;
                        done <= 1'b1;
                    end else begin
                        // Process next row
                        read_cnt <= 6'd0;
                        
                        if (current_row + 1 < mat_width) begin
                            // Read next source row
                            read_addr <= calc_src_addr(current_row + 1, mat_width);
                        end
                        
                        $display("[DEBUG] Moving to next row: %d", current_row + 1);
                        state <= S_READ_ROW;
                    end
                end
            endcase
        end
    end

endmodule 