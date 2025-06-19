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
    localparam S_READ_BLOCK     = 3'd1;
    localparam S_WRITE_2X2      = 3'd2;
    localparam S_WRITE_RESP     = 3'd3;
    localparam S_DONE           = 3'd4;

    reg [2:0] state;
    
    // Configuration registers
    reg [31:0] src_addr;
    reg [31:0] dst_addr;
    reg [5:0]  mat_width;
    reg        done;
    
    // Current position in output matrix (0-based)
    reg [5:0]  out_row;     // Current output row (0 to mat_width+1)  
    reg [5:0]  out_col;     // Current output column (0 to mat_width+1)
    
    // Source reading buffer (최대 4개의 원소를 읽을 수 있음)
    reg [31:0] src_buffer [0:3];
    reg [1:0]  src_read_cnt;
    reg [3:0]  src_read_len;  // 실제로 읽어야 할 원소 개수
    
    // AXI control signals
    reg        ar_valid;
    reg [31:0] ar_addr;
    reg [3:0]  ar_len;
    reg        r_ready;
    
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
    
    // 현재 2x2 블록이 원본 매트릭스와 겹치는지 확인
    function need_source_read;
        input [5:0] out_r;
        input [5:0] out_c;
        input [5:0] width;
        begin
            // 2x2 블록이 원본 영역(1~width, 1~width)과 겹치는지 확인
            need_source_read = ((out_r + 1 >= 1) && (out_r <= width)) && 
                              ((out_c + 1 >= 1) && (out_c <= width));
        end
    endfunction
    
    // 읽어야 할 원소의 개수와 시작 주소 계산
    function [4:0] calc_read_info; // [4] = need_read, [3:0] = length
        input [5:0] out_r;
        input [5:0] out_c;
        input [5:0] width;
        reg [5:0] src_start_r, src_start_c;
        reg [5:0] src_end_r, src_end_c;
        reg [5:0] read_rows, read_cols;
        begin
            if (!need_source_read(out_r, out_c, width)) begin
                calc_read_info = 5'b0_0000; // no read needed
            end else begin
                // 읽기 시작점 (원본 매트릭스 내에서)
                src_start_r = (out_r >= 1) ? out_r : 1;
                src_start_c = (out_c >= 1) ? out_c : 1;
                
                // 읽기 끝점 (원본 매트릭스 내에서)  
                src_end_r = ((out_r + 1) <= width) ? (out_r + 1) : width;
                src_end_c = ((out_c + 1) <= width) ? (out_c + 1) : width;
                
                read_rows = src_end_r - src_start_r + 1;
                read_cols = src_end_c - src_start_c + 1;
                
                calc_read_info = {1'b1, read_rows[1:0], read_cols[1:0]};
            end
        end
    endfunction
    
    // 읽기 주소 계산
    function [31:0] calc_read_addr;
        input [5:0] out_r;
        input [5:0] out_c;
        input [5:0] width;
        reg [5:0] src_r, src_c;
        begin
            src_r = (out_r >= 1) ? out_r : 1;
            src_c = (out_c >= 1) ? out_c : 1;
            
            // 0-based indexing for memory address
            calc_read_addr = src_addr + ((src_r - 1) * width + (src_c - 1)) * 4;
        end
    endfunction
    
    // 미러 패딩을 적용한 값 가져오기
    function [31:0] get_padded_value;
        input [5:0] out_r;      // Output matrix row (0-based)
        input [5:0] out_c;      // Output matrix col (0-based)  
        input [5:0] width;
        input [1:0] rel_r;      // Relative position in 2x2 (0 or 1)
        input [1:0] rel_c;      // Relative position in 2x2 (0 or 1)
        reg [5:0] abs_r, abs_c; // Absolute position in output matrix
        reg [5:0] src_r, src_c; // Corresponding source position (1-based)
        reg [5:0] buf_r, buf_c; // Buffer index calculation
        reg [1:0] buf_idx;
        begin
            // Calculate absolute position in output matrix
            abs_r = out_r + rel_r;
            abs_c = out_c + rel_c;
            
            // Apply mirror padding to get source coordinates (1-based)
            if (abs_r == 0) begin
                src_r = 1;  // Mirror from first row
            end else if (abs_r == width + 1) begin
                src_r = width;  // Mirror from last row
            end else begin
                src_r = abs_r;  // Normal case (already 1-based for source)
            end
            
            if (abs_c == 0) begin
                src_c = 1;  // Mirror from first column
            end else if (abs_c == width + 1) begin
                src_c = width;  // Mirror from last column
            end else begin
                src_c = abs_c;  // Normal case (already 1-based for source)
            end
            
            // Check if we read this from source
            if (need_source_read(out_r, out_c, width)) begin
                // Calculate buffer position based on what we actually read
                reg [5:0] read_start_r, read_start_c;
                read_start_r = (out_r >= 1) ? out_r : 1;
                read_start_c = (out_c >= 1) ? out_c : 1;
                
                buf_r = src_r - read_start_r;
                buf_c = src_c - read_start_c;
                buf_idx = buf_r * 2 + buf_c;
                
                get_padded_value = src_buffer[buf_idx];
            end else begin
                // All padding case - use a default value (should not happen in practice)
                get_padded_value = 32'd0;
            end
        end
    endfunction
    
    // 2x2 블록의 쓰기 주소 계산
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
            src_read_cnt <= 2'd0;
            src_read_len <= 4'd0;
            
            aw_valid <= 1'b0;
            aw_addr <= 32'd0;
            w_valid <= 1'b0;
            w_data <= 32'd0;
            w_last <= 1'b0;
            b_ready <= 1'b0;
            write_cnt <= 2'd0;
            
            src_buffer[0] <= 32'd0;
            src_buffer[1] <= 32'd0;
            src_buffer[2] <= 32'd0;
            src_buffer[3] <= 32'd0;
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
                        if (need_source_read(6'd0, 6'd0, mat_width_i)) begin
                            state <= S_READ_BLOCK;
                            ar_valid <= 1'b1;
                            ar_addr <= calc_read_addr(6'd0, 6'd0, mat_width_i);
                            ar_len <= calc_read_info(6'd0, 6'd0, mat_width_i)[2:0] - 1; // AXI length is actual-1
                            src_read_len <= calc_read_info(6'd0, 6'd0, mat_width_i)[3:0];
                            r_ready <= 1'b1;
                            src_read_cnt <= 2'd0;
                        end else begin
                            // All padding
                            state <= S_WRITE_2X2;
                            aw_valid <= 1'b1;
                            aw_addr <= calc_write_addr(6'd0, 6'd0, mat_width_i);
                            write_cnt <= 2'd0;
                        end
                    end else begin
                        done <= 1'b1;
                    end
                end
                
                S_READ_BLOCK: begin
                    if (ar_handshake) begin
                        ar_valid <= 1'b0;
                    end
                    
                    if (r_handshake) begin
                        src_buffer[src_read_cnt] <= rdata_i;
                        src_read_cnt <= src_read_cnt + 1;
                        
                        if (rlast_i || (src_read_cnt + 1 == src_read_len)) begin
                            r_ready <= 1'b0;
                            src_read_cnt <= 2'd0;
                            state <= S_WRITE_2X2;
                            aw_valid <= 1'b1;
                            aw_addr <= calc_write_addr(out_row, out_col, mat_width);
                            write_cnt <= 2'd0;
                        end
                    end
                end
                
                S_WRITE_2X2: begin
                    if (aw_handshake) begin
                        aw_valid <= 1'b0;
                        w_valid <= 1'b1;
                        b_ready <= 1'b1;
                        w_data <= get_padded_value(out_row, out_col, mat_width, 2'd0, 2'd0); // top-left
                        w_last <= 1'b0;
                    end
                    
                    if (w_handshake) begin
                        write_cnt <= write_cnt + 1;
                        
                        case (write_cnt)
                            2'd0: begin
                                w_data <= get_padded_value(out_row, out_col, mat_width, 2'd0, 2'd1); // top-right
                                w_last <= 1'b0;
                            end
                            2'd1: begin
                                w_data <= get_padded_value(out_row, out_col, mat_width, 2'd1, 2'd0); // bottom-left
                                w_last <= 1'b0;
                            end
                            2'd2: begin
                                w_data <= get_padded_value(out_row, out_col, mat_width, 2'd1, 2'd1); // bottom-right
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
                        if (out_col + 2 < mat_width + 2) begin
                            out_col <= out_col + 2;
                        end else begin
                            out_col <= 6'd0;
                            out_row <= out_row + 2;
                        end
                        
                        // Check if done
                        if (out_row + 2 >= mat_width + 2) begin
                            state <= S_DONE;
                        end else begin
                            // Prepare next block coordinates
                            reg [5:0] next_row, next_col;
                            if (out_col + 2 < mat_width + 2) begin
                                next_row = out_row;
                                next_col = out_col + 2;
                            end else begin
                                next_row = out_row + 2;
                                next_col = 6'd0;
                            end
                            
                            // Check if next block needs reading
                            if (need_source_read(next_row, next_col, mat_width)) begin
                                state <= S_READ_BLOCK;
                                ar_valid <= 1'b1;
                                ar_addr <= calc_read_addr(next_row, next_col, mat_width);
                                ar_len <= calc_read_info(next_row, next_col, mat_width)[2:0] - 1;
                                src_read_len <= calc_read_info(next_row, next_col, mat_width)[3:0];
                                r_ready <= 1'b1;
                                src_read_cnt <= 2'd0;
                            end else begin
                                // All padding for this block
                                state <= S_WRITE_2X2;
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