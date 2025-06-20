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

    // State Machine - SGDMAC 패턴 따라하기
    localparam IDLE         = 3'd0;
    localparam READ_ADDR    = 3'd1;
    localparam READ_DATA    = 3'd2;
    localparam WRITE_ADDR   = 3'd3;
    localparam WRITE_DATA   = 3'd4;
    localparam WRITE_RESP   = 3'd5;

    reg [2:0] state;
    
    // Configuration registers
    reg [31:0] src_addr;
    reg [31:0] dst_addr;
    reg [5:0]  mat_width;
    
    // Current processing row (0 to mat_width+1)
    reg [5:0]  current_row;
    
    // Source row buffer
    reg [31:0] src_row_buffer [0:63];
    
    // Read state variables
    reg [31:0] read_addr;
    reg [5:0]  read_cnt;        // 읽은 데이터 개수
    reg [3:0]  read_burst_cnt;  // 현재 burst의 남은 beat 수
    
    // Write state variables  
    reg [31:0] write_addr;
    reg [5:0]  write_cnt;       // 쓴 데이터 개수
    reg [3:0]  write_burst_cnt; // 현재 burst의 남은 beat 수
    reg [31:0] output_data;     // 현재 출력할 데이터
    
    // Handshake signals - SGDMAC 스타일
    wire ar_handshake = arvalid_o & arready_i;
    wire r_handshake = rvalid_i & rready_o;
    wire aw_handshake = awvalid_o & awready_i;
    wire w_handshake = wvalid_o & wready_i;
    wire b_handshake = bvalid_i & bready_o;
    
    // Control signals - SGDMAC 스타일
    wire read_burst_complete = r_handshake & rlast_i;
    wire write_burst_complete = w_handshake & wlast_o;
    wire is_last_read_beat = (read_burst_cnt == 4'd0);
    wire is_last_write_beat = (write_burst_cnt == 4'd0);
    wire read_done = (current_row >= mat_width);
    wire all_done = (current_row >= mat_width + 2);
    
    // Calculate burst length
    wire [3:0] calc_arlen = mat_width - 1;        // 소스 행 길이
    wire [3:0] calc_awlen = mat_width + 1;        // 출력 행 길이 (width+2)
    
    // Mirror padding function
    function [31:0] get_padded_value;
        input [5:0] out_row;  // 0-based output row (0 to width+1)
        input [5:0] out_col;  // 0-based output col (0 to width+1)
        input [5:0] width;
        reg [5:0] src_row, src_col;
        begin
            // Mirror padding mapping - 정답 매트릭스 기준
            if (out_row == 0) begin
                src_row = 1;  // out[0] -> src[1] (두 번째 행)
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
            
            // 현재 읽은 소스 행의 데이터에서 가져오기
            if (src_row == current_row - 1 && current_row > 0 && current_row <= width) begin
                get_padded_value = src_row_buffer[src_col];
            end else begin
                // 경계 행의 경우, 다른 처리 (추후 구현)
                get_padded_value = 32'd0;
            end
        end
    endfunction
    
    // AXI Output assignments - SGDMAC 스타일
    assign done_o = (state == IDLE) && !start_i;
    
    // AR channel
    assign arid_o = 4'd0;
    assign araddr_o = read_addr;
    assign arlen_o = calc_arlen;
    assign arsize_o = 3'b010;
    assign arburst_o = 2'b01;
    assign arvalid_o = (state == READ_ADDR) && !read_done;
    assign rready_o = (state == READ_DATA);
    
    // AW channel  
    assign awid_o = 4'd0;
    assign awaddr_o = write_addr;
    assign awlen_o = calc_awlen;
    assign awsize_o = 3'b010;
    assign awburst_o = 2'b01;
    assign awvalid_o = (state == WRITE_ADDR);
    
    // W channel
    assign wid_o = 4'd0;
    assign wdata_o = output_data;
    assign wstrb_o = 4'hF;
    assign wlast_o = (state == WRITE_DATA) & is_last_write_beat;
    assign wvalid_o = (state == WRITE_DATA);
    
    // B channel
    assign bready_o = (state == WRITE_DATA) | (state == WRITE_RESP);
    
    // Main state machine - SGDMAC 패턴
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            src_addr <= 32'd0;
            dst_addr <= 32'd0;
            mat_width <= 6'd0;
            current_row <= 6'd0;
            read_addr <= 32'd0;
            read_cnt <= 6'd0;
            read_burst_cnt <= 4'd0;
            write_addr <= 32'd0;
            write_cnt <= 6'd0;
            write_burst_cnt <= 4'd0;
            output_data <= 32'd0;
        end else begin
            case (state)
                IDLE: begin
                    if (start_i) begin
                        src_addr <= src_addr_i;
                        dst_addr <= dst_addr_i;
                        mat_width <= mat_width_i;
                        current_row <= 6'd0;
                        
                        // 첫 번째 소스 행 주소 계산 (소스 행 0)
                        read_addr <= src_addr_i;
                        read_cnt <= 6'd0;
                        
                        $display("[DEBUG] Starting DMA: src=%h, dst=%h, width=%d", src_addr_i, dst_addr_i, mat_width_i);
                        state <= READ_ADDR;
                    end
                end
                
                READ_ADDR: begin
                    if (!read_done && ar_handshake) begin
                        state <= READ_DATA;
                        read_burst_cnt <= calc_arlen;
                        
                        $display("[DEBUG] Read ADDR: row=%d, addr=%h, len=%d", current_row-1, read_addr, calc_arlen);
                    end else if (read_done) begin
                        // 소스 읽기 완료, 바로 출력 행 쓰기 시작
                        write_addr <= dst_addr + (current_row * (mat_width + 2)) * 4;
                        write_cnt <= 6'd0;
                        output_data <= get_padded_value(current_row, 6'd0, mat_width);
                        state <= WRITE_ADDR;
                    end
                end
                
                READ_DATA: begin
                    if (r_handshake) begin
                        src_row_buffer[read_cnt] <= rdata_i;
                        read_cnt <= read_cnt + 1;
                        read_burst_cnt <= read_burst_cnt - 1;
                        
                        $display("[DEBUG] Read DATA: buffer[%d] = %d", read_cnt, rdata_i);
                        
                        if (read_burst_complete) begin
                            // 현재 소스 행 읽기 완료, 출력 행 쓰기 준비
                            write_addr <= dst_addr + (current_row * (mat_width + 2)) * 4;
                            write_cnt <= 6'd0;
                            output_data <= get_padded_value(current_row, 6'd0, mat_width);
                            state <= WRITE_ADDR;
                        end
                    end
                end
                
                WRITE_ADDR: begin
                    if (aw_handshake) begin
                        state <= WRITE_DATA;
                        write_burst_cnt <= calc_awlen;
                        
                        $display("[DEBUG] Write ADDR: row=%d, addr=%h, len=%d", current_row, write_addr, calc_awlen);
                    end
                end
                
                WRITE_DATA: begin
                    if (w_handshake) begin
                        write_cnt <= write_cnt + 1;
                        write_burst_cnt <= write_burst_cnt - 1;
                        
                        $display("[DEBUG] Write DATA: col=%d, data=%d", write_cnt, output_data);
                        
                        // 다음 출력 데이터 준비
                        if (!is_last_write_beat) begin
                            output_data <= get_padded_value(current_row, write_cnt + 1, mat_width);
                        end
                        
                        if (write_burst_complete) begin
                            if (b_handshake) begin
                                // Write response도 함께 완료된 경우
                                current_row <= current_row + 1;
                                
                                if (all_done) begin
                                    state <= IDLE;
                                    $display("[DEBUG] All done!");
                                end else begin
                                    // 다음 행 처리
                                    if (current_row + 1 < mat_width) begin
                                        // 다음 소스 행 읽기
                                        read_addr <= src_addr + ((current_row + 1) * mat_width) * 4;
                                        read_cnt <= 6'd0;
                                        state <= READ_ADDR;
                                    end else begin
                                        // 경계 행 처리 (소스 읽기 없이 바로 쓰기)
                                        write_addr <= dst_addr + ((current_row + 1) * (mat_width + 2)) * 4;
                                        write_cnt <= 6'd0;
                                        output_data <= get_padded_value(current_row + 1, 6'd0, mat_width);
                                        state <= WRITE_ADDR;
                                    end
                                end
                            end else begin
                                // Write response 대기
                                state <= WRITE_RESP;
                            end
                        end
                    end
                end
                
                WRITE_RESP: begin
                    if (b_handshake) begin
                        current_row <= current_row + 1;
                        
                        if (all_done) begin
                            state <= IDLE;
                            $display("[DEBUG] All done!");
                        end else begin
                            // 다음 행 처리
                            if (current_row + 1 < mat_width) begin
                                // 다음 소스 행 읽기
                                read_addr <= src_addr + ((current_row + 1) * mat_width) * 4;
                                read_cnt <= 6'd0;
                                state <= READ_ADDR;
                            end else begin
                                // 경계 행 처리
                                write_addr <= dst_addr + ((current_row + 1) * (mat_width + 2)) * 4;
                                write_cnt <= 6'd0;
                                output_data <= get_padded_value(current_row + 1, 6'd0, mat_width);
                                state <= WRITE_ADDR;
                            end
                        end
                    end
                end
            endcase
        end
    end

endmodule 