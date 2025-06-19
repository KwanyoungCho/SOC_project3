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
    localparam S_READ_ROW       = 3'd1;
    localparam S_WRITE_BLOCK    = 3'd2;
    localparam S_WRITE_RESP     = 3'd3;
    localparam S_DONE           = 3'd4;

    reg [2:0] state;
    
    // Configuration registers
    reg [31:0] src_addr;
    reg [31:0] dst_addr;
    reg [5:0]  mat_width;
    reg        done;
    
    // Current position in output matrix (0-based, top-left of 2x2 block)
    reg [5:0]  out_row;     
    reg [5:0]  out_col;     
    
    // Source data cache - store 2 rows
    reg [31:0] src_row0 [0:63];  // Current row
    reg [31:0] src_row1 [0:63];  // Next row  
    reg        row0_valid;
    reg        row1_valid;
    reg [5:0]  cached_row;       // Which source row is in row0 (1-based)
    
    // AXI control signals
    reg        ar_valid;
    reg [31:0] ar_addr;
    reg [5:0]  ar_len;
    reg        r_ready;
    reg [5:0]  read_cnt;
    reg        reading_row1;
    
    reg        aw_valid;
    reg [31:0] aw_addr;
    reg        w_valid;
    reg [31:0] w_data;
    reg        w_last;
    reg        b_ready;
    reg [2:0]  write_cnt;  // 0,1,2,3 for 2x2 block: TL, TR, BL, BR
    
    // 2x2 block data
    reg [31:0] block_data [0:3];  // TL, TR, BL, BR
    
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
    assign arlen_o = ar_len[3:0];
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
    
    // 출력 좌표를 소스 행으로 변환 (미러 패딩 적용)
    // 반환되는 값은 원본 행의 0-based 인덱스이다.
    function [5:0] get_source_row;
        input [5:0] out_r;
        input [5:0] width;
        begin
            if (out_r == 0) begin
                // 첫 번째 패딩 라인은 두 번째 행을 복사한다.
                get_source_row = 6'd1;
            end else if (out_r == width + 1) begin
                // 마지막 패딩 라인은 끝에서 두 번째 행을 복사한다.
                get_source_row = width - 2;
            end else begin
                // 내부 영역은 1을 빼서 원본 행 인덱스를 얻는다.
                get_source_row = out_r - 1;
            end
        end
    endfunction

    // 출력 좌표를 소스 열로 변환 (미러 패딩 적용)
    // 반환되는 값은 원본 열의 0-based 인덱스이다.
    function [5:0] get_source_col;
        input [5:0] out_c;
        input [5:0] width;
        begin
            if (out_c == 0) begin
                get_source_col = 6'd1;
            end else if (out_c == width + 1) begin
                get_source_col = width - 2;
            end else begin
                get_source_col = out_c - 1;
            end
        end
    endfunction
    
    // 2x2 블록의 특정 위치에서 미러 패딩된 값 가져오기
    function [31:0] get_padded_value;
        input [5:0] out_r;
        input [5:0] out_c;
        input [5:0] width;
        input [1:0] rel_r;  // 2x2 블록 내 상대 행 (0 or 1)
        input [1:0] rel_c;  // 2x2 블록 내 상대 열 (0 or 1)
        reg [5:0] abs_r, abs_c;
        reg [5:0] src_r, src_c;
        begin
            // 절대 출력 좌표 계산
            abs_r = out_r + rel_r;
            abs_c = out_c + rel_c;
            
            // 미러 패딩 적용하여 소스 좌표 계산
            src_r = get_source_row(abs_r, width);
            src_c = get_source_col(abs_c, width);
            
            // 캐시에서 값 가져오기
            if (src_r == cached_row) begin
                get_padded_value = src_row0[src_c];
            end else if (src_r == cached_row + 1) begin
                get_padded_value = src_row1[src_c];
            end else begin
                get_padded_value = 32'd0;  // Error case
            end
        end
    endfunction
    
    // 현재 2x2 블록에 필요한 소스 행들이 캐시되어 있는지 확인
    function need_read_source;
        input [5:0] out_r;
        input [5:0] out_c;
        input [5:0] width;
        reg [5:0] src_r0, src_r1;
        begin
            // 출력 위치에 대응되는 소스 행 계산 (미러 패딩 적용)
            src_r0 = get_source_row(out_r, width);
            src_r1 = get_source_row(out_r + 1, width);
            
            // 필요한 행들이 캐시되어 있지 않으면 읽기 필요
            need_read_source = (!row0_valid || cached_row != src_r0) ||
                              (!row1_valid || cached_row + 1 != src_r1 || cached_row + 1 >= width);
        end
    endfunction
    
    // 2x2 블록을 위한 출력 주소 계산 (첫 번째 요소 TL)
    function [31:0] calc_write_addr;
        input [5:0] out_r;
        input [5:0] out_c;
        input [5:0] width;
        begin
            // 출력 매트릭스 크기: (width+2) x (width+2) 
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
            row0_valid <= 1'b0;
            row1_valid <= 1'b0;
            cached_row <= 6'd0;
            
            ar_valid <= 1'b0;
            ar_addr <= 32'd0;
            ar_len <= 6'd0;
            r_ready <= 1'b0;
            read_cnt <= 6'd0;
            reading_row1 <= 1'b0;
            
            aw_valid <= 1'b0;
            aw_addr <= 32'd0;
            w_valid <= 1'b0;
            w_data <= 32'd0;
            w_last <= 1'b0;
            b_ready <= 1'b0;
            write_cnt <= 3'd0;
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
                        row0_valid <= 1'b0;
                        row1_valid <= 1'b0;
                        
                        // 첫 번째 블록을 위해 첫 두 행 읽기 시작
                        state <= S_READ_ROW;
                        ar_valid <= 1'b1;
                        ar_addr <= src_addr_i;  // 소스 매트릭스 첫 번째 행
                        ar_len <= mat_width_i - 1;  // AXI length = actual - 1
                        r_ready <= 1'b1;
                        read_cnt <= 6'd0;
                        reading_row1 <= 1'b0;
                        cached_row <= 6'd1;  // 읽고 있는 소스 행 번호 (1-based)
                    end else begin
                        done <= 1'b1;
                    end
                end
                
                S_READ_ROW: begin
                    if (ar_handshake) begin
                        ar_valid <= 1'b0;
                    end
                    
                    if (r_handshake) begin
                        if (!reading_row1) begin
                            src_row0[read_cnt] <= rdata_i;
                        end else begin
                            src_row1[read_cnt] <= rdata_i;
                        end
                        read_cnt <= read_cnt + 1;
                        
                        if (rlast_i) begin
                            if (!reading_row1 && cached_row < mat_width) begin
                                // 첫 번째 행 읽기 완료, 두 번째 행 읽기 시작
                                row0_valid <= 1'b1;
                                reading_row1 <= 1'b1;
                                read_cnt <= 6'd0;
                                ar_valid <= 1'b1;
                                ar_addr <= src_addr + cached_row * mat_width * 4;  // 다음 행
                                ar_len <= mat_width - 1;
                            end else begin
                                // 두 번째 행 읽기 완료 또는 마지막 행
                                if (reading_row1) begin
                                    row1_valid <= 1'b1;
                                end else begin
                                    row0_valid <= 1'b1;
                                end
                                reading_row1 <= 1'b0;
                                r_ready <= 1'b0;
                                read_cnt <= 6'd0;
                                
                                // 2x2 블록 데이터 준비
                                block_data[0] <= get_padded_value(out_row, out_col, mat_width, 2'd0, 2'd0); // TL
                                block_data[1] <= get_padded_value(out_row, out_col, mat_width, 2'd0, 2'd1); // TR
                                block_data[2] <= get_padded_value(out_row, out_col, mat_width, 2'd1, 2'd0); // BL
                                block_data[3] <= get_padded_value(out_row, out_col, mat_width, 2'd1, 2'd1); // BR
                                
                                state <= S_WRITE_BLOCK;
                                aw_valid <= 1'b1;
                                aw_addr <= calc_write_addr(out_row, out_col, mat_width);
                                write_cnt <= 3'd0;
                            end
                        end
                    end
                end
                
                S_WRITE_BLOCK: begin
                    if (aw_handshake) begin
                        aw_valid <= 1'b0;
                        w_valid <= 1'b1;
                        b_ready <= 1'b1;
                        w_data <= block_data[0]; // TL
                        w_last <= 1'b0;
                    end
                    
                    if (w_handshake) begin
                        write_cnt <= write_cnt + 1;
                        
                        case (write_cnt)
                            3'd0: begin
                                w_data <= block_data[1]; // TR
                                w_last <= 1'b0;
                            end
                            3'd1: begin  
                                w_data <= block_data[2]; // BL
                                w_last <= 1'b0;
                            end
                            3'd2: begin
                                w_data <= block_data[3]; // BR
                                w_last <= 1'b1;
                            end
                            3'd3: begin
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
                        write_cnt <= 3'd0;
                        
                        // Move to next 2x2 block  
                        if (out_col + 2 < mat_width + 2) begin
                            out_col <= out_col + 2;
                        end else begin
                            out_col <= 6'd0;
                            out_row <= out_row + 2;
                        end
                        
                        // Check if done
                        if ((out_row + 2 >= mat_width + 2) && (out_col + 2 >= mat_width + 2)) begin
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
                            
                            // Check if we need to read more source data
                            if (need_read_source(next_row, next_col, mat_width)) begin
                                reg [5:0] req_row;
                                req_row = get_source_row(next_row, mat_width);
                                
                                state <= S_READ_ROW;
                                
                                // Shift cache if needed
                                if (req_row == cached_row + 1) begin
                                    // Shift: row1 -> row0, read new row1
                                    integer i;
                                    for (i = 0; i < 64; i = i + 1) begin
                                        src_row0[i] <= src_row1[i];
                                    end
                                    row0_valid <= row1_valid;
                                    row1_valid <= 1'b0;
                                    cached_row <= cached_row + 1;
                                    reading_row1 <= 1'b1;
                                end else begin
                                    // Read completely new rows
                                    row0_valid <= 1'b0;
                                    row1_valid <= 1'b0;
                                    cached_row <= req_row;
                                    reading_row1 <= 1'b0;
                                end
                                
                                ar_valid <= 1'b1;
                                ar_addr <= src_addr + (req_row - 1) * mat_width * 4;
                                ar_len <= mat_width - 1;
                                r_ready <= 1'b1;
                                read_cnt <= 6'd0;
                            end else begin
                                // Can use cached data - prepare 2x2 block
                                block_data[0] <= get_padded_value(next_row, next_col, mat_width, 2'd0, 2'd0); // TL
                                block_data[1] <= get_padded_value(next_row, next_col, mat_width, 2'd0, 2'd1); // TR  
                                block_data[2] <= get_padded_value(next_row, next_col, mat_width, 2'd1, 2'd0); // BL
                                block_data[3] <= get_padded_value(next_row, next_col, mat_width, 2'd1, 2'd1); // BR
                                
                                state <= S_WRITE_BLOCK;
                                aw_valid <= 1'b1;
                                aw_addr <= calc_write_addr(next_row, next_col, mat_width);
                                write_cnt <= 3'd0;
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