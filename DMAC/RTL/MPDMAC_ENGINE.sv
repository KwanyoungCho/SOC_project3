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

    // State definitions
    localparam                  S_IDLE      = 3'd0,
                                S_RREQ      = 3'd1,
                                S_RDATA     = 3'd2,
                                S_PROCESS   = 3'd3,
                                S_WREQ      = 3'd4,
                                S_WDATA     = 3'd5,
                                S_WRESP     = 3'd6;  // SGDMAC 패턴: response 대기 상태 추가

    reg [2:0]                   state, state_n;
    
    // Configuration registers
    reg [31:0]              src_addr, src_addr_n;
    reg [31:0]              dst_addr, dst_addr_n;
    reg [5:0]               mat_width, mat_width_n;
    
    // Block processing variables
    reg [5:0]               block_x, block_x_n;     // 현재 블록 x 좌표 (0 ~ blocks_per_row-1)
    reg [5:0]               block_y, block_y_n;     // 현재 블록 y 좌표 (0 ~ blocks_per_col-1)
    reg [5:0]               blocks_per_row, blocks_per_row_n;
    reg [5:0]               blocks_per_col, blocks_per_col_n;
    
    // 5x5 Buffer (x,y 좌표로 접근)
    reg [31:0]              buffer [0:4][0:4];      // buffer[x][y]
    
    // Burst read variables
    reg [1:0]               read_row, read_row_n;   // 4x4 블록 내 읽고 있는 행 (0~3)
    reg [1:0]               read_col, read_col_n;   // burst 내 읽고 있는 열 (0~3)
    
    // Burst write variables - SGDMAC 패턴
    reg [4:0]                   write_cnt, write_cnt_n; // 출력 중인 데이터 개수 (최대 25개)
    reg [4:0]                   write_len, write_len_n; // 현재 블록의 출력 길이
    reg [4:0]                   burst_cnt, burst_cnt_n; // SGDMAC 패턴: burst counter
    reg [2:0]                   write_row, write_row_n; // 현재 쓰고 있는 행 (0~4)
    
    // Block type detection
    reg [3:0]               block_type, block_type_n;
    localparam TYPE_TL    = 4'd0;  // Top-Left corner
    localparam TYPE_TR    = 4'd1;  // Top-Right corner  
    localparam TYPE_BL    = 4'd2;  // Bottom-Left corner
    localparam TYPE_BR    = 4'd3;  // Bottom-Right corner
    localparam TYPE_T     = 4'd4;  // Top edge
    localparam TYPE_B     = 4'd5;  // Bottom edge
    localparam TYPE_L     = 4'd6;  // Left edge
    localparam TYPE_R     = 4'd7;  // Right edge
    localparam TYPE_INNER = 4'd8;  // Inner (no edge)
    
    // Control signals - SGDMAC 패턴
    reg                     arvalid;
    reg                     rready;
    reg                     awvalid;
    reg                     wvalid;
    reg                     done, done_n;
    reg [31:0]              wdata_reg;
    
    // SGDMAC handshake patterns
    wire ar_handshake = arvalid_o & arready_i;
    wire r_handshake = rvalid_i & rready_o;
    wire aw_handshake = awvalid_o & awready_i;
    wire w_handshake = wvalid_o & wready_i;
    wire b_handshake = bvalid_i & bready_o;
    
    // Read control - SGDMAC 패턴
    wire read_burst_complete = r_handshake & rlast_i;
    wire read_4x4_complete = (read_row == 2'd3) & read_burst_complete;
    
    // Write control - SGDMAC 패턴  
    wire is_last_beat = (burst_cnt == 5'd0);
    wire burst_done = w_handshake & is_last_beat;
    wire write_data_available = 1'b1;  // 버퍼에서 항상 데이터 사용 가능
    
    // Helper signals  
    wire current_block_done = (block_x == blocks_per_row - 1) && (block_y == blocks_per_col - 1);
    wire all_blocks_done = current_block_done;
    
    // Block type name for debug
    function string get_block_type_name;
        input [3:0] btype;
        begin
            case (btype)
                TYPE_TL:    get_block_type_name = "TL";
                TYPE_TR:    get_block_type_name = "TR";
                TYPE_BL:    get_block_type_name = "BL";
                TYPE_BR:    get_block_type_name = "BR";
                TYPE_T:     get_block_type_name = "T";
                TYPE_B:     get_block_type_name = "B";
                TYPE_L:     get_block_type_name = "L";
                TYPE_R:     get_block_type_name = "R";
                TYPE_INNER: get_block_type_name = "INNER";
                default:    get_block_type_name = "UNKNOWN";
            endcase
        end
    endfunction
    
    // Block type detection function
    function [3:0] detect_block_type;
        input [5:0] bx, by;
        input [5:0] blocks_row, blocks_col;
        begin
            // Corner cases
            if (bx == 0 && by == 0) 
                detect_block_type = TYPE_TL;
            else if (bx == blocks_row-1 && by == 0)
                detect_block_type = TYPE_TR;
            else if (bx == 0 && by == blocks_col-1)
                detect_block_type = TYPE_BL;
            else if (bx == blocks_row-1 && by == blocks_col-1)
                detect_block_type = TYPE_BR;
            // Edge cases
            else if (by == 0)
                detect_block_type = TYPE_T;
            else if (by == blocks_col-1)
                detect_block_type = TYPE_B;
            else if (bx == 0)
                detect_block_type = TYPE_L;
            else if (bx == blocks_row-1)
                detect_block_type = TYPE_R;
            // Inner case
            else
                detect_block_type = TYPE_INNER;
        end
    endfunction
    
    // Calculate source read address
    function [31:0] calc_read_addr;
        input [5:0] bx, by;
        input [1:0] row;
        input [5:0] width;
        input [31:0] base_addr;
        reg [5:0] src_row, src_col;
        begin
            src_row = by * 4 + row;
            src_col = bx * 4;
            calc_read_addr = base_addr + (src_row * width + src_col) * 4;
        end
    endfunction
    
    // Calculate destination write address - 현재 행의 시작 주소
    function [31:0] calc_write_addr;
        input [5:0] bx, by;
        input [2:0] wrow;    // 현재 쓰고 있는 행 (0~4)
        input [5:0] width;
        input [31:0] base_addr;
        input [3:0] btype;
        reg [5:0] dst_row, dst_col;
        begin
            // 패딩된 매트릭스에서의 올바른 시작 위치 계산
            case (btype)
                TYPE_TL: begin
                    // TL: (bx*4, by*4)에서 시작
                    dst_row = by * 4 + wrow;
                    dst_col = bx * 4;
                end
                TYPE_TR: begin  
                    // TR: (bx*4+4, by*4)에서 시작
                    dst_row = by * 4 + wrow;
                    dst_col = bx * 4 + 1;
                end
                TYPE_BL: begin
                    // BL: (bx*4, by*4+4)에서 시작  
                    dst_row = by * 4 + 1 + wrow;
                    dst_col = bx * 4;
                end
                TYPE_BR: begin
                    // BR: (bx*4+4, by*4+4)에서 시작
                    dst_row = by * 4 + 1 + wrow;
                    dst_col = bx * 4 + 1;
                end
                TYPE_T: begin
                    // T: (1,0)에서 시작 (top padding)
                    dst_row = by * 4 + wrow;
                    dst_col = bx * 4 + 1;
                end
                TYPE_B: begin
                    // B: (1,5)에서 시작 (bottom padding)
                    dst_row = by * 4 + 5 + wrow;
                    dst_col = bx * 4 + 1;
                end
                TYPE_L: begin
                    // L: (0,1)에서 시작 (left padding)
                    dst_row = by * 4 + 1 + wrow;
                    dst_col = bx * 4;
                end
                TYPE_R: begin
                    // R: (5,1)에서 시작 (right padding)
                    dst_row = by * 4 + 1 + wrow;
                    dst_col = bx * 4 + 5;
                end
                TYPE_INNER: begin
                    // INNER: (1,1)에서 시작 (no padding)
                    dst_row = by * 4 + 1 + wrow;
                    dst_col = bx * 4 + 1;
                end
            endcase
            // 패딩된 매트릭스 크기는 (width + 2) x (width + 2)
            calc_write_addr = base_addr + (dst_row * (width + 2) + dst_col) * 4;
        end
    endfunction
    
    // Get buffer base coordinates for storing read data
    function [2:0] get_base_x;  // 3비트로 수정 (0~4 표현 가능)
        input [3:0] btype;
        begin
            case (btype)
                TYPE_TL:    get_base_x = 3'd1;
                TYPE_TR:    get_base_x = 3'd0;
                TYPE_BL:    get_base_x = 3'd1;
                TYPE_BR:    get_base_x = 3'd0;
                TYPE_T:     get_base_x = 3'd0;
                TYPE_B:     get_base_x = 3'd0;
                TYPE_L:     get_base_x = 3'd1;
                TYPE_R:     get_base_x = 3'd0;
                default:    get_base_x = 3'd1; // TYPE_INNER
            endcase
        end
    endfunction
    
    function [2:0] get_base_y;  // 3비트로 수정
        input [3:0] btype;
        begin
            case (btype)
                TYPE_TL:    get_base_y = 3'd1;
                TYPE_TR:    get_base_y = 3'd1;
                TYPE_BL:    get_base_y = 3'd0;
                TYPE_BR:    get_base_y = 3'd0;
                TYPE_T:     get_base_y = 3'd1;
                TYPE_B:     get_base_y = 3'd0;
                TYPE_L:     get_base_y = 3'd0;
                TYPE_R:     get_base_y = 3'd0;
                default:    get_base_y = 3'd1; // TYPE_INNER
            endcase
        end
    endfunction
    
    // Get total rows for each block type
    function [2:0] get_total_rows;
        input [3:0] btype;
        begin
            case (btype)
                TYPE_TL, TYPE_TR, TYPE_BL, TYPE_BR: get_total_rows = 3'd5;  // 5 rows
                TYPE_T, TYPE_B: get_total_rows = 3'd4;                      // 4 rows  
                TYPE_L, TYPE_R: get_total_rows = 3'd5;                      // 5 rows
                TYPE_INNER: get_total_rows = 3'd4;                          // 4 rows
                default: get_total_rows = 3'd4;
            endcase
        end
    endfunction
    
    // Get row length for each block type and row
    function [2:0] get_row_len;
        input [3:0] btype;
        input [2:0] row;
        begin
            case (btype)
                TYPE_TL, TYPE_TR, TYPE_BL, TYPE_BR: get_row_len = 3'd5;     // 5 per row
                TYPE_T, TYPE_B: get_row_len = 3'd5;                         // 5 per row
                TYPE_L, TYPE_R: get_row_len = 3'd4;                         // 4 per row  
                TYPE_INNER: get_row_len = 3'd4;                             // 4 per row
                default: get_row_len = 3'd4;
            endcase
        end
    endfunction
    
    // Get output data from buffer based on write counter and row
    function [31:0] get_output_data;
        input [2:0] cnt;     // 현재 행 내 인덱스 (0~4)
        input [2:0] row;     // 현재 행 (0~4)
        input [3:0] btype;
        reg [2:0] buf_x, buf_y;
        begin
            // 현재 행의 cnt번째 데이터 반환
            buf_y = row;
            buf_x = cnt;
            get_output_data = buffer[buf_x][buf_y];
        end
    endfunction
    
    // Sequential logic
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state <= S_IDLE;
            src_addr <= 32'd0;
            dst_addr <= 32'd0;
            mat_width <= 6'd0;
            block_x <= 6'd0;
            block_y <= 6'd0;
            blocks_per_row <= 6'd0;
            blocks_per_col <= 6'd0;
            read_row <= 2'd0;
            read_col <= 2'd0;
            write_cnt <= 5'd0;
            write_len <= 5'd0;
            burst_cnt <= 5'd0;
            write_row <= 3'd0;
            block_type <= 4'd0;
            done <= 1'b1;
        end else begin
            state <= state_n;
            src_addr <= src_addr_n;
            dst_addr <= dst_addr_n;
            mat_width <= mat_width_n;
            block_x <= block_x_n;
            block_y <= block_y_n;
            blocks_per_row <= blocks_per_row_n;
            blocks_per_col <= blocks_per_col_n;
            read_row <= read_row_n;
            read_col <= read_col_n;
            write_cnt <= write_cnt_n;
            write_len <= write_len_n;
            burst_cnt <= burst_cnt_n;
            write_row <= write_row_n;
            block_type <= block_type_n;
            done <= done_n;
            
            // Update wdata_reg when in WDATA state
            if (state == S_WDATA || state_n == S_WDATA) begin
                wdata_reg <= get_output_data(write_cnt, write_row, block_type);
            end
        end
    end

    // Combinational logic
    always_comb begin
        state_n = state;
        src_addr_n = src_addr;
        dst_addr_n = dst_addr;
        mat_width_n = mat_width;
        block_x_n = block_x;
        block_y_n = block_y;
        blocks_per_row_n = blocks_per_row;
        blocks_per_col_n = blocks_per_col;
        read_row_n = read_row;
        read_col_n = read_col;
        write_cnt_n = write_cnt;
        write_len_n = write_len;
        burst_cnt_n = burst_cnt;
        write_row_n = write_row;
        block_type_n = block_type;
        done_n = done;
        
        arvalid = 1'b0;
        rready = 1'b0;
        awvalid = 1'b0;
        wvalid = 1'b0;

            case (state)
                S_IDLE: begin
                done_n = 1'b1;
                if (start_i && mat_width_i != 0) begin
                    done_n = 1'b0;
                    src_addr_n = src_addr_i;
                    dst_addr_n = dst_addr_i;
                    mat_width_n = mat_width_i;
                    block_x_n = 6'd0;
                    block_y_n = 6'd0;
                    blocks_per_row_n = mat_width_i >> 2;  // /4
                    blocks_per_col_n = mat_width_i >> 2;  // /4
                    read_row_n = 2'd0;
                    read_col_n = 2'd0;
                    block_type_n = detect_block_type(6'd0, 6'd0, mat_width_i >> 2, mat_width_i >> 2);
                    
                    $display("[DEBUG] Starting DMA: src=%h, dst=%h, width=%d", 
                            src_addr_i, dst_addr_i, mat_width_i);
                    $display("[DEBUG] Blocks: %dx%d", mat_width_i >> 2, mat_width_i >> 2);
                    
                    state_n = S_RREQ;
                end
            end
            
            S_RREQ: begin
                arvalid = 1'b1;
                if (ar_handshake) begin
                    state_n = S_RDATA;
                    read_col_n = 2'd0;
                    
                    $display("[DEBUG] Read REQ: block(%d,%d), type=%s, row=%d, addr=%h", 
                            block_x, block_y, get_block_type_name(block_type), read_row, araddr_o);
                end
            end
            
            S_RDATA: begin
                rready = 1'b1;
                if (r_handshake) begin
                    read_col_n = read_col + 1;
                    
                    $display("[DEBUG] Read DATA[%d,%d] = %d -> buffer[%d][%d]", 
                            read_row, read_col, rdata_i,
                            get_base_x(block_type) + read_col, get_base_y(block_type) + read_row);
                    
                    if (read_burst_complete) begin
                        // One row completed
                        read_col_n = 2'd0;  // Reset column for next row
                        if (read_4x4_complete) begin
                            // All 4 rows read, process padding
                            state_n = S_PROCESS;
                        end else begin
                            // Read next row
                            read_row_n = read_row + 1;
                            state_n = S_RREQ;
                        end
                    end
                end
            end
            
            S_PROCESS: begin
                // Apply padding based on block type - 한 번만 실행
                write_cnt_n = 3'd0;
                write_row_n = 3'd0; 
                write_len_n = get_row_len(block_type, 3'd0); // 첫 번째 행 길이
                state_n = S_WREQ;
                
                $display("[DEBUG] Processing block type %d, total_rows=%d", block_type, get_total_rows(block_type));
                $display("[DEBUG] Starting row 0, len=%d, awlen will be %d", get_row_len(block_type, 3'd0), get_row_len(block_type, 3'd0) - 1);
            end
            
            S_WREQ: begin
                awvalid = 1'b1;
                if (aw_handshake) begin
                    state_n = S_WDATA;
                    burst_cnt_n = awlen_o;  // RTL_new 패턴: awlen_o 값으로 초기화
                    
                    $display("[DEBUG] Write REQ: block(%d,%d), type=%s, row=%d, len=%d, addr=%h", 
                            block_x, block_y, get_block_type_name(block_type), write_row, awlen_o, awaddr_o);
                    $display("[DEBUG] write_len=%d, awlen_o=%d, burst_cnt_n=%d", 
                            write_len, awlen_o, awlen_o);
                    $display("[DEBUG] Buffer row %d: [0]=%d, [1]=%d, [2]=%d, [3]=%d, [4]=%d", 
                            write_row, buffer[0][write_row], buffer[1][write_row], buffer[2][write_row], buffer[3][write_row], buffer[4][write_row]);
                end
            end
            
            S_WDATA: begin
                wvalid = write_data_available;
                if (w_handshake) begin
                    write_cnt_n = write_cnt + 1;
                    burst_cnt_n = burst_cnt - 1;  // SGDMAC 패턴: burst counter 감소
                    
                    $display("[DEBUG] Write DATA[%d] = %d (from buffer[%d][%d])", 
                            write_cnt, wdata_o, write_cnt, write_row);
                    
                    if (is_last_beat) begin
                        // RTL_new 패턴: wlast와 동시에 response 처리
                        if (b_handshake) begin
                            // Same cycle: write complete + response received
                            if (write_row == get_total_rows(block_type) - 1) begin
                                // 현재 블록의 모든 행 완료
                                if (all_blocks_done) begin
                                    state_n = S_IDLE;
                                    done_n = 1'b1;
                                    $display("[DEBUG] All blocks completed!");
                                end else begin
                                    // Move to next block immediately (row-major order)
                                    if (block_x == blocks_per_row - 1) begin
                                        block_x_n = 6'd0;
                                        block_y_n = block_y + 1;
                                    end else begin
                                        block_x_n = block_x + 1;
                                        block_y_n = block_y;
                                    end
                                    read_row_n = 2'd0;
                                    block_type_n = detect_block_type(block_x_n, block_y_n, blocks_per_row, blocks_per_col);
                                    state_n = S_RREQ;
                                    $display("[DEBUG] Block (%d,%d) type=%s completed, moving to (%d,%d)", 
                                            block_x, block_y, get_block_type_name(block_type), block_x_n, block_y_n);
                                end
                            end else begin
                                // 다음 행으로 이동
                                write_row_n = write_row + 1;
                                write_cnt_n = 3'd0;
                                write_len_n = get_row_len(block_type, write_row + 1);
                                state_n = S_WREQ;
                                $display("[DEBUG] Row %d completed, moving to row %d", write_row, write_row + 1);
                            end
                        end else begin
                            // Wait for response in next state
                            state_n = S_WRESP;
                        end
                    end
                    end
                end
                
            S_WRESP: begin
                // SGDMAC 패턴: response 대기 상태
                if (b_handshake) begin
                    if (write_row == get_total_rows(block_type) - 1) begin
                        // 현재 블록의 모든 행 완료
                        if (all_blocks_done) begin
                            state_n = S_IDLE;
                            done_n = 1'b1;
                            $display("[DEBUG] All blocks completed!");
                        end else begin
                            // Move to next block (row-major order)
                            if (block_x == blocks_per_row - 1) begin
                                block_x_n = 6'd0;
                                block_y_n = block_y + 1;
                            end else begin
                                block_x_n = block_x + 1;
                                block_y_n = block_y;
                            end
                            
                            read_row_n = 2'd0;
                            block_type_n = detect_block_type(block_x_n, block_y_n, blocks_per_row, blocks_per_col);
                            state_n = S_RREQ;
                            
                            $display("[DEBUG] Block (%d,%d) type=%s completed, moving to (%d,%d)", 
                                    block_x, block_y, get_block_type_name(block_type), block_x_n, block_y_n);
                        end
                    end else begin
                        // 다음 행으로 이동
                        write_row_n = write_row + 1;
                        write_cnt_n = 3'd0;
                        write_len_n = get_row_len(block_type, write_row + 1);
                        state_n = S_WREQ;
                        $display("[DEBUG] Row %d completed, moving to row %d", write_row, write_row + 1);
                    end
                end
            end
        endcase
    end
    
    // Buffer management in always_ff block
    integer i, j, k, l;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            // Initialize buffer to zero
            for (k = 0; k < 5; k = k + 1) begin
                for (l = 0; l < 5; l = l + 1) begin
                    buffer[k][l] <= 32'd0;
                end
            end
        end else begin
            // Store read data in buffer
            if (state == S_RDATA && rvalid_i && rready_o) begin
                buffer[get_base_x(block_type) + read_col][get_base_y(block_type) + read_row] <= rdata_i;
                $display("[DEBUG] Storing rdata=%d at buffer[%d][%d]", rdata_i, 
                        get_base_x(block_type) + read_col, get_base_y(block_type) + read_row);
            end
            
            // Apply padding in S_PROCESS state
            if (state == S_PROCESS) begin
                case (block_type)
                    TYPE_TL: begin
                        // TL: (1,1) 기준으로 4x4 저장됨, padding 적용
                        buffer[0][0] <= buffer[2][2];  // corner = 첫 번째 데이터
                        for (i = 1; i < 5; i = i + 1) buffer[i][0] <= buffer[i][2];  // top row
                        for (j = 1; j < 5; j = j + 1) buffer[0][j] <= buffer[2][j];  // left col
                        $display("[DEBUG] TL padding: corner buffer[0][0]=%d from buffer[2][2]", buffer[2][2]);
                        $display("[DEBUG] TL padding values: [1][0]=%d, [2][0]=%d, [3][0]=%d, [4][0]=%d", 
                                buffer[1][2], buffer[2][2], buffer[3][2], buffer[4][2]);
                    end
                    TYPE_TR: begin
                        // TR: (0,1) 기준으로 4x4 저장됨
                        buffer[4][0] <= buffer[2][2];  // corner = 마지막 첫행 데이터
                        for (i = 0; i < 4; i = i + 1) buffer[i][0] <= buffer[i][2];  // top row
                        for (j = 1; j < 5; j = j + 1) buffer[4][j] <= buffer[2][j];  // right col
                    end
                    TYPE_BL: begin
                        // BL: (1,0) 기준으로 4x4 저장됨
                        buffer[0][4] <= buffer[2][2];  // corner = 첫 마지막행 데이터
                        for (i = 1; i < 5; i = i + 1) buffer[i][4] <= buffer[i][2];  // bottom row
                        for (j = 0; j < 4; j = j + 1) buffer[0][j] <= buffer[2][j];  // left col
                    end
                    TYPE_BR: begin
                        // BR: (0,0) 기준으로 4x4 저장됨
                        buffer[4][4] <= buffer[2][2];  // corner = 마지막 마지막 데이터
                        for (i = 0; i < 4; i = i + 1) buffer[i][4] <= buffer[i][2];  // bottom row
                        for (j = 0; j < 4; j = j + 1) buffer[4][j] <= buffer[2][j];  // right col
                    end
                    TYPE_T: begin
                        // T: (0,1) 기준으로 4x4 저장됨, top padding만
                        for (i = 0; i < 4; i = i + 1) buffer[i][0] <= buffer[i][2];  // top row
                    end
                    TYPE_B: begin
                        // B: (0,0) 기준으로 4x4 저장됨, bottom padding만
                        for (i = 0; i < 4; i = i + 1) buffer[i][4] <= buffer[i][2];  // bottom row
                    end
                    TYPE_L: begin
                        // L: (1,0) 기준으로 4x4 저장됨, left padding만
                        for (j = 0; j < 4; j = j + 1) buffer[0][j] <= buffer[2][j];  // left col
                    end
                    TYPE_R: begin
                        // R: (0,0) 기준으로 4x4 저장됨, right padding만
                        for (j = 0; j < 4; j = j + 1) buffer[4][j] <= buffer[2][j];  // right col
                    end
                    TYPE_INNER: begin
                        // INNER: (1,1) 기준으로 4x4 저장됨, padding 없음
                    end
                endcase
            end
        end
    end

    // Output assignments
    assign done_o = done;

    assign awid_o = 4'd0;
    assign awaddr_o = calc_write_addr(block_x, block_y, write_row, mat_width, dst_addr, block_type);
    assign awlen_o = write_len - 1;   // burst length (0-based)
    assign awsize_o = 3'b010;         // 4 bytes per transfer
    assign awburst_o = 2'b01;         // incremental
    assign awvalid_o = awvalid;

    assign wid_o = 4'd0;
    assign wdata_o = wdata_reg;
    assign wstrb_o = 4'b1111;         // all bytes valid
    assign wlast_o = (state == S_WDATA) & is_last_beat;  // SGDMAC 패턴
    assign wvalid_o = wvalid;

    assign bready_o = (state == S_WDATA) & is_last_beat | (state == S_WRESP);  // RTL_new 패턴

    assign arid_o = 4'd0;
    assign araddr_o = calc_read_addr(block_x, block_y, read_row, mat_width, src_addr);
    assign arlen_o = 4'd3;            // 4-beat burst (0~3)
    assign arsize_o = 3'b010;         // 4 bytes per transfer
    assign arburst_o = 2'b01;         // incremental
    assign arvalid_o = arvalid;

    assign rready_o = rready;

endmodule 