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
    localparam IDLE         = 3'd0;
    localparam READ_ADDR    = 3'd1;
    localparam READ_DATA    = 3'd2;
    localparam PROCESS      = 3'd3;
    localparam WRITE_ADDR   = 3'd4;
    localparam WRITE_DATA   = 3'd5;
    localparam WRITE_RESP   = 3'd6;

    reg [2:0] state;
    
    // Configuration registers
    reg [31:0] src_addr;
    reg [31:0] dst_addr;
    reg [5:0]  mat_width;
    
    // Block processing variables
    reg [5:0] block_x, block_y;          // 현재 처리 중인 블록 좌표
    reg [5:0] blocks_per_row;            // 한 행당 블록 개수 (mat_width/4)
    reg [5:0] total_blocks;              // 전체 블록 개수
    reg [5:0] current_block;             // 현재 블록 번호 (0부터 시작)
    
    // 5x5 Buffer (x,y 좌표로 접근)
    reg [31:0] buffer [0:4][0:4];        // buffer[x][y]
    
    // Read state variables
    reg [31:0] read_addr;
    reg [1:0]  read_cnt;                 // 읽고 있는 행 인덱스 (0~3)
    reg [1:0]  read_col;                 // burst 내 컬럼 인덱스
    reg [3:0]  read_burst_cnt;
    
    // Write state variables  
    reg [31:0] write_addr;
    reg [2:0]  write_cnt;                // 최대 5개씩 쓰므로 0~4
    reg [3:0]  write_burst_cnt;
    reg [31:0] output_data;
    
    // Block type detection
    reg [3:0] block_type;
    localparam TYPE_TL    = 4'd0;  // Top-Left corner
    localparam TYPE_TR    = 4'd1;  // Top-Right corner  
    localparam TYPE_BL    = 4'd2;  // Bottom-Left corner
    localparam TYPE_BR    = 4'd3;  // Bottom-Right corner
    localparam TYPE_T     = 4'd4;  // Top edge
    localparam TYPE_B     = 4'd5;  // Bottom edge
    localparam TYPE_L     = 4'd6;  // Left edge
    localparam TYPE_R     = 4'd7;  // Right edge
    localparam TYPE_INNER = 4'd8;  // Inner (no edge)
    
    // Handshake signals
    wire ar_handshake = arvalid_o & arready_i;
    wire r_handshake = rvalid_i & rready_o;
    wire aw_handshake = awvalid_o & awready_i;
    wire w_handshake = wvalid_o & wready_i;
    wire b_handshake = bvalid_i & bready_o;
    
    // Control signals
    wire read_burst_complete = r_handshake & rlast_i;
    wire write_burst_complete = w_handshake & wlast_o;
    wire is_last_write_beat = (write_burst_cnt == 4'd0);
    wire all_blocks_done = (current_block >= total_blocks);
    
    // Block type detection function - 매트릭스 크기 기반 동적 판단
    function [3:0] detect_block_type;
        input [5:0] bx, by;           // block coordinates
        input [5:0] mat_width;        // 실제 매트릭스 크기 (8, 16, 32)
        reg [5:0] blocks_per_row, blocks_per_col;
        begin
            blocks_per_row = mat_width >> 2;  // mat_width / 4
            blocks_per_col = mat_width >> 2;  // mat_width / 4
            
            // Corner cases
            if (bx == 0 && by == 0) 
                detect_block_type = TYPE_TL;
            else if (bx == blocks_per_row-1 && by == 0)
                detect_block_type = TYPE_TR;
            else if (bx == 0 && by == blocks_per_col-1)
                detect_block_type = TYPE_BL;
            else if (bx == blocks_per_row-1 && by == blocks_per_col-1)
                detect_block_type = TYPE_BR;
            // Edge cases
            else if (by == 0)
                detect_block_type = TYPE_T;
            else if (by == blocks_per_col-1)
                detect_block_type = TYPE_B;
            else if (bx == 0)
                detect_block_type = TYPE_L;
            else if (bx == blocks_per_row-1)
                detect_block_type = TYPE_R;
            // Inner case
            else
                detect_block_type = TYPE_INNER;
        end
    endfunction

    // Apply padding to 5x5 buffer based on block type
    task automatic apply_padding;
        integer x;
        integer y;
        begin
            case (block_type)
                TYPE_TL: begin
                    buffer[0][0] <= buffer[2][2];
                    for (x = 0; x < 5; x++) buffer[x][0] <= buffer[x][2];
                    for (y = 0; y < 5; y++) buffer[0][y] <= buffer[2][y];
                end
                TYPE_TR: begin
                    for (x = 0; x < 5; x++) buffer[x][0] <= buffer[x][2];
                    for (y = 0; y < 5; y++) buffer[4][y] <= buffer[2][y];
                end
                TYPE_BL: begin
                    for (x = 0; x < 5; x++) buffer[x][4] <= buffer[x][2];
                    for (y = 0; y < 5; y++) buffer[0][y] <= buffer[2][y];
                end
                TYPE_BR: begin
                    for (x = 0; x < 5; x++) buffer[x][4] <= buffer[x][2];
                    for (y = 0; y < 5; y++) buffer[4][y] <= buffer[2][y];
                end
                TYPE_T: begin
                    for (x = 0; x < 5; x++) buffer[x][0] <= buffer[x][2];
                end
                TYPE_B: begin
                    for (x = 0; x < 5; x++) buffer[x][4] <= buffer[x][2];
                end
                TYPE_L: begin
                    for (y = 0; y < 5; y++) buffer[0][y] <= buffer[2][y];
                end
                TYPE_R: begin
                    for (y = 0; y < 5; y++) buffer[4][y] <= buffer[2][y];
                end
                default: begin
                    // TYPE_INNER : no padding
                end
            endcase
        end
    endtask

    // Buffer base coordinate for each block type
    function [1:0] base_x;
        input [3:0] btype;
        begin
            case (btype)
                TYPE_TL:    base_x = 2'd1;
                TYPE_TR:    base_x = 2'd0;
                TYPE_BL:    base_x = 2'd1;
                TYPE_BR:    base_x = 2'd0;
                TYPE_T:     base_x = 2'd0;
                TYPE_B:     base_x = 2'd0;
                TYPE_L:     base_x = 2'd1;
                TYPE_R:     base_x = 2'd0;
                default:    base_x = 2'd1; // TYPE_INNER
            endcase
        end
    endfunction

    function [1:0] base_y;
        input [3:0] btype;
        begin
            case (btype)
                TYPE_TL:    base_y = 2'd1;
                TYPE_TR:    base_y = 2'd1;
                TYPE_BL:    base_y = 2'd0;
                TYPE_BR:    base_y = 2'd0;
                TYPE_T:     base_y = 2'd1;
                TYPE_B:     base_y = 2'd0;
                TYPE_L:     base_y = 2'd0;
                TYPE_R:     base_y = 2'd0;
                default:    base_y = 2'd1; // TYPE_INNER
            endcase
        end
    endfunction
    
    // Calculate source address for 4x4 block read - 각 행별로 읽기
    function [31:0] calc_read_addr;
        input [5:0] bx, by;
        input [1:0] row_idx;    // 0~3 (블록 내 행 인덱스)
        input [5:0] width;
        reg [5:0] src_row, src_col;
        begin
            // 4x4 블록의 시작 좌표
            src_row = by * 4 + row_idx;  // 블록 시작 행 + 행 오프셋
            src_col = bx * 4;            // 블록의 시작 열
            
            calc_read_addr = src_addr + (src_row * width + src_col) * 4;
        end
    endfunction
    
    // Calculate write address  
    function [31:0] calc_write_addr;
        input [5:0] bx, by;
        input [2:0] write_idx;   // 0~4 (최대 5번의 쓰기)
        input [5:0] width;
        input [3:0] btype;
        reg [5:0] dst_row, dst_col;
        reg [5:0] out_width;
        begin
            out_width = width + 2;  // 출력 매트릭스 폭
            
            // 블록 타입에 따른 출력 시작 위치 계산
            case (btype)
                TYPE_TL, TYPE_TR, TYPE_BL, TYPE_BR: begin
                    dst_row = by * 4 + write_idx;    // 코너는 5개 행 출력
                    dst_col = bx * 4;
                end
                TYPE_T, TYPE_B: begin
                    dst_row = by * 4 + write_idx;    // 상하 가장자리는 4개 행 
                    dst_col = bx * 4 + 1;            // 1칸 오프셋
                end
                TYPE_L, TYPE_R: begin
                    dst_row = by * 4 + 1 + write_idx; // 좌우 가장자리는 4개 행, 1칸 오프셋
                    dst_col = bx * 4;
                end
                TYPE_INNER: begin
                    dst_row = by * 4 + 1 + write_idx; // 내부는 4x4 그대로, 1칸씩 오프셋
                    dst_col = bx * 4 + 1;
                end
            endcase
            
            calc_write_addr = dst_addr + (dst_row * out_width + dst_col) * 4;
        end
    endfunction
    
    // Get buffer value for output - 실제 출력할 데이터 선택
    function [31:0] get_buffer_output;
        input [2:0] write_idx;  // 현재 출력 인덱스 (0~4)
        input [3:0] btype;
        reg [2:0] buf_x, buf_y;
        begin
            // 블록 타입에 따른 버퍼 읽기 위치 결정 (5x5 버퍼에서)
            case (btype)
                TYPE_TL, TYPE_TR, TYPE_BL, TYPE_BR: begin
                    // 코너: 5x5 전체 출력 (row 기준으로 순차 출력)
                    buf_y = write_idx / 5;     // 행 인덱스
                    buf_x = write_idx % 5;     // 열 인덱스
                end
                TYPE_T, TYPE_B: begin
                    // 상하 가장자리: 5x4 출력 (가로 5개, 세로 4개)
                    buf_y = write_idx / 5;
                    buf_x = write_idx % 5;
                end
                TYPE_L, TYPE_R: begin
                    // 좌우 가장자리: 4x5 출력 (가로 4개, 세로 5개)
                    buf_y = write_idx / 4;
                    buf_x = write_idx % 4;
                end
                TYPE_INNER: begin
                    // 내부: 4x4 출력 (패딩 없이 원본 데이터만)
                    buf_y = write_idx / 4 + 1;  // 1~4 행
                    buf_x = write_idx % 4 + 1;  // 1~4 열
                end
                default: begin
                    buf_x = 0;
                    buf_y = 0;
                end
            endcase
            
            get_buffer_output = buffer[buf_x][buf_y];
        end
    endfunction
    
    // Calculate burst length
    wire [3:0] calc_arlen = 4'd3;     // 항상 4개씩 읽기 (0~3)
    
    // 블록 타입에 따른 출력 버스트 길이 계산
    function [3:0] calc_awlen_func;
        input [3:0] btype;
        begin
            case (btype)
                TYPE_TL, TYPE_TR, TYPE_BL, TYPE_BR: calc_awlen_func = 4'd24;  // 5x5 = 25개 (0~24)
                TYPE_T, TYPE_B: calc_awlen_func = 4'd19;                      // 5x4 = 20개 (0~19)  
                TYPE_L, TYPE_R: calc_awlen_func = 4'd19;                      // 4x5 = 20개 (0~19)
                TYPE_INNER: calc_awlen_func = 4'd15;                          // 4x4 = 16개 (0~15)
                default: calc_awlen_func = 4'd15;
            endcase
        end
    endfunction
    
    wire [3:0] calc_awlen = calc_awlen_func(block_type);
    
    // AXI Output assignments
    assign done_o = (state == IDLE) && !start_i;
    
    // AR channel
    assign arid_o = 4'd0;
    assign araddr_o = read_addr;
    assign arlen_o = calc_arlen;
    assign arsize_o = 3'b010;
    assign arburst_o = 2'b01;
    assign arvalid_o = (state == READ_ADDR);
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

    // Main state machine
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            src_addr <= 32'd0;
            dst_addr <= 32'd0;
            mat_width <= 6'd0;
            block_x <= 6'd0;
            block_y <= 6'd0;
            blocks_per_row <= 6'd0;
            total_blocks <= 6'd0;
            current_block <= 6'd0;
            read_addr <= 32'd0;
            read_cnt <= 2'd0;
            read_col <= 2'd0;
            read_burst_cnt <= 4'd0;
            write_addr <= 32'd0;
            write_cnt <= 3'd0;
            write_burst_cnt <= 4'd0;
            output_data <= 32'd0;
            block_type <= 4'd0;
        end else begin
            case (state)
                IDLE: begin
                    if (start_i) begin
                        src_addr <= src_addr_i;
                        dst_addr <= dst_addr_i;
                        mat_width <= mat_width_i;
                        
                        // 블록 계산
                        blocks_per_row <= mat_width_i >> 2;  // divide by 4
                        total_blocks <= (mat_width_i >> 2) * (mat_width_i >> 2);
                        current_block <= 6'd0;
                        block_x <= 6'd0;
                        block_y <= 6'd0;
                        read_addr <= calc_read_addr(6'd0, 6'd0, 2'd0, mat_width_i);
                        block_type <= detect_block_type(6'd0, 6'd0, mat_width_i);

                        $display("[DEBUG] Starting DMA: src=%h, dst=%h, width=%d",
                                src_addr_i, dst_addr_i, mat_width_i);
                        $display("[DEBUG] Blocks per row: %d, Total blocks: %d", 
                                mat_width_i >> 2, (mat_width_i >> 2) * (mat_width_i >> 2));
                        
                        state <= READ_ADDR;
                    end
                end
                
                READ_ADDR: begin
                    if (current_block < total_blocks && ar_handshake) begin
                        state <= READ_DATA;
                        read_burst_cnt <= calc_arlen;
                        read_col <= 2'd0;
                        
                        // 현재 블록의 타입을 동적으로 결정
                        block_type <= detect_block_type(block_x, block_y, mat_width);
                        
                        $display("[DEBUG] Read ADDR: block(%d,%d), row=%d, addr=%h, type=%d", 
                                block_x, block_y, read_cnt, read_addr, detect_block_type(block_x, block_y, mat_width));
                    end else if (current_block >= total_blocks) begin
                        state <= IDLE;
                        $display("[DEBUG] All blocks completed!");
                    end
                end
                
                READ_DATA: begin
                    if (r_handshake) begin
                        buffer[base_x(block_type)+read_col][base_y(block_type)+read_cnt] <= rdata_i;
                        read_col <= read_col + 1;
                        read_burst_cnt <= read_burst_cnt - 1;

                        $display("[DEBUG] Read DATA[%d,%d] = %d", read_cnt, read_col, rdata_i);

                        if (read_burst_complete) begin
                            read_col <= 2'd0;
                            if (read_cnt == 2'd3) begin
                                state <= PROCESS;
                            end else begin
                                read_cnt <= read_cnt + 1;
                                read_addr <= calc_read_addr(block_x, block_y, read_cnt + 1, mat_width);
                                state <= READ_ADDR;
                            end
                        end
                    end
                end
                
                PROCESS: begin
                    // 버퍼에 패딩 적용
                    apply_padding();

                    $display("[DEBUG] Processing block type: %d", block_type);

                    // 처리 완료 후 쓰기 준비
                    write_addr <= calc_write_addr(block_x, block_y, 3'd0, mat_width, block_type);
                    write_cnt <= 3'd0;
                    write_burst_cnt <= calc_awlen;
                    output_data <= get_buffer_output(3'd0, block_type);

                    state <= WRITE_ADDR;
                end
                
                WRITE_ADDR: begin
                    if (aw_handshake) begin
                        state <= WRITE_DATA;
                        
                        $display("[DEBUG] Write ADDR: block(%d,%d), addr=%h, len=%d", 
                                block_x, block_y, write_addr, calc_awlen);
                    end
                end
                
                WRITE_DATA: begin
                    if (w_handshake) begin
                        write_cnt <= write_cnt + 1;
                        write_burst_cnt <= write_burst_cnt - 1;
                        
                        $display("[DEBUG] Write DATA[%d] = %d", write_cnt, output_data);
                        
                        // 다음 출력 데이터 준비
                        if (!is_last_write_beat) begin
                            output_data <= get_buffer_output(write_cnt + 1, block_type);
                        end
                        
                        if (write_burst_complete) begin
                            if (b_handshake) begin
                                // 다음 블록으로 이동
                                current_block <= current_block + 1;
                                
                                if (block_x == blocks_per_row - 1) begin
                                    block_x <= 6'd0;
                                    block_y <= block_y + 1;
                                end else begin
                                    block_x <= block_x + 1;
                                end
                                
                                if (current_block + 1 < total_blocks) begin
                                    // 다음 블록 정보 계산
                                    reg [5:0] next_bx, next_by;
                                    if (block_x == blocks_per_row - 1) begin
                                        next_bx = 6'd0;
                                        next_by = block_y + 1;
                                    end else begin
                                        next_bx = block_x + 1;
                                        next_by = block_y;
                                    end
                                    
                                    read_addr <= calc_read_addr(next_bx, next_by, 2'd0, mat_width);
                                    read_cnt <= 2'd0;
                                    state <= READ_ADDR;
                                end else begin
                                    state <= IDLE;
                                    $display("[DEBUG] All blocks completed!");
                                end
                            end else begin
                                state <= WRITE_RESP;
                            end
                        end
                    end
                end
                
                WRITE_RESP: begin
                    if (b_handshake) begin
                        // 다음 블록으로 이동
                        current_block <= current_block + 1;
                        
                        if (block_x == blocks_per_row - 1) begin
                            block_x <= 6'd0;
                            block_y <= block_y + 1;
                        end else begin
                            block_x <= block_x + 1;
                        end
                        
                        if (current_block + 1 < total_blocks) begin
                            // 다음 블록 정보 계산
                            reg [5:0] next_bx, next_by;
                            if (block_x == blocks_per_row - 1) begin
                                next_bx = 6'd0;
                                next_by = block_y + 1;
                            end else begin
                                next_bx = block_x + 1;
                                next_by = block_y;
                            end
                            
                            read_addr <= calc_read_addr(next_bx, next_by, 2'd0, mat_width);
                            read_cnt <= 2'd0;
                            state <= READ_ADDR;
                        end else begin
                            state <= IDLE;
                            $display("[DEBUG] All blocks completed!");
                        end
                    end
                end
            endcase
        end
    end

endmodule 