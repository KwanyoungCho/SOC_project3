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
    localparam S_IDLE           = 4'd0;
    localparam S_READ_TL        = 4'd1;
    localparam S_READ_TR        = 4'd2;
    localparam S_READ_BL        = 4'd3;
    localparam S_READ_BR        = 4'd4;
    localparam S_WRITE_TL       = 4'd5;
    localparam S_WRITE_TR       = 4'd6;
    localparam S_WRITE_BL       = 4'd7;
    localparam S_WRITE_BR       = 4'd8;
    localparam S_NEXT_BLOCK     = 4'd9;
    localparam S_DONE           = 4'd10;

    reg [3:0] state;
    
    // Configuration registers
    reg [31:0] src_addr;
    reg [31:0] dst_addr;
    reg [5:0]  mat_width;
    reg        done;
    
    // Current 2x2 block position in output matrix (0-based)
    reg [5:0]  block_row;  // 0, 2, 4, ... (width+2-2)
    reg [5:0]  block_col;  // 0, 2, 4, ... (width+2-2)
    
    // Output data for 2x2 block
    reg [31:0] output_block [0:3];  // TL, TR, BL, BR
    
    // AXI control signals
    reg        ar_valid;
    reg [31:0] ar_addr;
    reg        r_ready;
    
    reg        aw_valid;
    reg [31:0] aw_addr;
    reg        w_valid;
    reg [31:0] w_data;
    reg        w_last;
    reg        b_ready;
    
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
    assign arlen_o = 4'd0;      // Single read
    assign arsize_o = 3'b010;   // 4 bytes
    assign arburst_o = 2'b01;   // INCR
    assign arvalid_o = ar_valid;
    assign rready_o = r_ready;
    
    // AXI AW channel  
    assign awid_o = 4'd0;
    assign awaddr_o = aw_addr;
    assign awlen_o = 4'd0;      // Single write transaction
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
    
    // 미러 패딩된 주소 계산 함수
    function [31:0] get_mirrored_addr;
        input signed [6:0] out_r;  // 0-based output row
        input signed [6:0] out_c;  // 0-based output col
        input [5:0] width;
        reg signed [6:0] src_r, src_c;  // 1-based source coordinates
        begin
            // Case별 명확한 처리 - 각 영역을 독립적으로 처리
            case ({(out_r == 0), (out_r == width + 1), (out_c == 0), (out_c == width + 1)})
                // 4'b0000: Normal region (1 ≤ r ≤ width, 1 ≤ c ≤ width)
                4'b0000: begin
                    src_r = out_r;
                    src_c = out_c;
                end
                
                // 4'b1000: Top edge (r=0, 1 ≤ c ≤ width) - 잘 되는 패턴
                4'b1000: begin
                    src_r = 2;  // mirror from row 2
                    src_c = out_c;
                end
                
                // 4'b0100: Bottom edge (r=width+1, 1 ≤ c ≤ width) - Top의 반대
                4'b0100: begin
                    src_r = width - 1;  // mirror from row (width-1)
                    src_c = out_c;
                end
                
                // 4'b0010: Left edge (1 ≤ r ≤ width, c=0) - 잘 되는 패턴
                4'b0010: begin
                    src_r = out_r;
                    src_c = 2;  // mirror from col 2
                end
                
                // 4'b0001: Right edge (1 ≤ r ≤ width, c=width+1) - Left의 반대
                4'b0001: begin
                    src_r = out_r;
                    src_c = width - 1;  // mirror from col (width-1)
                end
                
                // 4'b1010: Top-Left corner (r=0, c=0) - 잘 되는 패턴
                4'b1010: begin
                    src_r = 2;  // mirror from row 2
                    src_c = 2;  // mirror from col 2
                end
                
                // 4'b1001: Top-Right corner (r=0, c=width+1) - TL의 column 반대
                4'b1001: begin
                    src_r = 2;  // mirror from row 2
                    src_c = width - 1;  // mirror from col (width-1)
                end
                
                // 4'b0110: Bottom-Left corner (r=width+1, c=0) - TL의 row 반대
                4'b0110: begin
                    src_r = width - 1;  // mirror from row (width-1)
                    src_c = 2;  // mirror from col 2
                end
                
                // 4'b0101: Bottom-Right corner (r=width+1, c=width+1) - TL의 완전 반대
                4'b0101: begin
                    src_r = width - 1;  // mirror from row (width-1)
                    src_c = width - 1;  // mirror from col (width-1)
                end
                
                default: begin
                    src_r = out_r;
                    src_c = out_c;
                end
            endcase
            
            // Return source address offset (1-based to 0-based conversion)
            get_mirrored_addr = ((src_r - 1) * width + (src_c - 1)) * 4;
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
            
            block_row <= 6'd0;
            block_col <= 6'd0;
            
            ar_valid <= 1'b0;
            ar_addr <= 32'd0;
            r_ready <= 1'b0;
            
            aw_valid <= 1'b0;
            aw_addr <= 32'd0;
            w_valid <= 1'b0;
            w_data <= 32'd0;
            w_last <= 1'b0;
            b_ready <= 1'b0;
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
                        
                        state <= S_READ_TL;
                    end else begin
                        done <= 1'b1;
                    end
                end
                
                S_READ_TL: begin
                    if (!ar_valid) begin
                        ar_valid <= 1'b1;
                        ar_addr <= src_addr + get_mirrored_addr(block_row, block_col, mat_width);
                        r_ready <= 1'b1;
                    end
                    
                    if (ar_handshake) begin
                        ar_valid <= 1'b0;
                    end
                    
                    if (r_handshake) begin
                        output_block[0] <= rdata_i;
                        r_ready <= 1'b0;
                        state <= S_READ_TR;
                    end
                end
                
                S_READ_TR: begin
                    if (!ar_valid) begin
                        ar_valid <= 1'b1;
                        ar_addr <= src_addr + get_mirrored_addr(block_row, block_col + 1, mat_width);
                        r_ready <= 1'b1;
                    end
                    
                    if (ar_handshake) begin
                        ar_valid <= 1'b0;
                    end
                    
                    if (r_handshake) begin
                        output_block[1] <= rdata_i;
                        r_ready <= 1'b0;
                        state <= S_READ_BL;
                    end
                end
                
                S_READ_BL: begin
                    if (!ar_valid) begin
                        ar_valid <= 1'b1;
                        ar_addr <= src_addr + get_mirrored_addr(block_row + 1, block_col, mat_width);
                        r_ready <= 1'b1;
                    end
                    
                    if (ar_handshake) begin
                        ar_valid <= 1'b0;
                    end
                    
                    if (r_handshake) begin
                        output_block[2] <= rdata_i;
                        r_ready <= 1'b0;
                        state <= S_READ_BR;
                    end
                end
                
                S_READ_BR: begin
                    if (!ar_valid) begin
                        ar_valid <= 1'b1;
                        ar_addr <= src_addr + get_mirrored_addr(block_row + 1, block_col + 1, mat_width);
                        r_ready <= 1'b1;
                    end
                    
                    if (ar_handshake) begin
                        ar_valid <= 1'b0;
                    end
                    
                    if (r_handshake) begin
                        output_block[3] <= rdata_i;
                        r_ready <= 1'b0;
                        state <= S_WRITE_TL;
                        
                        // Start write transaction for TL
                        aw_valid <= 1'b1;
                        aw_addr <= dst_addr + (block_row * (mat_width + 2) + block_col) * 4;  // TL address
                    end
                end
                
                S_WRITE_TL: begin
                    if (aw_handshake) begin
                        aw_valid <= 1'b0;
                        w_valid <= 1'b1;
                        w_data <= output_block[0];  // TL
                        w_last <= 1'b1;
                        b_ready <= 1'b1;
                    end
                    
                    if (w_handshake) begin
                        w_valid <= 1'b0;
                        w_last <= 1'b0;
                    end
                    
                    if (b_handshake) begin
                        b_ready <= 1'b0;
                        state <= S_WRITE_TR;
                        
                        // Start write transaction for TR
                        aw_valid <= 1'b1;
                        aw_addr <= dst_addr + (block_row * (mat_width + 2) + block_col + 1) * 4;  // TR address
                    end
                end
                
                S_WRITE_TR: begin
                    if (aw_handshake) begin
                        aw_valid <= 1'b0;
                        w_valid <= 1'b1;
                        w_data <= output_block[1];  // TR
                        w_last <= 1'b1;
                        b_ready <= 1'b1;
                    end
                    
                    if (w_handshake) begin
                        w_valid <= 1'b0;
                        w_last <= 1'b0;
                    end
                    
                    if (b_handshake) begin
                        b_ready <= 1'b0;
                        state <= S_WRITE_BL;
                        
                        // Start write transaction for BL
                        aw_valid <= 1'b1;
                        aw_addr <= dst_addr + ((block_row + 1) * (mat_width + 2) + block_col) * 4;  // BL address
                    end
                end
                
                S_WRITE_BL: begin
                    if (aw_handshake) begin
                        aw_valid <= 1'b0;
                        w_valid <= 1'b1;
                        w_data <= output_block[2];  // BL
                        w_last <= 1'b1;
                        b_ready <= 1'b1;
                    end
                    
                    if (w_handshake) begin
                        w_valid <= 1'b0;
                        w_last <= 1'b0;
                    end
                    
                    if (b_handshake) begin
                        b_ready <= 1'b0;
                        state <= S_WRITE_BR;
                        
                        // Start write transaction for BR
                        aw_valid <= 1'b1;
                        aw_addr <= dst_addr + ((block_row + 1) * (mat_width + 2) + block_col + 1) * 4;  // BR address
                    end
                end
                
                S_WRITE_BR: begin
                    if (aw_handshake) begin
                        aw_valid <= 1'b0;
                        w_valid <= 1'b1;
                        w_data <= output_block[3];  // BR
                        w_last <= 1'b1;
                        b_ready <= 1'b1;
                    end
                    
                    if (w_handshake) begin
                        w_valid <= 1'b0;
                        w_last <= 1'b0;
                    end
                    
                    if (b_handshake) begin
                        b_ready <= 1'b0;
                        state <= S_NEXT_BLOCK;
                    end
                end
                
                S_NEXT_BLOCK: begin
                    // Move to next 2x2 block
                    if (block_col + 2 < mat_width + 2) begin
                        block_col <= block_col + 2;
                    end else begin
                        block_col <= 6'd0;
                        block_row <= block_row + 2;
                    end
                    
                    // Check if done - fixed completion condition
                    if (block_row >= mat_width + 2) begin
                        state <= S_DONE;
                    end else begin
                        state <= S_READ_TL;
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