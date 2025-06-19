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
    // Design here

    // State Machine
    localparam S_IDLE           = 4'd0;
    localparam S_READ_MATRIX    = 4'd1;
    localparam S_WRITE_TOP      = 4'd2;
    localparam S_WRITE_MID      = 4'd3;
    localparam S_WRITE_BOT      = 4'd4;
    localparam S_DONE           = 4'd5;

    reg [3:0] state, state_n;
    
    // Internal registers
    reg [31:0] src_addr, src_addr_n;
    reg [31:0] dst_addr, dst_addr_n;
    reg [5:0]  mat_width, mat_width_n;
    reg [5:0]  row_cnt, row_cnt_n;
    reg [5:0]  col_cnt, col_cnt_n;
    reg        done, done_n;
    
    // Matrix buffer to store entire matrix (max 32x32)
    reg [31:0] matrix [0:31][0:31];
    reg [31:0] matrix_n [0:31][0:31];
    
    // Read channel control
    reg        ar_valid, ar_valid_n;
    reg [31:0] ar_addr, ar_addr_n;
    reg [3:0]  ar_len, ar_len_n;
    reg        r_ready, r_ready_n;
    reg [5:0]  read_cnt, read_cnt_n;
    
    // Write channel control
    reg        aw_valid, aw_valid_n;
    reg [31:0] aw_addr, aw_addr_n;
    reg [3:0]  aw_len, aw_len_n;
    reg        w_valid, w_valid_n;
    reg [31:0] w_data, w_data_n;
    reg        w_last, w_last_n;
    reg        b_ready, b_ready_n;
    reg [5:0]  write_cnt, write_cnt_n;
    reg [5:0]  write_row, write_row_n;
    reg [5:0]  write_col, write_col_n;
    
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
    
    // Sequential logic
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_IDLE;
            src_addr <= 32'd0;
            dst_addr <= 32'd0;
            mat_width <= 6'd0;
            row_cnt <= 6'd0;
            col_cnt <= 6'd0;
            done <= 1'b0;
            
            ar_valid <= 1'b0;
            ar_addr <= 32'd0;
            ar_len <= 4'd0;
            r_ready <= 1'b0;
            read_cnt <= 6'd0;
            
            aw_valid <= 1'b0;
            aw_addr <= 32'd0;
            aw_len <= 4'd0;
            w_valid <= 1'b0;
            w_data <= 32'd0;
            w_last <= 1'b0;
            b_ready <= 1'b0;
            write_cnt <= 6'd0;
            write_row <= 6'd0;
            write_col <= 6'd0;
            
            for (integer i = 0; i < 32; i = i + 1) begin
                for (integer j = 0; j < 32; j = j + 1) begin
                    matrix[i][j] <= 32'd0;
                end
            end
        end else begin
            state <= state_n;
            src_addr <= src_addr_n;
            dst_addr <= dst_addr_n;
            mat_width <= mat_width_n;
            row_cnt <= row_cnt_n;
            col_cnt <= col_cnt_n;
            done <= done_n;
            
            ar_valid <= ar_valid_n;
            ar_addr <= ar_addr_n;
            ar_len <= ar_len_n;
            r_ready <= r_ready_n;
            read_cnt <= read_cnt_n;
            
            aw_valid <= aw_valid_n;
            aw_addr <= aw_addr_n;
            aw_len <= aw_len_n;
            w_valid <= w_valid_n;
            w_data <= w_data_n;
            w_last <= w_last_n;
            b_ready <= b_ready_n;
            write_cnt <= write_cnt_n;
            write_row <= write_row_n;
            write_col <= write_col_n;
            
            for (integer i = 0; i < 32; i = i + 1) begin
                for (integer j = 0; j < 32; j = j + 1) begin
                    matrix[i][j] <= matrix_n[i][j];
                end
            end
        end
    end
    
    // Combinational logic
    always @(*) begin
        // Default values
        state_n = state;
        src_addr_n = src_addr;
        dst_addr_n = dst_addr;
        mat_width_n = mat_width;
        row_cnt_n = row_cnt;
        col_cnt_n = col_cnt;
        done_n = done;
        
        ar_valid_n = ar_valid;
        ar_addr_n = ar_addr;
        ar_len_n = ar_len;
        r_ready_n = r_ready;
        read_cnt_n = read_cnt;
        
        aw_valid_n = aw_valid;
        aw_addr_n = aw_addr;
        aw_len_n = aw_len;
        w_valid_n = w_valid;
        w_data_n = w_data;
        w_last_n = w_last;
        b_ready_n = b_ready;
        write_cnt_n = write_cnt;
        write_row_n = write_row;
        write_col_n = write_col;
        
        for (integer i = 0; i < 32; i = i + 1) begin
            for (integer j = 0; j < 32; j = j + 1) begin
                matrix_n[i][j] = matrix[i][j];
            end
        end
        
        case (state)
            S_IDLE: begin
                done_n = 1'b0;
                if (start_i) begin
                    src_addr_n = src_addr_i;
                    dst_addr_n = dst_addr_i;
                    mat_width_n = mat_width_i;
                    row_cnt_n = 6'd0;
                    col_cnt_n = 6'd0;
                    read_cnt_n = 6'd0;
                    state_n = S_READ_MATRIX;
                    
                    // Setup first read
                    ar_valid_n = 1'b1;
                    ar_addr_n = src_addr_i;
                    ar_len_n = mat_width_i - 1; // Read one row
                    r_ready_n = 1'b1;
                end
            end
            
            S_READ_MATRIX: begin
                // AR channel handshake
                if (ar_valid && arready_i) begin
                    ar_valid_n = 1'b0;
                end
                
                // R channel data reception
                if (rvalid_i && r_ready) begin
                    matrix_n[row_cnt][col_cnt] = rdata_i;
                    col_cnt_n = col_cnt + 1;
                    
                    if (rlast_i) begin
                        row_cnt_n = row_cnt + 1;
                        col_cnt_n = 6'd0;
                        
                        if (row_cnt == mat_width - 1) begin
                            // All data read, start writing
                            r_ready_n = 1'b0;
                            state_n = S_WRITE_TOP;
                            write_row_n = 6'd0;
                            write_col_n = 6'd0;
                            write_cnt_n = 6'd0;
                        end else begin
                            // Read next row
                            ar_valid_n = 1'b1;
                            ar_addr_n = src_addr + ((row_cnt + 1) * mat_width * 4);
                            ar_len_n = mat_width - 1;
                        end
                    end
                end
            end
            
            S_WRITE_TOP: begin
                // Write top padding row
                if (!aw_valid || (aw_valid && awready_i)) begin
                    aw_valid_n = 1'b1;
                    aw_addr_n = dst_addr;
                    aw_len_n = mat_width + 1; // N+2 elements
                    w_valid_n = 1'b1;
                    write_cnt_n = 6'd0;
                    b_ready_n = 1'b1;
                end
                
                if (aw_valid && awready_i) begin
                    aw_valid_n = 1'b0;
                end
                
                if (w_valid && wready_i) begin
                    if (write_cnt == 0) begin
                        // Top-left corner
                        w_data_n = matrix[1][1];
                    end else if (write_cnt <= mat_width) begin
                        // Top edge
                        w_data_n = matrix[1][write_cnt - 1];
                    end else begin
                        // Top-right corner
                        w_data_n = matrix[1][mat_width - 2];
                    end
                    
                    write_cnt_n = write_cnt + 1;
                    
                    if (write_cnt == mat_width + 1) begin
                        w_last_n = 1'b1;
                    end else begin
                        w_last_n = 1'b0;
                    end
                    
                    if (w_last) begin
                        w_valid_n = 1'b0;
                        w_last_n = 1'b0;
                        state_n = S_WRITE_MID;
                        write_row_n = 6'd0;
                        write_cnt_n = 6'd0;
                    end
                end
            end
            
            S_WRITE_MID: begin
                // Write middle rows with padding
                if (!aw_valid || (aw_valid && awready_i)) begin
                    aw_valid_n = 1'b1;
                    aw_addr_n = dst_addr + ((write_row + 1) * (mat_width + 2) * 4);
                    aw_len_n = mat_width + 1;
                    w_valid_n = 1'b1;
                    write_cnt_n = 6'd0;
                end
                
                if (aw_valid && awready_i) begin
                    aw_valid_n = 1'b0;
                end
                
                if (w_valid && wready_i) begin
                    if (write_cnt == 0) begin
                        // Left padding
                        w_data_n = matrix[write_row][1];
                    end else if (write_cnt <= mat_width) begin
                        // Original data
                        w_data_n = matrix[write_row][write_cnt - 1];
                    end else begin
                        // Right padding
                        w_data_n = matrix[write_row][mat_width - 2];
                    end
                    
                    write_cnt_n = write_cnt + 1;
                    
                    if (write_cnt == mat_width + 1) begin
                        w_last_n = 1'b1;
                    end else begin
                        w_last_n = 1'b0;
                    end
                    
                    if (w_last) begin
                        w_valid_n = 1'b0;
                        w_last_n = 1'b0;
                        write_row_n = write_row + 1;
                        write_cnt_n = 6'd0;
                        
                        if (write_row == mat_width - 1) begin
                            state_n = S_WRITE_BOT;
                            write_cnt_n = 6'd0;
                        end
                    end
                end
            end
            
            S_WRITE_BOT: begin
                // Write bottom padding row
                if (!aw_valid || (aw_valid && awready_i)) begin
                    aw_valid_n = 1'b1;
                    aw_addr_n = dst_addr + ((mat_width + 1) * (mat_width + 2) * 4);
                    aw_len_n = mat_width + 1;
                    w_valid_n = 1'b1;
                    write_cnt_n = 6'd0;
                end
                
                if (aw_valid && awready_i) begin
                    aw_valid_n = 1'b0;
                end
                
                if (w_valid && wready_i) begin
                    if (write_cnt == 0) begin
                        // Bottom-left corner
                        w_data_n = matrix[mat_width - 2][1];
                    end else if (write_cnt <= mat_width) begin
                        // Bottom edge
                        w_data_n = matrix[mat_width - 2][write_cnt - 1];
                    end else begin
                        // Bottom-right corner
                        w_data_n = matrix[mat_width - 2][mat_width - 2];
                    end
                    
                    write_cnt_n = write_cnt + 1;
                    
                    if (write_cnt == mat_width + 1) begin
                        w_last_n = 1'b1;
                    end else begin
                        w_last_n = 1'b0;
                    end
                    
                    if (w_last) begin
                        w_valid_n = 1'b0;
                        w_last_n = 1'b0;
                        state_n = S_DONE;
                    end
                end
            end
            
            S_DONE: begin
                if (bvalid_i && b_ready) begin
                    b_ready_n = 1'b0;
                    done_n = 1'b1;
                    state_n = S_IDLE;
                end
            end
        endcase
    end

endmodule