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
    localparam S_IDLE           = 5'd0;
    localparam S_READ_PREV_ROW  = 5'd1;
    localparam S_READ_CURR_ROW  = 5'd2;
    localparam S_READ_NEXT_ROW  = 5'd3;
    localparam S_WRITE_TOP      = 5'd4;
    localparam S_WRITE_MID      = 5'd5;
    localparam S_WRITE_BOT      = 5'd6;
    localparam S_WAIT_WRITE     = 5'd7;
    localparam S_DONE           = 5'd8;

    reg [4:0] state, state_n;
    
    // Internal registers
    reg [31:0] src_addr, src_addr_n;
    reg [31:0] dst_addr, dst_addr_n;
    reg [5:0]  mat_width, mat_width_n;
    reg        done, done_n;
    
    // Row buffers to store 3 rows (previous, current, next)
    reg [31:0] prev_row [0:31];
    reg [31:0] curr_row [0:31];
    reg [31:0] next_row [0:31];
    reg [31:0] prev_row_n [0:31];
    reg [31:0] curr_row_n [0:31];
    reg [31:0] next_row_n [0:31];
    
    // Position counters
    reg [5:0]  row_idx, row_idx_n;
    reg [5:0]  col_idx, col_idx_n;
    reg [5:0]  write_col, write_col_n;
    
    // Read channel control
    reg        ar_valid, ar_valid_n;
    reg [31:0] ar_addr, ar_addr_n;
    reg [3:0]  ar_len, ar_len_n;
    reg        r_ready, r_ready_n;
    reg [5:0]  read_col, read_col_n;
    reg [1:0]  read_row_type, read_row_type_n; // 0: prev, 1: curr, 2: next
    
    // Write channel control
    reg        aw_valid, aw_valid_n;
    reg [31:0] aw_addr, aw_addr_n;
    reg [3:0]  aw_len, aw_len_n;
    reg        w_valid, w_valid_n;
    reg [31:0] w_data, w_data_n;
    reg        w_last, w_last_n;
    reg        b_ready, b_ready_n;
    reg [5:0]  w_cnt, w_cnt_n;
    
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
            done <= 1'b0;
            
            row_idx <= 6'd0;
            col_idx <= 6'd0;
            write_col <= 6'd0;
            
            ar_valid <= 1'b0;
            ar_addr <= 32'd0;
            ar_len <= 4'd0;
            r_ready <= 1'b0;
            read_col <= 6'd0;
            read_row_type <= 2'd0;
            
            aw_valid <= 1'b0;
            aw_addr <= 32'd0;
            aw_len <= 4'd0;
            w_valid <= 1'b0;
            w_data <= 32'd0;
            w_last <= 1'b0;
            b_ready <= 1'b0;
            w_cnt <= 6'd0;
            
            for (integer i = 0; i < 32; i = i + 1) begin
                prev_row[i] <= 32'd0;
                curr_row[i] <= 32'd0;
                next_row[i] <= 32'd0;
            end
        end else begin
            state <= state_n;
            src_addr <= src_addr_n;
            dst_addr <= dst_addr_n;
            mat_width <= mat_width_n;
            done <= done_n;
            
            row_idx <= row_idx_n;
            col_idx <= col_idx_n;
            write_col <= write_col_n;
            
            ar_valid <= ar_valid_n;
            ar_addr <= ar_addr_n;
            ar_len <= ar_len_n;
            r_ready <= r_ready_n;
            read_col <= read_col_n;
            read_row_type <= read_row_type_n;
            
            aw_valid <= aw_valid_n;
            aw_addr <= aw_addr_n;
            aw_len <= aw_len_n;
            w_valid <= w_valid_n;
            w_data <= w_data_n;
            w_last <= w_last_n;
            b_ready <= b_ready_n;
            w_cnt <= w_cnt_n;
            
            for (integer i = 0; i < 32; i = i + 1) begin
                prev_row[i] <= prev_row_n[i];
                curr_row[i] <= curr_row_n[i];
                next_row[i] <= next_row_n[i];
            end
        end
    end
    
    // Get padded value function
    function [31:0] get_padded_value;
        input [5:0] row;
        input [5:0] col;
        input [5:0] width;
        reg [5:0] src_row, src_col;
        begin
            // Top padding
            if (row == 0) begin
                src_row = 1;
            // Bottom padding
            end else if (row == width + 1) begin
                src_row = width - 2;
            // Original rows
            end else begin
                src_row = row - 1;
            end
            
            // Left padding
            if (col == 0) begin
                src_col = 1;
            // Right padding
            end else if (col == width + 1) begin
                src_col = width - 2;
            // Original columns
            end else begin
                src_col = col - 1;
            end
            
            // Return value from appropriate buffer
            if (row_idx == 0) begin
                // Writing first padded row
                get_padded_value = curr_row[src_col];
            end else if (row_idx == width + 1) begin
                // Writing last padded row
                get_padded_value = prev_row[src_col];
            end else begin
                // Writing middle rows
                if (src_row == row_idx - 2) begin
                    get_padded_value = prev_row[src_col];
                end else if (src_row == row_idx - 1) begin
                    get_padded_value = curr_row[src_col];
                end else begin
                    get_padded_value = next_row[src_col];
                end
            end
        end
    endfunction
    
    // Combinational logic
    always @(*) begin
        // Default values
        state_n = state;
        src_addr_n = src_addr;
        dst_addr_n = dst_addr;
        mat_width_n = mat_width;
        done_n = done;
        
        row_idx_n = row_idx;
        col_idx_n = col_idx;
        write_col_n = write_col;
        
        ar_valid_n = ar_valid;
        ar_addr_n = ar_addr;
        ar_len_n = ar_len;
        r_ready_n = r_ready;
        read_col_n = read_col;
        read_row_type_n = read_row_type;
        
        aw_valid_n = aw_valid;
        aw_addr_n = aw_addr;
        aw_len_n = aw_len;
        w_valid_n = w_valid;
        w_data_n = w_data;
        w_last_n = w_last;
        b_ready_n = b_ready;
        w_cnt_n = w_cnt;
        
        for (integer i = 0; i < 32; i = i + 1) begin
            prev_row_n[i] = prev_row[i];
            curr_row_n[i] = curr_row[i];
            next_row_n[i] = next_row[i];
        end
        
        case (state)
            S_IDLE: begin
                done_n = 1'b0;
                if (start_i) begin
                    src_addr_n = src_addr_i;
                    dst_addr_n = dst_addr_i;
                    mat_width_n = mat_width_i;
                    row_idx_n = 6'd0;
                    col_idx_n = 6'd0;
                    read_col_n = 6'd0;
                    write_col_n = 6'd0;
                    state_n = S_READ_CURR_ROW;
                    
                    // Setup first read (row 0)
                    ar_valid_n = 1'b1;
                    ar_addr_n = src_addr_i;
                    ar_len_n = mat_width_i - 1;
                    r_ready_n = 1'b1;
                    read_row_type_n = 2'd1; // curr
                end
            end
            
            S_READ_CURR_ROW: begin
                // AR channel handshake
                if (ar_valid && arready_i) begin
                    ar_valid_n = 1'b0;
                end
                
                // R channel data reception
                if (rvalid_i && r_ready) begin
                    curr_row_n[read_col] = rdata_i;
                    read_col_n = read_col + 1;
                    
                    if (rlast_i) begin
                        read_col_n = 6'd0;
                        
                        if (row_idx == 0) begin
                            // First row read, read second row
                            state_n = S_READ_NEXT_ROW;
                            ar_valid_n = 1'b1;
                            ar_addr_n = src_addr + mat_width * 4;
                            ar_len_n = mat_width - 1;
                        end else begin
                            // Start writing current row
                            state_n = S_WRITE_TOP;
                            r_ready_n = 1'b0;
                        end
                    end
                end
            end
            
            S_READ_NEXT_ROW: begin
                // AR channel handshake
                if (ar_valid && arready_i) begin
                    ar_valid_n = 1'b0;
                end
                
                // R channel data reception
                if (rvalid_i && r_ready) begin
                    next_row_n[read_col] = rdata_i;
                    read_col_n = read_col + 1;
                    
                    if (rlast_i) begin
                        read_col_n = 6'd0;
                        r_ready_n = 1'b0;
                        state_n = S_WRITE_TOP;
                    end
                end
            end
            
            S_WRITE_TOP: begin
                // Write first padded row
                if (!aw_valid || (aw_valid && awready_i)) begin
                    aw_valid_n = 1'b1;
                    aw_addr_n = dst_addr;
                    aw_len_n = mat_width + 1;
                    w_valid_n = 1'b1;
                    w_cnt_n = 6'd0;
                    write_col_n = 6'd0;
                    b_ready_n = 1'b1;
                end
                
                if (aw_valid && awready_i) begin
                    aw_valid_n = 1'b0;
                end
                
                if (w_valid && wready_i) begin
                    w_data_n = get_padded_value(row_idx, write_col, mat_width);
                    write_col_n = write_col + 1;
                    w_cnt_n = w_cnt + 1;
                    
                    if (w_cnt == mat_width + 1) begin
                        w_last_n = 1'b1;
                    end else begin
                        w_last_n = 1'b0;
                    end
                    
                    if (w_last) begin
                        w_valid_n = 1'b0;
                        w_last_n = 1'b0;
                        row_idx_n = 6'd1;
                        write_col_n = 6'd0;
                        state_n = S_WAIT_WRITE;
                    end
                end
            end
            
            S_WRITE_MID: begin
                // Write middle rows with padding
                if (!aw_valid || (aw_valid && awready_i)) begin
                    aw_valid_n = 1'b1;
                    aw_addr_n = dst_addr + (row_idx * (mat_width + 2) * 4);
                    aw_len_n = mat_width + 1;
                    w_valid_n = 1'b1;
                    w_cnt_n = 6'd0;
                    write_col_n = 6'd0;
                end
                
                if (aw_valid && awready_i) begin
                    aw_valid_n = 1'b0;
                end
                
                if (w_valid && wready_i) begin
                    w_data_n = get_padded_value(row_idx, write_col, mat_width);
                    write_col_n = write_col + 1;
                    w_cnt_n = w_cnt + 1;
                    
                    if (w_cnt == mat_width + 1) begin
                        w_last_n = 1'b1;
                    end else begin
                        w_last_n = 1'b0;
                    end
                    
                    if (w_last) begin
                        w_valid_n = 1'b0;
                        w_last_n = 1'b0;
                        write_col_n = 6'd0;
                        state_n = S_WAIT_WRITE;
                    end
                end
            end
            
            S_WRITE_BOT: begin
                // Write last padded row
                if (!aw_valid || (aw_valid && awready_i)) begin
                    aw_valid_n = 1'b1;
                    aw_addr_n = dst_addr + ((mat_width + 1) * (mat_width + 2) * 4);
                    aw_len_n = mat_width + 1;
                    w_valid_n = 1'b1;
                    w_cnt_n = 6'd0;
                    write_col_n = 6'd0;
                end
                
                if (aw_valid && awready_i) begin
                    aw_valid_n = 1'b0;
                end
                
                if (w_valid && wready_i) begin
                    w_data_n = get_padded_value(row_idx, write_col, mat_width);
                    write_col_n = write_col + 1;
                    w_cnt_n = w_cnt + 1;
                    
                    if (w_cnt == mat_width + 1) begin
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
            
            S_WAIT_WRITE: begin
                if (bvalid_i && b_ready) begin
                    b_ready_n = 1'b0;
                    
                    if (row_idx == mat_width + 1) begin
                        // All done
                        state_n = S_DONE;
                    end else if (row_idx == mat_width) begin
                        // Write last padded row
                        row_idx_n = mat_width + 1;
                        state_n = S_WRITE_BOT;
                    end else begin
                        // Prepare for next row
                        row_idx_n = row_idx + 1;
                        
                        // Shift row buffers
                        for (integer i = 0; i < 32; i = i + 1) begin
                            prev_row_n[i] = curr_row[i];
                            curr_row_n[i] = next_row[i];
                        end
                        
                        // Read next row if not at the end
                        if (row_idx < mat_width - 1) begin
                            ar_valid_n = 1'b1;
                            ar_addr_n = src_addr + ((row_idx + 1) * mat_width * 4);
                            ar_len_n = mat_width - 1;
                            r_ready_n = 1'b1;
                            read_col_n = 6'd0;
                            state_n = S_READ_NEXT_ROW;
                        end else begin
                            // Last row, no more reads needed
                            state_n = S_WRITE_MID;
                        end
                    end
                end
            end
            
            S_DONE: begin
                if (bvalid_i && b_ready) begin
                    b_ready_n = 1'b0;
                end
                done_n = 1'b1;
                if (!start_i) begin
                    state_n = S_IDLE;
                end
            end
        endcase
    end

endmodule