/*
 * Module Name: tr_packer
 * Author(s):   Kiet Le
 * Description:
 * The TX Gearbox (Encoder) for the ML-KEM Transcoder.
 * This module reads 256 mathematical coefficients from Polynomial Memory,
 * compresses them using the FIPS 203 exact Barret module, and tightly
 * packs them into continuous 64-bit AXI-Stream beats (ByteEncode_d).
 *
 * Architecture:
 * - Uses a 128-bit shift register to accumulate 4 * d bits per cycle.
 * - Implements a safe "in-flight bit tracking" mechanism to stall memory
 * reads if the downstream AXI bus is backpressured, preventing overflow.
 * - Because ML-KEM coefficient totals (256 * d) are always perfectly
 * divisible by 64 bits, no padding or sub-beat logic is required.
 */

`default_nettype none
`timescale 1ns / 1ps

module tr_packer #(
    parameter int POLY_ID_W = 6,
    parameter int COEFF_W   = 12
) (
    input  wire logic                      clk,
    input  wire logic                      rst,

    // Control from FSM
    input  wire logic                      start_i,
    output      logic                      done_o,
    input  wire logic [3:0]                d_param_i,
    input  wire logic [POLY_ID_W-1:0]      poly_id_i,

    // Data Stream to Router (AXI TX)
    output      logic [63:0]               m_tdata_o,
    output      logic                      m_tvalid_o,
    input  wire logic                      m_tready_i,

    // Top-Level Polynomial Memory (Read Channel Only)
    output      logic                      poly_req_o,
    input  wire logic                      poly_stall_i,
    output      logic                      poly_rd_en_o,
    output      logic [POLY_ID_W-1:0]      poly_rd_poly_id_o,
    output      logic [3:0][7:0]           poly_rd_idx_o,
    output      logic [3:0]                poly_rd_lane_valid_o,
    input  wire logic                      poly_rd_valid_i,
    input  wire logic [3:0][COEFF_W-1:0]   poly_rd_data_i
);

    // ====================================================================
    // Internal State & Registers
    // ====================================================================
    typedef enum logic [1:0] {
        ST_IDLE,
        ST_READING,
        ST_DRAINING
    } state_t;
    state_t state, next_state;

    logic [3:0]           d_param_reg;
    logic [POLY_ID_W-1:0] poly_id_reg;

    // Memory tracking
    logic [5:0]           rd_counter; // 0 to 63 (since 64 reads * 4 = 256 coeffs)
    logic                 read_fire;

    // Gearbox registers
    logic [127:0]         shift_reg;
    logic [7:0]           bit_count;     // Current bits stored in shift_reg
    logic [7:0]           inflight_bits; // Bits requested but not yet returned
    logic [7:0]           bits_per_cycle;

    // ====================================================================
    // Compress Instantiation
    // ====================================================================
    logic [3:0][COEFF_W-1:0] comp_coeffs;

    compress u_compress (
        .coeff_i (poly_rd_data_i),
        .d_i     (d_param_reg),
        .coeff_o (comp_coeffs)
    );

    // ====================================================================
    // Dynamic Bit Packer (ByteEncode_d)
    // ====================================================================
    logic [47:0] packed_4d;

    // Dynamically crushes the 4x12-bit compressed array down to EXACTLY 4*d bits.
    // LSBs of lane 0 become the absolute LSBs of the packed word.
    always_comb begin
        packed_4d = '0;
        bits_per_cycle = 8'd0;
        case (d_param_reg)
            4'd1:  begin bits_per_cycle = 8'd4;  packed_4d[3:0]   = {comp_coeffs[3][0],    comp_coeffs[2][0],    comp_coeffs[1][0],    comp_coeffs[0][0]}; end
            4'd4:  begin bits_per_cycle = 8'd16; packed_4d[15:0]  = {comp_coeffs[3][3:0],  comp_coeffs[2][3:0],  comp_coeffs[1][3:0],  comp_coeffs[0][3:0]}; end
            4'd5:  begin bits_per_cycle = 8'd20; packed_4d[19:0]  = {comp_coeffs[3][4:0],  comp_coeffs[2][4:0],  comp_coeffs[1][4:0],  comp_coeffs[0][4:0]}; end
            4'd10: begin bits_per_cycle = 8'd40; packed_4d[39:0]  = {comp_coeffs[3][9:0],  comp_coeffs[2][9:0],  comp_coeffs[1][9:0],  comp_coeffs[0][9:0]}; end
            4'd11: begin bits_per_cycle = 8'd44; packed_4d[43:0]  = {comp_coeffs[3][10:0], comp_coeffs[2][10:0], comp_coeffs[1][10:0], comp_coeffs[0][10:0]}; end
            4'd12: begin bits_per_cycle = 8'd48; packed_4d[47:0]  = {comp_coeffs[3][11:0], comp_coeffs[2][11:0], comp_coeffs[1][11:0], comp_coeffs[0][11:0]}; end
            default: begin bits_per_cycle = 8'd0; end
        endcase
    end

    // ====================================================================
    // Memory Backpressure Control
    // ====================================================================
    // To prevent the 128-bit shift_reg from overflowing, we stall memory
    // reads if the currently buffered bits PLUS the bits currently requested
    // from memory (in flight) exceed the remaining space.
    logic stall_read;
    assign stall_read = (bit_count + inflight_bits + bits_per_cycle) > 8'd128;

    assign poly_req_o           = (state == ST_READING);
    assign poly_rd_en_o         = (state == ST_READING) && !stall_read;
    assign poly_rd_poly_id_o    = poly_id_reg;
    assign poly_rd_lane_valid_o = 4'b1111; // Always read 4 lanes at once

    // Address generation: {rd_counter, 2'bxx}
    assign poly_rd_idx_o[0] = {rd_counter, 2'b00};
    assign poly_rd_idx_o[1] = {rd_counter, 2'b01};
    assign poly_rd_idx_o[2] = {rd_counter, 2'b10};
    assign poly_rd_idx_o[3] = {rd_counter, 2'b11};

    assign read_fire = poly_rd_en_o && !poly_stall_i;

    // ====================================================================
    // Main Control FSM
    // ====================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            state <= ST_IDLE;
        end else begin
            state <= next_state;
        end
    end

    always_comb begin
        next_state = state;
        case (state)
            ST_IDLE: begin
                if (start_i) next_state = ST_READING;
            end
            ST_READING: begin
                if (read_fire && (rd_counter == 6'd63)) next_state = ST_DRAINING;
            end
            ST_DRAINING: begin
                // Done when all in-flight memory returns and buffer is empty
                if ((inflight_bits == 0) && (bit_count == 0)) next_state = ST_IDLE;
            end
            default: next_state = ST_IDLE;
        endcase
    end

    // ====================================================================
    // Datapath & Gearbox Sequential Logic
    // ====================================================================
    logic axim_tx_fire;
    assign axim_tx_fire = m_tvalid_o && m_tready_i;

    assign m_tvalid_o = (bit_count >= 64);
    assign m_tdata_o  = shift_reg[63:0];

    always_ff @(posedge clk) begin
        if (rst) begin
            d_param_reg   <= '0;
            poly_id_reg   <= '0;
            rd_counter    <= '0;
            shift_reg     <= '0;
            bit_count     <= '0;
            inflight_bits <= '0;
            done_o        <= 1'b0;
        end else begin
            done_o <= 1'b0; // Default off

            if (state == ST_IDLE) begin
                if (start_i) begin
                    d_param_reg <= d_param_i;
                    poly_id_reg <= poly_id_i;
                    rd_counter  <= '0;
                end
            end
            else begin
                // Track memory read requests
                if (read_fire) rd_counter <= rd_counter + 1;

                // Track in-flight bits
                if (read_fire && !poly_rd_valid_i)      inflight_bits <= inflight_bits + bits_per_cycle;
                else if (!read_fire && poly_rd_valid_i) inflight_bits <= inflight_bits - bits_per_cycle;

                // Gearbox Shift Register Operations
                if (axim_tx_fire && poly_rd_valid_i) begin
                    // Simultaneous Push and Pop
                    shift_reg <= (shift_reg >> 64) | (128'(packed_4d) << (bit_count - 64));
                    bit_count <= bit_count - 64 + bits_per_cycle;
                end
                else if (axim_tx_fire) begin
                    // Pop only
                    shift_reg <= shift_reg >> 64;
                    bit_count <= bit_count - 64;
                end
                else if (poly_rd_valid_i) begin
                    // Push only
                    shift_reg <= shift_reg | (128'(packed_4d) << bit_count);
                    bit_count <= bit_count + bits_per_cycle;
                end

                // Completion signal
                if ((state == ST_DRAINING) && (next_state == ST_IDLE)) begin
                    done_o <= 1'b1;
                end
            end
        end
    end

endmodule

`default_nettype wire
