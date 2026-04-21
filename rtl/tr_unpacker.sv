/*
 * Module Name: tr_unpacker
 * Author(s):   Kiet Le
 * Description:
 * The RX Gearbox (Decoder) for the ML-KEM Transcoder.
 * This module absorbs 64-bit continuous AXI-Stream beats, slices them into
 * exact 4 x d-bit chunks (ByteDecode_d), decompresses them using the
 * FIPS 203 exact math module, and writes 256 coefficients to Polynomial Memory.
 *
 * Architecture:
 * - Accumulator: A 128-bit shift register safely caches incoming 64-bit beats.
 * - Slicer: A combinational multiplexer extracts exactly 4 coefficients worth
 * of bits per cycle based on the active security parameter (d).
 * - Backpressure: If the memory stalls, slicing pauses. If the 128-bit buffer
 * fills up (>64 bits held), AXI 'ready' is dropped to prevent overflow.
 */

`default_nettype none
`timescale 1ns / 1ps

module tr_unpacker #(
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

    // Data Stream from Router (AXI RX)
    input  wire logic [63:0]               s_tdata_i,
    input  wire logic                      s_tvalid_i,
    output      logic                      s_tready_o,

    // Top-Level Polynomial Memory (Write Channel Only)
    output      logic                      poly_req_o,
    input  wire logic                      poly_stall_i,
    output      logic [3:0]                poly_wr_en_o,
    output      logic [POLY_ID_W-1:0]      poly_wr_poly_id_o,
    output      logic [3:0][7:0]           poly_wr_idx_o,
    output      logic [3:0][COEFF_W-1:0]   poly_wr_data_o
);

    // ====================================================================
    // Internal State & Registers
    // ====================================================================
    typedef enum logic [1:0] {
        ST_IDLE,
        ST_INGEST,
        ST_DONE
    } state_t;
    state_t state, next_state;

    logic [3:0]           d_param_reg;
    logic [POLY_ID_W-1:0] poly_id_reg;

    // Memory tracking
    logic [5:0]           wr_counter; // 0 to 63 (64 writes * 4 = 256 coeffs)

    // Gearbox registers
    logic [127:0]         shift_reg;
    logic [7:0]           bit_count;  // Current valid bits in shift_reg
    logic [7:0]           bits_per_cycle;

    // ====================================================================
    // Dynamic Bit Slicer (ByteDecode_d)
    // ====================================================================
    logic [47:0]             sliced_bits;
    logic [3:0][COEFF_W-1:0] decomp_in;

    assign sliced_bits = shift_reg[47:0]; // Always observe the bottom bits

    // Slices exactly 4 coefficients worth of bits from the LSBs of the
    // shift register and zero-pads them up to 12 bits for the decompressor.
    always_comb begin
        decomp_in      = '0;
        bits_per_cycle = 8'd0;

        case (d_param_reg)
            4'd1:  begin
                bits_per_cycle = 8'd4;
                decomp_in[0] = {11'd0, sliced_bits[0]};
                decomp_in[1] = {11'd0, sliced_bits[1]};
                decomp_in[2] = {11'd0, sliced_bits[2]};
                decomp_in[3] = {11'd0, sliced_bits[3]};
            end
            4'd4:  begin
                bits_per_cycle = 8'd16;
                decomp_in[0] = {8'd0, sliced_bits[3:0]};
                decomp_in[1] = {8'd0, sliced_bits[7:4]};
                decomp_in[2] = {8'd0, sliced_bits[11:8]};
                decomp_in[3] = {8'd0, sliced_bits[15:12]};
            end
            4'd5:  begin
                bits_per_cycle = 8'd20;
                decomp_in[0] = {7'd0, sliced_bits[4:0]};
                decomp_in[1] = {7'd0, sliced_bits[9:5]};
                decomp_in[2] = {7'd0, sliced_bits[14:10]};
                decomp_in[3] = {7'd0, sliced_bits[19:15]};
            end
            4'd10: begin
                bits_per_cycle = 8'd40;
                decomp_in[0] = {2'd0, sliced_bits[9:0]};
                decomp_in[1] = {2'd0, sliced_bits[19:10]};
                decomp_in[2] = {2'd0, sliced_bits[29:20]};
                decomp_in[3] = {2'd0, sliced_bits[39:30]};
            end
            4'd11: begin
                bits_per_cycle = 8'd44;
                decomp_in[0] = {1'd0, sliced_bits[10:0]};
                decomp_in[1] = {1'd0, sliced_bits[21:11]};
                decomp_in[2] = {1'd0, sliced_bits[32:22]};
                decomp_in[3] = {1'd0, sliced_bits[43:33]};
            end
            4'd12: begin
                bits_per_cycle = 8'd48;
                decomp_in[0] = sliced_bits[11:0];
                decomp_in[1] = sliced_bits[23:12];
                decomp_in[2] = sliced_bits[35:24];
                decomp_in[3] = sliced_bits[47:36];
            end
            default: bits_per_cycle = 8'd0;
        endcase
    end

    // ====================================================================
    // Decompress Instantiation
    // ====================================================================
    logic [3:0][COEFF_W-1:0] decomp_out;

    decompress u_decompress (
        .coeff_i (decomp_in),
        .d_i     (d_param_reg),
        .coeff_o (decomp_out)
    );

    // ====================================================================
    // Backpressure & Fire Logic
    // ====================================================================
    logic axi_rx_fire;
    logic mem_wr_fire;

    // We can accept a new 64-bit beat as long as we have 64 bits of free space.
    assign s_tready_o  = (bit_count <= 8'd64) && (state == ST_INGEST);
    assign axi_rx_fire = s_tvalid_i && s_tready_o;

    // We can write to memory if we have enough bits sliced and memory isn't stalled.
    assign mem_wr_fire = (bit_count >= bits_per_cycle) && !poly_stall_i && (state == ST_INGEST);

    // Memory write assignments
    assign poly_req_o        = (state == ST_INGEST);
    assign poly_wr_en_o      = mem_wr_fire ? 4'b1111 : 4'b0000; // Write 4 lanes at once
    assign poly_wr_poly_id_o = poly_id_reg;
    assign poly_wr_data_o    = decomp_out;

    assign poly_wr_idx_o[0]  = {wr_counter, 2'b00};
    assign poly_wr_idx_o[1]  = {wr_counter, 2'b01};
    assign poly_wr_idx_o[2]  = {wr_counter, 2'b10};
    assign poly_wr_idx_o[3]  = {wr_counter, 2'b11};

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
                if (start_i) next_state = ST_INGEST;
            end
            ST_INGEST: begin
                // Transition out once the 64th write (index 63) successfully fires
                if (mem_wr_fire && (wr_counter == 6'd63)) next_state = ST_DONE;
            end
            ST_DONE: begin
                next_state = ST_IDLE;
            end
            default: next_state = ST_IDLE;
        endcase
    end

    // ====================================================================
    // Datapath & Gearbox Sequential Logic
    // ====================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            d_param_reg <= '0;
            poly_id_reg <= '0;
            wr_counter  <= '0;
            shift_reg   <= '0;
            bit_count   <= '0;
            done_o      <= 1'b0;
        end else begin
            done_o <= (state == ST_DONE);

            if (state == ST_IDLE) begin
                if (start_i) begin
                    d_param_reg <= d_param_i;
                    poly_id_reg <= poly_id_i;
                    wr_counter  <= '0;
                    shift_reg   <= '0;
                    bit_count   <= '0;
                end
            end
            else if (state == ST_INGEST) begin

                // Track memory write progress
                if (mem_wr_fire) wr_counter <= wr_counter + 1;

                // Gearbox Shift Register Operations
                if (axi_rx_fire && mem_wr_fire) begin
                    // Simultaneous Push and Pop
                    // 1. Shift out the used bits
                    // 2. OR in the new 64 bits offset by the remaining bit_count
                    shift_reg <= (shift_reg >> bits_per_cycle) | (128'(s_tdata_i) << (128'(bit_count) - 128'(bits_per_cycle)));
                    bit_count <= bit_count - bits_per_cycle + 64;
                end
                else if (axi_rx_fire) begin
                    // Push only (Accumulate)
                    shift_reg <= shift_reg | (128'(s_tdata_i) << bit_count);
                    bit_count <= bit_count + 64;
                end
                else if (mem_wr_fire) begin
                    // Pop only (Slice)
                    shift_reg <= shift_reg >> bits_per_cycle;
                    bit_count <= bit_count - bits_per_cycle;
                end

            end
        end
    end

endmodule

`default_nettype wire
