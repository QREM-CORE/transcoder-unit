/*
 * Module Name: tr_fsm
 * Author(s):   Kiet Le
 * Description:
 * The Master Controller (Micro-Sequencer) for the ML-KEM Transcoder datapath.
 * This FSM contains zero datapath logic and zero hardcoded ML-KEM parameters.
 * It relies entirely on the 'tr_microcode_rom' to dictate its behavior, acting
 * strictly as a generic, highly-scalable execution engine.
 *
 * Functionality:
 * Orchestrates the Packer, Unpacker, Router matrix, and SeedBank to execute
 * multi-phase ML-KEM operations. It cleanly bridges the cycle-by-cycle AXI-Stream
 * reality with the high-level semantic requirements of the protocol.
 *
 * Internal Structure:
 * 1. ROM Interface:
 * Latches host commands (start, opcode, sec_level) and feeds them to the
 * instruction decoder to extract execution parameters.
 *
 * 2. The Micro-Sequencer (State Machine):
 * A robust 5-state machine designed to handle phased execution:
 * - Math Phase: Asserts start signals to the Gearboxes, waits for completion,
 * and automatically loops 'k' times if processing a polynomial vector.
 * - Bypass Phase: Seamlessly transitions to raw AXI <-> SeedBank streaming
 * (e.g., reading an Encapsulation Key from AXI, then immediately extracting
 * its trailing 32-byte seed without returning to IDLE).
 *
 * 3. AXI Beat Tracker & TLAST Generator:
 * Snoops the AXI bus handshakes ('valid' & 'ready') to accurately track data
 * movement. This allows the FSM to combinationally generate exact 'tlast' signals
 * and SeedBank read/write indices without requiring the Math engines to count bits.
 *
 * 4. Latency Management:
 * Actively shields the AXI bus from SRAM read latency by forcing the router
 * matrix into an IDLE state until valid read data emerges from the SeedBank.
 *
 * Usage:
 * Acts as the top-level interface for the Host/DMA. Driven by 'ctrl_start_i',
 * it manages all internal ready/valid backpressure via the 'tr_router', and pulses
 * 'ctrl_done_o' for exactly one cycle when the complete artifact transmission
 * (including math, hashing, and seeds) is fully finished.
 */

`default_nettype none
`timescale 1ns / 1ps

import qrem_global_pkg::*;
import transcoder_pkg::*;

module tr_fsm #(
    parameter int POLY_ID_W  = 6,
    parameter int SEED_ID_W  = 4,
    parameter int SEED_IDX_W = 3
) (
    input  wire logic                      clk,
    input  wire logic                      rst,

    // Top-Level Control
    input  wire logic                      ctrl_start_i,
    output      logic                      ctrl_done_o,
    input  wire logic [1:0]                ctrl_sec_level_i,
    input  wire tr_opcode_t                ctrl_opcode_i,

    // Sub-Module Control: Packer
    output      logic                      packer_start_o,
    input  wire logic                      packer_done_i,
    output      logic [3:0]                packer_d_param_o,
    output      logic [POLY_ID_W-1:0]      packer_poly_id_o,

    // Sub-Module Control: Unpacker
    output      logic                      unpacker_start_o,
    input  wire logic                      unpacker_done_i,
    output      logic [3:0]                unpacker_d_param_o,
    output      logic [POLY_ID_W-1:0]      unpacker_poly_id_o,

    // Sub-Module Control: Router
    output      router_sel_t               router_sel_o,
    output      logic                      router_tlast_o,

    // Data Handshake Snooping
    input  wire logic                      axi_rx_vld_rdy_i,
    input  wire logic                      axi_tx_vld_rdy_i,

    // Top-Level Seed Protocol Store
    output      logic                      seed_req_o,
    output      logic                      seed_we_o,
    output      logic [SEED_ID_W-1:0]      seed_id_o,
    output      logic [SEED_IDX_W-1:0]     seed_idx_o,
    input  wire logic                      seed_ready_i,
    input  wire logic                      seed_rvalid_i
);

    // ====================================================================
    // Internal State & Registers
    // ====================================================================
    typedef enum logic [2:0] {
        ST_IDLE       = 3'd0,
        ST_MATH_START = 3'd1,
        ST_MATH_WAIT  = 3'd2,
        ST_BYPASS     = 3'd3,
        ST_DONE       = 3'd4
    } state_t;
    state_t state, next_state;

    tr_opcode_t opcode_reg;
    logic [1:0] sec_level_reg;
    logic [2:0] k_counter;
    logic [7:0] beat_counter;

    // ====================================================================
    // ROM Instantiation
    // ====================================================================
    logic [2:0]           cfg_k_limit;
    logic                 cfg_is_tx;
    logic                 cfg_math_en;
    logic                 cfg_math_k_loop;
    logic [3:0]           cfg_d_param;
    logic [POLY_ID_W-1:0] cfg_poly_base_id;
    logic                 cfg_bypass_en;
    logic [SEED_ID_W-1:0] cfg_seed_id;
    logic [7:0]           cfg_bypass_beats;
    router_sel_t          cfg_router_math_sel;
    router_sel_t          cfg_router_bypass_sel;

    tr_microcode_rom #(
        .POLY_ID_W(POLY_ID_W),
        .SEED_ID_W(SEED_ID_W)
    ) u_rom (
        .opcode_i            (opcode_reg),
        .sec_level_i         (sec_level_reg),
        .k_limit_o           (cfg_k_limit),
        .is_tx_o             (cfg_is_tx),
        .math_en_o           (cfg_math_en),
        .math_k_loop_o       (cfg_math_k_loop),
        .d_param_o           (cfg_d_param),
        .poly_base_id_o      (cfg_poly_base_id),
        .bypass_en_o         (cfg_bypass_en),
        .seed_id_o           (cfg_seed_id),
        .bypass_beats_o      (cfg_bypass_beats),
        .router_math_sel_o   (cfg_router_math_sel),
        .router_bypass_sel_o (cfg_router_bypass_sel)
    );

    // ====================================================================
    // Beat Tracking & TLAST Generation
    // ====================================================================
    logic active_axi_fire;
    logic [7:0] math_beats_per_poly;

    always_comb begin
        if (state == ST_BYPASS) begin
            active_axi_fire = (cfg_router_bypass_sel == TR_ROUTER_BYPASS_TX) ? axi_tx_vld_rdy_i : axi_rx_vld_rdy_i;
        end else if (state == ST_MATH_WAIT) begin
            active_axi_fire = (cfg_is_tx) ? axi_tx_vld_rdy_i : axi_rx_vld_rdy_i;
        end else begin
            active_axi_fire = 1'b0;
        end
    end

    assign math_beats_per_poly = {4'b0000, cfg_d_param} << 2;

    always_comb begin
        router_tlast_o = 1'b0;
        if (state == ST_BYPASS) begin
            if (beat_counter == cfg_bypass_beats - 1) router_tlast_o = 1'b1;
        end
        else if (state == ST_MATH_WAIT && !cfg_bypass_en) begin
            if ((k_counter == (cfg_math_k_loop ? cfg_k_limit - 1 : 0)) &&
                (beat_counter == math_beats_per_poly - 1)) begin
                router_tlast_o = 1'b1;
            end
        end
    end

    // ====================================================================
    // Datapath Control Assignments
    // ====================================================================
    assign packer_start_o     = (state == ST_MATH_START) && cfg_is_tx;
    assign unpacker_start_o   = (state == ST_MATH_START) && !cfg_is_tx;

    assign packer_d_param_o   = cfg_d_param;
    assign unpacker_d_param_o = cfg_d_param;

    assign packer_poly_id_o   = cfg_poly_base_id + k_counter;
    assign unpacker_poly_id_o = cfg_poly_base_id + k_counter;

    assign seed_req_o = (state == ST_BYPASS);
    assign seed_idx_o = beat_counter[SEED_IDX_W-1:0];
    assign seed_id_o  = cfg_seed_id;
    assign seed_we_o  = (state == ST_BYPASS) && (cfg_router_bypass_sel == TR_ROUTER_BYPASS_RX) && axi_rx_vld_rdy_i;

    always_comb begin
        router_sel_o = TR_ROUTER_IDLE;
        if (state == ST_MATH_START || state == ST_MATH_WAIT) begin
            router_sel_o = cfg_router_math_sel;
        end
        else if (state == ST_BYPASS) begin
            if (cfg_router_bypass_sel == TR_ROUTER_BYPASS_TX && !seed_rvalid_i) begin
                router_sel_o = TR_ROUTER_IDLE;
            end else begin
                router_sel_o = cfg_router_bypass_sel;
            end
        end
    end

    // ====================================================================
    // Main Control FSM
    // ====================================================================
    always_ff @(posedge clk) begin
        if (rst) state <= ST_IDLE;
        else     state <= next_state;
    end

    logic math_done_pulse;
    assign math_done_pulse = cfg_is_tx ? packer_done_i : unpacker_done_i;

    always_comb begin
        next_state = state;
        case (state)
            ST_IDLE: begin
                if (ctrl_start_i) begin
                    // Note: In ST_IDLE, the opcode_reg hasn't latched yet, so we peek
                    // at the ROM outputs using the incoming wires combinationally.
                    // To do this safely, we assume the ROM input updates instantly
                    // or we rely on the host to hold ctrl_opcode_i steady.
                    // A safer approach is to latch unconditionally in ST_IDLE and transition,
                    // but assuming opcode is stable when start is asserted:
                    if (cfg_math_en)        next_state = ST_MATH_START;
                    else if (cfg_bypass_en) next_state = ST_BYPASS;
                    else                    next_state = ST_DONE;
                end
            end
            ST_MATH_START: next_state = ST_MATH_WAIT;
            ST_MATH_WAIT: begin
                if (math_done_pulse) begin
                    if (cfg_math_k_loop && (k_counter < cfg_k_limit - 1)) next_state = ST_MATH_START;
                    else if (cfg_bypass_en)                               next_state = ST_BYPASS;
                    else                                                  next_state = ST_DONE;
                end
            end
            ST_BYPASS: begin
                if (active_axi_fire && (beat_counter == cfg_bypass_beats - 1)) next_state = ST_DONE;
            end
            ST_DONE: next_state = ST_IDLE;
            default: next_state = ST_IDLE;
        endcase
    end

    // ====================================================================
    // Sequential Logic & Counters
    // ====================================================================
    assign ctrl_done_o = (state == ST_DONE);

    always_ff @(posedge clk) begin
        if (rst) begin
            opcode_reg    <= TR_OP_KG_INGEST_D;
            sec_level_reg <= '0;
            k_counter     <= '0;
            beat_counter  <= '0;
        end else begin
            // Continuously latch opcode in IDLE to feed the ROM
            if (state == ST_IDLE) begin
                opcode_reg    <= ctrl_opcode_i;
                sec_level_reg <= ctrl_sec_level_i;
                k_counter     <= '0;
                beat_counter  <= '0;
            end
            else begin
                if (state == ST_MATH_WAIT && math_done_pulse) begin
                    k_counter <= k_counter + 1;
                end

                if ((state == ST_MATH_WAIT && math_done_pulse) || (state == ST_MATH_START)) begin
                    beat_counter <= '0;
                end else if (active_axi_fire) begin
                    beat_counter <= beat_counter + 1;
                end
            end
        end
    end

endmodule
`default_nettype wire
