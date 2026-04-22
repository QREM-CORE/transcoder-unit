/*
 * Module Name: transcoder_unit
 * Author(s):   Kiet Le
 * Description: Top level module for the ML-KEM transcoder unit.
 * Integrates the Control Path (FSM, Microcode ROM), Datapath Crossbar (Router),
 * and Mathematical Engines (Packer, Unpacker) into a single decoupled architecture.
 */

`default_nettype none
`timescale 1ns / 1ps

import qrem_global_pkg::*;
import transcoder_pkg::*;

module transcoder_unit (
    input  wire logic                            clk,
    input  wire logic                            rst,

    // ==========================================
    // Control Interface (From Main FSM)
    // ==========================================
    input  wire logic                            ctrl_start,
    output      logic                            ctrl_done,
    input  wire logic [1:0]                      ctrl_sec_level, // 00: ML-KEM-512, 01: 768, 10: 1024
    input  wire logic [4:0]                      ctrl_opcode,

    // ==========================================
    // Polynomial Memory Interface (Internal)
    // 4 coefficients per cycle = 4 * 12 bits = 48 bits
    // ==========================================
    // Global Memory Control
    output      logic                            poly_req_o,
    input  wire logic                            poly_stall_i,

    // Read Request Channel (For Compress/Encode) - Driven by Packer
    output      logic                            poly_rd_en_o,
    output      logic [POLY_ID_WIDTH-1:0]        poly_rd_poly_id_o,
    output      logic [3:0][7:0]                 poly_rd_idx_o,
    output      logic [3:0]                      poly_rd_lane_valid_o,

    // Write Request Channel (For Decode/Decompress) - Driven by Unpacker
    output      logic [3:0]                      poly_wr_en_o,
    output      logic [POLY_ID_WIDTH-1:0]        poly_wr_poly_id_o,
    output      logic [3:0][7:0]                 poly_wr_idx_o,
    output      logic [3:0][COEFF_WIDTH-1:0]     poly_wr_data_o,

    // Read Response Channel (Data returning from memory) -> Packer
    input  wire logic                            poly_rd_valid_i,
    input  wire logic [POLY_ID_WIDTH-1:0]        poly_rd_poly_id_i,
    input  wire logic [3:0][7:0]                 poly_rd_idx_i,
    input  wire logic [3:0]                      poly_rd_lane_valid_i,
    input  wire logic [3:0][COEFF_WIDTH-1:0]     poly_rd_data_i,

    // ==========================================
    // Seed / Protocol Store Interface (ID-Based)
    // ==========================================
    output      logic                            seed_req_o,
    output      logic                            seed_we_o,
    output      seed_id_e                        seed_id_o,
    output      logic [$clog2(SEED_BEATS)-1:0]   seed_idx_o,
    output      logic [SEED_W-1:0]               seed_wdata_o,
    input  wire logic                            seed_ready_i,

    // Read Response Channel
    input  wire logic                            seed_rvalid_i,
    input  wire logic [SEED_W-1:0]               seed_rdata_i,

    // ==========================================
    // HASH SNOOP INTERFACE - TRANSCODER -> KECCAK
    // ==========================================
    output      logic [63:0]                     hash_snoop_data_o,
    output      logic [7:0]                      hash_snoop_keep_o,
    output      logic                            hash_snoop_valid_o,
    input  wire logic                            hash_snoop_ready_i,
    output      logic                            hash_snoop_last_o,

    // ==========================================
    // External Data Stream (Host / Keccak Hash)
    // ==========================================
    // AXI4-Stream Ingest (Host -> Transcoder)
    input  wire logic [63:0]                     s_axis_tdata,
    input  wire logic                            s_axis_tvalid,
    output      logic                            s_axis_tready,
    input  wire logic                            s_axis_tlast,

    // AXI4-Stream Egest (Transcoder -> Host)
    output      logic [63:0]                     m_axis_tdata,
    output      logic                            m_axis_tvalid,
    input  wire logic                            m_axis_tready,
    output      logic [7:0]                      m_axis_tkeep,
    output      logic                            m_axis_tlast
);

    // ====================================================================
    // Internal Interconnect Wires
    // ====================================================================

    // FSM <-> Math Engines
    logic                 packer_start;
    logic                 packer_done;
    logic [3:0]           packer_d_param;
    logic [POLY_ID_WIDTH-1:0] packer_poly_id;
    logic                 packer_poly_req;

    logic                 unpacker_start;
    logic                 unpacker_done;
    logic [3:0]           unpacker_d_param;
    logic [POLY_ID_WIDTH-1:0] unpacker_poly_id;
    logic                 unpacker_poly_req;

    // FSM <-> Router
    router_sel_t          router_sel;
    logic                 router_tlast;

    // Math Engines <-> Router
    logic [63:0]          packer_tdata;
    logic                 packer_tvalid;
    logic                 packer_tready;

    logic [63:0]          unpacker_tdata;
    logic                 unpacker_tvalid;
    logic                 unpacker_tready;

    // ====================================================================
    // FSM Snoop & Global Logic
    // ====================================================================
    // The FSM requires handshake snoops to track beat boundaries cleanly.
    logic axi_rx_vld_rdy;
    logic axi_tx_vld_rdy;
    logic internal_rx_vld_rdy;
    logic internal_tx_vld_rdy;

    assign axi_rx_vld_rdy      = s_axis_tvalid & s_axis_tready;
    assign axi_tx_vld_rdy      = m_axis_tvalid & m_axis_tready;
    assign internal_rx_vld_rdy = seed_rvalid_i & unpacker_tready; // SeedBank -> Unpacker
    assign internal_tx_vld_rdy = packer_tvalid & seed_ready_i;    // Packer -> SeedBank

    // PolyMem access is mutually exclusive between Packer and Unpacker based on FSM state.
    assign poly_req_o = packer_poly_req | unpacker_poly_req;


    // ====================================================================
    // Sub-Module Instantiations
    // ====================================================================

    // 1. The Micro-Sequencer
    tr_fsm u_tr_fsm (
        .clk                   (clk),
        .rst                   (rst),

        // Top-Level Control
        .ctrl_start_i          (ctrl_start),
        .ctrl_done_o           (ctrl_done),
        .ctrl_sec_level_i      (ctrl_sec_level),
        .ctrl_opcode_i         (tr_opcode_t'(ctrl_opcode)),

        // Sub-Module Control: Packer
        .packer_start_o        (packer_start),
        .packer_done_i         (packer_done),
        .packer_d_param_o      (packer_d_param),
        .packer_poly_id_o      (packer_poly_id),

        // Sub-Module Control: Unpacker
        .unpacker_start_o      (unpacker_start),
        .unpacker_done_i       (unpacker_done),
        .unpacker_d_param_o    (unpacker_d_param),
        .unpacker_poly_id_o    (unpacker_poly_id),

        // Sub-Module Control: Router
        .router_sel_o          (router_sel),
        .router_tlast_o        (router_tlast),

        // Data Handshake Snooping
        .axi_rx_vld_rdy_i      (axi_rx_vld_rdy),
        .axi_tx_vld_rdy_i      (axi_tx_vld_rdy),
        .internal_rx_vld_rdy_i (internal_rx_vld_rdy),
        .internal_tx_vld_rdy_i (internal_tx_vld_rdy),

        // Top-Level Seed Protocol Store
        .seed_req_o            (seed_req_o),
        .seed_we_o             (seed_we_o),
        .seed_id_o             (seed_id_o),
        .seed_idx_o            (seed_idx_o),
        .seed_ready_i          (seed_ready_i),
        .seed_rvalid_i         (seed_rvalid_i)
    );

    // 2. The Datapath Crossbar
    tr_router u_tr_router (
        .router_sel_i          (router_sel),
        .router_tlast_i        (router_tlast),

        // Host AXI Streams
        .s_axis_tdata_i        (s_axis_tdata),
        .s_axis_tvalid_i       (s_axis_tvalid),
        .s_axis_tready_o       (s_axis_tready),
        .m_axis_tdata_o        (m_axis_tdata),
        .m_axis_tvalid_o       (m_axis_tvalid),
        .m_axis_tready_i       (m_axis_tready),
        .m_axis_tkeep_o        (m_axis_tkeep),
        .m_axis_tlast_o        (m_axis_tlast),

        // Math Engines
        .unpacker_tdata_o      (unpacker_tdata),
        .unpacker_tvalid_o     (unpacker_tvalid),
        .unpacker_tready_i     (unpacker_tready),
        .packer_tdata_i        (packer_tdata),
        .packer_tvalid_i       (packer_tvalid),
        .packer_tready_o       (packer_tready),

        // Hash Snoop
        .hash_snoop_data_o     (hash_snoop_data_o),
        .hash_snoop_keep_o     (hash_snoop_keep_o),
        .hash_snoop_valid_o    (hash_snoop_valid_o),
        .hash_snoop_ready_i    (hash_snoop_ready_i),
        .hash_snoop_last_o     (hash_snoop_last_o),

        // SeedBank Interface
        .seed_rdata_i          (seed_rdata_i),
        .seed_rvalid_i         (seed_rvalid_i),
        .seed_wdata_o          (seed_wdata_o),
        .seed_ready_i          (seed_ready_i)
    );

    // 3. Mathematical TX Engine (Compress/Encode)
    tr_packer u_tr_packer (
        .clk                   (clk),
        .rst                   (rst),

        // Control
        .start_i               (packer_start),
        .done_o                (packer_done),
        .d_param_i             (packer_d_param),
        .poly_id_i             (packer_poly_id),

        // Data to Router
        .m_tdata_o             (packer_tdata),
        .m_tvalid_o            (packer_tvalid),
        .m_tready_i            (packer_tready),

        // PolyMem Read Channel
        .poly_req_o            (packer_poly_req),
        .poly_stall_i          (poly_stall_i),
        .poly_rd_en_o          (poly_rd_en_o),
        .poly_rd_poly_id_o     (poly_rd_poly_id_o),
        .poly_rd_idx_o         (poly_rd_idx_o),
        .poly_rd_lane_valid_o  (poly_rd_lane_valid_o),
        .poly_rd_valid_i       (poly_rd_valid_i),
        .poly_rd_data_i        (poly_rd_data_i)
    );

    // 4. Mathematical RX Engine (Decode/Decompress)
    tr_unpacker u_tr_unpacker (
        .clk                   (clk),
        .rst                   (rst),

        // Control
        .start_i               (unpacker_start),
        .done_o                (unpacker_done),
        .d_param_i             (unpacker_d_param),
        .poly_id_i             (unpacker_poly_id),

        // Data from Router
        .s_tdata_i             (unpacker_tdata),
        .s_tvalid_i            (unpacker_tvalid),
        .s_tready_o            (unpacker_tready),

        // PolyMem Write Channel
        .poly_req_o            (unpacker_poly_req),
        .poly_stall_i          (poly_stall_i),
        .poly_wr_en_o          (poly_wr_en_o),
        .poly_wr_poly_id_o     (poly_wr_poly_id_o),
        .poly_wr_idx_o         (poly_wr_idx_o),
        .poly_wr_data_o        (poly_wr_data_o)
    );

endmodule

`default_nettype wire
