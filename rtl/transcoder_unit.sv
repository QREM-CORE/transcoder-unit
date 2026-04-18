/*
 * Module Name: transcoder_unit
 * Author(s):
 * Description: Top level module for the transcoder unit.
 */

`default_nettype none
`timescale 1ns / 1ps

import transcoder_pkg::*;

module transcoder (
    input  logic        clk,
    input  logic        rst,

    // ==========================================
    // Control Interface (From Main FSM)
    // ==========================================
    input  logic        ctrl_start,
    output logic        ctrl_done,
    input  logic [1:0]  ctrl_sec_level, // 00: ML-KEM-512, 01: 768, 10: 1024

    // === High-Level Artifact Opcodes ===
    // --- KEYGEN ---
    // 3'b000: TR_OP_KG_INGEST_D      (AXI-RX -> SeedBank(d))
    // 3'b001: TR_OP_KG_EXPORT_DK_PKE (PolyMem(s) -> Encode -> AXI-TX)
    // 3'b010: TR_OP_KG_EXPORT_EK_PKE (PolyMem(t) -> Encode -> AXI-TX/HSU, Seed(rho) -> AXI-TX, Seed(rho) -> HSU)
    // 3'b011: TR_OP_KG_EXPORT_HEK    (HSU(H(ek)) -> AXI-TX)

    // --- ENCAP ---
    // 3'b001: TR_OP_EN_INGEST_EK (AXI-RX -> Decode -> PolyMem(t), extract Seed(rho) -> seedbank)
    // 3'b100: TR_OP_EN_EXPORT_CT (PolyMem(u,v) -> Compress/Encode -> AXI-TX)

    // --- DECAP ---
    // 3'b011: TR_OP_DC_INGEST_DK (AXI-RX -> Decode -> PolyMem(s), extract ek, H(ek), z)
    // 3'b101: TR_OP_DC_INGEST_CT (AXI-RX -> Decode/Decompress -> PolyMem(u,v))
    // 3'b110: TR_OP_DC_MSG_ENC   (AXI-RX -> Decode_1/Decomp_1 -> PolyMem(mu))
    // 3'b111: TR_OP_DC_MSG_DEC   (PolyMem(w) -> Comp_1/Encode_1 -> AXI-TX)
    input  logic [2:0]  ctrl_opcode,

    // ==========================================
    // Polynomial Memory Interface (Internal)
    // 4 coefficients per cycle = 4 * 12 bits = 48 bits
    // ==========================================
    // Global Memory Control
    output logic                       poly_req_o,
    input  logic                       poly_stall_i,

    // Read Request Channel (For Compress/Encode)
    output logic                       poly_rd_en_o,
    output logic [POLY_ID_W-1:0]       poly_rd_poly_id_o,
    output logic [3:0][7:0]            poly_rd_idx_o,
    output logic [3:0]                 poly_rd_lane_valid_o,

    // Write Request Channel (For Decode/Decompress)
    output logic [3:0]                 poly_wr_en_o,
    output logic [POLY_ID_W-1:0]       poly_wr_poly_id_o,
    output logic [3:0][7:0]            poly_wr_idx_o,
    output logic [3:0][11:0]           poly_wr_data_o,

    // Read Response Channel (Data returning from memory)
    input  logic                       poly_rd_valid_i,
    input  logic [POLY_ID_W-1:0]       poly_rd_poly_id_i,
    input  logic [3:0][7:0]            poly_rd_idx_i,
    input  logic [3:0]                 poly_rd_lane_valid_i,
    input  logic [3:0][11:0]           poly_rd_data_i,

    // ==========================================
    // Seed / Protocol Store Interface
    // ==========================================
    // Routes seeds, hashes, and keys directly to/from the
    // unified memory subsystem during Raw Passthrough modes.
    output logic                          seed_req_o,
    output logic                          seed_we_o,
    output logic [$clog2(SEED_DEPTH)-1:0] seed_addr_o,
    output logic [SEED_W-1:0]             seed_wdata_o,
    input  logic                          seed_ready_i,
    input  logic                          seed_rvalid_i,
    input  logic [SEED_W-1:0]             seed_rdata_i,

    // ==========================================
    // HASH SNOOP INTERFACE - TRANSCODER -> KECCAK
    // ==========================================
    // Driven simultaneously with AXI-S TX during specific operations.
    output logic [63:0]        hash_snoop_data_o,
    output logic [7:0]         hash_snoop_keep_o, // Can be hardwired to 8'hFF
    output logic               hash_snoop_valid_o,
    input  logic               hash_snoop_ready_i,
    output logic               hash_snoop_last_o

    // ==========================================
    // External Data Stream (Host / Keccak Hash)
    // ==========================================
    // AXI4-Stream Ingest (Host -> Transcoder)
    input  logic [63:0] s_axis_tdata,
    input  logic        s_axis_tvalid,
    output logic        s_axis_tready,
    input  logic        s_axis_tlast,

    // AXI4-Stream Egest (Transcoder -> Host)
    output logic [63:0] m_axis_tdata,
    output logic        m_axis_tvalid,
    input  logic        m_axis_tready,
    output logic [7:0]  m_axis_tkeep,   // Necessary because the final beat might not be a full 64 bits
    output logic        m_axis_tlast
);

endmodule

`default_nettype wire
