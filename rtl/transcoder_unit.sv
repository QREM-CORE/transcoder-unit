/*
 * Module Name: transcoder_unit
 * Author(s):   Kiet Le
 * Description: Top level module for the transcoder unit.
 */

`default_nettype none
`timescale 1ns / 1ps

import transcoder_pkg::*;

module transcoder (
    input  logic                            clk,
    input  logic                            rst,

    // ==========================================
    // Control Interface (From Main FSM)
    // ==========================================
    input  logic                            ctrl_start,
    output logic                            ctrl_done,
    input  logic [1:0]                      ctrl_sec_level, // 00: ML-KEM-512, 01: 768, 10: 1024

    // === High-Level Artifact Opcodes ===
    // --- KEYGEN ---
    // 5'b00000: TR_OP_KG_INGEST_D          (AXI-RX -> SeedBank(d))
    // 5'b00001: TR_OP_KG_EXPORT_DK_PKE     (PolyMem(s) -> Encode12 -> AXI-TX)
    // 5'b00010: TR_OP_KG_EXPORT_EK_PKE     (PolyMem(t) -> Encode12 -> AXI-TX/HSU,
    //                                       SeedBank(rho) -> AXI-TX,
    // 5'b00011: TR_OP_KG_EXPORT_HEK        (HSU(H(ek)) -> AXI-TX)
    //                                       SeedBank(rho) -> HSU (transcoder not responsible for this))

    // --- ENCAP ---
    // 5'b00100: TR_OP_EN_INGEST_M          (AXI-RX -> SeedBank(m))
    // 5'b00101: TR_OP_EN_INGEST_EK         (AXI-RX(ek:encoded(t-hat), rho) -> Decode(t-hat)/HSU(ek) -> PolyMem(t), extract Seed(rho) -> seedbank)
    // 5'b00110: TR_OP_EN_MSG_DEC           (SeedBank(m) -> DECODE1/DECOMP1 -> PolyMem(mu))
    // 5'b00111: TR_OP_EN_EXPORT_CT_1       (PolyMem(u) -> Compress_DU/Encode_DU -> AXI-TX)
    // 5'b01000: TR_OP_EN_EXPORT_CT_2       (PolyMem(v) -> Compress_DV/Encode_DV -> AXI-TX)
    // 5'b01001: TR_OP_EN_EXPORT_K          (Seedbank(k) -> AXI-TX)

    // --- DECAP ---
    // 5'b01010: TR_OP_DC_INGEST_DK_PKE     (AXI-RX -> Decode12 -> PolyMem(s))
    // 5'b01011: TR_OP_DC_INGEST_C1         (AXI-RX -> Decode_DU/Decompress_DU -> PolyMem(u'))
    // 5'b01100: TR_OP_DC_INGEST_C2         (AXI-RX -> Decode_DV/Decompress_DV -> PolyMem(v'))
    // 5'b01101: TR_OP_DC_INGEST_Z          (AXI-RX -> SeedBank(z))
    // 5'b01110: TR_OP_DC_MSG_ENC           (PolyMem(w) -> Decode_1/Decomp_1 -> Seedbank(m'))
    // 5'b01111: TR_OP_DC_EXPORT_K          (Seedbank(k) -> AXI-TX)
    // 5'b10000: TR_OP_DC_EXPORT_K_BAR      (Seedbank(k-bar) -> AXI-TX)
    // 5'b10001: TR_OP_DC_EXPORT_R          (Seedbank(r) -> AXI-TX)
    input  logic [4:0]                      ctrl_opcode,

    // ==========================================
    // Polynomial Memory Interface (Internal)
    // 4 coefficients per cycle = 4 * 12 bits = 48 bits
    // ==========================================
    // Global Memory Control
    output logic                            poly_req_o,
    input  logic                            poly_stall_i,

    // Read Request Channel (For Compress/Encode)
    output logic                            poly_rd_en_o,
    output logic [POLY_ID_W-1:0]            poly_rd_poly_id_o,
    output logic [3:0][7:0]                 poly_rd_idx_o,
    output logic [3:0]                      poly_rd_lane_valid_o,

    // Write Request Channel (For Decode/Decompress)
    output logic [3:0]                      poly_wr_en_o,
    output logic [POLY_ID_W-1:0]            poly_wr_poly_id_o,
    output logic [3:0][7:0]                 poly_wr_idx_o,
    output logic [3:0][11:0]                poly_wr_data_o,

    // Read Response Channel (Data returning from memory)
    input  logic                            poly_rd_valid_i,
    input  logic [POLY_ID_W-1:0]            poly_rd_poly_id_i,
    input  logic [3:0][7:0]                 poly_rd_idx_i,
    input  logic [3:0]                      poly_rd_lane_valid_i,
    input  logic [3:0][11:0]                poly_rd_data_i,

    // ==========================================
    // Seed / Protocol Store Interface (ID-Based)
    // ==========================================
    // Routes seeds, hashes, and keys directly to/from the
    // unified memory subsystem using semantic IDs instead of addresses.
    output logic                            seed_req_o,
    output logic                            seed_we_o,
    output logic [SEED_ID_W-1:0]            seed_id_o,      // The semantic seed variable (e.g., RHO, Z)
    output logic [SEED_IDX_W-1:0]           seed_idx_o,     // The 64-bit word offset within the seed
    output logic [SEED_W-1:0]               seed_wdata_o,
    input  logic                            seed_ready_i,

    // Read Response Channel
    input  logic                            seed_rvalid_i,
    input  logic [SEED_ID_W-1:0]            seed_rdata_id_i, // Returns the ID for tracking read responses
    input  logic [SEED_IDX_W-1:0]           seed_rdata_idx_i,// Returns the offset for tracking
    input  logic [SEED_W-1:0]               seed_rdata_i,

    // ==========================================
    // HASH SNOOP INTERFACE - TRANSCODER -> KECCAK
    // ==========================================
    output logic [63:0]                     hash_snoop_data_o,
    output logic [7:0]                      hash_snoop_keep_o, // Can be hardwired to 8'hFF
    output logic                            hash_snoop_valid_o,
    input  logic                            hash_snoop_ready_i,
    output logic                            hash_snoop_last_o

    // ==========================================
    // External Data Stream (Host / Keccak Hash)
    // ==========================================
    // AXI4-Stream Ingest (Host -> Transcoder)
    input  logic [63:0]                     s_axis_tdata,
    input  logic                            s_axis_tvalid,
    output logic                            s_axis_tready,
    input  logic                            s_axis_tlast,

    // AXI4-Stream Egest (Transcoder -> Host)
    output logic [63:0]                     m_axis_tdata,
    output logic                            m_axis_tvalid,
    input  logic                            m_axis_tready,
    output logic [7:0]                      m_axis_tkeep,   // Necessary because the final beat might not be a full 64 bits
    output logic                            m_axis_tlast
);

endmodule

`default_nettype wire
