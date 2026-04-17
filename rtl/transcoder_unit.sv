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
    input  logic [3:0]  ctrl_d_param,   // 1, 4, 5, 10, 11, or 12
    input  logic [5:0]  ctrl_poly_idx,  // Which polynomial in memory we are targeting
    // input  logic [2:0]  ctrl_seed_idx,  // Which seed in memory we are targeting

    // Core Transcoder Modes:
    // 2'b00 (Enc)     : PolyMem -> Compress_d -> ByteEncode_d -> AXI-TX (and Hash Snoop)
    // 2'b01 (Dec)     : AXI-RX  -> ByteDecode_d -> Decompress_d -> PolyMem
    // 2'b10 (Msg_Enc) : AXI-RX  -> ByteDecode_1 -> Decompress_1 -> PolyMem
    // 2'b11 (Msg_Dec) : PolyMem -> Compress_1   -> ByteEncode_1 -> AXI-TX
    input  logic [1:0]  ctrl_mode,

    // ==========================================
    // Polynomial Memory Interface (Internal)
    // 4 coefficients per cycle = 4 * 12 bits = 48 bits
    // ==========================================
    // Read Channel (For Compress/Encode)
    output logic        poly_rd_req,
    output logic [5:0]  poly_rd_addr,   // 0 to 63 (since 256 / 4 = 64)
    input  logic [47:0] poly_rd_data,

    // Write Channel (For Decode/Decompress)
    output logic        poly_wr_req,
    output logic [5:0]  poly_wr_addr,
    output logic [47:0] poly_wr_data,

    // ==========================================
    // Seed Bank Interface
    // ==========================================
    // If seeds are packed alongside public keys (e.g., the 32-byte rho),
    // the transcoder might need to route them directly to/from the seed bank.
    output logic        seed_wr_req,
    output logic [3:0]  seed_wr_addr,
    output logic [63:0] seed_wr_data,

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
