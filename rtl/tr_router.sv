/*
 * Module Name: tr_router
 * Author(s):   Kiet Le
 * Description:
 * Purely combinational datapath crossbar for the ML-KEM Transcoder.
 * This module isolates the variable-stride math engines (Packer/Unpacker)
 * from the external 64-bit communication buses. It strictly manages
 * data routing and AXI-Stream handshaking (valid/ready backpressure)
 * based on the active operational mode dictated by the Transcoder FSM.
 *
 * Architectural Role:
 * Acts as the unified "Traffic Cop" of the Decoupled Dual-Datapath architecture.
 * It routes 64-bit streams between four distinct domains:
 * 1. The Host (AXI4-Stream RX/TX)
 * 2. The Internal Math Pipelines (Packer/Unpacker)
 * 3. The Keccak Hash Core (Hash Snoop interface)
 * 4. The Unified Memory Subsystem (Seed/Protocol Store)
 *
 * Backpressure Handling:
 * When broadcasting a single source to multiple destinations (e.g., routing
 * an Encapsulation Key to both AXI-TX and the Hash Snoop simultaneously),
 * this module enforces strict backpressure. It gates the source's 'ready'
 * signal with a logical AND of all target destinations, ensuring zero data loss.
 *
 * Routing Modes (router_sel_i):
 * +------+-----------------------+-------------+-----------------------------+
 * | Mode | Name                  | Data Source | Data Destination(s)         |
 * +------+-----------------------+-------------+-----------------------------+
 * | 0000 | IDLE                  | None        | None (Zeroed to save power) |
 * | 0001 | MATH_TX               | Packer      | AXI-TX                      |
 * | 0010 | MATH_TX_SNOOP         | Packer      | AXI-TX & Hash Snoop         |
 * | 0011 | MATH_RX               | AXI-RX      | Unpacker                    |
 * | 0100 | MATH_RX_SNOOP         | AXI-RX      | Unpacker & Hash Snoop       |
 * | 0101 | RAW_BYPASS_TX         | SeedBank    | AXI-TX                      |
 * | 0110 | RAW_BYPASS_RX         | AXI-RX      | SeedBank                    |
 * | 0111 | MATH_RX_FROM_SEEDBANK | SeedBank    | Unpacker (Internal)         |
 * | 1000 | MATH_TX_TO_SEEDBANK   | Packer      | SeedBank (Internal)         |
 * +------+-----------------------+-------------+-----------------------------+
 */

`default_nettype none
`timescale 1ns / 1ps

import qrem_global_pkg::*;
import transcoder_pkg::*;

module tr_router (
    // Control
    input  wire router_sel_t               router_sel_i,
    input  wire logic                      router_tlast_i,

    // AXI4-Stream RX (Host -> Transcoder)
    input  wire logic [63:0]               s_axis_tdata_i,
    input  wire logic                      s_axis_tvalid_i,
    output      logic                      s_axis_tready_o,

    // AXI4-Stream TX (Transcoder -> Host)
    output      logic [63:0]               m_axis_tdata_o,
    output      logic                      m_axis_tvalid_o,
    input  wire logic                      m_axis_tready_i,
    output      logic [7:0]                m_axis_tkeep_o,
    output      logic                      m_axis_tlast_o,

    // Math Engine: Unpacker (RX)
    output      logic [63:0]               unpacker_tdata_o,
    output      logic                      unpacker_tvalid_o,
    input  wire logic                      unpacker_tready_i,

    // Math Engine: Packer (TX)
    input  wire logic [63:0]               packer_tdata_i,
    input  wire logic                      packer_tvalid_i,
    output      logic                      packer_tready_o,

    // Keccak Hash Snoop
    output      logic [63:0]               hash_snoop_data_o,
    output      logic [7:0]                hash_snoop_keep_o,
    output      logic                      hash_snoop_valid_o,
    input  wire logic                      hash_snoop_ready_i,
    output      logic                      hash_snoop_last_o,

    // SeedBank (Bypass & Internal Crossbar)
    input  wire logic [63:0]               seed_rdata_i,
    input  wire logic                      seed_rvalid_i,
    output      logic [63:0]               seed_wdata_o,
    input  wire logic                      seed_ready_i
);

    // ====================================================================
    // Static Assignments
    // ====================================================================
    // FIPS 203 artifacts (EK, DK, CT, Messages) are all strictly
    // byte-aligned and perfectly divisible by 8 bytes (64 bits).
    // Therefore, tkeep is permanently tied high.
    assign m_axis_tkeep_o    = 8'hFF;
    assign hash_snoop_keep_o = 8'hFF;

    always_comb begin
        // Default Assignments (IDLE / Safe State)
        s_axis_tready_o    = 1'b0;

        m_axis_tdata_o     = '0;
        m_axis_tvalid_o    = 1'b0;
        m_axis_tlast_o     = 1'b0;

        unpacker_tdata_o   = '0;
        unpacker_tvalid_o  = 1'b0;

        packer_tready_o    = 1'b0;

        hash_snoop_data_o  = '0;
        hash_snoop_valid_o = 1'b0;
        hash_snoop_last_o  = 1'b0;

        seed_wdata_o       = '0;

        case (router_sel_i)
            TR_ROUTER_IDLE: begin
                // Defaults hold
            end

            TR_ROUTER_MATH_TX: begin // Packer -> AXI TX
                m_axis_tdata_o  = packer_tdata_i;
                m_axis_tvalid_o = packer_tvalid_i;
                m_axis_tlast_o  = router_tlast_i;
                packer_tready_o = m_axis_tready_i;
            end

            TR_ROUTER_MATH_TX_SNOOP: begin // Packer -> AXI TX & Hash Snoop
                m_axis_tdata_o     = packer_tdata_i;
                m_axis_tvalid_o    = packer_tvalid_i;
                m_axis_tlast_o     = router_tlast_i;

                hash_snoop_data_o  = packer_tdata_i;
                hash_snoop_valid_o = packer_tvalid_i;
                hash_snoop_last_o  = router_tlast_i;

                packer_tready_o    = m_axis_tready_i & hash_snoop_ready_i;
            end

            TR_ROUTER_MATH_RX: begin // AXI RX -> Unpacker
                unpacker_tdata_o  = s_axis_tdata_i;
                unpacker_tvalid_o = s_axis_tvalid_i;
                s_axis_tready_o   = unpacker_tready_i;
            end

            TR_ROUTER_MATH_RX_SNOOP: begin // AXI RX -> Unpacker & Hash Snoop
                unpacker_tdata_o   = s_axis_tdata_i;
                unpacker_tvalid_o  = s_axis_tvalid_i;

                hash_snoop_data_o  = s_axis_tdata_i;
                hash_snoop_valid_o = s_axis_tvalid_i;
                hash_snoop_last_o  = router_tlast_i;

                // Backpressure: Only ready if BOTH receivers are ready
                s_axis_tready_o    = unpacker_tready_i & hash_snoop_ready_i;
            end

            TR_ROUTER_BYPASS_TX: begin // SeedBank -> AXI TX (Bypass)
                m_axis_tdata_o  = seed_rdata_i;
                // In bypass mode, the FSM is responsible for incrementing the seed
                // address and ensuring data is valid. We route the rvalid directly.
                m_axis_tvalid_o = seed_rvalid_i;
                m_axis_tlast_o  = router_tlast_i;
            end

            TR_ROUTER_BYPASS_RX: begin // AXI RX -> SeedBank (Bypass)
                seed_wdata_o    = s_axis_tdata_i;
                s_axis_tready_o = seed_ready_i;
            end

            TR_ROUTER_MATH_RX_FROM_SEEDBANK: begin // SeedBank -> Unpacker (Internal Crossbar)
                unpacker_tdata_o  = seed_rdata_i;
                unpacker_tvalid_o = seed_rvalid_i;
            end

            TR_ROUTER_MATH_TX_TO_SEEDBANK: begin // Packer -> SeedBank (Internal Crossbar)
                seed_wdata_o    = packer_tdata_i;
                packer_tready_o = seed_ready_i;
            end

            default: begin
                // Defaults hold
            end
        endcase
    end

endmodule

`default_nettype wire
