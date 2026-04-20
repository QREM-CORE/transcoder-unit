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
 * +------+----------------+------------------+-----------------------------+
 * | Mode | Name           | Data Source      | Data Destination(s)         |
 * +------+----------------+------------------+-----------------------------+
 * | 000  | IDLE           | None             | None (Zeroed to save power) |
 * | 001  | MATH_TX        | Packer           | AXI-TX                      |
 * | 010  | MATH_TX_SNOOP  | Packer           | AXI-TX AND Hash Snoop       |
 * | 011  | MATH_RX        | AXI-RX           | Unpacker                    |
 * | 100  | MATH_RX_SNOOP  | AXI-RX           | Unpacker AND Hash Snoop     |
 * | 101  | RAW_BYPASS_TX  | SeedBank Read    | AXI-TX                      |
 * | 110  | RAW_BYPASS_RX  | AXI-RX           | SeedBank Write              |
 * +------+----------------+------------------+-----------------------------+
 *
 * Mode Utilization Details:
 * - MATH_TX/RX: Active during Polynomial mathematical operations (Compress,
 * Decompress, ByteEncode, ByteDecode) for Keys, Ciphertexts, and Messages.
 * - SNOOP Modes: Active when FIPS 203 requires an artifact (e.g., Public Key)
 * to be exported to the host AND hashed by Keccak at the exact same time.
 * - RAW_BYPASS: Bypasses math pipelines entirely. Active when streaming
 * raw 32-byte seeds (rho, z, shared secret K) to/from the SeedBank.
 */

`default_nettype none
`timescale 1ns / 1ps

module tr_router #(
    parameter int SEED_W = 64
) (
    // Control from FSM
    input  wire logic [2:0]                router_sel_i,
    input  wire logic                      router_tlast_i,

    // Top-Level External AXI-Stream I/O
    input  wire logic [63:0]               s_axis_tdata_i,
    input  wire logic                      s_axis_tvalid_i,
    output      logic                      s_axis_tready_o,

    output      logic [63:0]               m_axis_tdata_o,
    output      logic                      m_axis_tvalid_o,
    input  wire logic                      m_axis_tready_i,
    output      logic [7:0]                m_axis_tkeep_o,
    output      logic                      m_axis_tlast_o,

    // Top-Level Hash Snoop I/O
    output      logic [63:0]               hash_snoop_data_o,
    output      logic [7:0]                hash_snoop_keep_o,
    output      logic                      hash_snoop_valid_o,
    input  wire logic                      hash_snoop_ready_i,
    output      logic                      hash_snoop_last_o,

    // Top-Level Seed Protocol Store (Data Only)
    input  wire logic [SEED_W-1:0]         seed_rdata_i,
    output      logic [SEED_W-1:0]         seed_wdata_o,

    // Internal Stream to/from Packer
    input  wire logic [63:0]               packer_tdata_i,
    input  wire logic                      packer_tvalid_i,
    output      logic                      packer_tready_o,

    // Internal Stream to/from Unpacker
    output      logic [63:0]               unpacker_tdata_o,
    output      logic                      unpacker_tvalid_o,
    input  wire logic                      unpacker_tready_i
);

    // ====================================================================
    // Static Assignments
    // ====================================================================
    // FIPS 203 artifacts (EK, DK, CT, Messages) are all strictly
    // byte-aligned and perfectly divisible by 8 bytes (64 bits).
    // Therefore, tkeep is permanently tied high.
    assign m_axis_tkeep_o    = 8'hFF;
    assign hash_snoop_keep_o = 8'hFF;

    // ====================================================================
    // Combinational Routing Matrix
    // ====================================================================
    always_comb begin
        // 1. Default to Idle/Zero states to prevent latches and bus collisions
        s_axis_tready_o    = 1'b0;

        m_axis_tdata_o     = '0;
        m_axis_tvalid_o    = 1'b0;
        m_axis_tlast_o     = 1'b0;

        hash_snoop_data_o  = '0;
        hash_snoop_valid_o = 1'b0;
        hash_snoop_last_o  = 1'b0;

        seed_wdata_o       = '0;

        packer_tready_o    = 1'b0;

        unpacker_tdata_o   = '0;
        unpacker_tvalid_o  = 1'b0;

        // 2. Crossbar Select
        unique case (router_sel_i)
            3'b001: begin // Packer -> AXI TX
                m_axis_tdata_o  = packer_tdata_i;
                m_axis_tvalid_o = packer_tvalid_i;
                m_axis_tlast_o  = router_tlast_i;
                packer_tready_o = m_axis_tready_i;
            end

            3'b010: begin // Packer -> AXI TX & Hash Snoop
                m_axis_tdata_o     = packer_tdata_i;
                m_axis_tvalid_o    = packer_tvalid_i;
                m_axis_tlast_o     = router_tlast_i;

                hash_snoop_data_o  = packer_tdata_i;
                hash_snoop_valid_o = packer_tvalid_i;
                hash_snoop_last_o  = router_tlast_i;

                // Backpressure: Only ready if BOTH receivers are ready
                packer_tready_o    = m_axis_tready_i & hash_snoop_ready_i;
            end

            3'b011: begin // AXI RX -> Unpacker
                unpacker_tdata_o  = s_axis_tdata_i;
                unpacker_tvalid_o = s_axis_tvalid_i;
                s_axis_tready_o   = unpacker_tready_i;
            end

            3'b100: begin // AXI RX -> Unpacker & Hash Snoop
                unpacker_tdata_o   = s_axis_tdata_i;
                unpacker_tvalid_o  = s_axis_tvalid_i;

                hash_snoop_data_o  = s_axis_tdata_i;
                hash_snoop_valid_o = s_axis_tvalid_i;
                hash_snoop_last_o  = router_tlast_i;

                // Backpressure: Only ready if BOTH receivers are ready
                s_axis_tready_o    = unpacker_tready_i & hash_snoop_ready_i;
            end

            3'b101: begin // SeedBank -> AXI TX (Bypass)
                m_axis_tdata_o  = seed_rdata_i;
                // In bypass mode, the FSM is responsible for incrementing the seed
                // address and ensuring data is valid. FSM must stall its internal
                // counters if m_axis_tready_i goes low.
                m_axis_tvalid_o = 1'b1;
                m_axis_tlast_o  = router_tlast_i;
            end

            3'b110: begin // AXI RX -> SeedBank (Bypass)
                seed_wdata_o    = s_axis_tdata_i;
                // Assuming Seed SRAM is always ready to write. The FSM acts on
                // s_axis_tvalid_i to pulse the actual seed_we_o signal.
                s_axis_tready_o = 1'b1;
            end
        endcase
    end

endmodule

`default_nettype wire
