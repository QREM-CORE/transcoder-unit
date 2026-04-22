/*
 * Module Name: tr_microcode_rom
 * Author(s):   Kiet Le
 * Description:
 * Purely combinational instruction decoder (Microcode ROM) for the ML-KEM Transcoder.
 * This module acts as the single source of truth for the ML-KEM protocol specifications,
 * decoupling the mathematical parameters from the FSM state traversal logic.
 *
 * Functionality:
 * Translates a high-level 5-bit artifact opcode and a 2-bit security level into a
 * wide, explicit control word. This control word dictates exactly how the Transcoder
 * datapath should route, slice, loop, and compress data for that specific instruction.
 *
 * Internal Structure:
 * 1. Security Level Resolution:
 * Dynamically decodes the FIPS 203 security level (ML-KEM-512, 768, 1024) to
 * establish the vector dimension (k) and ciphertext compression parameters (du, dv).
 *
 * 2. Opcode Decoder (The ROM):
 * A massive multiplexer that maps the requested opcode to specific execution flags:
 * - Math Config: Determines if the Packer/Unpacker runs, the base memory ID,
 * the active 'd' compression parameter, and whether to loop 'k' times (Vectors)
 * or execute once (Scalars).
 * - Bypass Config: Determines if a raw 32-byte seed transfer is required,
 * setting the target SeedBank ID and beat limits.
 * - Router Config: Selects the crossbar modes required for both the Math phase
 * and the Bypass phase (e.g., routing to AXI-TX AND Hash Snoop simultaneously).
 * - Internal Math Enable: Flags operations where the Math Engines communicate
 * directly with the internal SeedBank instead of the external AXI bus.
 *
 * Usage:
 * Instantiated exclusively within 'tr_fsm'. It is fully combinational and requires
 * the inputs (opcode_i, sec_level_i) to be latched and held stable by the parent FSM.
 */

`default_nettype none
`timescale 1ns / 1ps

import qrem_global_pkg::*;
import transcoder_pkg::*;

module tr_microcode_rom #(
    parameter int POLY_ID_W = 6,
    parameter int SEED_ID_W = 4
) (
    input  wire tr_opcode_t           opcode_i,
    input  wire logic [1:0]           sec_level_i,

    // Parameter Outputs
    output      logic [2:0]           k_limit_o, // Vector loop limit (2, 3, or 4)

    // Math Phase Configuration
    output      logic                 is_tx_o,
    output      logic                 math_en_o,
    output      logic                 math_k_loop_o,
    output      logic                 internal_math_en_o,
    output      logic [3:0]           d_param_o,
    output      logic [POLY_ID_W-1:0] poly_base_id_o,

    // Bypass Phase Configuration
    output      logic                 bypass_en_o,
    output      logic [SEED_ID_W-1:0] seed_id_o,
    output      logic [7:0]           bypass_beats_o,

    // Router Configuration
    output      router_sel_t          router_math_sel_o,
    output      router_sel_t          router_bypass_sel_o
);

    // ====================================================================
    // Memory & Seed Semantic Identifiers
    // ====================================================================
    localparam logic [POLY_ID_W-1:0] MEM_S  = 6'd0;
    localparam logic [POLY_ID_W-1:0] MEM_T  = 6'd4;
    localparam logic [POLY_ID_W-1:0] MEM_U  = 6'd8;
    localparam logic [POLY_ID_W-1:0] MEM_V  = 6'd12;
    localparam logic [POLY_ID_W-1:0] MEM_W  = 6'd16;
    localparam logic [POLY_ID_W-1:0] MEM_MU = 6'd20;

    localparam logic [SEED_ID_W-1:0] SEED_D     = 4'd0;
    localparam logic [SEED_ID_W-1:0] SEED_RHO   = 4'd1;
    localparam logic [SEED_ID_W-1:0] SEED_M     = 4'd2;
    localparam logic [SEED_ID_W-1:0] SEED_K     = 4'd3;
    localparam logic [SEED_ID_W-1:0] SEED_Z     = 4'd4;
    localparam logic [SEED_ID_W-1:0] SEED_K_BAR = 4'd5;
    localparam logic [SEED_ID_W-1:0] SEED_R     = 4'd6;
    localparam logic [SEED_ID_W-1:0] SEED_HEK   = 4'd7;

    // ====================================================================
    // Security Level Resolution
    // ====================================================================
    logic [3:0] cfg_du;
    logic [3:0] cfg_dv;

    always_comb begin
        case (sec_level_i)
            2'b00:   begin k_limit_o = 3'd2; cfg_du = 4'd10; cfg_dv = 4'd4; end // ML-KEM-512
            2'b01:   begin k_limit_o = 3'd3; cfg_du = 4'd10; cfg_dv = 4'd4; end // ML-KEM-768
            2'b10:   begin k_limit_o = 3'd4; cfg_du = 4'd11; cfg_dv = 4'd5; end // ML-KEM-1024
            default: begin k_limit_o = 3'd2; cfg_du = 4'd10; cfg_dv = 4'd4; end
        endcase
    end

    // ====================================================================
    // Opcode Decoder
    // ====================================================================
    always_comb begin
        // Safe Defaults
        is_tx_o             = 1'b0;
        math_en_o           = 1'b0;
        math_k_loop_o       = 1'b0;
        internal_math_en_o  = 1'b0;
        d_param_o           = 4'd12;
        poly_base_id_o      = '0;
        bypass_en_o         = 1'b0;
        seed_id_o           = '0;
        bypass_beats_o      = 8'd4; // 32 bytes = 4x 64-bit beats
        router_math_sel_o   = TR_ROUTER_IDLE;
        router_bypass_sel_o = TR_ROUTER_IDLE;

        case (opcode_i)
            TR_OP_IDLE: begin
                // Defaults hold. FSM is at rest.
            end

            // ==========================================
            // KEYGEN OPCODES
            // ==========================================
            TR_OP_KG_INGEST_D: begin
                // Ingesting a 32-byte seed 'd'. No math required.
                // We bypass straight from AXI-RX to the internal SeedBank.
                bypass_en_o         = 1'b1;
                seed_id_o           = SEED_D;
                router_bypass_sel_o = TR_ROUTER_BYPASS_RX;
            end
            TR_OP_KG_EXPORT_DK_PKE: begin
                // Exporting the decapsulation key (vector s).
                // Math required to encode 12-bit coeffs to bytes.
                // It's a vector, so we must loop 'k' times.
                math_en_o           = 1'b1;
                is_tx_o             = 1'b1;
                math_k_loop_o       = 1'b1;
                poly_base_id_o      = MEM_S;
                router_math_sel_o   = TR_ROUTER_MATH_TX;
            end
            TR_OP_KG_EXPORT_EK_PKE_1: begin
                // Exporting part 1 of the encapsulation key (vector t).
                // Same math/loop setup as 's', just pointing to MEM_T.
                math_en_o           = 1'b1;
                is_tx_o             = 1'b1;
                math_k_loop_o       = 1'b1;
                poly_base_id_o      = MEM_T;
                router_math_sel_o   = TR_ROUTER_MATH_TX;
            end
            TR_OP_KG_EXPORT_EK_PKE_2: begin
                // Exporting part 2 of the encapsulation key (seed rho).
                // Pure bypass from SeedBank out to Host.
                bypass_en_o         = 1'b1;
                seed_id_o           = SEED_RHO;
                router_bypass_sel_o = TR_ROUTER_BYPASS_TX;
            end
            TR_OP_KG_EXPORT_HEK: begin
                // Exporting the hash of the encapsulation key.
                // Pure bypass from SeedBank out to Host.
                bypass_en_o         = 1'b1;
                seed_id_o           = SEED_HEK;
                router_bypass_sel_o = TR_ROUTER_BYPASS_TX;
            end

            // ==========================================
            // ENCAP OPCODES
            // ==========================================
            TR_OP_EN_INGEST_M: begin
                // Ingesting the 32-byte message 'm' from Host.
                // Pure bypass from AXI-RX to SeedBank.
                bypass_en_o         = 1'b1;
                seed_id_o           = SEED_M;
                router_bypass_sel_o = TR_ROUTER_BYPASS_RX;
            end
            TR_OP_EN_INGEST_EK_1: begin
                // Ingesting the 't' vector of the Encapsulation Key.
                // Math required to decode bytes into 12-bit coeffs.
                // It's a vector, so loop 'k' times.
                math_en_o           = 1'b1;
                math_k_loop_o       = 1'b1;
                poly_base_id_o      = MEM_T;
                router_math_sel_o   = TR_ROUTER_MATH_RX;
            end
            TR_OP_EN_INGEST_EK_2: begin
                // Ingesting the 'rho' seed of the Encapsulation Key.
                // Pure bypass from AXI-RX to SeedBank.
                bypass_en_o         = 1'b1;
                seed_id_o           = SEED_RHO;
                router_bypass_sel_o = TR_ROUTER_BYPASS_RX;
            end
            TR_OP_EN_MSG_DEC: begin
                // OPTION B ARCHITECTURE: Internal Crossbar.
                // We decode 1-bit bytes into a polynomial (mu).
                // 'm' is already in the SeedBank, so we enable the internal crossbar
                // to route SeedBank -> Unpacker. It's a scalar (1 poly), so k_loop is 0.
                math_en_o           = 1'b1;
                internal_math_en_o  = 1'b1;
                math_k_loop_o       = 1'b0;
                d_param_o           = 4'd1;     // Decode_1 compression level
                poly_base_id_o      = MEM_MU;
                seed_id_o           = SEED_M;
                router_math_sel_o   = TR_ROUTER_MATH_RX_FROM_SEEDBANK;
            end
            TR_OP_EN_EXPORT_CT_1: begin
                // Exporting vector 'u' of the ciphertext.
                // Math required to compress and encode. Loop 'k' times.
                // Compression parameter depends on security level (cfg_du).
                math_en_o           = 1'b1;
                is_tx_o             = 1'b1;
                math_k_loop_o       = 1'b1;
                d_param_o           = cfg_du;
                poly_base_id_o      = MEM_U;
                router_math_sel_o   = TR_ROUTER_MATH_TX;
            end
            TR_OP_EN_EXPORT_CT_2: begin
                // Exporting scalar 'v' of the ciphertext.
                // Math required, but it's only 1 polynomial, so k_loop is 0.
                // Compression parameter depends on security level (cfg_dv).
                math_en_o           = 1'b1;
                is_tx_o             = 1'b1;
                math_k_loop_o       = 1'b0;
                d_param_o           = cfg_dv;
                poly_base_id_o      = MEM_V;
                router_math_sel_o   = TR_ROUTER_MATH_TX;
            end
            TR_OP_EN_EXPORT_K: begin
                // Exporting the derived shared secret 'K'.
                // Pure bypass from SeedBank out to Host.
                bypass_en_o         = 1'b1;
                seed_id_o           = SEED_K;
                router_bypass_sel_o = TR_ROUTER_BYPASS_TX;
            end

            // ==========================================
            // DECAP OPCODES
            // ==========================================
            TR_OP_DC_INGEST_DK_PKE: begin
                // Ingesting the Decapsulation Key (vector s).
                // Math required to decode bytes to 12-bit coeffs. Loop 'k' times.
                math_en_o           = 1'b1;
                math_k_loop_o       = 1'b1;
                poly_base_id_o      = MEM_S;
                router_math_sel_o   = TR_ROUTER_MATH_RX;
            end
            TR_OP_DC_INGEST_C1: begin
                // Ingesting ciphertext vector 'u'. Math required, k loops, dynamic d_u.
                // CRITICAL: The Hash core needs to snoop this incoming byte stream
                // to perform the Fujisaki-Okamoto re-encryption validation check.
                math_en_o           = 1'b1;
                math_k_loop_o       = 1'b1;
                d_param_o           = cfg_du;
                poly_base_id_o      = MEM_U;
                router_math_sel_o   = TR_ROUTER_MATH_RX_SNOOP; // Snooping enabled
            end
            TR_OP_DC_INGEST_C2: begin
                // Ingesting ciphertext scalar 'v'. Math required, k=0, dynamic d_v.
                // CRITICAL: Must also route to Hash Snoop for validation check.
                math_en_o           = 1'b1;
                math_k_loop_o       = 1'b0;
                d_param_o           = cfg_dv;
                poly_base_id_o      = MEM_V;
                router_math_sel_o   = TR_ROUTER_MATH_RX_SNOOP; // Snooping enabled
            end
            TR_OP_DC_INGEST_Z: begin
                // Ingesting the implicit rejection seed 'z'.
                // Pure bypass from AXI-RX to SeedBank.
                bypass_en_o         = 1'b1;
                seed_id_o           = SEED_Z;
                router_bypass_sel_o = TR_ROUTER_BYPASS_RX;
            end
            TR_OP_DC_MSG_ENC: begin
                // OPTION B ARCHITECTURE: Internal Crossbar.
                // We compress polynomial 'w' into 1-bit bytes to recover message 'm'.
                // We enable the internal crossbar to route Packer -> SeedBank.
                // It's a scalar (1 poly), so k_loop is 0.
                math_en_o           = 1'b1;
                is_tx_o             = 1'b1;
                internal_math_en_o  = 1'b1;
                math_k_loop_o       = 1'b0;
                d_param_o           = 4'd1;
                poly_base_id_o      = MEM_W;
                seed_id_o           = SEED_M;
                router_math_sel_o   = TR_ROUTER_MATH_TX_TO_SEEDBANK;
            end
            TR_OP_DC_EXPORT_K: begin
                // Exporting final shared secret 'K' (Successful Decap).
                bypass_en_o         = 1'b1;
                seed_id_o           = SEED_K;
                router_bypass_sel_o = TR_ROUTER_BYPASS_TX;
            end
            TR_OP_DC_EXPORT_K_BAR: begin
                // Exporting implicit rejection secret 'K-bar' (Failed Decap).
                bypass_en_o         = 1'b1;
                seed_id_o           = SEED_K_BAR;
                router_bypass_sel_o = TR_ROUTER_BYPASS_TX;
            end
            TR_OP_DC_EXPORT_R: begin
                // Exporting re-encryption seed 'r'.
                bypass_en_o         = 1'b1;
                seed_id_o           = SEED_R;
                router_bypass_sel_o = TR_ROUTER_BYPASS_TX;
            end
            default: begin end
        endcase
    end
endmodule

`default_nettype wire
