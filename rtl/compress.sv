/*
 * Module Name: compress
 * Author(s):   Mai Komar, Kiet Le
 * Description:
 *   4-lane parallel, bit-exact sequential FIPS 203 compression.
 *   Hardware-optimized Barrett reduction (M=161271) implemented via
 *   a 2-stage pipelined CSD adder-tree to reduce logic depth.
 *
 * Architecture:
 *   - Stage 1: Partial Barrett product (CSD term-recycling tree).
 *   - Stage 2: Final shifting, rounding, and modulo-masking.
 */

`default_nettype none
import transcoder_pkg::*;

// Compress_d(x) = round(2^d * x / q) mod 2^d
// 4 lanes in parallel, 2-stage sequential pipeline.
// d=12 is a passthrough (no compression).
module compress (
    input  wire logic                           clk,
    input  wire logic                           rst,
    input  wire logic [3:0][COEFF_WIDTH-1:0]    coeff_i,
    input  wire logic [3:0]                     d_i,
    output      logic [3:0][COEFF_WIDTH-1:0]    coeff_o
);
    // Barrett Exact approach with Hand-Optimized CSD Tree
    // M = 161271
    // M = 2^17 + 2^15 - 2^11 - 2^9 - 2^3 - 2^0
    //   = 5*(2^15) - 5*(2^9) - 9*(2^0)

    // ====================================================================
    // Stage 1: Pipeline Internal Registers & Arithmetic
    // ====================================================================
    logic [3:0][COEFF_WIDTH-1:0] x1_reg;
    logic [3:0][28:0]            xM1_reg;
    logic [3:0]                  d1_reg;

    // Optimize bit-widths for 3328 max input
    // x*5: 3328 * 5 = 16640 (15 bits)
    // x*9: 3328 * 9 = 29952 (15 bits)
    logic [3:0][14:0] x_mul_5, x_mul_9;
    logic [3:0][23:0] sub1;
    logic [3:0][28:0] xM_comb;

    // Stage 1 Combinational: Calculate Barrett product components
    always_comb begin
        for (int i = 0; i < 4; i++) begin
            x_mul_5[i] = {coeff_i[i], 2'b0} + coeff_i[i];
            x_mul_9[i] = {coeff_i[i], 3'b0} + coeff_i[i];
            sub1[i]    = {x_mul_5[i], 6'b0} - x_mul_5[i];
            xM_comb[i] = {sub1[i], 9'b0} - x_mul_9[i];
        end
    end

    // Sequential Boundary: Cut point for the 11-level path
    always_ff @(posedge clk) begin
        if (rst) begin
            x1_reg   <= '0;
            xM1_reg  <= '0;
            d1_reg   <= '0;
        end else begin
            x1_reg   <= coeff_i;
            xM1_reg  <= xM_comb;
            d1_reg   <= d_i;
        end
    end

    // ====================================================================
    // Stage 2: Final Shifting, Rounding, and Masking
    // ====================================================================
    logic [3:0][39:0] shifted_xM;
    logic [3:0][40:0] shifted_sum;
    logic [11:0]      mask;

    always_comb begin
        // Prevent latches by providing defaults
        shifted_xM  = '0;
        shifted_sum = '0;

        // Determine mask based on registered d
        mask = '0;
        case (d1_reg)
            4'd4:  mask = 12'h00F;
            4'd5:  mask = 12'h01F;
            4'd10: mask = 12'h3FF;
            4'd11: mask = 12'h7FF;
            4'd12: mask = 12'hFFF;
            default: mask = '0;
        endcase

        for (int i = 0; i < 4; i++) begin
            if (d1_reg == 4'd12) begin
                coeff_o[i] = x1_reg[i];
            end else if (d1_reg == 4'd1) begin
                coeff_o[i] = ((x1_reg[i] >= 12'd833) && (x1_reg[i] <= 12'd2496)) ? 12'd1 : 12'd0;
            end else begin
                // Case-based shift to avoid variable shifter penalty
                case (d1_reg)
                    4'd4:  shifted_xM[i] = {7'b0, xM1_reg[i], 4'b0};
                    4'd5:  shifted_xM[i] = {6'b0, xM1_reg[i], 5'b0};
                    4'd10: shifted_xM[i] = {1'b0, xM1_reg[i], 10'b0};
                    4'd11: shifted_xM[i] = {      xM1_reg[i], 11'b0};
                    default: shifted_xM[i] = '0;
                endcase

                shifted_sum[i] = 40'(shifted_xM[i]) + 41'd268354944;
                coeff_o[i]     = shifted_sum[i][40:29] & mask;
            end
        end
    end

endmodule

`default_nettype wire
