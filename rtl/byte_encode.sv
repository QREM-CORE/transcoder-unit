/*
 * Module Name: byte_encode
 * Author(s): Mai Komar
 * Description: Packs 256 d-bit coefficients into a byte array of length 32*d (FIPS 203 Algorithm 5).
 *
 * Optimization notes:
 *   - coeff_i bits are packed into a flat 3072-bit wire via always_comb using
 *     constant-width part-selects — no runtime index arithmetic, resolves to wires.
 *   - bytes_o is wired directly from that flat vector via generate (zero gates).
 *   - The case mux on d_i collapses entirely when d is constant at elaboration time.
 *   - No arithmetic logic at all — this module is pure wiring in every supported d mode.
 */

`default_nettype none
import transcoder_pkg::*;

module byte_encode (
    input  logic [255:0][COEFF_WIDTH-1:0] coeff_i,
    input  logic [3:0]                    d_i,
    output logic [383:0][7:0]             bytes_o
);

    // Flat bit stream: bit k = the k-th coefficient bit in FIPS 203 LSB-first order.
    // Driven by always_comb below; bytes_o is wired from this via generate.
    logic [3071:0] bits;

    // Pack coeff_i into the flat stream.
    // Each case branch uses a constant part-select width — pure wire assignment.
    always_comb begin
        bits = '0;
        case (d_i)
            4'd1:  for (int i = 0; i < 256; i++) bits[i*1  +: 1]  = coeff_i[i][0:0];
            4'd4:  for (int i = 0; i < 256; i++) bits[i*4  +: 4]  = coeff_i[i][3:0];
            4'd5:  for (int i = 0; i < 256; i++) bits[i*5  +: 5]  = coeff_i[i][4:0];
            4'd10: for (int i = 0; i < 256; i++) bits[i*10 +: 10] = coeff_i[i][9:0];
            4'd11: for (int i = 0; i < 256; i++) bits[i*11 +: 11] = coeff_i[i][10:0];
            4'd12: for (int i = 0; i < 256; i++) bits[i*12 +: 12] = coeff_i[i][11:0];
            default: bits = '0;
        endcase
    end

    // Wire flat bit stream into packed byte output.
    // bytes_o[k/8][k%8] = bits[k] — all pure wires, zero gates.
    genvar b;
    generate
        for (b = 0; b < 3072; b++) begin : g_pack
            assign bytes_o[b >> 3][b & 3'h7] = bits[b];
        end
    endgenerate

endmodule

`default_nettype wire