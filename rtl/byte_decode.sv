/*
 * Module Name: byte_decode
 * Author(s): Mai Komar
 * Description: Unpacks a byte array of length 32*d into 256 d-bit coefficients (FIPS 203 Algorithm 6).
 *
 * Optimization notes:
 *   - bytes_i is flattened to a 3072-bit vector via generate (pure wires, zero gates).
 *   - Each case branch uses a constant-width part-select [i*d +: d] which synthesis
 *     resolves to direct wire connections — no arithmetic logic generated.
 *   - The case mux collapses to nothing when d is constant at elaboration time.
 *   - Only the d=12 mod-q reduction requires actual gates (~3 GE per coefficient:
 *     comparator + subtractor + mux).
 */

`default_nettype none
import transcoder_pkg::*;

module byte_decode (
    input  logic [383:0][7:0]             bytes_i,
    input  logic [3:0]                    d_i,
    output logic [255:0][COEFF_WIDTH-1:0] coeff_o
);

    // Flatten packed byte array to a single bit stream.
    // bit k = bytes_i[k/8][k%8] — FIPS 203 little-endian bit order.
    // Pure wire rearrangement: zero gates.
    logic [3071:0] bits;

    genvar b;
    generate
        for (b = 0; b < 3072; b++) begin : g_flatten
            assign bits[b] = bytes_i[b >> 3][b & 3'h7];
        end
    endgenerate

    // Combinational decode. For every d value the part-select width is a
    // literal so synthesis maps each coeff_o[i] directly to input wires.
    logic [11:0] v12;

    always_comb begin
        coeff_o = '0;
        v12     = '0;

        case (d_i)
            4'd1: begin
                for (int i = 0; i < 256; i++)
                    coeff_o[i] = 12'(bits[i*1  +: 1]);
            end
            4'd4: begin
                for (int i = 0; i < 256; i++)
                    coeff_o[i] = 12'(bits[i*4  +: 4]);
            end
            4'd5: begin
                for (int i = 0; i < 256; i++)
                    coeff_o[i] = 12'(bits[i*5  +: 5]);
            end
            4'd10: begin
                for (int i = 0; i < 256; i++)
                    coeff_o[i] = 12'(bits[i*10 +: 10]);
            end
            4'd11: begin
                for (int i = 0; i < 256; i++)
                    coeff_o[i] = 12'(bits[i*11 +: 11]);
            end
            4'd12: begin
                for (int i = 0; i < 256; i++) begin
                    v12 = bits[i*12 +: 12];
                    coeff_o[i] = (v12 >= 12'(Q)) ? (v12 - 12'(Q)) : v12;
                end
            end
            default: coeff_o = '0;
        endcase
    end

endmodule

`default_nettype wire