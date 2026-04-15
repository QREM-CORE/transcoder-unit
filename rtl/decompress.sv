/*
 * Module Name: decompress
 * Author(s): Mai Komar, Kiet Le
 * Description:
 *   4-lane parallel, bit-exact combinational FIPS 203 decompression.
 *   Implements Decompress_d(y) = round(q * y / 2^d).
 *   Optimized for area and timing using statically-sized, case-based
 *   shift-and-add operations. Includes a dedicated combinational fast path
 *   for the d=1 edge case, and sizes intermediate math paths exactly to
 *   their highly constrained, per-case theoretical maximum bit-widths.
 */

`default_nettype none
import transcoder_pkg::*;

// Decompress_d(y) = round(q * y / 2^d)
// 4 lanes in parallel, purely combinational.
// d=12 is a passthrough (no decompression).
module decompress (
    input  wire logic [3:0][COEFF_WIDTH-1:0] coeff_i,
    input  wire logic [3:0]                  d_i,
    output wire logic [3:0][COEFF_WIDTH-1:0] coeff_o
);
    function automatic logic [COEFF_WIDTH-1:0] decompress_one(
        input logic [COEFF_WIDTH-1:0] y,
        input logic [3:0]             d
    );
        logic [15:0] qy_4;
        logic [16:0] qy_5;
        logic [21:0] qy_10;
        logic [22:0] qy_11;
    begin
        if (d == 4'd12) begin
            decompress_one = y;
        end else if (d == 4'd1) begin
            decompress_one = y[0] ? 12'd1665 : 12'd0;
        end else begin
            case (d)
                4'd4: begin
                    qy_4 = (16'(y[3:0]) << 11) + (16'(y[3:0]) << 10) + (16'(y[3:0]) << 8) + 16'(y[3:0]);
                    decompress_one = 12'((qy_4 + 16'd8) >> 4);
                end
                4'd5: begin
                    qy_5 = (17'(y[4:0]) << 11) + (17'(y[4:0]) << 10) + (17'(y[4:0]) << 8) + 17'(y[4:0]);
                    decompress_one = 12'((qy_5 + 17'd16) >> 5);
                end
                4'd10: begin
                    qy_10 = (22'(y[9:0]) << 11) + (22'(y[9:0]) << 10) + (22'(y[9:0]) << 8) + 22'(y[9:0]);
                    decompress_one = 12'((qy_10 + 22'd512) >> 10);
                end
                4'd11: begin
                    qy_11 = (23'(y[10:0]) << 11) + (23'(y[10:0]) << 10) + (23'(y[10:0]) << 8) + 23'(y[10:0]);
                    decompress_one = 12'((qy_11 + 23'd1024) >> 11);
                end
                default: decompress_one = '0;
            endcase
        end
    end
    endfunction

    genvar i;
    generate
        for (i = 0; i < 4; i++) begin : g_decompress
            assign coeff_o[i] = decompress_one(coeff_i[i], d_i);
        end
    endgenerate

endmodule

`default_nettype wire
