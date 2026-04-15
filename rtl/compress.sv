/*
 * Module Name: compress
 * Author(s): Mai Komar, Kiet Le
 * Description: 4-lane parallel combinational Compress_d implementation (FIPS 203).
 */

`default_nettype none
import transcoder_pkg::*;

// Compress_d(x) = round(2^d * x / q) mod 2^d
// 4 lanes in parallel, purely combinational.
// d=12 is a passthrough (no compression).
module compress (
    input  wire logic [3:0][COEFF_WIDTH-1:0] coeff_i,
    input  wire logic [3:0]                  d_i,
    output wire logic [3:0][COEFF_WIDTH-1:0] coeff_o
);
    // Mathematically Exact MUX-then-add datapath
    // Barrett Magic M = 161271, K = 29. Constant QHALF*M = 268354944.
    // Multiplier is 12-bit x 18-bit constant.

    function automatic logic [COEFF_WIDTH-1:0] compress_one(
        input logic [COEFF_WIDTH-1:0] x,
        input logic [3:0]             d
    );
        logic [28:0] xM;
        logic [39:0] shifted_xM;
        logic [40:0] shifted_sum;
        logic [11:0] mask;
    begin
        if (d == 4'd12) begin
            compress_one = x;
        end else if (d == 4'd1) begin
            // d=1 bypass using comparators (Compress_1(x) = 1 iff 833 <= x <= 2496)
            compress_one = ((x >= 12'd833) && (x <= 12'd2496)) ? 12'd1 : 12'd0;
        end else begin
            // Mathematical exactness without correction requires calculating ((x << d) + 1664) * M >> K
            // By multiplying `x` first: (x*M << d) + 1664*M >> K.
            // Barrett Magic M = 161271, K = 29. Constant QHALF*M = 1664 * 161271 = 268354944.
            xM = 29'(x) * 29'd161271; // 12-bit x 18-bit constant multiply

            case (d)
                // d logic: max shift 11. xM is 29 bit. + 11 = 40 bit.
                4'd4:  begin shifted_xM = {7'b0, xM, 4'b0};   mask = 12'h00F; end
                4'd5:  begin shifted_xM = {6'b0, xM, 5'b0};   mask = 12'h01F; end
                4'd10: begin shifted_xM = {1'b0, xM, 10'b0};  mask = 12'h3FF; end
                4'd11: begin shifted_xM = {      xM, 11'b0};  mask = 12'h7FF; end
                default: begin shifted_xM = '0;               mask = '0; end
            endcase

            // Add QHALF*M
            shifted_sum = 41'(shifted_xM) + 41'd268354944;

            compress_one = shifted_sum[40:29] & mask;
        end
    end
    endfunction

    genvar i;
    generate
        for (i = 0; i < 4; i++) begin : g_compress
            assign coeff_o[i] = compress_one(coeff_i[i], d_i);
        end
    endgenerate

endmodule

`default_nettype wire
