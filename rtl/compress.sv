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
    // Barrett Exact approach with Hand-Optimized CSD Tree
    // M = 161271
    // M = 2^17 + 2^15 - 2^11 - 2^9 - 2^3 - 2^0
    //   = 5*(2^15) - 5*(2^9) - 9*(2^0)

    function automatic logic [COEFF_WIDTH-1:0] compress_one(
        input logic [COEFF_WIDTH-1:0] x,
        input logic [3:0]             d
    );
        // Optimize bit-widths for 3328 max input
        // x*5: 3328 * 5 = 16640 (15 bits)
        // x*9: 3328 * 9 = 29952 (15 bits)
        logic [14:0] x_mul_5;
        logic [14:0] x_mul_9;

        logic [23:0] sub1;
        logic [28:0] xM;
        logic [39:0] shifted_xM;
        logic [40:0] shifted_sum;
        logic [11:0] mask;
    begin
        if (d == 4'd12) begin
            compress_one = x;
        end else if (d == 4'd1) begin
            compress_one = ((x >= 12'd833) && (x <= 12'd2496)) ? 12'd1 : 12'd0;
        end else begin
            x_mul_5 = {x, 2'b0} + x;
            x_mul_9 = {x, 3'b0} + x;

            // t1 = x_mul_5 << 15
            // t2 = x_mul_5 << 9
            // sub1 = (x_mul_5 << 6) - x_mul_5 (shifted by 9)
            sub1 = {x_mul_5, 6'b0} - x_mul_5;

            // xM = (sub1 << 9) - x_mul_9
            xM = {sub1, 9'b0} - x_mul_9;

            case (d)
                4'd4:  begin shifted_xM = {7'b0, xM, 4'b0};   mask = 12'h00F; end
                4'd5:  begin shifted_xM = {6'b0, xM, 5'b0};   mask = 12'h01F; end
                4'd10: begin shifted_xM = {1'b0, xM, 10'b0};  mask = 12'h3FF; end
                4'd11: begin shifted_xM = {      xM, 11'b0};  mask = 12'h7FF; end
                default: begin shifted_xM = '0;               mask = '0; end
            endcase

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
