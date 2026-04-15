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
    input  logic [3:0][COEFF_WIDTH-1:0] coeff_i,
    input  logic [3:0]                  d_i,
    output logic [3:0][COEFF_WIDTH-1:0] coeff_o
);
    // Option C: Specialized datapath per ML-KEM 'd' value
    // Barrett Magic M = 10080, K = 26
    localparam logic [13:0] M_WIDE = 14'd10080;

    function automatic logic [COEFF_WIDTH-1:0] compress_one(
        input logic [COEFF_WIDTH-1:0] x,
        input logic [3:0]             d
    );
        logic [23:0] n;
        logic [37:0] product;
        logic [11:0] mask;
    begin
        if (d == 4'd12) begin
            compress_one = x;
        end else begin
            case (d)
                // ML-KEM valid d parameters (FIPS 203)
                4'd1:  begin n = {10'b0, x, 2'b0} + 24'(Q);  mask = 12'h001; end
                4'd4:  begin n = {7'b0, x, 5'b0} + 24'(Q);   mask = 12'h00F; end
                4'd5:  begin n = {6'b0, x, 6'b0} + 24'(Q);   mask = 12'h01F; end
                4'd10: begin n = {1'b0, x, 11'b0} + 24'(Q);  mask = 12'h3FF; end
                4'd11: begin n = {x, 12'b0} + 24'(Q);        mask = 12'h7FF; end
                default: begin n = '0; mask = '0; end
            endcase

            // Single shared massive multiplier inferred, shifted by K=26
            product = 38'(n) * 38'(M_WIDE);

            compress_one = COEFF_WIDTH'(product[37:26]) & mask;
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
