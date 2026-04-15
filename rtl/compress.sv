/*
 * Module Name: compress
 * Author(s): Mai Komar
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
    localparam logic [11:0] BARRETT_M = 12'd2519; // floor(2^24 / 2q)
    localparam logic [13:0] TWO_Q     = 14'd6658; // 2 * q

    function automatic logic [COEFF_WIDTH-1:0] compress_one(
        input logic [COEFF_WIDTH-1:0] x,
        input logic [3:0]             d
    );
        logic [23:0] n;
        logic [35:0] product;
        logic [11:0] t;
        logic [23:0] r;
        logic [11:0] mask;
    begin
        if (d == 4'd12) begin
            compress_one = x;
        end else begin
            n       = (24'(x) << (d + 4'd1)) + 24'(Q); // rounding absorbed into numerator
            product = 36'(n) * 36'(BARRETT_M);
            t       = product[35:24];                    // Barrett quotient
            r       = n - 24'(t) * 24'(TWO_Q);
            if (r >= 24'(TWO_Q)) t = t + 12'd1;         // one correction step
            mask         = COEFF_WIDTH'((12'd1 << d) - 12'd1);
            compress_one = t & mask;
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
