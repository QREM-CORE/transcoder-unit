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
    // Option C: Specialized datapath per ML-KEM 'd' value
    // Barrett Magic M = 10080, K = 26
    localparam logic [13:0] M_WIDE = 14'd10080;

    function automatic logic [COEFF_WIDTH-1:0] compress_d4(input logic [COEFF_WIDTH-1:0] x);
        logic [16:0] n;
        logic [30:0] product;
    begin
        n = {x, 5'b0} + 17'(Q);
        product = 31'(n) * 31'(M_WIDE);
        compress_d4 = COEFF_WIDTH'(product[29:26]);
    end
    endfunction

    function automatic logic [COEFF_WIDTH-1:0] compress_d5(input logic [COEFF_WIDTH-1:0] x);
        logic [17:0] n;
        logic [31:0] product;
    begin
        n = {x, 6'b0} + 18'(Q);
        product = 32'(n) * 32'(M_WIDE);
        compress_d5 = COEFF_WIDTH'(product[30:26]);
    end
    endfunction

    function automatic logic [COEFF_WIDTH-1:0] compress_d10(input logic [COEFF_WIDTH-1:0] x);
        logic [22:0] n;
        logic [36:0] product;
    begin
        n = {x, 11'b0} + 23'(Q);
        product = 37'(n) * 37'(M_WIDE);
        compress_d10 = COEFF_WIDTH'(product[35:26]);
    end
    endfunction

    function automatic logic [COEFF_WIDTH-1:0] compress_d11(input logic [COEFF_WIDTH-1:0] x);
        logic [23:0] n;
        logic [37:0] product;
    begin
        n = {x, 12'b0} + 24'(Q);
        product = 38'(n) * 38'(M_WIDE);
        compress_d11 = COEFF_WIDTH'(product[36:26]);
    end
    endfunction

    genvar i;
    generate
        for (i = 0; i < 4; i++) begin : g_compress
            always_comb begin
                case (d_i)
                    4'd4:    coeff_o[i] = compress_d4(coeff_i[i]);
                    4'd5:    coeff_o[i] = compress_d5(coeff_i[i]);
                    4'd10:   coeff_o[i] = compress_d10(coeff_i[i]);
                    4'd11:   coeff_o[i] = compress_d11(coeff_i[i]);
                    4'd12:   coeff_o[i] = coeff_i[i];
                    default: coeff_o[i] = '0;
                endcase
            end
        end
    endgenerate

endmodule

`default_nettype wire
