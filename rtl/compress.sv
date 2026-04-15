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
    // Hybrid bits-extraction approach (Inha algorithm, smaller K)
    // Compress_d: t = m * x (m = 2580335)
    //             y = (t >> (33 - d)) + t[32 - d]
    // Uses 12-bit x 22-bit constant multiply. Narrower than Inha m=10321340.

    function automatic logic [COEFF_WIDTH-1:0] compress_one(
        input logic [COEFF_WIDTH-1:0] x,
        input logic [3:0]             d
    );
        logic [32:0] t;
        logic [11:0] shifted_t;
        logic        round_bit;
        logic [11:0] mask;
    begin
        if (d == 4'd12) begin
            compress_one = x;
        end else if (d == 4'd1) begin
            // d=1 bypass using comparators (Compress_1(x) = 1 iff 833 <= x <= 2496)
            compress_one = ((x >= 12'd833) && (x <= 12'd2496)) ? 12'd1 : 12'd0;
        end else begin
            t = 33'(x) * 33'd2580335; // 12-bit x 22-bit constant multiply

            // Extract bits [32 : 33-d] and round_bit t[32-d]
            case (d)
                4'd4:  begin shifted_t = 12'(t[32:29]); round_bit = t[28]; mask = 12'h00F; end
                4'd5:  begin shifted_t = 12'(t[32:28]); round_bit = t[27]; mask = 12'h01F; end
                4'd10: begin shifted_t = 12'(t[32:23]); round_bit = t[22]; mask = 12'h3FF; end
                4'd11: begin shifted_t = 12'(t[32:22]); round_bit = t[21]; mask = 12'h7FF; end
                default: begin shifted_t = '0;          round_bit = 1'b0;  mask = '0; end
            endcase

            // Add single-bit round and apply mask
            compress_one = (shifted_t + 12'(round_bit)) & mask;
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
