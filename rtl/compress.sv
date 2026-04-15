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
    // Inha University Algorithm 7 approach
    // Compress_d: t = m * x (m = 10321340)
    //             y = (t >> (35 - d)) + t[34 - d]
    // Uses 12-bit x 24-bit constant multiply, saving wide MUX and adder overhead.

    function automatic logic [COEFF_WIDTH-1:0] compress_one(
        input logic [COEFF_WIDTH-1:0] x,
        input logic [3:0]             d
    );
        logic [34:0] t;
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
            t = 35'(x) * 35'd10321340; // 12-bit x 24-bit constant multiply

            // Extract bits [34 : 35-d] and round_bit t[34-d]
            case (d)
                4'd4:  begin shifted_t = 12'(t[34:31]); round_bit = t[30]; mask = 12'h00F; end
                4'd5:  begin shifted_t = 12'(t[34:30]); round_bit = t[29]; mask = 12'h01F; end
                4'd10: begin shifted_t = 12'(t[34:25]); round_bit = t[24]; mask = 12'h3FF; end
                4'd11: begin shifted_t = 12'(t[34:24]); round_bit = t[23]; mask = 12'h7FF; end
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
