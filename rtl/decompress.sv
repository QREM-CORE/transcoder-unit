`default_nettype none
import poly_arith_pkg::*;

// Decompress_d(y) = round(q * y / 2^d)
// 4 lanes in parallel, purely combinational.
// d=12 is a passthrough (no decompression).
module decompress (
    input  logic [3:0][COEFF_WIDTH-1:0] coeff_i,
    input  logic [3:0]                  d_i,
    output logic [3:0][COEFF_WIDTH-1:0] coeff_o
);
    function automatic logic [COEFF_WIDTH-1:0] decompress_one(
        input logic [COEFF_WIDTH-1:0] y,
        input logic [3:0]             d
    );
        logic [22:0] qy;
        logic [22:0] rounded;
    begin
        if (d == 4'd12) begin
            decompress_one = y;
        end else begin
            // q*y via shift-and-add: 3329 = 2^11 + 2^10 + 2^8 + 1
            qy             = (23'(y) << 11) + (23'(y) << 10) + (23'(y) << 8) + 23'(y);
            rounded        = qy + 23'(1 << (d - 4'd1)); // round to nearest
            decompress_one = COEFF_WIDTH'(rounded >> d);
        end
    end
    endfunction

    genvar i;
    generate
        for (i = 0; i < 4; i++) begin : G_LANE
            assign coeff_o[i] = decompress_one(coeff_i[i], d_i);
        end
    endgenerate

endmodule

`default_nettype wire
