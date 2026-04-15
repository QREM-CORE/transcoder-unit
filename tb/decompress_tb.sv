`timescale 1ns/1ps

module decompress_tb;

    // Parameters
    localparam int COEFF_WIDTH = 12;

    // DUT Interface Signals
    logic [3:0][COEFF_WIDTH-1:0] coeff_i;
    logic [3:0]                  d_i;
    logic [3:0][COEFF_WIDTH-1:0] coeff_o;

    // Testbench tracking variables
    int error_count = 0;
    int test_count  = 0;

    // Instantiate the Datapath Module
    decompress #(
        // .COEFF_WIDTH(COEFF_WIDTH) // Uncomment if parameterized
    ) dut (
        .coeff_i (coeff_i),
        .d_i     (d_i),
        .coeff_o (coeff_o)
    );

    // ====================================================================
    // Golden Reference Model
    // Implements FIPS 203 rounding: round((3329 / 2^d) * y)
    // ====================================================================
    function automatic logic [COEFF_WIDTH-1:0] golden_decompress(
        input logic [COEFF_WIDTH-1:0] y,
        input logic [3:0]             d
    );
        logic [23:0] scaled_y;
        logic [23:0] half_denom;
        logic [23:0] rounded_val;

        // 1. Scale the input: y * q
        scaled_y = y * 3329;

        // 2. Add half the denominator for "round half up" logic
        // Denominator is 2^d, so half is 2^(d-1)
        half_denom = (1 << (d - 1));

        // 3. Divide by 2^d (which is just a right shift by d)
        rounded_val = (scaled_y + half_denom) >> d;

        return rounded_val[COEFF_WIDTH-1:0];
    endfunction

    // ====================================================================
    // Stimulus & Verification Sequence
    // ====================================================================
    initial begin
        // The specific d values required by ML-KEM per FIPS 203
        int d_values[5] = '{1, 4, 5, 10, 11};
        logic [COEFF_WIDTH-1:0] expected_out [3:0];
        int max_y;

        $display("====================================================");
        $display("Starting ML-KEM Decompress Module Exhaustive Testing");
        $display("====================================================");

        // Sweep all valid 'd' parameters
        for (int d_idx = 0; d_idx < 5; d_idx++) begin
            d_i = d_values[d_idx];
            max_y = (1 << d_i); // 2^d is the exclusive upper bound

            $display("Testing d = %0d (Input range: 0 to %0d)...", d_i, max_y - 1);

            // Sweep all possible d-bit inputs
            // Stepping by 4 since the datapath is 4 lanes wide
            for (int y = 0; y < max_y; y += 4) begin

                // 1. Drive inputs and calculate expected results
                for (int lane = 0; lane < 4; lane++) begin
                    if ((y + lane) < max_y) begin
                        coeff_i[lane]      = y + lane;
                        expected_out[lane] = golden_decompress(coeff_i[lane], d_i);
                    end else begin
                        // Pad out-of-bounds lanes with 0
                        coeff_i[lane]      = 0;
                        expected_out[lane] = golden_decompress(12'd0, d_i);
                    end
                end

                // 2. Wait for combinational logic to propagate
                #10;

                // 3. Self-Check the outputs
                for (int lane = 0; lane < 4; lane++) begin
                    if ((y + lane) < max_y) begin
                        test_count++;

                        if (coeff_o[lane] !== expected_out[lane]) begin
                            $error("Mismatch in Lane %0d! d=%0d, y=%0d | Expected: %0d, Got: %0d",
                                   lane, d_i, (y + lane), expected_out[lane], coeff_o[lane]);
                            error_count++;
                        end
                    end
                end

                if (error_count > 20) begin
                    $display("Too many errors. Halting simulation for debug.");
                    $stop;
                end
            end
        end

        // ====================================================================
        // Final Reporting
        // ====================================================================
        $display("====================================================");
        if (error_count == 0) begin
            $display("SUCCESS: All %0d test cases passed!", test_count);
            $display("The combinational decompress logic is bit-exact to FIPS 203.");
        end else begin
            $display("FAILURE: %0d errors found out of %0d test cases.", error_count, test_count);
        end
        $display("====================================================");
        $finish;
    end

endmodule
