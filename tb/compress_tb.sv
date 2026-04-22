/*
 * Module Name: compress_tb
 * Author(s): Kiet Le
 * Description: Exhaustive testbench for the ML-KEM Compress_d module.
 *              Verified against a golden reference model using double-
 *              precision logic (rounded-division equivalent) to ensure
 *              the hardware approximation is bit-exact for all valid d in
 *              {1, 4, 5, 10, 11} across the entire input range [0, 3328].
 *              Includes 4-lane parallel verification.
 */

`timescale 1ns/1ps

import qrem_global_pkg::*;

module compress_tb;

    // Parameters
    // localparam int COEFF_WIDTH = 12;

    // DUT Interface Signals
    logic                        clk;
    logic                        rst;
    logic [3:0][COEFF_WIDTH-1:0] coeff_i;
    logic [3:0]                  d_i;
    logic [3:0][COEFF_WIDTH-1:0] coeff_o;

    // Testbench tracking variables
    int error_count = 0;
    int test_count  = 0;

    // Clock Generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Instantiate the Datapath Module
    compress dut (
        .clk     (clk),
        .rst     (rst),
        .coeff_i (coeff_i),
        .d_i     (d_i),
        .coeff_o (coeff_o)
    );

    // ====================================================================
    // Golden Reference Model
    // Implements FIPS 203 rounding: round((x * 2^d) / 3329) mod 2^d
    // ====================================================================
    function automatic logic [COEFF_WIDTH-1:0] golden_compress(
        input logic [COEFF_WIDTH-1:0] x,
        input logic [3:0]             d
    );
        logic [23:0] scaled_x;
        logic [23:0] rounded_div;
        logic [11:0] mod_mask;

        // Scale the input: x * 2^d
        scaled_x = x * (1 << d);

        // Integer division with "round half up" logic for q = 3329
        // Formula: (A + (B/2)) / B
        rounded_div = (scaled_x + 1664) / 3329;

        // Modulo 2^d (equivalent to keeping the bottom 'd' bits)
        mod_mask = (1 << d) - 1;

        return rounded_div[COEFF_WIDTH-1:0] & mod_mask;
    endfunction

    // ====================================================================
    // Stimulus & Verification Sequence
    // ====================================================================
    initial begin : drive_and_verify
        // The specific d values required by ML-KEM per FIPS 203
        automatic int d_values[5] = '{1, 4, 5, 10, 11};
        logic [COEFF_WIDTH-1:0] expected_out [3:0];

        $display("==================================================");
        $display("Starting ML-KEM Compress Module Pipelined Testing");
        $display("==================================================");

        // Reset
        rst = 1;
        coeff_i = '0;
        d_i = '0;
        repeat(5) @(posedge clk);
        rst = 0;
        @(posedge clk);

        // Sweep all valid 'd' parameters
        for (int d_idx = 0; d_idx < 5; d_idx++) begin
            $display("Testing d = %0d...", d_values[d_idx]);

            // Sweep all possible modulo-q inputs (0 to 3328)
            // Stepping by 4 since the datapath is 4 lanes wide
            for (int x = 0; x < 3329; x += 4) begin

                // 1. Drive inputs and calculate expected results
                @(posedge clk);
                d_i = d_values[d_idx];
                for (int lane = 0; lane < 4; lane++) begin
                    if ((x + lane) < 3329) begin
                        coeff_i[lane]      = x + lane;
                        expected_out[lane] = golden_compress(coeff_i[lane], d_i);
                    end else begin
                        coeff_i[lane]      = 0;
                        expected_out[lane] = golden_compress(12'd0, d_i);
                    end
                end

                // 2. Wait for 1-cycle pipeline propagation
                @(posedge clk);
                #1; // Delay for comb logic after register

                // 3. Self-Check the outputs
                for (int lane = 0; lane < 4; lane++) begin
                    if ((x + lane) < 3329) begin
                        test_count++;

                        if (coeff_o[lane] !== expected_out[lane]) begin
                            $error("Mismatch in Lane %0d! d=%0d, x=%0d | Expected: %0d, Got: %0d",
                                   lane, d_i, (x + lane), expected_out[lane], coeff_o[lane]);
                            error_count++;
                        end
                    end
                end

                // Halt immediately if there are too many errors
                if (error_count > 20) begin
                    $display("Too many errors. Halting simulation.");
                    $stop;
                end
            end
        end

        // ====================================================================
        // Final Reporting
        // ===================================================
        $display("==================================================");
        if (error_count == 0) begin
            $display("SUCCESS: All %0d test cases passed!", test_count);
        end else begin
            $display("FAILURE: %0d errors found out of %0d test cases.", error_count, test_count);
        end
        $display("==================================================");
        $finish;
    end

endmodule

