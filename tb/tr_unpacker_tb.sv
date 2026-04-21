/*
 * Module Name: tr_unpacker_tb
 * Author(s):   Kiet Le
 * Description:
 * Comprehensive verification environment for the ML-KEM Transcoder RX
 * Gearbox (tr_unpacker). Verifies dynamic bit-slicing (ByteDecode_d),
 * FIPS 203 decompression, and strict shift register buffer management.
 *
 * Verification Architecture:
 * - Mock Polynomial Memory: Captures the 256 writes and supports random stalls.
 * - Golden Model (Oracle): A software function that packs randomized d-bit
 * bounds into an exact 64-bit stream, simulating a Host AXI transmission.
 * - Driver/Monitor: Asynchronously drives the AXI stream observing 'ready',
 * while a monitor validates the final memory state after 'done' is asserted.
 */

`default_nettype none
`timescale 1ns / 1ps

module tr_unpacker_tb;

    // ====================================================================
    // Parameters & Signals
    // ====================================================================
    parameter int POLY_ID_W = 6;
    parameter int COEFF_W   = 12;

    logic                      clk;
    logic                      rst;

    // Control
    logic                      start;
    logic                      done;
    logic [3:0]                d_param;
    logic [POLY_ID_W-1:0]      poly_id;

    // AXI RX
    logic [63:0]               s_tdata;
    logic                      s_tvalid;
    logic                      s_tready;

    // Memory Write Channel
    logic                      poly_req;
    logic                      poly_stall;
    logic [3:0]                poly_wr_en;
    logic [POLY_ID_W-1:0]      poly_wr_poly_id;
    logic [3:0][7:0]           poly_wr_idx;
    logic [3:0][COEFF_W-1:0]   poly_wr_data;

    // ====================================================================
    // DUT Instantiation
    // ====================================================================
    tr_unpacker #(
        .POLY_ID_W(POLY_ID_W),
        .COEFF_W(COEFF_W)
    ) dut (
        .clk                 (clk),
        .rst                 (rst),
        .start_i             (start),
        .done_o              (done),
        .d_param_i           (d_param),
        .poly_id_i           (poly_id),
        .s_tdata_i           (s_tdata),
        .s_tvalid_i          (s_tvalid),
        .s_tready_o          (s_tready),
        .poly_req_o          (poly_req),
        .poly_stall_i        (poly_stall),
        .poly_wr_en_o        (poly_wr_en),
        .poly_wr_poly_id_o   (poly_wr_poly_id),
        .poly_wr_idx_o       (poly_wr_idx),
        .poly_wr_data_o      (poly_wr_data)
    );

    // ====================================================================
    // Clock Generation
    // ====================================================================
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // ====================================================================
    // Mock Polynomial Memory Subsystem
    // ====================================================================
    logic [3:0][COEFF_W-1:0] mock_mem [0:63]; // 64 writes * 4 coeffs = 256

    always_ff @(posedge clk) begin
        if (poly_wr_en[0]) mock_mem[poly_wr_idx[0][7:2]][0] <= poly_wr_data[0];
        if (poly_wr_en[1]) mock_mem[poly_wr_idx[1][7:2]][1] <= poly_wr_data[1];
        if (poly_wr_en[2]) mock_mem[poly_wr_idx[2][7:2]][2] <= poly_wr_data[2];
        if (poly_wr_en[3]) mock_mem[poly_wr_idx[3][7:2]][3] <= poly_wr_data[3];
    end

    task automatic clear_mock_mem();
        for (int i = 0; i < 64; i++) begin
            mock_mem[i] = '0;
        end
    endtask

    // ====================================================================
    // SystemVerilog Assertions (SVAs)
    // ====================================================================
    // 1. Prevent Shift Register Overflow
    property p_no_overflow;
        @(posedge clk) disable iff (rst)
        (dut.bit_count <= 8'd128);
    endproperty
    assert property (p_no_overflow) else $error("SVA: RX buffer overflowed 128 bits!");

    // 2. Safe AXI Backpressure
    // If the buffer has more than 64 bits, it CANNOT accept another 64-bit beat safely.
    property p_safe_axi_stall;
        @(posedge clk) disable iff (rst)
        (dut.bit_count > 8'd64) |-> (s_tready == 1'b0);
    endproperty
    assert property (p_safe_axi_stall) else $error("SVA: Failed to drop AXI ready when buffer was full!");

    // 3. Safe Memory Writes
    // Do not assert write enable if we don't have enough valid sliced bits.
    property p_safe_mem_write;
        @(posedge clk) disable iff (rst)
        (poly_wr_en != 4'b0) |-> (dut.bit_count >= dut.bits_per_cycle);
    endproperty
    assert property (p_safe_mem_write) else $error("SVA: Attempted to write incomplete bits to memory!");

    // ====================================================================
    // Golden Model Oracle
    // ====================================================================
    logic [63:0] stimulus_queue [$];
    logic [11:0] expected_coeffs [0:255];

    // Hardware-exact reference model for Decompression
    function automatic logic [11:0] decompress_ref(input logic [11:0] y, input logic [3:0] d);
        logic [15:0] qy_4; logic [16:0] qy_5; logic [21:0] qy_10; logic [22:0] qy_11;
        if (d == 12) return y;
        if (d == 1)  return y[0] ? 12'd1665 : 12'd0;
        case (d)
            4:  begin qy_4  = (16'(y[3:0])<<11)  + (16'(y[3:0])<<10)  + (16'(y[3:0])<<8)  + 16'(y[3:0]);  return 12'((qy_4 + 8) >> 4); end
            5:  begin qy_5  = (17'(y[4:0])<<11)  + (17'(y[4:0])<<10)  + (17'(y[4:0])<<8)  + 17'(y[4:0]);  return 12'((qy_5 + 16) >> 5); end
            10: begin qy_10 = (22'(y[9:0])<<11)  + (22'(y[9:0])<<10)  + (22'(y[9:0])<<8)  + 22'(y[9:0]);  return 12'((qy_10 + 512) >> 10); end
            11: begin qy_11 = (23'(y[10:0])<<11) + (23'(y[10:0])<<10) + (23'(y[10:0])<<8) + 23'(y[10:0]); return 12'((qy_11 + 1024) >> 11); end
            default: return '0;
        endcase
    endfunction

    // Generates perfect 64-bit AXI beats and the 256 expected decompressed coefficients
    task automatic generate_stimulus(input logic [3:0] d, input bit edge_case = 0);
        logic [127:0] shift_buf = 0;
        int           buf_count = 0;
        logic [11:0]  raw_val;

        stimulus_queue.delete();

        for (int i = 0; i < 256; i++) begin
            // 1. Generate bounded random data
            if (edge_case) raw_val = (i % 2 == 0) ? '0 : ((1 << d) - 1); // Alternate 0s and Max
            else           raw_val = $urandom_range(0, (1 << d) - 1);

            // 2. Calculate expected hardware output
            expected_coeffs[i] = decompress_ref(raw_val, d);

            // 3. Pack bits for AXI transmission
            shift_buf = shift_buf | (128'(raw_val) << buf_count);
            buf_count += d;

            while (buf_count >= 64) begin
                stimulus_queue.push_back(shift_buf[63:0]);
                shift_buf = shift_buf >> 64;
                buf_count -= 64;
            end
        end
        // Push residual bits (though FIPS 203 sizes guarantee perfect alignment)
        if (buf_count > 0) stimulus_queue.push_back(shift_buf[63:0]);
    endtask

    // ====================================================================
    // Driver & Monitor Tasks
    // ====================================================================
    task automatic drive_axi();
        // 1. Prime the pump immediately if queue has data
        if (stimulus_queue.size() > 0) begin
            s_tdata  <= stimulus_queue[0];
            s_tvalid <= 1'b1;
        end

        while (stimulus_queue.size() > 0) begin
            @(posedge clk);

            // 2. If a handshake just occurred, pop and update
            if (s_tready && s_tvalid) begin
                stimulus_queue.pop_front();

                if (stimulus_queue.size() > 0) begin
                    s_tdata  <= stimulus_queue[0];
                    s_tvalid <= 1'b1;
                end else begin
                    s_tvalid <= 1'b0;
                    s_tdata  <= '0; // Clean the bus
                end
            end
        end
        @(posedge clk);
        s_tvalid <= 1'b0;
    endtask

    task automatic check_memory(input string test_name);
        int err_count = 0;
        for (int i = 0; i < 64; i++) begin
            if (mock_mem[i][0] !== expected_coeffs[i*4 + 0]) err_count++;
            if (mock_mem[i][1] !== expected_coeffs[i*4 + 1]) err_count++;
            if (mock_mem[i][2] !== expected_coeffs[i*4 + 2]) err_count++;
            if (mock_mem[i][3] !== expected_coeffs[i*4 + 3]) err_count++;

            if (err_count > 0 && err_count < 3) begin
                $error("[%s] Mismatch at block %0d. Expected: {%0d, %0d, %0d, %0d}, Got: {%0d, %0d, %0d, %0d}",
                    test_name, i,
                    expected_coeffs[i*4+3], expected_coeffs[i*4+2], expected_coeffs[i*4+1], expected_coeffs[i*4+0],
                    mock_mem[i][3], mock_mem[i][2], mock_mem[i][1], mock_mem[i][0]);
            end
        end
        if (err_count == 0) $display("[%s] PASSED.", test_name);
        else                $error("[%s] FAILED with %0d coefficient mismatches.", test_name, err_count);
    endtask

    task automatic run_test(input logic [3:0] d, input string name, input bit edge_c = 0);
        $display("Running %s (d=%0d)...", name, d);
        d_param = d;
        poly_id = 6'd2;
        clear_mock_mem();
        generate_stimulus(d, edge_c);

        @(posedge clk) start = 1;
        @(posedge clk) start = 0;

        fork
            drive_axi();
        join_none

        wait(done);
        @(posedge clk);
        check_memory(name);
    endtask

    // ====================================================================
    // Main Test Sequence
    // ====================================================================
    initial begin
        $display("Starting tr_unpacker verification...");
        rst        = 1;
        start      = 0;
        s_tvalid   = 0;
        s_tdata    = 0;
        poly_stall = 0;
        @(posedge clk); rst = 0; @(posedge clk);

        // ---------------------------------------------------------
        // Phase 2: Directed Datapath Testing
        // ---------------------------------------------------------
        $display("--- Phase 2: Ideal Flow ---");
        run_test(1,  "Test 1: Sweep D=1");
        run_test(4,  "Test 1: Sweep D=4");
        run_test(5,  "Test 1: Sweep D=5");
        run_test(10, "Test 1: Sweep D=10");
        run_test(11, "Test 1: Sweep D=11");
        run_test(12, "Test 1: Sweep D=12");

        $display("--- Phase 2: Edge Cases ---");
        run_test(1, "Test 2: Edge Pattern D=1", 1);
        run_test(10, "Test 2: Edge Pattern D=10", 1);

        // ---------------------------------------------------------
        // Phase 3: Backpressure & Flow Control
        // ---------------------------------------------------------
        $display("--- Phase 3: Severe SRAM Backpressure ---");
        fork
            run_test(11, "Test 3: Memory Stalls (Buffer Fill)");
            begin
                // Pulse poly_stall high to force the accumulator to fill up
                repeat(20) begin
                    @(posedge clk); poly_stall = 1;
                    repeat($urandom_range(5, 15)) @(posedge clk);
                    poly_stall = 0;
                    repeat(2) @(posedge clk); // let it drain briefly
                end
            end
        join

        $display("--- Phase 3: AXI Starvation ---");
        fork
            // Test 4 uses the driver's natural wait states by injecting delays
            begin
                $display("Running Test 4: AXI Starvation (d=5)...");
                d_param = 5;
                clear_mock_mem();
                generate_stimulus(5);

                @(posedge clk) start = 1;
                @(posedge clk) start = 0;

                while (stimulus_queue.size() > 0) begin
                    // 1. Starvation Phase: Randomly delay BEFORE asserting valid
                    if ($urandom_range(0,2) == 0) begin
                        s_tvalid <= 1'b0;
                        repeat($urandom_range(2, 6)) @(posedge clk);
                    end

                    // 2. Drive Phase: Put data on bus and hold valid high
                    s_tdata  <= stimulus_queue[0];
                    s_tvalid <= 1'b1;

                    // 3. Wait for the AXI Handshake
                    do begin
                        @(posedge clk);
                    end while (!(s_tready && s_tvalid));

                    // 4. Handshake complete! Pop queue and drop valid to allow starvation
                    stimulus_queue.pop_front();
                    s_tvalid <= 1'b0;
                end
            end
            begin
                wait(done);
                @(posedge clk);
                check_memory("Test 4: AXI Starvation (Buffer Drain)");
            end
        join

        $display("--- Phase 3: The Jitter Test ---");
        fork
            run_test(10, "Test 5: Dual Randomized Stalls");
            begin
                // Note: s_tvalid jitter is handled by the starvation test.
                // Here we randomize the memory stall aggressively.
                repeat(200) begin
                    @(posedge clk);
                    poly_stall = $urandom_range(0, 1);
                end
                poly_stall = 0;
            end
        join

        $display("All tr_unpacker tests completed successfully.");
        $finish;
    end

endmodule
`default_nettype wire
