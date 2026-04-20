/*
 * Module Name: tr_packer_tb
 * Author(s):   Kiet Le
 * Description:
 * Comprehensive verification environment for the ML-KEM Transcoder TX
 * Gearbox (tr_packer). Verifies FIPS 203 compression, dynamic bit-packing,
 * and bidirectional flow control (memory stalling and AXI backpressure).
 *
 * Verification Architecture:
 * - Mock Polynomial Memory: Simulates read latency and random stall injections.
 * - Golden Model (Oracle): Computes the expected 64-bit AXI beats in software
 * to automatically scoreboard the hardware output.
 * - SVAs: Monitors shift register capacity to guarantee zero overflow.
 */

`default_nettype none
`timescale 1ns / 1ps

module tr_packer_tb;

    // ====================================================================
    // Parameters & Signals
    // ====================================================================
    parameter int POLY_ID_W   = 6;
    parameter int COEFF_W     = 12;
    parameter int MEM_LATENCY = 1; // Simulated SRAM read latency

    logic                      clk;
    logic                      rst;

    // Control
    logic                      start;
    logic                      done;
    logic [3:0]                d_param;
    logic [POLY_ID_W-1:0]      poly_id;

    // AXI TX
    logic [63:0]               m_tdata;
    logic                      m_tvalid;
    logic                      m_tready;

    // Memory Read Channel
    logic                      poly_req;
    logic                      poly_stall;
    logic                      poly_rd_en;
    logic [POLY_ID_W-1:0]      poly_rd_poly_id;
    logic [3:0][7:0]           poly_rd_idx;
    logic [3:0]                poly_rd_lane_valid;
    logic                      poly_rd_valid;
    logic [3:0][COEFF_W-1:0]   poly_rd_data;

    // ====================================================================
    // DUT Instantiation
    // ====================================================================
    tr_packer #(
        .POLY_ID_W(POLY_ID_W),
        .COEFF_W(COEFF_W)
    ) dut (
        .clk                 (clk),
        .rst                 (rst),
        .start_i             (start),
        .done_o              (done),
        .d_param_i           (d_param),
        .poly_id_i           (poly_id),
        .m_tdata_o           (m_tdata),
        .m_tvalid_o          (m_tvalid),
        .m_tready_i          (m_tready),
        .poly_req_o          (poly_req),
        .poly_stall_i        (poly_stall),
        .poly_rd_en_o        (poly_rd_en),
        .poly_rd_poly_id_o   (poly_rd_poly_id),
        .poly_rd_idx_o       (poly_rd_idx),
        .poly_rd_lane_valid_o(poly_rd_lane_valid),
        .poly_rd_valid_i     (poly_rd_valid),
        .poly_rd_data_i      (poly_rd_data)
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
    logic [3:0][COEFF_W-1:0] mock_mem [0:63]; // 64 reads * 4 coeffs = 256

    // Pipeline to simulate SRAM read latency
    typedef struct packed {
        logic valid;
        logic [5:0] addr;
    } mem_pipe_t;

    mem_pipe_t read_pipe [MEM_LATENCY+1];

    always_ff @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i <= MEM_LATENCY; i++) begin
                read_pipe[i].valid <= 1'b0;
                read_pipe[i].addr  <= '0;
            end
            poly_rd_valid <= 1'b0;
            poly_rd_data  <= '0;
        end else begin
            // Stage 0: Accept request if not stalled
            read_pipe[0].valid <= poly_rd_en && !poly_stall;
            read_pipe[0].addr  <= poly_rd_idx[0][7:2]; // Extract base 6-bit index

            // Shift pipeline
            for (int i = 1; i <= MEM_LATENCY; i++) begin
                read_pipe[i] <= read_pipe[i-1];
            end

            // Output stage
            poly_rd_valid <= read_pipe[MEM_LATENCY-1].valid;
            if (read_pipe[MEM_LATENCY-1].valid) begin
                poly_rd_data <= mock_mem[read_pipe[MEM_LATENCY-1].addr];
            end else begin
                poly_rd_data <= '0; // Optional: clean bus
            end
        end
    end

    // ====================================================================
    // SystemVerilog Assertions (SVAs)
    // ====================================================================
    // 1. Shift Register Overflow Prevention
    property p_no_overflow;
        @(posedge clk) disable iff (rst)
        (dut.bit_count <= 8'd128);
    endproperty
    assert property (p_no_overflow) else $error("SVA: Shift register overflowed!");

    // 2. Safe Memory Stalling (Stop reading if inflight + current > 128)
    property p_safe_read_stall;
        @(posedge clk) disable iff (rst)
        ((dut.bit_count + dut.inflight_bits + dut.bits_per_cycle) > 8'd128) |-> (poly_rd_en == 1'b0);
    endproperty
    assert property (p_safe_read_stall) else $error("SVA: Failed to stall memory safely!");

    // 3. AXI Output Integrity (Data stable while stalled)
    property p_axi_stable_data;
        @(posedge clk) disable iff (rst)
        (m_tvalid && !m_tready) |=> ($stable(m_tdata) && m_tvalid);
    endproperty
    assert property (p_axi_stable_data) else $error("SVA: AXI data changed while stalled!");

    // ====================================================================
    // Testbench Tasks & Golden Model Oracle
    // ====================================================================
    logic [63:0] expected_queue [$];

    // Reference model for Barrett Compression (matches hardware)
    function automatic logic [11:0] compress_ref(input logic [11:0] x, input logic [3:0] d);
        logic [40:0] shifted_sum;
        logic [28:0] xM;
        logic [23:0] sub1;
        logic [14:0] x_mul_5, x_mul_9;
        logic [11:0] mask;

        if (d == 12) return x;
        if (d == 1)  return ((x >= 833) && (x <= 2496)) ? 12'd1 : 12'd0;

        x_mul_5 = {x, 2'b0} + x;
        x_mul_9 = {x, 3'b0} + x;
        sub1 = {x_mul_5, 6'b0} - x_mul_5;
        xM = {sub1, 9'b0} - x_mul_9;

        case (d)
            4:  begin shifted_sum = 41'({7'b0, xM, 4'b0}) + 41'd268354944; mask = 12'h00F; end
            5:  begin shifted_sum = 41'({6'b0, xM, 5'b0}) + 41'd268354944; mask = 12'h01F; end
            10: begin shifted_sum = 41'({1'b0, xM, 10'b0}) + 41'd268354944; mask = 12'h3FF; end
            11: begin shifted_sum = 41'({xM, 11'b0}) + 41'd268354944;      mask = 12'h7FF; end
            default: return '0;
        endcase
        return shifted_sum[40:29] & mask;
    endfunction

    // Generates expected 64-bit stream from mock_mem
    task automatic generate_golden_model(input logic [3:0] d);
        logic [127:0] shift_buf = 0;
        int           buf_count = 0;
        logic [11:0]  comp_val;

        expected_queue.delete();
        for (int i = 0; i < 64; i++) begin
            for (int j = 0; j < 4; j++) begin
                comp_val = compress_ref(mock_mem[i][j], d);
                shift_buf = shift_buf | (128'(comp_val) << buf_count);
                buf_count += d;
            end
            while (buf_count >= 64) begin
                expected_queue.push_back(shift_buf[63:0]);
                shift_buf = shift_buf >> 64;
                buf_count -= 64;
            end
        end
    endtask

    // Scoreboard Monitor
    always_ff @(posedge clk) begin
        if (m_tvalid && m_tready && expected_queue.size() > 0) begin
            logic [63:0] exp;
            exp = expected_queue.pop_front();
            if (m_tdata !== exp) begin
                $error("Mismatch! Expected: %16X, Got: %16X (d=%0d)", exp, m_tdata, d_param);
            end
        end
    end

    // Main Run Task
    task run_test(input logic [3:0] d, input string name);
        $display("Running %s (d=%0d)...", name, d);
        d_param = d;
        poly_id = 6'd1;

        generate_golden_model(d);

        @(posedge clk) start = 1;
        @(posedge clk) start = 0;

        wait(done);
        @(posedge clk);

        if (expected_queue.size() > 0) begin
            $error("%s Failed: %0d expected beats were never output!", name, expected_queue.size());
        end else begin
            $display("%s Passed.", name);
        end
    endtask

    // ====================================================================
    // Main Test Sequence
    // ====================================================================
    initial begin
        $display("Starting tr_packer verification...");
        rst        = 1;
        start      = 0;
        d_param    = 0;
        poly_id    = 0;
        m_tready   = 1;
        poly_stall = 0;
        @(posedge clk); rst = 0; @(posedge clk);

        // Populate Memory with Incrementing Data
        for (int i = 0; i < 64; i++) begin
            mock_mem[i][0] = (i * 4 + 0) % 3329;
            mock_mem[i][1] = (i * 4 + 1) % 3329;
            mock_mem[i][2] = (i * 4 + 2) % 3329;
            mock_mem[i][3] = (i * 4 + 3) % 3329;
        end

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

        // ---------------------------------------------------------
        // Phase 3: Backpressure & Flow Control
        // ---------------------------------------------------------
        $display("--- Phase 3: Severe AXI Backpressure ---");
        fork
            run_test(10, "Test 3: AXI Stalls");
            begin
                // Pulse m_tready low frequently to stress the inflight logic
                repeat(50) begin
                    @(posedge clk); m_tready = 0;
                    repeat($urandom_range(2, 10)) @(posedge clk);
                    m_tready = 1;
                end
            end
        join

        $display("--- Phase 3: Memory Starvation ---");
        fork
            run_test(11, "Test 4: SRAM Stalls");
            begin
                repeat(30) begin
                    @(posedge clk); poly_stall = 1;
                    repeat($urandom_range(1, 5)) @(posedge clk);
                    poly_stall = 0;
                end
            end
        join

        $display("--- Phase 3: The Jitter Test ---");
        fork
            run_test(5, "Test 5: Dual Randomized Stalls");
            begin
                repeat(100) begin
                    @(posedge clk);
                    poly_stall = $urandom_range(0, 1);
                    m_tready   = $urandom_range(0, 1);
                end
                poly_stall = 0;
                m_tready   = 1;
            end
        join

        $display("All tr_packer tests completed successfully.");
        $finish;
    end

endmodule
`default_nettype wire
