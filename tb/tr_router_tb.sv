/*
 * Module Name: tr_router_tb
 * Author(s): Kiet Le
 * Description:
 * Comprehensive verification environment for the ML-KEM Transcoder datapath
 * crossbar (tr_router). This testbench validates the purely combinational
 * routing matrix, focusing heavily on strict backpressure enforcement and
 * data isolation across all operational modes, including the new internal
 * SRAM crossbar paths.
 *
 * Verification Architecture:
 * Although the DUT is combinational, the testbench drives stimulus and samples
 * outputs synchronously using a clock to mirror the real FSM interaction.
 * The surrounding top-level components (AXI buses, Hash Snoop, SeedBank,
 * Packer, and Unpacker) are mocked using directed tasks.
 */

`default_nettype none
`timescale 1ns / 1ps

import qrem_global_pkg::*;
import transcoder_pkg::*;

module tr_router_tb;

    // Parameters & Signals
    // ====================================================================

    logic                clk;
    logic                rst;

    // Control
    router_sel_t         router_sel;
    logic                router_tlast;

    // AXI-Stream RX
    logic [63:0]         s_axis_tdata;
    logic                s_axis_tvalid;
    logic                s_axis_tready;

    // AXI-Stream TX
    logic [63:0]         m_axis_tdata;
    logic                m_axis_tvalid;
    logic                m_axis_tready;
    logic [7:0]          m_axis_tkeep;
    logic                m_axis_tlast;

    // Hash Snoop
    logic [63:0]         hash_snoop_data;
    logic [7:0]          hash_snoop_keep;
    logic                hash_snoop_valid;
    logic                hash_snoop_ready;
    logic                hash_snoop_last;

    // SeedBank
    logic [SEED_W-1:0]   seed_rdata;
    logic                seed_rvalid; // New Handshake
    logic [SEED_W-1:0]   seed_wdata;
    logic                seed_ready;  // New Handshake

    // Packer (TX)
    logic [63:0]         packer_tdata;
    logic                packer_tvalid;
    logic                packer_tready;

    // Unpacker (RX)
    logic [63:0]         unpacker_tdata;
    logic                unpacker_tvalid;
    logic                unpacker_tready;

    // ====================================================================
    // DUT Instantiation
    // ====================================================================
    tr_router dut (
        .router_sel_i       (router_sel),
        .router_tlast_i     (router_tlast),

        .s_axis_tdata_i     (s_axis_tdata),
        .s_axis_tvalid_i    (s_axis_tvalid),
        .s_axis_tready_o    (s_axis_tready),

        .m_axis_tdata_o     (m_axis_tdata),
        .m_axis_tvalid_o    (m_axis_tvalid),
        .m_axis_tready_i    (m_axis_tready),
        .m_axis_tkeep_o     (m_axis_tkeep), // Ensure Router RTL still has this
        .m_axis_tlast_o     (m_axis_tlast),

        .hash_snoop_data_o  (hash_snoop_data),
        .hash_snoop_keep_o  (hash_snoop_keep), // Ensure Router RTL still has this
        .hash_snoop_valid_o (hash_snoop_valid),
        .hash_snoop_ready_i (hash_snoop_ready),
        .hash_snoop_last_o  (hash_snoop_last),

        .seed_rdata_i       (seed_rdata),
        .seed_rvalid_i      (seed_rvalid),
        .seed_wdata_o       (seed_wdata),
        .seed_ready_i       (seed_ready),

        .packer_tdata_i     (packer_tdata),
        .packer_tvalid_i    (packer_tvalid),
        .packer_tready_o    (packer_tready),

        .unpacker_tdata_o   (unpacker_tdata),
        .unpacker_tvalid_o  (unpacker_tvalid),
        .unpacker_tready_i  (unpacker_tready)
    );

    // ====================================================================
    // Clock Generation
    // ====================================================================
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // ====================================================================
    // Testbench Tasks
    // ====================================================================
    task reset_signals();
        rst              = 1;
        router_sel       = TR_ROUTER_IDLE;
        router_tlast     = 0;
        s_axis_tdata     = '0;
        s_axis_tvalid    = 0;
        m_axis_tready    = 0;
        hash_snoop_ready = 0;
        seed_rdata       = '0;
        seed_rvalid      = 0;
        seed_ready       = 0;
        packer_tdata     = '0;
        packer_tvalid    = 0;
        unpacker_tready  = 0;
        @(posedge clk);
        rst              = 0;
    endtask

    task drive_stimulus(
        input logic [63:0] axi_rx_d, input logic axi_rx_v,
        input logic [63:0] pack_d,   input logic pack_v,
        input logic [63:0] seed_d,   input logic seed_rv,
        input logic tlast
    );
        s_axis_tdata  = axi_rx_d;
        s_axis_tvalid = axi_rx_v;
        packer_tdata  = pack_d;
        packer_tvalid = pack_v;
        seed_rdata    = seed_d;
        seed_rvalid   = seed_rv;
        router_tlast  = tlast;
    endtask

    task set_dest_ready(
        input logic axi_tx,
        input logic snoop,
        input logic unpack,
        input logic srdy
    );
        m_axis_tready    = axi_tx;
        hash_snoop_ready = snoop;
        unpacker_tready  = unpack;
        seed_ready       = srdy;
    endtask

    // ====================================================================
    // SystemVerilog Assertions (SVAs)
    // ====================================================================
    // 1. Backpressure gate for MATH_TX_SNOOP (Mode 2)
    property p_snoop_tx_backpressure;
        @(posedge clk) (router_sel == 4'b0010) |->
                       (packer_tready == (m_axis_tready & hash_snoop_ready));
    endproperty
    assert property (p_snoop_tx_backpressure) else $error("SVA: Mode 2 backpressure failed!");

    // 2. Backpressure gate for MATH_RX_SNOOP (Mode 4)
    property p_snoop_rx_backpressure;
        @(posedge clk) (router_sel == 4'b0100) |->
                       (s_axis_tready == (unpacker_tready & hash_snoop_ready));
    endproperty
    assert property (p_snoop_rx_backpressure) else $error("SVA: Mode 4 backpressure failed!");

    // 3. Data Leakage Isolation
    property p_isolation_rx;
        @(posedge clk) (router_sel == 4'b0011) |-> (m_axis_tvalid == 1'b0);
    endproperty
    assert property (p_isolation_rx) else $error("SVA: Data leaked to AXI TX during RX mode!");

    // ====================================================================
    // Main Test Sequence
    // ====================================================================
    initial begin
        $display("Starting tr_router verification...");
        reset_signals();

        // ---------------------------------------------------------
        // Test 0: IDLE Mode
        // ---------------------------------------------------------
        $display("[TEST 0] IDLE Mode Verification");
        router_sel = TR_ROUTER_IDLE;
        // axi_rx_d, axi_rx_v, pack_d, pack_v, seed_d, seed_rv, tlast
        drive_stimulus(64'hAAAA, 1, 64'hBBBB, 1, 64'hCCCC, 1, 1);
        set_dest_ready(1, 1, 1, 1);
        #1;
        // Wait for combinational logic
        if (m_axis_tvalid || hash_snoop_valid || unpacker_tvalid || s_axis_tready || packer_tready)
            $error("Test 0 Failed: Signals active during IDLE mode!");

        // ---------------------------------------------------------
        // Test 1: Single-Destination Routing
        // ---------------------------------------------------------
        $display("[TEST 1] Single-Destination Verification");

        @(posedge clk);
        router_sel = TR_ROUTER_MATH_TX; // MATH_TX
        #1;
        if (m_axis_tdata !== 64'hBBBB || m_axis_tvalid !== 1) $error("MATH_TX Routing Failed!");

        @(posedge clk);
        router_sel = TR_ROUTER_MATH_RX; // MATH_RX
        #1;
        if (unpacker_tdata !== 64'hAAAA || unpacker_tvalid !== 1) $error("MATH_RX Routing Failed!");

        @(posedge clk);
        router_sel = TR_ROUTER_BYPASS_TX; // RAW_BYPASS_TX
        #1;
        if (m_axis_tdata !== 64'hCCCC || m_axis_tvalid !== 1) $error("BYPASS_TX Routing Failed!");

        @(posedge clk);
        router_sel = TR_ROUTER_BYPASS_RX; // RAW_BYPASS_RX
        #1;
        if (seed_wdata !== 64'hAAAA || s_axis_tready !== 1) $error("BYPASS_RX Routing Failed!");

        // --- NEW INTERNAL CROSSBAR TESTS ---
        @(posedge clk);
        router_sel = TR_ROUTER_MATH_RX_FROM_SEEDBANK; // MATH_RX_FROM_SEEDBANK
        #1;
        if (unpacker_tdata !== 64'hCCCC || unpacker_tvalid !== 1) $error("MATH_RX_FROM_SEEDBANK Routing Failed!");

        @(posedge clk);
        router_sel = TR_ROUTER_MATH_TX_TO_SEEDBANK; // MATH_TX_TO_SEEDBANK
        #1;
        if (seed_wdata !== 64'hBBBB || packer_tready !== 1) $error("MATH_TX_TO_SEEDBANK Routing Failed!");

        // ---------------------------------------------------------
        // Test 2: Backpressure Matrix (Mode 2)
        // ---------------------------------------------------------
        $display("[TEST 2] Backpressure Matrix Verification");
        @(posedge clk);
        router_sel = TR_ROUTER_MATH_TX_SNOOP; // MATH_TX_SNOOP

        // Both Ready
        set_dest_ready(1, 1, 0, 1); #1;
        if (packer_tready !== 1) $error("Mode 2: Should be ready when both targets ready");

        // AXI TX Stalls
        set_dest_ready(0, 1, 0, 1); #1;
        if (packer_tready !== 0) $error("Mode 2: Failed to stall when AXI TX stalled");

        // Snoop Stalls
        set_dest_ready(1, 0, 0, 1); #1;
        if (packer_tready !== 0) $error("Mode 2: Failed to stall when Snoop stalled");

        // ---------------------------------------------------------
        // Test 3: Randomized Stress Test (100 Cycles)
        // ---------------------------------------------------------
        $display("[TEST 3] Randomized Stress Test (100 Cycles)");
        for (int i = 0; i < 100; i++) begin
            @(posedge clk);
            router_sel    = router_sel_t'($urandom_range(0, 8)); // Now testing all 8 active modes
            router_tlast  = $urandom_range(0, 1);
            s_axis_tdata  = {$urandom(), $urandom()};
            packer_tdata  = {$urandom(), $urandom()};
            seed_rdata    = {$urandom(), $urandom()};
            s_axis_tvalid = $urandom_range(0, 1);
            packer_tvalid = $urandom_range(0, 1);
            seed_rvalid   = $urandom_range(0, 1);
            set_dest_ready($urandom_range(0,1), $urandom_range(0,1), $urandom_range(0,1), $urandom_range(0,1));
        end

        $display("Verification Complete!");
        $finish;
    end

    // ====================================================================
    // Automated Checker (Scoreboard)
    // ====================================================================
    // Checks output states on the negative edge, after comb logic settles
    always @(negedge clk) begin
        case (router_sel)
            TR_ROUTER_MATH_TX: begin
                if (m_axis_tdata !== packer_tdata) $error("Checker: m_axis_tdata mismatch in Mode 1");
                if (packer_tready !== m_axis_tready) $error("Checker: backpressure mismatch in Mode 1");
            end
            TR_ROUTER_MATH_TX_SNOOP: begin
                if (m_axis_tdata !== packer_tdata) $error("Checker: m_axis_tdata mismatch in Mode 2");
                if (hash_snoop_data !== packer_tdata) $error("Checker: snoop data mismatch in Mode 2");
            end
            TR_ROUTER_MATH_RX: begin
                if (unpacker_tdata !== s_axis_tdata) $error("Checker: unpacker_tdata mismatch in Mode 3");
            end
            TR_ROUTER_MATH_RX_FROM_SEEDBANK: begin // New Mode Check
                if (unpacker_tdata !== seed_rdata) $error("Checker: unpacker_tdata mismatch in Mode 7");
                if (unpacker_tvalid !== seed_rvalid) $error("Checker: unpacker_tvalid mismatch in Mode 7");
            end
            TR_ROUTER_MATH_TX_TO_SEEDBANK: begin // New Mode Check
                if (seed_wdata !== packer_tdata) $error("Checker: seed_wdata mismatch in Mode 8");
                if (packer_tready !== seed_ready) $error("Checker: backpressure mismatch in Mode 8");
            end
            // ... Additional precise scoreboard checks can be added here
            default: ;
        endcase
    end

endmodule
`default_nettype wire
