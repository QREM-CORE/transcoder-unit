# **High-Level Architecture Overview**

The Transcoder acts as a "Smart Gearbox." It bridges your **64-bit external domains** (AXI-Stream, Hash Snoop, SeedBank) with your **48-bit internal domains** (Polynomial Memory).

By decoupling the paths, the architecture is split into four distinct submodules:
1.  **The Brain:** `tr_fsm` (State machine, byte counting, orchestration).
2.  **The Input Engine:** `tr_unpacker` (Bit-slicing and Decompression).
3.  **The Output Engine:** `tr_packer` (Compression and Bit-packing).
4.  **The Traffic Cop:** `tr_router` (Combinational muxing and bypass paths).

---

### **1. Submodule: `tr_fsm.sv` (The Master Controller)**
This is the only module that understands the ML-KEM protocol. It translates high-level opcodes into low-level hardware control signals.

* **Inputs:** `ctrl_start`, `ctrl_opcode`, `ctrl_sec_level`.
* **Outputs:** * `d_param` for the packer/unpacker.
    * Memory IDs (`poly_id_o`, `seed_id_o`) and read/write enables.
    * Mux select lines for `tr_router`.
    * `axis_tx_tlast` and `hash_snoop_last` generation.
* **Internal Mechanics:** It contains a **Byte Counter**. When an opcode fires (e.g., `TR_OP_KG_EXPORT_EK_PKE`), it looks at the `sec_level` to determine the exact number of bytes required for the polynomial portion ($384 \times k$). Once that count is reached, it seamlessly flips the router multiplexers to fetch the remaining 32 bytes from the SeedBank to finish the packet.

### **2. Submodule: `tr_unpacker.sv` (The RX Gearbox)**
This module handles all data flowing **INTO** the Polynomial Memory. It never touches seed data.

* **Inputs:** 64-bit valid words from `tr_router`, plus `d_param`.
* **Outputs:** 48-bit coefficient blocks (4x12-bits) to `poly_wr_data_o`.
* **Internal Mechanics:**
    * **The Accumulator:** A shift register (e.g., 128-bits wide) that absorbs 64-bit beats.
    * **The Slicer:** Combinational logic that slices exactly $4 \times d$ bits off the accumulator every cycle.
    * **The Math:** Instantiates your `decompress.sv` module. It passes the sliced bits through the decompressor to output four valid 12-bit coefficients.

### **3. Submodule: `tr_packer.sv` (The TX Gearbox)**
This module handles all mathematical data flowing **OUT** of the Polynomial Memory.

* **Inputs:** 48-bit coefficient blocks from `poly_rd_data_i`, plus `d_param`.
* **Outputs:** Perfectly formatted 64-bit valid words to `tr_router`.
* **Internal Mechanics:**
    * **The Math:** Instantiates your `compress.sv` module. It takes four 12-bit coefficients and crushes them down to exactly $4 \times d$ bits.
    * **The Packer:** A shift register that receives those $4 \times d$ bits every cycle. Once it has $\ge 64$ bits stored, it fires a `valid` pulse and ejects a 64-bit word to the router.

### **4. Submodule: `tr_router.sv` (The Datapath Crossbar)**
This is a purely combinational module (no flip-flops). It is a collection of multiplexers driven by the `tr_fsm` to route the 64-bit data streams.

* **Mode A: Math TX:** Routes `tr_packer` output to `m_axis_tdata` and/or `hash_snoop_data_o`.
* **Mode B: Math RX:** Routes `s_axis_tdata` into the `tr_unpacker`.
* **Mode C: Raw Bypass TX:** Connects `seed_rdata_i` directly to `m_axis_tdata` (Used for streaming seeds out).
* **Mode D: Raw Bypass RX:** Connects `s_axis_tdata` directly to `seed_wdata_o` (Used for ingesting seeds).

---

### **Execution Example: `TR_OP_EN_INGEST_EK` (Encapsulation Key Ingest)**

Here is how these four modules work together to execute a multi-part opcode:

1.  **Phase 1 (Polynomial Ingestion):**
    * `tr_fsm` sees the opcode. It sets $d=12$.
    * `tr_fsm` tells `tr_router` to connect `AXI-RX` to `tr_unpacker`, and connects `AXI-RX` to `hash_snoop` simultaneously (since $H(ek)$ is needed).
    * `tr_unpacker` starts eating 64-bit words, decoding them, and writing to Polynomial Memory.
    * `tr_fsm` counts the bytes. Once it hits exactly $384 \times k$ bytes, Phase 1 ends.
2.  **Phase 2 (Seed Ingestion):**
    * `tr_fsm` instantly tells `tr_router` to switch to **Raw Bypass RX**.
    * The `tr_unpacker` goes to sleep.
    * The next 32 bytes (4 beats) from `AXI-RX` bypass the math pipelines and route directly into the SeedBank via `seed_wdata_o` at ID `SEED_RHO`.
    * Once 32 bytes are counted, `tr_fsm` asserts `ctrl_done`.

--

This architecture completely isolates the complex bit-shifting logic from the protocol orchestration logic.
