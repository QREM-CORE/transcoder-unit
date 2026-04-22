# Transcoder Unit

This repository contains the hardware implementation of the **ML-KEM (FIPS 203)** Transcoder Unit. Evolving beyond simple ByteEncode/Decode logic, this module operates as a high-performance **Stateless Formatting Engine and Macro-Sequencing Coprocessor** for the post-quantum cryptographic core.

## Purpose & Architectural Philosophy

The primary responsibility of the Transcoder Unit is to bridge the internal 12-bit polynomial coefficient domain with byte-aligned external interfaces through structural formatting (Pack/Unpack) and mathematical scaling (Compress/Decompress).

**Philosophy: Top-Level Orchestrated Macro-Sequencing**
This design explicitly avoids deeply nested, monolithic FIPS 203 state machines (such as the Fujisaki-Okamoto transform or implicit rejection checks) inside the Transcoder. Instead, it offers a lean instruction set of **21 micro-operations**. The **ML-KEM Top-Level Controller** acts as the macro-sequencer, explicitly calling these hardware primitives to assemble the full protocol. This decoupled approach maximizes hardware reuse, eliminates redundant datapaths, and keeps the silicon footprint incredibly light and highly verifiable.

---

## Module Breakdown

The decoupled architecture consists of the following core modules:

* **`tr_fsm.sv` (The Micro-Sequencer):** The master controller that navigates multi-phase execution (math and bypass). It actively snoops datapath handshakes (`valid`/`ready`) to track data beats seamlessly without requiring the math engines to count bits, and handles latency shielding for SRAM reads.
* **`tr_microcode_rom.sv` (The Instruction Decoder):** A purely combinational "single source of truth" that dynamically resolves security levels (ML-KEM-512/768/1024) into explicit vector loops (`k_limit`), compression parameters (`d_param`), and crossbar routing signals.
* **`tr_router.sv` (The Datapath Crossbar):** A 4-bit, purely combinational routing matrix that manages all AXI backpressure and internal interconnect paths, safely gating data across multiple simultaneous destinations.
* **`tr_packer.sv` / `tr_unpacker.sv` (The Math Engines):** The highly optimized, variable-stride execution pipelines responsible for all compression, decompression, encoding, and decoding operations.

---

## Hardware Interfaces

The `transcoder_unit.sv` wrapper exposes several strictly decoupled interfaces:

1. **Host AXI4-Stream (`s_axis` / `m_axis`):** For external data ingress and egress.
2. **Hash Snoop (`hash_snoop`):** A dedicated output port that snoops incoming ciphertext streams and multicasts them to the Keccak core during Decapsulation re-encryption validation.
3. **PolyMem Subsystem (`poly_rd` / `poly_wr`):** Mutually exclusive read/write channels for interfacing with the global 12-bit polynomial SRAM.
4. **SeedBank Crossbar (`seed_rd` / `seed_wr`):** Includes internal crossbar routing that allows the Math Engines to read and write 32-byte seeds (like the recovered message m') directly to/from the internal SeedBank, entirely avoiding external AXI bus congestion.

---

## Instruction Set Architecture (ISA)

The Transcoder is driven by a 21-instruction ISA (`tr_opcode_t`). 
*(Note: Random bit generation for z and constant-time comparisons for implicit rejection c == c' are handled by the top-level orchestrator.)*

| Opcode Family | Instructions | Description |
| :--- | :--- | :--- |
| **KeyGen** | `KG_INGEST_D`, `KG_EXPORT_DK_PKE`, `KG_EXPORT_EK_PKE_1`, `KG_EXPORT_EK_PKE_2`, `KG_EXPORT_HEK`, `KG_EXPORT_Z` | Handles d ingestion and the structural packing of the Encapsulation and Decapsulation keys. |
| **Encap** | `EN_INGEST_M`, `EN_INGEST_EK_1`, `EN_INGEST_EK_2`, `EN_MSG_DEC`, `EN_EXPORT_CT_1`, `EN_EXPORT_CT_2`, `EN_EXPORT_K` | Handles message decoding and ciphertext compression (u and v). |
| **Decap** | `DC_INGEST_DK_PKE`, `DC_INGEST_EK_1`, `DC_INGEST_EK_2`, `DC_INGEST_HEK`, `DC_INGEST_C1`, `DC_INGEST_C2`, `DC_INGEST_Z`, `DC_MSG_ENC`, `DC_EXPORT_C1_PRIME`, `DC_EXPORT_C2_PRIME`, `DC_EXPORT_K`, `DC_EXPORT_K_BAR`, `DC_EXPORT_R` | Handles ciphertext ingestion (with concurrent Keccak snooping) and message packing into the internal SeedBank. |

Opcode definitions are in [qrem_global_pkg.sv](https://github.com/QREM-CORE/common-rtl/blob/main/rtl/qrem_global_pkg.sv)

---

## Verification Strategy

The Transcoder Unit has been subjected to rigorous, state-machine-driven verification via `transcoder_unit_tb.sv`:

* **Cycle-Accurate BFMs:** The testbench relies on robust, cycle-accurate Bus Functional Models (BFMs) to mock the PolyMem, SeedBank, and AXI streams.
* **Firehose AXI Testing:** Aggressive, randomized backpressure is applied to the `valid`/`ready` signals to stress-test the router's multi-destination stalling logic and the FSM's SRAM latency shielding.
* **Exhaustive Sweep:** The CI sequence automatically executes all 21 opcodes across all 3 NIST security levels (ML-KEM-512, 768, 1024), validating 63 distinct execution paths.
* **Concurrent SVAs:** SystemVerilog Assertions actively monitor internal boundaries to prevent PolyMem read/write collisions and enforce strict `tlast` protocol timings.

---

## Getting Started

### 1. Clone the Repository
Since this project relies on a centralized build system, you **must** clone with the `--recursive` flag to pull the necessary `build-tools`.

```bash
# Clone with submodules
git clone --recursive https://github.com/QREM-CORE/transcoder-unit.git
```

### 2. Initialize Submodules (If already cloned)
If you forgot to clone recursively, run the following to initialize the `build-tools` and any shared RTL libraries:

```bash
git submodule update --init --recursive
```

### 3. Build & Simulation
This repo utilizes our standard industry-grade build flow.
* **To run all testbenches (ModelSim):** `make`
* **To run all testbenches (Verilator):** `make SIM=verilator`
* **To clean build artifacts:** `make clean`
