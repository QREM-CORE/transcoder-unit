# Transcoder Unit

This repository contains the hardware implementation of the data formatting and representation modules for the **ML-KEM (FIPS 203)** post-quantum cryptographic core. The primary goal of this unit is to bridge the gap between the internal 12-bit polynomial coefficients and the byte-aligned external interfaces.

## Purpose
The Transcoder Unit is responsible for four critical "Sign-off" operations:
* **ByteEncode / ByteDecode**: Efficiently packing and unpacking 12-bit polynomial coefficients into 8-bit byte strings.
* **Compress / Decompress**: Implementing the scaling and rounding logic required to reduce ciphertext size and meet NIST security level requirements.
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
