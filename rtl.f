# 1. Submodules (i.e. -f lib/keccak-fips202-sv/rtl.f)
-f lib/common-rtl/rtl.f

# 2. Local Packages (i.e., rtl/my_pkg.sv)
rtl/transcoder_pkg.sv

# 3. Local RTL (i.e., rtl/transcoder_unit.sv)
rtl/compress.sv
rtl/decompress.sv
rtl/tr_router.sv
rtl/tr_packer.sv
rtl/tr_unpacker.sv
rtl/tr_fsm.sv
rtl/tr_microcode_rom.sv

# 4. Top Level
rtl/transcoder_unit.sv
