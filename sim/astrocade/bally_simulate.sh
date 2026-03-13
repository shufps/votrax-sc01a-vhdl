#!/bin/bash
set -e
ghdl -a --std=08 -O3 votrax_tb_vectors.vhd \
    ../../rtl/sc01a_rom.vhd \
    ../../rtl/sc01_rom.vhd \
    ../../rtl/f1_rom.vhd \
    ../../rtl/f2v_rom.vhd \
    ../../rtl/f2n_rom.vhd \
    ../../rtl/f3_rom.vhd \
    ../../rtl/f4_rom.vhd \
    ../../rtl/fx_rom.vhd \
    ../../rtl/fn_rom.vhd \
    ../../rtl/sc01a_coeff_scales_pkg.vhd \
    ../../rtl/sc01a_resamp.vhd \
    ../../rtl/iir_filter_slow.vhd \
    ../../rtl/sc01a_filter.vhd \
    ../../rtl/sc01a.vhd \
    ../../rtl/astrocade/votrax_sound.vhd \
    ../../rtl/astrocade/bally_rc_filter.vhd \
    bally.vhd \
    bally_tb.vhd
ghdl -e --std=08 -O3 bally_tb
ghdl -r bally_tb #--wave=wave.ghw
