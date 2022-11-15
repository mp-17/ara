# Copyright 2021 ETH Zurich and University of Bologna.
# Solderpad Hardware License, Version 0.51, see LICENSE for details.
# SPDX-License-Identifier: SHL-0.51
#
# Author: Matheus Cavalcante <matheusd@iis.ee.ethz.ch>

add wave -noupdate -group Ara -group core /ara_tb/dut/i_ara_soc/i_system/i_ara/*

add wave -noupdate -group Ara -group dispatcher /ara_tb/dut/i_ara_soc/i_system/i_ara/i_dispatcher/*
add wave -noupdate -group Ara -group sequencer /ara_tb/dut/i_ara_soc/i_system/i_ara/i_sequencer/*

# Add waves from all the lanes
for {set lane 0}  {$lane < [examine -radix dec ara_tb.NrLanes]} {incr lane} {
    do ../scripts/wave_lane.tcl $lane
}

add wave -noupdate -group Ara -group masku /ara_tb/dut/i_ara_soc/i_system/i_ara/i_masku/*

add wave -noupdate -group Ara -group sldu /ara_tb/dut/i_ara_soc/i_system/i_ara/i_sldu/*

add wave -noupdate -group Ara -group vlsu -group addrgen /ara_tb/dut/i_ara_soc/i_system/i_ara/i_vlsu/i_addrgen/*
add wave -noupdate -group Ara -group vlsu -group vldu /ara_tb/dut/i_ara_soc/i_system/i_ara/i_vlsu/i_vldu/*
add wave -noupdate -group Ara -group vlsu -group vstu /ara_tb/dut/i_ara_soc/i_system/i_ara/i_vlsu/i_vstu/*
add wave -noupdate -group Ara -group vlsu /ara_tb/dut/i_ara_soc/i_system/i_ara/i_vlsu/*

add wave -noupdate -group Ara -group bc_buffer /ara_tb/dut/i_ara_soc/i_system/i_ara/i_bc_buffer/*
add wave -noupdate -group Ara -group bc_buffer -group re_readable_fifo[0] /ara_tb/dut/i_ara_soc/i_system/i_ara/i_bc_buffer/gen_re_readable_buffer[0]/i_re_readable_fifo/*
add wave -noupdate -group Ara -group bc_buffer -group re_readable_fifo[1] /ara_tb/dut/i_ara_soc/i_system/i_ara/i_bc_buffer/gen_re_readable_buffer[1]/i_re_readable_fifo/*