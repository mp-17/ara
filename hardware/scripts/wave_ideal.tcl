# Copyright 2021 ETH Zurich and University of Bologna.
# Solderpad Hardware License, Version 0.51, see LICENSE for details.
# SPDX-License-Identifier: SHL-0.51
#
# Author: Matheus Cavalcante <matheusd@iis.ee.ethz.ch>

onerror {resume}
quietly WaveActivateNextPane {} 0

# Add Ara's waveforms
for {set grp 0}  {$grp < [examine -radix dec ara_tb.NrGroups]} {incr grp} {
	do ../scripts/wave_ara_ideal.tcl $grp
}
