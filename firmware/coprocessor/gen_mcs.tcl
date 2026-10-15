# ============================================================================
# File    : gen_mcs.tcl
# Purpose : Convert top.bit into a configuration-memory programming file
#           (top.mcs) for Program Configuration Memory Device.
#
# Usage   : run from the directory containing coprocessor.runs/impl_1/top.bit
#           (normally firmware/coprocessor, after build_bit.tcl):
#
#             vivado -mode batch -source gen_mcs.tcl
#
# Output  : coprocessor.runs/impl_1/top.mcs
#           -> feed it to "Program Configuration Memory Device"
#              (that dialog accepts .mcs/.bin only, NOT .bit)
#
# Notes   : -size 32  = 32Mbit (matches on-board 25Q032-family flash)
#           -interface SPIx1 = most conservative load mode; the bitstream
#           uses default SPI_BUS_WIDTH=1, so x1 is guaranteed compatible.
# ============================================================================

set bit ./coprocessor.runs/impl_1/top.bit

if { ![file exists $bit] } {
    puts "MISSING: [file normalize $bit]"
    puts "Run build_bit.tcl first."
    exit 1
}

write_cfgmem -format mcs -size 32 -interface SPIx1 \
    -loadbit [list up 0x00000000 $bit] \
    -file ./coprocessor.runs/impl_1/top.mcs -force

set mcs [file normalize ./coprocessor.runs/impl_1/top.mcs]
if { [file exists $mcs] } {
    puts "MCS_OK: $mcs"
} else {
    puts "MCS_MISSING: $mcs"
    exit 1
}
