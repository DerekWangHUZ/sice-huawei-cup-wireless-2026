# ============================================================================
# File    : program_bit.tcl
# Purpose : Headless JTAG download of top.bit into the FPGA (VOLATILE:
#           configuration is lost on power-off). For persistent programming
#           use gen_mcs.tcl + "Program Configuration Memory Device".
#
# Usage   : run from the directory containing coprocessor.runs/impl_1/top.bit
#           (normally firmware/coprocessor, after build_bit.tcl):
#
#             vivado -mode batch -source program_bit.tcl
#
#           "vivado" is typically NOT on PATH; call it by full path, e.g.
#             C:\AMDDesignTools\2026.1\Vivado\bin\vivado.bat -mode batch -source program_bit.tcl
#
# Notes   : - Guarded: refuses to program unless the detected device part
#             matches xc7a35t, so a stray JTAG cable cannot be written by
#             accident.
#           - connect_hw_server launches the local hw_server automatically.
# ============================================================================

set part_expected "xc7a35t"
set bit ./coprocessor.runs/impl_1/top.bit

if { ![file exists $bit] } {
    puts "MISSING: [file normalize $bit]"
    puts "Run build_bit.tcl first."
    exit 1
}

open_hw_manager
connect_hw_server
open_hw_target

set dev [lindex [get_hw_devices] 0]
if { $dev eq "" } {
    puts "NO_DEVICE: no hw_device on the opened target"
    close_hw_manager
    exit 1
}

set part ""
catch { set part [get_property PART $dev] }
if { ![string match "${part_expected}*" $part] } {
    puts "PART_MISMATCH: device=$dev part=$part expected=${part_expected}*"
    close_hw_manager
    exit 1
}

current_hw_device $dev
refresh_hw_device -update_hw_probes false $dev

set_property PROGRAM.FILE $bit $dev
program_hw_devices $dev
refresh_hw_device $dev

puts "PROGRAM_OK: $dev ($part) <- [file normalize $bit]"
close_hw_manager
