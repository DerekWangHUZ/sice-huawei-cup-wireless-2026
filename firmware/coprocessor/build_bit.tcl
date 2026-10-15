# ============================================================================
# File    : build_bit.tcl
# Purpose : Headless bitstream build for the coprocessor project.
#           No GUI. Synthesis -> Implementation -> write_bitstream.
#
# Usage   : run from the directory that contains coprocessor.xpr
#           (normally firmware/coprocessor, after rebuild.tcl):
#
#             vivado -mode batch -source build_bit.tcl
#
# Output  : coprocessor.runs/impl_1/top.bit
#
# NOTE: ASCII-only so Windows PowerShell / cmd / tcl all handle it safely.
# ============================================================================

open_project ./coprocessor.xpr

# Sources/constraints may have changed since the last run: always reset
# both runs so stale results are never reused.
reset_run synth_1
reset_run impl_1

launch_runs synth_1
wait_on_run synth_1
puts "SYNTH: [get_property STATUS [get_runs synth_1]]"

launch_runs impl_1 -to_step write_bitstream
wait_on_run impl_1
puts "IMPL:  [get_property STATUS [get_runs impl_1]]"

set bit [file normalize ./coprocessor.runs/impl_1/top.bit]
if {[file exists $bit]} {
    puts "BITSTREAM_OK: $bit"
} else {
    puts "BITSTREAM_MISSING: $bit"
}

close_project
