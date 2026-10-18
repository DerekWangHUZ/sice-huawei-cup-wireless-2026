# ============================================================================
# File    : program_mcs.tcl
# Purpose : Headless programming of the on-board configuration flash with
#           top.mcs (persistent: survives power-off, unlike program_bit.tcl
#           which only loads the FPGA SRAM).
#
# Usage   : run from the directory containing coprocessor.runs/impl_1/top.mcs
#           (normally firmware/coprocessor):
#
#             vivado -mode batch -source gen_mcs.tcl        # 1) make top.mcs
#             vivado -mode batch -source program_mcs.tcl    # 2) burn to flash
#
# Notes   : - Guarded: refuses to program unless the detected device part
#             matches xc7a35t.
#           - Memory part pinned explicitly: n25q32-3.3v-spi-x1_x2_x4
#             (matches gen_mcs.tcl's -size 32 / SPIx1). Update MEM_PART here
#             if the flash is ever replaced.
#           - ERASE + PROGRAM + VERIFY on; BLANK_CHECK off (slow, unnecessary
#             for a known-good part).
#
# KNOWN LIMITATION on this board (see docs/FPGA实验板资料参考.md §9b):
#           A successful burn does NOT make the board self-start on power-up.
#           After powering on, the CONFIG button must be pressed once. The
#           flash image itself is good - the button proves it - so this is a
#           board-level power-on trigger issue, not a bitstream/flash issue.
# ============================================================================

set part_expected "xc7a35t"
set mem_part      "n25q32-3.3v-spi-x1_x2_x4"
set mcs           ./coprocessor.runs/impl_1/top.mcs

if { ![file exists $mcs] } {
    puts "MCS_MISSING: [file normalize $mcs]"
    puts "Run gen_mcs.tcl first."
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

set mem_dev [lindex [get_cfgmem_parts $mem_part] 0]
if { $mem_dev eq "" } {
    puts "CFGMEM_PART_MISSING: $mem_part not found in this Vivado installation"
    close_hw_manager
    exit 1
}

current_hw_device $dev
refresh_hw_device -update_hw_probes false $dev

# ---------------------------------------------------------------------------
# STEP 1 (CRITICAL): load the SPI bridge bitstream into the FPGA.
#
# JTAG cannot reach the SPI config flash directly. Vivado ships a small
# "bridge" design (spi_<part>_pullnone.bit) that turns the FPGA into a
# temporary SPI master; program_hw_cfgmem drives the flash through it.
#
# The bridge bitstreams ship inside
#   <Vivado>/data/xicom/cfgmem/bitfile.zip
# and PROGRAM.HW_CFGMEM_BITFILE may point at a bitfile/ directory that does
# not exist if that archive was never expanded. In that case Vivado reports
#   "[Labtools 27-3347] Failure to set flash parameters. Verify that the
#    selected flash part matches the actual flash device"
# which is misleading - readback_hw_cfgmem gives the real reason:
#   "[Labtools 27-3138] ... indirect programming bitstream was not found".
#
# So: (1) use the advertised path if it exists, else (2) extract the single
# needed member from bitfile.zip into a temp dir. If neither works, stop with
# a clear message rather than letting the misleading error through.
# ---------------------------------------------------------------------------
proc mcs_find_bridge { devobj part } {
    # (1) prefer the path Vivado advertises, if the file is really there
    set advertised ""
    catch { set advertised [string trim [get_property PROGRAM.HW_CFGMEM_BITFILE $devobj]] }
    if { $advertised ne "" && [file exists $advertised] } {
        return $advertised
    }

    # (2) fall back to extracting from bitfile.zip. Do not rely on
    # `info nameofexecutable` (under -mode batch it points at bin/unwrapped,
    # which has no data/ tree); derive the data root from the advertised path:
    #   <Vivado>/data/xicom/cfgmem/bitfile/spi_<part>_pullnone.bit
    set zipfile ""
    if { $advertised ne "" } {
        set cfgdir [file dirname [file dirname $advertised]]
        set cand [file join $cfgdir bitfile.zip]
        if { [file exists $cand] } {
            set zipfile $cand
        }
    }
    if { $zipfile eq "" } {
        foreach base [list "C:/AMDDesignTools/2026.1/Vivado" \
                           $::env(XILINX_VIVADO)] {
            if { $base ne "" } {
                set cand [file join $base data xicom cfgmem bitfile.zip]
                if { [file exists $cand] } { set zipfile $cand; break }
            }
        }
    }
    if { $zipfile eq "" || ![file exists $zipfile] } {
        puts "MCS_BRIDGE_NOTE: bitfile.zip not found (advertised='$advertised')"
        return ""
    }

    set member "bitfile/spi_${part}_pullnone.bit"
    set outdir [file join $::env(TEMP) vivado_cfgmem_bitfile]
    file mkdir $outdir
    # tar keeps the member's own directory prefix, so the file lands in
    # <outdir>/bitfile/, not directly in <outdir>/.
    set out [file join $outdir bitfile "spi_${part}_pullnone.bit"]
    if { [file exists $out] } {
        return $out
    }

    # Expand just the one member we need. tar.exe ships with Windows 10+
    # and reads zip archives, so no extra tooling is required. (Vivado's own
    # Tcl has no zipfile::decode package in 2026.1 - verified.)
    if { [catch {
        exec tar.exe -xf $zipfile -C $outdir $member
    } e] } {
        puts "MCS_BRIDGE_NOTE: tar could not expand $zipfile ($e)"
        return ""
    }
    return [expr { [file exists $out] ? $out : "" }]
}

set bridge [mcs_find_bridge $dev [get_property PART $dev]]

if { $bridge eq "" } {
    puts "CFGMEM_BRIDGE_MISSING: no SPI bridge bitstream for [get_property PART $dev]"
    puts "  Looked for PROGRAM.HW_CFGMEM_BITFILE and for the member"
    puts "    bitfile/spi_<part>_pullnone.bit"
    puts "  inside <Vivado>/data/xicom/cfgmem/bitfile.zip."
    puts "  Without it the flash cannot be reached over JTAG."
    puts "  Fix (one-off, needs admin rights): extract bitfile.zip into"
    puts "    <Vivado>/data/xicom/cfgmem/   so that bitfile/ appears there,"
    puts "  or just use the GUI: Hardware Manager -> right-click the device ->"
    puts "  Program Configuration Memory Device (it locates the bridge itself)."
    close_hw_manager
    exit 3
}

puts "MCS_BRIDGE: [file normalize $bridge]"
set_property PROGRAM.FILE $bridge $dev
if { [catch { program_hw_devices $dev } err] } {
    puts "MCS_BRIDGE_FAILED: $err"
    puts "  Could not load the SPI bridge bitstream into the FPGA."
    close_hw_manager
    exit 4
}
refresh_hw_device -update_hw_probes false $dev
puts "MCS_BRIDGE_OK: FPGA now runs the SPI bridge"

# ---------------------------------------------------------------------------
# STEP 2: attach the cfgmem device, then program it.
# Order matters: set the options on a freshly created hw_cfgmem, refresh the
# device so the object is bound into the target, then RE-FETCH the handle and
# set PROGRAM.FILES (refresh invalidates the pre-refresh handle).
# ---------------------------------------------------------------------------
create_hw_cfgmem -hw_device $dev $mem_dev
set cfgmem [get_property PROGRAM.HW_CFGMEM $dev]

set_property PROGRAM.BLANK_CHECK    0                            $cfgmem
set_property PROGRAM.ERASE          1                            $cfgmem
set_property PROGRAM.CFG_PROGRAM    1                            $cfgmem
set_property PROGRAM.VERIFY         1                            $cfgmem

refresh_hw_device $dev

set cfgmem [get_property PROGRAM.HW_CFGMEM $dev]
set_property PROGRAM.FILES                     [list [file normalize $mcs]] $cfgmem
set_property PROGRAM.UNUSED_PIN_TERMINATION    pull-none                    $cfgmem

puts "MCS_BURN_START: $mem_part <- [file normalize $mcs]"

if { [catch { program_hw_cfgmem -hw_cfgmem [get_property PROGRAM.HW_CFGMEM $dev] } err] } {
    puts "MCS_FLASH_INIT_FAILED: $mem_part"
    puts "  raw: $err"
    puts "  Even after the device reset the flash did not answer, so this is"
    puts "  now genuinely about the flash device itself:"
    puts "    a) the board's config flash is not the part named above"
    puts "       -> GUI: Add Configuration Memory Device, use AUTO-DETECT,"
    puts "          then edit MEM_PART in this script to the detected name."
    puts "    b) a boot-mode jumper keeps the flash off the SPI bus."
    puts "  Nothing is damaged: program_bit.tcl still works and the flash"
    puts "  content is unchanged."
    close_hw_manager
    exit 2
}

refresh_hw_device $dev

puts "MCS_PROGRAM_OK: $dev ($part) flash=$mem_part <- [file normalize $mcs]"
puts "MCS_NOTE: power-cycle the board to boot from flash (no JTAG needed)."
puts "MCS_NOTE: the FPGA currently runs the SPI bridge; a power cycle (or"
puts "          program_bit.tcl) replaces it with the real design."
close_hw_manager
