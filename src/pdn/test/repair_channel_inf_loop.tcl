# Retain partial output when parallel straps leave unrepairable channels.

set test_name repair_channel_inf_loop

read_lef repair_channel_inf_loop/tech_macro.lef
read_def repair_channel_inf_loop/floorplan.def

source "helpers.tcl"

set_voltage_domain -power VDD -ground GND

define_pdn_grid -name "Core"

add_pdn_stripe -followpins -layer M1 -width 2.630

add_pdn_stripe -layer M3 -width 11.280 -pitch 225.600 -spacing 16.920 -offset 113.550
add_pdn_stripe -layer M4 -width 11.280 -pitch 526.400 -spacing 16.920 -offset 375.625

add_pdn_connect -layers {M3 M4}
add_pdn_connect -layers {M1 M3}

set report [make_result_file ${test_name}.rpt]
catch { pdngen -failed_via_report $report } err
puts $err

check "original build error" { set err } PDN-0179
set block [ord::get_db_block]
set category [$block findMarkerCategory PDN]
check "repair channel markers" {
  [$category findMarkerCategory "Repair channels"] getMarkerCount
} 2
foreach net {VDD GND} {
  check "partial $net grid" {
    expr { [llength [[$block findNet $net] getSWires]] > 0 }
  } 1
}
set stream [open $report]
set markers [read $stream]
close $stream
check "repair channel report" { string match {*Repair channels*} $markers } 1
exit_summary
