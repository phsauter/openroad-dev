# Verify pdngen forwards the opt-in via-conflict repair flag.
source "helpers.tcl"

read_lef Nangate45/Nangate45_tech.lef
read_lef Nangate45/Nangate45_stdcell.lef
read_def nangate_gcd/floorplan.def

rename pdn::run_pdngen pdn::run_pdngen_original
proc pdn::run_pdngen { trim add_pins report_file repair_via_conflicts } {
  set ::repair_via_conflicts $repair_via_conflicts
}
pdngen
check "via repair defaults off" { expr { !$::repair_via_conflicts } } 1
pdngen -repair_via_conflicts
check "via repair can be enabled" { expr { $::repair_via_conflicts } } 1
rename pdn::run_pdngen {}
rename pdn::run_pdngen_original pdn::run_pdngen
exit_summary
