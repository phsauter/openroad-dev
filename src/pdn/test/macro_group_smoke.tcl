# The macro-grid binding accepts a group of fixed instances.
source "helpers.tcl"
read_lef Nangate45/Nangate45.lef
read_lef nangate_macros/fakeram45_64x32.lef
read_def nangate_macros/floorplan.def
add_global_connection -net VDD -pin_pattern {^VDD$} -power
add_global_connection -net VSS -pin_pattern {^VSS$} -ground
set_voltage_domain -power VDD -ground VSS
define_pdn_grid -macro -name group -instances {.*mem} -group
check "group created" {pdn::has_grid group} 1
exit_summary
