# Check for repair channels with straps in same direction and reject
# this avoids the infinate loop reported in: https://github.com/The-OpenROAD-Project/OpenROAD/issues/5905
source "helpers.tcl"

read_lef repair_channel_inf_loop/tech_macro.lef
read_def repair_channel_inf_loop/floorplan.def

set_voltage_domain -power VDD -ground GND

define_pdn_grid -name "Core"

add_pdn_stripe -followpins -layer M1 -width 2.630

add_pdn_stripe -layer M3 -width 11.280 -pitch 225.600 -spacing 16.920 -offset 113.550
add_pdn_stripe -layer M4 -width 11.280 -pitch 526.400 -spacing 16.920 -offset 375.625

add_pdn_connect -layers {M3 M4}
add_pdn_connect -layers {M1 M3}

catch { pdngen } err
puts $err

proc find_marker_category { parent name } {
  foreach category [$parent getMarkerCategories] {
    if { [$category getName] == $name } {
      return $category
    }
  }
  return ""
}

set block [ord::get_db_block]
set pdn_category [find_marker_category $block "PDN"]
set repair_category ""
if { $pdn_category != "" } {
  set repair_category [find_marker_category $pdn_category "Repair channels"]
}

check "repair channel markers" {
  expr { $repair_category != "" ? [llength [$repair_category getMarkers]] : 0 }
} 2
check "partial VDD grid" { expr { [llength [[$block findNet VDD] getSWires]] > 0 } } 1
check "partial GND grid" { expr { [llength [[$block findNet GND] getSWires]] > 0 } } 1

exit_summary
