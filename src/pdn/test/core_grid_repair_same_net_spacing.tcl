# test repair of same-net spacing between grid shapes
source "helpers.tcl"

read_lef Nangate45/Nangate45.lef
read_def core_grid_repair_same_net_spacing.def

set_voltage_domain -power VDD -ground VSS

define_pdn_grid -name "Core"
add_pdn_stripe -layer metal4 -width 3.0 -spacing 3.0 -pitch 200.0 \
  -offset 10.0 -number_of_straps 1 -nets VDD
add_pdn_stripe -layer metal5 -width 0.5 -spacing 1.0 -pitch 200.0 \
  -offset 10.0 -number_of_straps 1 -nets VDD
add_pdn_connect -layers {metal4 metal5}

pdngen -skip_trim

proc count_repaired_shapes { } {
  set count 0
  set vdd [[ord::get_db_block] findNet VDD]
  foreach swire [$vdd getSWires] {
    foreach wire [$swire getWires] {
      if { ![$wire isVia] && [[$wire getTechLayer] getName] == "metal4"
        && [$wire xMin] == 37000 && [$wire yMin] == 62800
        && [$wire xMax] == 43000 && [$wire yMax] == 63800 } {
        incr count
      }
    }
  }
  return $count
}

check "same-net spacing repair" { count_repaired_shapes } 1

exit_summary
