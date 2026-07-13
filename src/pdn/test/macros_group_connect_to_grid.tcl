# test grouped macro ring and pin repair from a core grid
source "helpers.tcl"

read_lef Nangate45/Nangate45.lef
read_lef nangate_macros/fakeram45_64x32.lef

read_def nangate_macros/floorplan.def

add_global_connection -net VDD -pin_pattern {^VDD$} -power
add_global_connection -net VDD -pin_pattern {^VDDPE$}
add_global_connection -net VDD -pin_pattern {^VDDCE$}
add_global_connection -net VSS -pin_pattern {^VSS$} -ground
add_global_connection -net VSS -pin_pattern {^VSSE$}

set_voltage_domain -power VDD -ground VSS

define_pdn_grid -name "Core"
add_pdn_ring -grid "Core" -layers {metal5 metal6} -widths 2.0 \
  -spacings 2.0 -core_offsets 2.0
add_pdn_stripe -grid "Core" -layer metal4 -width 0.48 -spacing 4.0 \
  -pitch 200.0 -offset 100.0 -number_of_straps 1 -nets {VDD} \
  -extend_to_core_ring
add_pdn_stripe -grid "Core" -layer metal5 -width 0.48 -spacing 4.0 \
  -pitch 200.0 -offset 50.0 -number_of_straps 1 -nets {VDD} \
  -extend_to_core_ring
add_pdn_connect -grid "Core" -layers {metal4 metal5}
add_pdn_connect -grid "Core" -layers {metal5 metal6}

define_pdn_grid -macro -name "srams" -instances {
  dcache.data.data_arrays_0.data_arrays_0_ext.mem
  frontend.icache.data_arrays_0.data_arrays_0_0_ext.mem
} -group
add_pdn_ring -grid "srams" -layers {metal3 metal4} -widths 1.0 \
  -spacings 1.0 -core_offsets 1.0
add_pdn_connect -grid "srams" -layers {metal3 metal4}
add_pdn_connect -grid "srams" -layers {metal4 metal5}

pdngen

proc count_vias_at_x { net_name lower_layer upper_layer x } {
  set count 0
  set net [[ord::get_db_block] findNet $net_name]
  foreach swire [$net getSWires] {
    foreach wire [$swire getWires] {
      if { ![$wire isVia] || [lindex [$wire getViaXY] 0] != $x } {
        continue
      }
      set has_lower 0
      set has_upper 0
      foreach box [$wire getViaBoxes 0] {
        set layer_name [[$box getTechLayer] getName]
        if { $layer_name == $lower_layer } {
          set has_lower 1
        } elseif { $layer_name == $upper_layer } {
          set has_upper 1
        }
      }
      if { $has_lower && $has_upper } {
        incr count
      }
    }
  }
  return $count
}

proc count_vias_in_box { net_name lower_layer upper_layer x_min y_min x_max y_max } {
  set count 0
  set net [[ord::get_db_block] findNet $net_name]
  foreach swire [$net getSWires] {
    foreach wire [$swire getWires] {
      if { ![$wire isVia] } {
        continue
      }
      set via_xy [$wire getViaXY]
      set x [lindex $via_xy 0]
      set y [lindex $via_xy 1]
      if { $x < $x_min || $x > $x_max || $y < $y_min || $y > $y_max } {
        continue
      }
      set has_lower 0
      set has_upper 0
      foreach box [$wire getViaBoxes 0] {
        set layer_name [[$box getTechLayer] getName]
        if { $layer_name == $lower_layer } {
          set has_lower 1
        } elseif { $layer_name == $upper_layer } {
          set has_upper 1
        }
      }
      if { $has_lower && $has_upper } {
        incr count
      }
    }
  }
  return $count
}

set dbu [[ord::get_db_block] getDbUnitsPerMicron]
set strap_x [expr { round(120.14 * $dbu) }]
check "VDD strap vias to grouped macro ring" \
  { count_vias_at_x VDD metal3 metal4 $strap_x } 2

set sram [[ord::get_db_block] findInst dcache.data.data_arrays_0.data_arrays_0_ext.mem]
set bbox [$sram getBBox]
check "VDD core stripe vias to macro pins" {
  expr [count_vias_in_box VDD metal4 metal5 \
    [$bbox xMin] [$bbox yMin] [$bbox xMax] [$bbox yMax]] > 0
} 1

exit_summary
