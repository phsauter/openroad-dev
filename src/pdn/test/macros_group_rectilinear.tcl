# test for rectilinear grouped macro grids
source "helpers.tcl"

read_lef Nangate45/Nangate45.lef
read_lef Nangate45/fakeram45_64x32.lef

read_def nangate_macros/floorplan.def

set block [ord::get_db_block]
set sram_master [[ord::get_db] findMaster fakeram45_64x32]

foreach inst [$block getInsts] {
  if { [[$inst getMaster] getName] != "fakeram45_64x32" } {
    odb::dbInst_destroy $inst
  }
}

proc place_sram { name x y } {
  set inst [[ord::get_db_block] findInst $name]
  $inst setPlacementStatus PLACED
  $inst setLocation $x $y
  $inst setOrient R0
  $inst setPlacementStatus LOCKED
  return $inst
}

set sram0 [place_sram dcache.data.data_arrays_0.data_arrays_0_ext.mem 80000 60000]
set sram1 [place_sram frontend.icache.data_arrays_0.data_arrays_0_0_ext.mem 139090 60000]
set sram2 [odb::dbInst_create $block $sram_master extra_sram]
$sram2 setLocation 80000 125800
$sram2 setOrient R0
$sram2 setPlacementStatus LOCKED

add_global_connection -net VDD -pin_pattern {^VDD$} -power
add_global_connection -net VDD -pin_pattern {^VDDPE$}
add_global_connection -net VDD -pin_pattern {^VDDCE$}
add_global_connection -net VSS -pin_pattern {^VSS$} -ground
add_global_connection -net VSS -pin_pattern {^VSSE$}

set_voltage_domain -power VDD -ground VSS

define_pdn_grid -macro -name "srams" -instances {
  dcache.data.data_arrays_0.data_arrays_0_ext.mem
  frontend.icache.data_arrays_0.data_arrays_0_0_ext.mem
  extra_sram
} -group -halo 10

cut_rows -endcap_master TAPCELL_X1

add_pdn_ring -grid "srams" -layers {metal5 metal6} -widths 2.0 \
  -spacings 2.0 -core_offsets 2.0
add_pdn_connect -grid "srams" -layers {metal5 metal6}

pdngen

proc count_ring_shapes { layer_name x_min y_min x_max y_max } {
  set count 0
  set block [ord::get_db_block]
  foreach net_name {VDD VSS} {
    foreach swire [[$block findNet $net_name] getSWires] {
      foreach wire [$swire getWires] {
        if { [$wire isVia] } {
          continue
        }
        if { [$wire getWireShapeType] != "RING" } {
          continue
        }
        if { [[$wire getTechLayer] getName] != $layer_name } {
          continue
        }
        if { [$wire xMax] > $x_min && [$wire xMin] < $x_max
          && [$wire yMax] > $y_min && [$wire yMin] < $y_max } {
          incr count
        }
      }
    }
  }
  return $count
}

proc count_rows_covering { x y } {
  set count 0
  foreach row [[ord::get_db_block] getRows] {
    set bbox [$row getBBox]
    if { [$bbox xMin] <= $x && $x < [$bbox xMax]
      && [$bbox yMin] <= $y && $y < [$bbox yMax] } {
      incr count
    }
  }
  return $count
}

check "concave vertical ring" {
  expr [count_ring_shapes metal6 200000 210000 216000 260000] > 0
} 1
check "concave horizontal ring" {
  expr [count_ring_shapes metal5 220000 193000 260000 210000] > 0
} 1
check "no rectangular top ring across missing corner" {
  count_ring_shapes metal5 220000 260000 260000 280000
} 0
check "row remains in missing halo corner" {
  expr [count_rows_covering 240000 240000] > 0
} 1

exit_summary
