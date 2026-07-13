# Layer-hopped pad connections should be removed as a unit when the
# routed side is blocked.
source "helpers.tcl"

read_lef Nangate45/Nangate45.lef
read_lef nangate_bsg_black_parrot/dummy_pads.lef
read_lef nangate_macros/fakeram45_64x32.lef

read_def nangate_macros/floorplan.def
initialize_floorplan -die_area [ord::get_die_area] \
  -core_area "50 20 185 185" \
  -site FreePDK45_38x28_10R_NP_162NW_34O

set block [ord::get_db_block]

set pad [odb::dbInst_create $block [[ord::get_db] findMaster PADCELL_VDD_H] vdd_pad]
$pad setLocation [ord::microns_to_dbu 10] [ord::microns_to_dbu 50]
$pad setOrient R0
$pad setPlacementStatus FIRM
[$pad findITerm VDD] connect [$block findNet VDD]
[$pad findITerm VSS] connect [$block findNet VSS]

add_global_connection -net VDD -pin_pattern {^VDD$} -power
add_global_connection -net VDD -pin_pattern {^VDDPE$}
add_global_connection -net VDD -pin_pattern {^VDDCE$}
add_global_connection -net VSS -pin_pattern {^VSS$} -ground
add_global_connection -net VSS -pin_pattern {^VSSE$}

set_voltage_domain -power VDD -ground VSS

# This blocks the metal5 landing used by the VDD pad connection.  The
# connection is built by hopping from the pad pin on metal4 to a short metal5
# route.  If the metal5 route is removed, the metal4 pad escape must be
# removed with it.
odb::dbObstruction_create $block [[ord::get_db_tech] findLayer metal5] \
  74000 136000 84000 146000

define_pdn_grid -name "Core"
add_pdn_ring -grid "Core" -layers {metal5 metal6} -widths 5.0 \
  -spacings 2.0 -core_offsets 2.0 -connect_to_pads \
  -connect_to_pad_layers metal4
add_pdn_connect -grid "Core" -layers {metal4 metal6}
add_pdn_connect -grid "Core" -layers {metal5 metal6}

define_pdn_grid -macro -name "Macro" \
  -instances "dcache.data.data_arrays_0.data_arrays_0_ext.mem"
add_pdn_ring -grid "Macro" -layers {metal5 metal6} -widths 1.0 \
  -spacings 1.0 -core_offsets 1.0
add_pdn_connect -grid "Macro" -layers {metal4 metal6}
add_pdn_connect -grid "Macro" -layers {metal5 metal6}

pdngen

proc count_vdd_pad_escape_shapes { } {
  set count 0
  set vdd [[ord::get_db_block] findNet VDD]
  foreach swire [$vdd getSWires] {
    foreach wire [$swire getWires] {
      if { [$wire isVia] } {
        continue
      }
      if { [[$wire getTechLayer] getName] != "metal4" } {
        continue
      }
      if { [$wire getWireShapeType] != "STRIPE" } {
        continue
      }

      set xmin [$wire xMin]
      set ymin [$wire yMin]
      set xmax [$wire xMax]
      set ymax [$wire yMax]
      if { $xmin <= 69800 && $xmax >= 80240 \
        && $ymin <= 141000 && $ymax >= 141000 } {
        incr count
      }
    }
  }
  return $count
}

check "orphaned VDD pad escapes" { count_vdd_pad_escape_shapes } 0

exit_summary
