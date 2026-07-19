# test bridge to a retained via array
source "helpers.tcl"

read_lef sky130hd/sky130hd.tlef
read_lef core_grid_via_spacing.lef
read_lef sky130hd/sky130_fd_sc_hd_merged.lef
read_def sky130_gcd/floorplan.def

add_global_connection -net VDD -pin_pattern VDD -power
add_global_connection -net VDD -pin_pattern VPWR
add_global_connection -net VSS -pin_pattern VSS -ground
add_global_connection -net VSS -pin_pattern VGND

set_voltage_domain -power VDD -ground VSS

define_pdn_grid -name grid
add_pdn_stripe -grid grid -layer met4 -width 2.0 -pitch 22.44 \
  -offset 11.44 -starts_with POWER
add_pdn_stripe -grid grid -layer met3 -width 0.4 -pitch 22.44 \
  -offset 12.24 -starts_with POWER
add_pdn_stripe -grid grid -layer met3 -width 2.4 -pitch 22.44 \
  -offset 10.2 -starts_with POWER
add_pdn_connect -grid grid -layers "met3 met4" \
  -fixed_vias "M3M4_SMALL M3M4_BIG"

pdn::check_setup
pdn::build_grids
[[ord::get_db_tech] findLayer via3] setSpacing 1000
pdn::write_to_db true ""

set bridge 0
set big_via 0
set small_via 0
set net [[ord::get_db_block] findNet VDD]
foreach swire [$net getSWires] {
  foreach wire [$swire getWires] {
    if { ![$wire isVia] } {
      if { [[$wire getTechLayer] getName] == "met3" \
        && [$wire getWireShapeType] == "DRCFILL" \
        && [$wire xMin] == 83480 && [$wire yMin] == 67160 \
        && [$wire xMax] == 85080 && [$wire yMax] == 67800 } {
        incr bridge
      }
      continue
    }
    lassign [$wire getViaXY] x y
    if { $x == 84280 && $y == 65960 } {
      incr big_via
    } elseif { $x == 84280 && $y == 68000 } {
      incr small_via
    }
  }
}

check "bridge" { set bridge } 1
check "larger transition" { set big_via } 1
check "removed transition" { set small_via } 0
exit_summary
