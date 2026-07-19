# test spacing between nearby via stacks
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

define_pdn_grid -name grid -pins "met4 met5"
add_pdn_stripe -grid grid -layer met4 -width 1.6 -pitch 22.44 \
  -offset 11.44 -starts_with POWER
add_pdn_stripe -grid grid -layer met5 -width 1.6 -pitch 22.44 \
  -offset 11.22 -starts_with POWER
add_pdn_stripe -grid grid -layer met1 -width 0.48 -followpins
add_pdn_connect -grid grid -layers "met1 met4"
add_pdn_connect -grid grid -layers "met4 met5"

define_pdn_grid -name nested
add_pdn_stripe -grid nested -layer met4 -width 1.6 -pitch 22.44 \
  -offset 11.44 -starts_with POWER
add_pdn_stripe -grid nested -layer met3 -width 2.4 -pitch 22.44 \
  -offset 10.2 -starts_with POWER
add_pdn_connect -grid nested -layers "met3 met4" -fixed_vias M3M4_BIG

define_pdn_grid -name nested23
add_pdn_stripe -grid nested23 -layer met2 -width 1.6 -pitch 22.44 \
  -offset 11.44 -starts_with POWER
add_pdn_stripe -grid nested23 -layer met3 -width 2.4 -pitch 22.44 \
  -offset 10.2 -starts_with POWER
add_pdn_connect -grid nested23 -layers "met2 met3" -fixed_vias M2M3_BIG

pdn::check_setup
pdn::build_grids
[[ord::get_db_tech] findLayer via2] setSpacing 1000
[[ord::get_db_tech] findLayer via3] setSpacing 1000
pdn::write_to_db true ""

set block [ord::get_db_block]
set stale_bridges 0
set lower_nested_vias 0
set upper_nested_vias 0
set lower_vias 0
set middle_vias 0
set upper_vias 0
foreach swire [[$block findNet VDD] getSWires] {
  foreach wire [$swire getWires] {
    if { ![$wire isVia] } {
      if { [[$wire getTechLayer] getName] == "met3" \
        && [$wire getWireShapeType] == "DRCFILL" \
        && [$wire xMin] == 83515 && [$wire yMin] == 67160 \
        && [$wire xMax] == 85045 && [$wire yMax] == 67835 } {
        incr stale_bridges
      }
      continue
    }
    lassign [$wire getViaXY] x y
    if { $x == 84280 && ($y == 65960 || $y == 68000) } {
      set layers {}
      foreach box [$wire getViaBoxes 0] {
        if { [[$box getTechLayer] getType] == "ROUTING" } {
          lappend layers [[$box getTechLayer] getName]
        }
      }
      set layers [lsort -unique $layers]
      if { $y == 65960 && $layers == "met2 met3" } {
        incr lower_nested_vias
      } elseif { $y == 65960 && $layers == "met3 met4" } {
        incr upper_nested_vias
      } elseif { $y == 68000 && $layers == "met1 met2" } {
        incr lower_vias
      } elseif { $y == 68000 && $layers == "met2 met3" } {
        incr middle_vias
      } elseif { $y == 68000 && $layers == "met3 met4" } {
        incr upper_vias
      }
    }
  }
}

check "stale transition bridge" { set stale_bridges } 0
check "larger lower transition" { set lower_nested_vias } 1
check "larger upper transition" { set upper_nested_vias } 1
check "lower via transition" { set lower_vias } 1
check "removed middle transition" { set middle_vias } 0
check "removed upper transition" { set upper_vias } 0
exit_summary
