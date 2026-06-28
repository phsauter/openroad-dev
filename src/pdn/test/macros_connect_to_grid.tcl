# test for connecting core straps to macro rings
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
  -pitch 200.0 -offset 145.35 -number_of_straps 1 -nets {VDD} \
  -extend_to_core_ring
add_pdn_connect -grid "Core" -layers {metal3 metal4}
add_pdn_connect -grid "Core" -layers {metal4 metal5}
add_pdn_connect -grid "Core" -layers {metal5 metal6}

define_pdn_grid -macro -name "sram" \
  -instances "dcache.data.data_arrays_0.data_arrays_0_ext.mem"
add_pdn_ring -grid "sram" -layers {metal3 metal4} -widths 1.0 \
  -spacings 1.0 -core_offsets 1.0
add_pdn_connect -grid "sram" -layers {metal3 metal4}

pdngen

set def_file [make_result_file macros_connect_to_grid.def]
write_def $def_file
diff_files macros_connect_to_grid.defok $def_file
