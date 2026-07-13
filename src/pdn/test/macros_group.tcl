# test for define_pdn_grid -macro -group
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

define_pdn_grid -macro -name "srams" -instances {
  dcache.data.data_arrays_0.data_arrays_0_ext.mem
  frontend.icache.data_arrays_0.data_arrays_0_0_ext.mem
} -group

add_pdn_ring -grid "srams" -layers {metal5 metal6} -widths 2.0 -spacings 2.0 -core_offsets 2.0
add_pdn_connect -grid "srams" -layers {metal5 metal6}

pdngen

set def_file [make_result_file macros_group.def]
write_def $def_file
diff_files macros_group.defok $def_file
