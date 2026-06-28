# pad connections should stop at the core ring
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

set def_file [make_result_file pads_connect_no_macro_repair.def]
write_def $def_file
diff_files pads_connect_no_macro_repair.defok $def_file
