# pad connections over pads should survive trim
source "helpers.tcl"

read_lef Nangate45/Nangate45.lef
read_lef nangate_bsg_black_parrot/dummy_pads_short.lef
read_lef nangate_macros/fakeram45_64x32.lef

read_def nangate_macros/floorplan.def
initialize_floorplan -die_area "0 0 400 400" \
  -core_area "50 20 350 220" \
  -site FreePDK45_38x28_10R_NP_162NW_34O

set block [ord::get_db_block]

set pad [odb::dbInst_create $block [[ord::get_db] findMaster PADCELL_VDD_V] vdd_pad]
$pad setLocation [ord::microns_to_dbu 100] [ord::microns_to_dbu 240]
$pad setOrient R0
$pad setPlacementStatus FIRM
[$pad findITerm VDD] connect [$block findNet VDD]
[$pad findITerm VSS] connect [$block findNet VSS]

add_global_connection -net VDD -pin_pattern {^VDD$} -power
add_global_connection -net VSS -pin_pattern {^VSS$} -ground

set_voltage_domain -power VDD -ground VSS

define_pdn_grid -name "Core" -starts_with "POWER"
add_pdn_ring -grid "Core" -layers {metal10 metal9} -widths 5.0 \
  -spacings 2.0 -core_offsets 2.0 -connect_to_pads
add_pdn_connect -grid "Core" -layers {metal9 metal10}

pdngen

set def_file [make_result_file pads_connect_over_pads_trim.def]
write_def $def_file
diff_files pads_connect_over_pads_trim.defok $def_file
