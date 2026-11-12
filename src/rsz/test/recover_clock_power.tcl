# Binding validation and a no-clock no-op, with semantic assertions.
source "helpers.tcl"
read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_verilog cpp/TestRecoverClockPower.v
link_design top
foreach args {{-max_skew -1} {-max_latency NaN} {-max_passes 0} {-max_trials 0} \
  {-max_trials -1} {-max_trials invalid} {-strategy sideways} \
  {-max_checkpoints 2} {-max_seconds 1} \
  {-max_timing_regression 0} {-max_intermediate_timing_regression 0} \
  {-strategy bottom_up_batch -max_timing_regression -1} \
  {-strategy bottom_up_batch -max_timing_regression NaN} \
  {-strategy bottom_up_batch -max_intermediate_timing_regression Inf} \
  {-strategy bottom_up_batch -max_timing_regression 2 -max_intermediate_timing_regression 1} \
  {-strategy bottom_up_batch -max_checkpoints 1} \
  {-strategy bottom_up_batch -max_checkpoints invalid} \
  {-strategy bottom_up_batch -max_seconds 0} \
  {-strategy bottom_up_batch -max_seconds Inf} \
  {-strategy bottom_up_batch -max_seconds NaN} \
  {-buffers NO_SUCH_CELL} {-buffers INV_X1} {-buffers {}} {-unknown 1}} {
  if { ![catch { recover_clock_power {*}$args }] } {
    error "invalid arguments accepted: $args"
  }
}
set before [llength [[ord::get_db_block] getInsts]]
if { [recover_clock_power -max_passes 1 -strategy top_down] != 0 } {
  error "a no-clock design was modified"
}
if { $before != [llength [[ord::get_db_block] getInsts]] } {
  error "instance population changed"
}
puts "pass"
