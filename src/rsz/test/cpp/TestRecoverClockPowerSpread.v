module top(input clk, input data, output q);
  wire root, early_a, early_b, middle_a, middle_b;
  wire late_a, late_b, useful_a, useful_b;
  wire unused_early, unused_middle, unused_late, unused_useful;
  assign q = data;
  BUF_X16 clock_root (.A(clk), .Z(root));
  BUF_X16 clock_early_anchor (.A(root), .Z(early_a));
  BUF_X16 clock_early_leaf (.A(early_a), .Z(early_b));
  BUF_X16 clock_middle_anchor (.A(root), .Z(middle_a));
  BUF_X16 clock_middle_leaf (.A(middle_a), .Z(middle_b));
  BUF_X16 clock_late_anchor (.A(root), .Z(late_a));
  BUF_X16 clock_late_leaf (.A(late_a), .Z(late_b));
  BUF_X16 clock_useful_anchor (.A(root), .Z(useful_a));
  BUF_X16 clock_useful_leaf (.A(useful_a), .Z(useful_b));
  DFF_X1 sink_early (.CK(early_b), .D(1'b0), .Q(unused_early));
  DFF_X1 sink_middle (.CK(middle_b), .D(1'b0), .Q(unused_middle));
  DFF_X1 sink_late (.CK(late_b), .D(1'b0), .Q(unused_late));
  DFF_X1 sink_useful (.CK(useful_b), .D(1'b0), .Q(unused_useful));
endmodule
