module top(input clk, input data, output q1, output q2, output q3);
  wire root, late_a, late_b, late_c, mid_a, mid_b, delayed;
  BUF_X16 clock_root (.A(clk), .Z(root));
  BUF_X16 clock_late_a (.A(root), .Z(late_a));
  BUF_X16 clock_late_b (.A(late_a), .Z(late_b));
  BUF_X16 clock_late_c (.A(late_b), .Z(late_c));
  BUF_X16 clock_mid_anchor (.A(root), .Z(mid_a));
  BUF_X16 clock_mid_leaf (.A(mid_a), .Z(mid_b));
  BUF_X2 data_delay (.A(data), .Z(delayed));
  DFF_X1 sink_early (.CK(root), .D(data), .Q(q1));
  DFF_X1 sink_late (.CK(late_c), .D(data), .Q(q2));
  DFF_X1 sink_mid (.CK(mid_b), .D(delayed), .Q(q3));
endmodule
