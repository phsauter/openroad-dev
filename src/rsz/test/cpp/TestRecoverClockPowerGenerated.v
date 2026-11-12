module top(input clk, input data, output q1, output q2, output q3, output q4);
  wire root, late_a, late_b, late_c, gen_a, gen_b, generated;
  wire generated_a, generated_b, useful_a, useful_b;
  BUF_X16 clock_root (.A(clk), .Z(root));
  BUF_X16 clock_late_a (.A(root), .Z(late_a));
  BUF_X16 clock_late_b (.A(late_a), .Z(late_b));
  BUF_X16 clock_late_c (.A(late_b), .Z(late_c));
  BUF_X16 clock_gen_anchor (.A(root), .Z(gen_a));
  BUF_X16 clock_gen_leaf (.A(gen_a), .Z(gen_b));
  DFF_X1 generator (.CK(gen_b), .D(data), .Q(generated));
  BUF_X16 clock_generated_anchor (.A(generated), .Z(generated_a));
  BUF_X16 clock_generated_leaf (.A(generated_a), .Z(generated_b));
  BUF_X16 clock_useful_anchor (.A(root), .Z(useful_a));
  BUF_X16 clock_useful_leaf (.A(useful_a), .Z(useful_b));
  DFF_X1 sink_early (.CK(root), .D(data), .Q(q1));
  DFF_X1 sink_late (.CK(late_c), .D(data), .Q(q2));
  DFF_X1 sink_generated (.CK(generated_b), .D(data), .Q(q3));
  DFF_X1 sink_useful (.CK(useful_b), .D(data), .Q(q4));
endmodule
