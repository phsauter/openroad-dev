module top(input clk, input data, output q1, output q2, output q3, output q4, output q5);
  wire root, bad, good, delayed;
  BUF_X16 clock_root (.A(clk), .Z(root));
  BUF_X16 clock_bad (.A(root), .Z(bad));
  BUF_X16 clock_good (.A(root), .Z(good));
  BUF_X1 data_delay (.A(data), .Z(delayed));
  DFF_X1 sink1 (.CK(bad), .D(data), .Q(q1));
  DFF_X1 sink2 (.CK(bad), .D(data), .Q(q2));
  DFF_X1 sink3 (.CK(bad), .D(data), .Q(q3));
  DFF_X1 sink4 (.CK(bad), .D(data), .Q(q4));
  DFF_X1 sink5 (.CK(good), .D(delayed), .Q(q5));
endmodule
