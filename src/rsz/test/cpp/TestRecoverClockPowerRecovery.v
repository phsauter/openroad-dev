module top(input clk, input data, output q);
  wire root, first, second, third;
  BUF_X16 clock_root (.A(clk), .Z(root));
  BUF_X16 clock_first (.A(root), .Z(first));
  BUF_X16 clock_second (.A(first), .Z(second));
  BUF_X8 clock_third (.A(second), .Z(third));
  DFF_X1 sink (.CK(third), .D(data), .Q(q));
endmodule
