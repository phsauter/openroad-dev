module top(input clk, input data, output q);
  wire root, first, second;
  BUF_X16 clock_root (.A(clk), .Z(root));
  BUF_X16 clock_first (.A(root), .Z(first));
  BUF_X16 clock_second (.A(first), .Z(second));
  DFF_X1 sink (.CK(second), .D(data), .Q(q));
endmodule
