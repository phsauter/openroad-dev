module top(input clk, input data, output q1, output q2);
  wire root, left, right;
  BUF_X16 clock_root (.A(clk), .Z(root));
  BUF_X16 clock_left (.A(root), .Z(left));
  BUF_X16 clock_right (.A(root), .Z(right));
  DFF_X1 sink1 (.CK(left), .D(data), .Q(q1));
  DFF_X1 sink2 (.CK(right), .D(data), .Q(q2));
endmodule
