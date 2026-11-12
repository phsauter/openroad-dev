module top(input clk, input data, output q1, output q2);
  wire root, branch;
  BUF_X16 clock_root (.A(clk), .Z(root));
  BUF_X16 clock_leaf (.A(root), .Z(branch));
  DFF_X1 sink1 (.CK(branch), .D(data), .Q(q1));
  DFF_X1 sink2 (.CK(root), .D(data), .Q(q2));
endmodule
