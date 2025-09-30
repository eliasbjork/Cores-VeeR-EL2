// SPDX-License-Identifier: Apache-2.0
// Minimal FP wrapper stub for R4 ops (Zfinx, GPR-backed). Integrator should replace internals with real FPU.
module el2_exu_fp_wrapper
  import el2_pkg::*;
#(
`include "el2_param.vh"
) (
  input  logic        clk,
  input  logic        rst_l,
  /*pragma coverage off*/
  input  logic        scan_mode,
  /*pragma coverage on*/

  input  el2_fp_pkt_t fp,
  input  logic [31:0] rs1,
  input  logic [31:0] rs2,
  input  logic [31:0] rs3,

  output logic [31:0] result,
  output logic        wren
);

  // Stub: no operation. Drive safe defaults.
  assign wren  = 1'b0;
  assign result = 32'b0;

endmodule
