// SPDX-License-Identifier: Apache-2.0
// Minimal FP wrapper stub for R4 ops (Zfinx, GPR-backed). Integrator should replace internals with real FPU.
module el2_exu_fpu_ctl
  import el2_pkg::*;
  import fpnew_pkg::*;
#(
    `include "el2_param.vh"
) (
    input logic clk,
    input logic rst_l,
    /*pragma coverage off*/
    input logic scan_mode,
    /*pragma coverage on*/

    input el2_fpu_pkt_t        fpu_p,
    input logic                cancel,
    input logic         [31:0] rs1,
    input logic         [31:0] rs2,
    input logic         [31:0] rs3,

    output logic [31:0] x_result,
    output logic [31:0] d_result,
    output logic        div_wren,
    output logic        d_result_valid
);

  logic [2:0][31:0] fpu_operands;

  operation_e fpu_op;

  fp_format_e fp_fmt;
  int_format_e int_fmt;

  logic [31:0] result;

  logic valid;

  logic valid_div;

  logic status, busy, in_ready;
  logic [2:0] rm;

  assign rm = (!&fpu_p.rm[2:0]) ? fpu_p.rm[2:0] : 3'b000;

  assign fp_fmt = FP32;
  assign int_fmt = INT32;

  assign d_result = result;
  assign div_wren = valid_div & valid;
  assign d_result_valid = valid;


//encoding from onehot to enum
  assign fpu_op[0] = fpu_p.fnmsub | fpu_p.mul | fpu_p.sqrt | fpu_p.minmax | fpu_p.classify | fpu_p.f2i | fpu_p.cpkab | fpu_p.adds;
  assign fpu_op[1] = fpu_p.add | fpu_p.mul | fpu_p.sgnj | fpu_p.minmax | fpu_p.f2f | fpu_p.f2i | fpu_p.cpkcd | fpu_p.adds;
  assign fpu_op[2] = fpu_p.div | fpu_p.sqrt | fpu_p.sgnj | fpu_p.minmax | fpu_p.i2f | fpu_p.cpkab | fpu_p.cpkcd | fpu_p.adds;
  assign fpu_op[3] = fpu_p.cmp | fpu_p.classify | fpu_p.f2f | fpu_p.f2i | fpu_p.i2f | fpu_p.cpkab | fpu_p.cpkcd | fpu_p.adds;

  always_comb begin
  unique case (fpu_op)
    FMADD, FNMSUB: begin
      fpu_operands[0] = rs1;
      fpu_operands[1] = rs2;
      fpu_operands[2] = rs3;
    end
    ADD, ADDS: begin
      fpu_operands[0] = '0;
      fpu_operands[1] = rs1;
      fpu_operands[2] = rs2;
    end
    SQRT, F2F, F2I, I2F, CLASSIFY: begin
      fpu_operands[0] = rs1;
      fpu_operands[1] = '0;
      fpu_operands[2] = '0;
    end
    default: begin
      fpu_operands[0] = rs1;
      fpu_operands[1] = rs2;
      fpu_operands[2] = '0;
    end
  endcase
  end


rvdffe #(32) result_ff    (.*, .clk(clk),  .din(result),   .dout(x_result),   .en(valid & ~valid_div));


  // FPU instance
  fpnew_top #(
      .Features(RV32F),
      .Implementation(DEFAULT_NOREGS),
      .TagType(logic),
      .DivSqrtSel(PULP)
  ) fpu (
      .clk_i(clk),
      .rst_ni(rst_l),
      .operands_i(fpu_operands),
      .rnd_mode_i(rm),
      .op_i(fpu_op),
      .op_mod_i(fpu_p.op_mod),
      .src_fmt_i(fp_fmt),
      .dst_fmt_i(fp_fmt),
      .int_fmt_i(int_fmt),
      .vectorial_op_i('0),
      .tag_i(fpu_p.fpu_div),
      .simd_mask_i('0),
      .in_valid_i(fpu_p.valid),
      .out_ready_i(1'b1),
      .flush_i(cancel),
      .result_o(result),
      .tag_o(valid_div),
      .out_valid_o(valid),
      .status_o(status),
      .busy_o(busy),
      .in_ready_o(in_ready)
  );

endmodule
