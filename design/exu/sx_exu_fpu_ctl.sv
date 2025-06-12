// wrapper for cvfpu/FPnew

module sx_exu_fpu_ctl
    import el2_pkg::*;
    import fpnew_pkg::*;
#(
`include "el2_param.vh"
)
(
    input logic         clk,          // top level clock
    input logic         rst_l,        // reset
    input fp_pkt_t      fp_p,         // one hot encoded operations
    input logic [31:0]  rs1,          // source operand 1
    input logic [31:0]  rs2,          // source operand 2
    input logic         cancel,       // flush
    input logic         wb_ready,     // writeback ready

    output logic        busy,
    output logic        result_ready,
    output logic [31:0] rd            // result
)

    // FPU configuration
    parameter fpnew_pkg::fpu_features_t       Features       = fpnew_pkg::RV32F,
    parameter fpnew_pkg::fpu_implementation_t Implementation = fpnew_pkg::DEFAULT_NOREGS,
    // DivSqrtSel chooses among PULP, TH32, or THMULTI (see documentation and fpnew_pkg.sv for further details)
    parameter fpnew_pkg::divsqrt_unit_t       DivSqrtSel     = fpnew_pkg::THMULTI,
    parameter type                            TagType        = logic, // this disables tags
    parameter int unsigned                    TrueSIMDClass  = 0,
    parameter int unsigned                    EnableSIMDMask = 0,
    // Do not change
    localparam int unsigned NumLanes     = fpnew_pkg::max_num_lanes(Features.Width, Features.FpFmtMask, Features.EnableVectors),
    localparam type         MaskType     = logic [NumLanes-1:0],
    localparam int unsigned WIDTH        = Features.Width,
    localparam int unsigned NUM_OPERANDS = 2

    // TODO
    fpnew_pkg::operation_e  op_i;
    logic                   op_mod_i;
    fpnew_pkg::fp_format_e  src_fmt_i   = fpnew_pkg::FP32;  // formats hardwired for now
    fpnew_pkg::fp_format_e  dst_fmt_i   = fpnew_pkg::FP32;
    fpnew_pkg::int_format_e int_fmt_i   = fpnew_pkg::INT32;
    TagType                 tag_i       = 'b0;              // disabled
    MaskType                simd_mask_i = 'b0;              // disabled
    logic                   in_valid_i;
    logic                   in_ready_o;
    logic                   flush_i;
    logic [31:0]            result_o;
    fpnew_pkg::status_t     status_o;
    TagType                 tag_o       = 'b0;              // disabled
    logic                   out_valid_o;
    logic                   out_ready_i;
    logic                   busy_o;
    
    assign out_ready_i = wb_ready;
    assign flush_i = cancel;
    assign result_ready = out_valid_o & wb_ready; // handshake
    assign busy = busy_o | ~in_ready_o;

    always_comb begin : op_def
        assign in_valid_i = 'b1;
        case (fp_p)
            // fadd
            'b100: begin
                assign op_i = fpnew_pkg::operation_e::ADD;
                assign op_mod_i = 'b0;
            end
            // fmul
            'b010: begin
                assign op_i = fpnew_pkg::operation_e::MUL;
                assign op_mod_i = 'b0;
            end
            // fdiv
            'b001: begin
                assign op_i = fpnew_pkg::operation_e::DIV;
                assign op_mod_i = 'b0;
            end
            default: assign in_valid_i = 'b0;
        endcase
    end

    // FPnew instance
    fpnew_pkg::fpnew_top #(
        .Features       (Features),
        .Implementation (Implementation),
        .DivSqrtSel     (DivSqrtSel),
        .TagType        (TagType),
        .TrueSIMDClass  (TrueSIMDClass),
        .EnableSIMDMask (EnableSIMDMask),
        .NumLanes       (NumLanes),
        .MaskType       (MaskType),
        .WIDTH          (WIDTH),
        .NUM_OPERANDS   (NUM_OPERANDS)
    ) fpu (
        .clk_i          (clk),
        .rst_ni         (rst_l),
        .operands_i     ({rs1, rs2}),
        .rnd_mode_i     (fpnew_pkg::roundmode_e::RTZ), // might have to un-hardcode later?
        .op_i           (op_i),
        .op_mod_i       (op_mod_i),
        .src_fmt_i      (src_fmt_i),
        .dst_fmt_i      (dst_fmt_i),
        .int_fmt_i      (int_fmt_i),
        .vectorial_op_i ('b0),       // todo?
        .tag_i          (tag_i),
        .simd_mask_i    (simd_mask_i),
        .in_valid_i     (in_valid_i),
        .in_ready_o     (in_ready_o),
        .flush_i        (flush_i),
        .result_o       (result_o),
        .status_o       (status_o),
        .tag_o          (tag_o),
        .out_valid_o    (out_valid_o),
        .out_ready_i    (out_ready_i),
        .busy_o         (busy_o),
    );

endmodule
