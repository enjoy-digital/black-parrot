/**
 *
 * Name:
 *   bp_cce_alu.v
 *
 * Description:
 *   A simple ALU for the CCE implementing addition, subtraction, logical shift, and bitwise
 *   operations (and, or, xor, negate).
 *
 *   Some microcode operations such as addi and inc are implemented via assembler/softwware
 *   transforms and appropriate source selection. For example, inc is really an add where opd_b_i
 *   is an immediate from the instruction (source select), and the immediate has value 1 (assembler
 *   transform). See bp_cce_inst.vh for SW supported operations.
 *
 *   The arithmetic width is parameterizable, and set based on the microarchitecture design.
 *
 */

module bp_cce_alu
  import bp_cce_pkg::*;
  #(parameter width_p = "inv"
  )
  (input [width_p-1:0]                       opd_a_i
   , input [width_p-1:0]                     opd_b_i
   , input bp_cce_inst_minor_alu_op_e        alu_op_i
   , output logic [width_p-1:0]              res_o
  );

  always_comb begin : alu
    unique case (alu_op_i)
      e_add_op: res_o = opd_a_i + opd_b_i;
      e_sub_op: res_o = opd_a_i - opd_b_i;
      e_lsh_op: res_o = opd_a_i << opd_b_i;
      e_rsh_op: res_o = opd_a_i >> opd_b_i;
      e_and_op: res_o = opd_a_i & opd_b_i;
      e_or_op:  res_o = opd_a_i | opd_b_i;
      e_xor_op: res_o = opd_a_i ^ opd_b_i;
      e_neg_op: res_o = ~opd_a_i;
      default:  res_o = '0;
    endcase
  end
endmodule
