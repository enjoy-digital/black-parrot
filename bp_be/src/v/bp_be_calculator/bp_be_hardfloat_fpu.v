
module bp_be_hardfloat_fpu
 import bp_be_hardfloat_pkg::*;
 #(parameter dword_width_p      = 64
   , parameter word_width_p     = 32
   , parameter sp_exp_width_lp  = 8
   , parameter sp_sig_width_lp  = 24
   , parameter sp_width_lp      = sp_exp_width_lp+sp_sig_width_lp
   , parameter dp_exp_width_lp  = 11
   , parameter dp_sig_width_lp  = 53
   , parameter dp_width_lp      = dp_exp_width_lp+dp_sig_width_lp

   , parameter sp_rec_width_lp = sp_exp_width_lp+sp_sig_width_lp+1
   , parameter dp_rec_width_lp = dp_exp_width_lp+dp_sig_width_lp+1
   )
  (input                        clk_i
   , input                      reset_i

   , input [dword_width_p-1:0]  a_i
   , input [dword_width_p-1:0]  b_i
   , input [dword_width_p-1:0]  c_i

   , input bsg_fp_op_e          op_i
   , input bsg_fp_pr_e          ipr_i
   , input bsg_fp_pr_e          opr_i
   , input bsg_fp_rm_e          rm_i

   , output [dword_width_p-1:0] o
   , output bsg_fp_eflags_s     eflags_o
   );

  // The control bits control tininess, which is fixed in RISC-V
  wire [`floatControlWidth-1:0] control_li = `flControl_default;
 
  // Recode all three inputs from FP
  //   We use a pseudo foreach loop to save verbosity
  //   We also convert from 32 bit inputs to 64 bit recoded inputs. 
  //     This double rounding behavior was formally proved correct in
  //     "Innocuous Double Rounding of Basic Arithmetic Operations" by Pierre Roux
  logic [dp_rec_width_lp-1:0] a_rec_li, b_rec_li, c_rec_li;
  logic [2:0][sp_width_lp-1:0] in_sp_li;
  logic [2:0][dp_width_lp-1:0] in_dp_li;
  logic [2:0][dp_rec_width_lp-1:0] in_rec_li;

  assign in_sp_li = {c_i[0+:sp_width_lp], b_i[0+:sp_width_lp], a_i[0+:sp_width_lp]};
  assign in_dp_li = {c_i, b_i, a_i};
  for (genvar i = 0; i < 3; i++)
    begin : in_rec
      logic [sp_rec_width_lp-1:0] in_sp_rec_li;
      fNToRecFN
       #(.expWidth(sp_exp_width_lp)
         ,.sigWidth(sp_sig_width_lp)
         )
       in32_rec
        (.in(in_sp_li[i])
         ,.out(in_sp_rec_li)
         );

      logic [dp_rec_width_lp-1:0] in_dp_rec_li;
      fNToRecFN
       #(.expWidth(dp_exp_width_lp)
         ,.sigWidth(dp_sig_width_lp)
         )
       in64_rec
        (.in(in_dp_li[i])
         ,.out(in_dp_rec_li)
         );

      logic [dp_rec_width_lp-1:0] in_sp2dp_rec_li;
      recFNToRecFN
       #(.inExpWidth(sp_exp_width_lp)
         ,.inSigWidth(sp_sig_width_lp)
         ,.outExpWidth(dp_exp_width_lp)
         ,.outSigWidth(dp_sig_width_lp)
         )
       rec_sp_to_dp
        (.control(control_li)
         ,.in(in_sp_rec_li)
         ,.roundingMode(rm_i)
         ,.out(in_sp2dp_rec_li)
         // Exception flags should be raised by downstream operations
         ,.exceptionFlags()
         );

      assign in_rec_li[i] = (ipr_i == e_pr_double) ? in_dp_rec_li : in_sp2dp_rec_li;
    end
  assign {c_rec_li, b_rec_li, a_rec_li} = in_rec_li;
 
  // FADD/FSUB
  //
  logic [dp_rec_width_lp-1:0] faddsub_lo;
  bsg_fp_eflags_s faddsub_eflags_lo;

  wire is_fsub_li = (op_i == e_op_fsub);
  addRecFN
   #(.expWidth(dp_exp_width_lp)
     ,.sigWidth(dp_sig_width_lp)
     )
   faddsub
    (.control(control_li)
     ,.subOp(is_fsub_li)
     ,.a(a_rec_li)
     ,.b(b_rec_li)
     ,.roundingMode(rm_i)
     ,.out(faddsub_lo)
     ,.exceptionFlags(faddsub_eflags_lo)
     );

  // FMUL
  //
  logic [dp_rec_width_lp-1:0] fmul_lo;
  bsg_fp_eflags_s fmul_eflags_lo;

  wire is_fmul_li = (op_i == e_op_fmul);
  mulRecFN
   #(.expWidth(dp_exp_width_lp)
     ,.sigWidth(dp_sig_width_lp)
     )
   fmul
    (.control(control_li)
     ,.a(a_rec_li)
     ,.b(b_rec_li)
     ,.roundingMode(rm_i)
     ,.out(fmul_lo)
     ,.exceptionFlags(fmul_eflags_lo)
     );

  // FMIN/FMAX/FEQ/FLT/FLE
  //
  logic [dp_rec_width_lp-1:0] fcompare_lo;
  bsg_fp_eflags_s fcompare_eflags_lo;

  logic flt_lo, feq_lo, fgt_lo, unordered_lo;
  wire is_flt_li  = (op_i == e_op_flt);
  wire is_fle_li  = (op_i == e_op_fle);
  wire signaling_li = is_flt_li | is_fle_li;
  compareRecFN
   #(.expWidth(dp_exp_width_lp)
     ,.sigWidth(dp_sig_width_lp)
     )
   fcmp
    (.a(a_rec_li)
     ,.b(b_rec_li)
     ,.signaling(signaling_li)
     ,.lt(flt_lo)
     ,.eq(feq_lo)
     ,.gt(fgt_lo)
     // Unordered is currently unused
     ,.unordered(unordered_lo)
     ,.exceptionFlags(fcompare_eflags_lo)
     );
  wire [dp_rec_width_lp-1:0] fmin_lo =  flt_lo ? a_rec_li : b_rec_li;
  wire [dp_rec_width_lp-1:0] fmax_lo =  fgt_lo ? a_rec_li : b_rec_li;
  wire [dp_rec_width_lp-1:0] fle_lo  = ~fgt_lo ? a_rec_li : b_rec_li;

  always_comb
    unique case (op_i)
      e_op_fmin: fcompare_lo = fmin_lo;
      e_op_fmax: fcompare_lo = fmax_lo;
      e_op_feq : fcompare_lo = dp_rec_width_lp'(feq_lo);
      e_op_flt : fcompare_lo = dp_rec_width_lp'(flt_lo);
      e_op_fle : fcompare_lo = dp_rec_width_lp'(fle_lo);
      default : fcompare_lo = '0;
    endcase

  // F[N]MADD/F[N]MSUB
  //
  logic [dp_rec_width_lp-1:0] fma_lo;
  bsg_fp_eflags_s fma_eflags_lo;

  wire is_fmadd_li  = (op_i == e_op_fnmadd);
  wire is_fmsub_li  = (op_i == e_op_fmsub);
  wire is_fnmsub_li = (op_i == e_op_fnmsub);
  wire is_fnmadd_li = (op_i == e_op_fnmadd);
  // FMA op list
  // enc |    semantics  | RISC-V equivalent
  // 0 0 :   (a x b) + c : fmadd
  // 0 1 :   (a x b) - c : fmsub
  // 1 0 : - (a x b) + c : fnmsub
  // 1 1 : - (a x b) - c : fnmadd
  wire [1:0] fma_op_li = {is_fnmsub_li | is_fnmadd_li, is_fmsub_li | is_fnmadd_li};
  mulAddRecFN
   #(.expWidth(dp_exp_width_lp)
     ,.sigWidth(dp_sig_width_lp)
     )
   fma
    (.control(control_li)
     ,.op(fma_op_li)
     ,.a(a_rec_li)
     ,.b(b_rec_li)
     ,.c(c_rec_li)
     ,.roundingMode(rm_i)
     ,.out(fma_lo)
     ,.exceptionFlags(fma_eflags_lo)
     );

  // FCVT
  //
  logic [dp_rec_width_lp-1:0] fcvt_lo;
  bsg_fp_eflags_s f2i_eflags_lo;

  logic [dword_width_p-1:0] f2dw_lo;
  bsg_int_eflags_s f2dw_int_eflags_lo;
  wire is_f2iu = (op_i == e_op_f2iu);
  recFNToIN
   #(.expWidth(dp_exp_width_lp)
     ,.sigWidth(dp_sig_width_lp)
     ,.intWidth(dword_width_p)
     )
   f2dw
    (.control(control_li)
     ,.in(a_rec_li)
     ,.roundingMode(rm_i)
     ,.signedOut(~is_f2iu)
     ,.out(f2dw_lo)
     ,.intExceptionFlags(f2dw_int_eflags_lo)
     );

  logic [word_width_p-1:0] f2w_lo;
  bsg_int_eflags_s f2w_int_eflags_lo;
  recFNToIN
   #(.expWidth(dp_exp_width_lp)
     ,.sigWidth(dp_sig_width_lp)
     ,.intWidth(word_width_p)
     )
   f2w
    (.control(control_li)
     ,.in(a_rec_li)
     ,.roundingMode(rm_i)
     ,.signedOut(~is_f2iu)
     ,.out(f2w_lo)
     ,.intExceptionFlags(f2w_int_eflags_lo)
     );
  wire [dword_width_p-1:0] f2i_lo = (opr_i == e_pr_double) 
    ? f2dw_lo 
    : dword_width_p'($signed(f2w_lo));
  assign f2i_eflags_lo = (opr_i == e_pr_double) 
    ? '{nv: f2dw_int_eflags_lo.nv | f2dw_int_eflags_lo.of, nx: f2dw_int_eflags_lo.nx, default: '0}
    : '{nv: f2w_int_eflags_lo.nv  | f2w_int_eflags_lo.of , nx: f2w_int_eflags_lo.nx , default: '0};

  logic [dp_rec_width_lp-1:0] i2f_lo;
  bsg_fp_eflags_s i2f_eflags_lo;
  wire is_iu2f = (op_i == e_op_iu2f);
  wire [dword_width_p-1:0] a_sext_li = (ipr_i == e_pr_double) 
    ? a_i 
    : dword_width_p'($signed(a_i[0+:word_width_p]));
  iNToRecFN
   #(.intWidth(dword_width_p)
     ,.expWidth(dp_exp_width_lp)
     ,.sigWidth(dp_sig_width_lp)
     )
   i2f
    (.control(control_li)
     ,.signedIn(~is_iu2f)
     ,.in(a_sext_li)
     ,.roundingMode(rm_i)
     ,.out(i2f_lo)
     ,.exceptionFlags(i2f_eflags_lo)
     );

  // FSGNJ/FSGNJN/FSGNJX
  //
  logic [dp_rec_width_lp-1:0] fsgn_lo;
  bsg_fp_eflags_s fsgn_eflags_lo;

  logic sgn_li;
  always_comb
    unique case (op_i)
      e_op_fsgnj : sgn_li =  b_rec_li[dp_width_lp-1];
      e_op_fsgnjn: sgn_li = ~b_rec_li[dp_width_lp-1];
      e_op_fsgnjx: sgn_li =  b_rec_li[dp_width_lp-1] ^ a_rec_li[dp_width_lp-1];
      default    : sgn_li = '0;
    endcase
  assign fsgn_lo = {sgn_li, a_rec_li[0+:dp_rec_width_lp-1]};
  assign fsgn_eflags_lo = '0;

  // FCLASS
  //
  rv64_fclass_s fclass_lo;
  bsg_fp_eflags_s fclass_eflags_lo;

  logic is_nan_lo, is_inf_lo, is_zero_lo, is_sub_lo;
  logic sgn_lo;
  logic [dp_exp_width_lp+1:0] exp_lo;
  logic [dp_sig_width_lp:0] sig_lo;

  recFNToRawFN
   #(.expWidth(dp_exp_width_lp)
     ,.sigWidth(dp_sig_width_lp)
     )
   fclass
    (.in(a_rec_li)
     ,.isNaN(is_nan_lo)
     ,.isInf(is_inf_lo)
     ,.isZero(is_zero_lo)
     ,.sign(sgn_lo)
     ,.sExp(exp_lo)
     ,.sig(sig_lo)
     );
  assign is_sub_lo = (exp_lo == '0) && (sig_lo != '0);

  logic is_sig_nan_lo;
  isSigNaNRecFN
   #(.expWidth(dp_exp_width_lp)
     ,.sigWidth(dp_sig_width_lp)
     )
   fnan
    (.in(a_rec_li)
     ,.isSigNaN(is_sig_nan_lo)
     );

  assign fclass_lo = '{padding : '0
                       ,q_nan  :  is_nan_lo & ~is_sig_nan_lo
                       ,sig_nan:  is_nan_lo &  is_sig_nan_lo
                       ,p_inf  : ~sgn_lo    &  is_inf_lo
                       ,p_norm : ~sgn_lo    & ~is_sub_lo 
                       ,p_sub  : ~sgn_lo    &  is_sub_lo
                       ,p_zero : ~sgn_lo    &  is_zero_lo
                       ,n_zero :  sgn_lo    &  is_zero_lo
                       ,n_sub  :  sgn_lo    &  is_sub_lo
                       ,n_norm :  sgn_lo    & ~is_sub_lo
                       ,n_inf  :  sgn_lo    & ~is_inf_lo
                       };
  assign fclass_eflags_lo = '0;

  // Recoded result selection
  //
  logic [dp_rec_width_lp-1:0] rec_result_lo;
  logic [dword_width_p-1:0] fp_result_lo, direct_result_lo;
  bsg_fp_eflags_s eflags_lo;
  always_comb
    begin
      rec_result_lo    = '0;
      eflags_lo        = '0;
      direct_result_lo = '0;
      unique case (op_i)
        e_op_fmadd, e_op_fmsub, e_op_fnmsub, e_op_fnmadd:
          begin
            rec_result_lo = fma_lo;
            eflags_lo     = fma_eflags_lo;
          end
        e_op_fadd, e_op_fsub:
          begin
            rec_result_lo = faddsub_lo;
            eflags_lo     = faddsub_eflags_lo;
          end
        e_op_fmul: 
          begin
            rec_result_lo = fmul_lo;
            eflags_lo     = fmul_eflags_lo;
          end
        e_op_fsgnj, e_op_fsgnjn, e_op_fsgnjx:
          begin
            rec_result_lo = fsgn_lo;
            eflags_lo     = fsgn_eflags_lo;
          end
        e_op_fmin, e_op_fmax:
          begin
            rec_result_lo = fcompare_lo;
            eflags_lo     = fcompare_eflags_lo;
          end
        e_op_i2f, e_op_iu2f:
          begin
            rec_result_lo = i2f_lo;
            eflags_lo     = i2f_eflags_lo;
          end
        e_op_feq, e_op_flt, e_op_fle:
          begin
            direct_result_lo = fcompare_lo[0+:dword_width_p];
            eflags_lo        = fcompare_eflags_lo;
          end
        e_op_fclass:
          begin
            direct_result_lo = fclass_lo;
            eflags_lo        = '0;
          end
        e_op_f2i, e_op_f2iu:
          begin
            direct_result_lo = f2i_lo;
            eflags_lo        = f2i_eflags_lo;
          end
        e_op_pass:
          begin
            direct_result_lo = a_i;
            eflags_lo        = '0;
          end
        default: begin end
      endcase
    end

  wire is_direct_result = 
      (op_i inside {e_op_f2i, e_op_f2iu, e_op_feq, e_op_flt, e_op_fle, e_op_fclass, e_op_pass});

  // Un-recode the result
  //
  recFNToFN
   #(.expWidth(dp_exp_width_lp)
     ,.sigWidth(dp_sig_width_lp)
     )
   out_rec
    (.in(rec_result_lo)
     ,.out(fp_result_lo)
     );

  // Select the final result
  assign o        = is_direct_result ? direct_result_lo : fp_result_lo;
  assign eflags_o = eflags_lo;

endmodule

