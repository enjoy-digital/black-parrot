
module bp_be_hardfloat_fpu
 import bp_common_pkg::*;
 import bp_common_rv64_pkg::*;
 import bp_be_pkg::*;
 import bp_be_hardfloat_pkg::*;
 #(parameter latency_p          = 4 // Used for retiming
   , parameter dword_width_p    = 64
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

   , input bp_be_fp_fu_op_e       op_i
   , input bp_be_fp_pr_e          ipr_i
   , input bp_be_fp_pr_e          opr_i
   , input rv64_frm_e             rm_i

   , output logic [dword_width_p-1:0] o
   , output rv64_fflags_s             eflags_o
   );

  // The control bits control tininess, which is fixed in RISC-V
  wire [`floatControlWidth-1:0] control_li = `flControl_default;
 
  // Recode all three inputs from FP
  //   We use a pseudo foreach loop to save verbosity
  //   We also convert from 32 bit inputs to 64 bit recoded inputs. 
  //     This double rounding behavior was formally proved correct in
  //     "Innocuous Double Rounding of Basic Arithmetic Operations" by Pierre Roux
  logic [dword_width_p-1:0] a_li, b_li, c_li;
  logic [dp_rec_width_lp-1:0] a_rec_li, b_rec_li, c_rec_li;
  logic a_sig_nan_li, b_sig_nan_li, c_sig_nan_li;
  logic [2:0][sp_width_lp-1:0] in_sp_li;
  logic [2:0][dp_width_lp-1:0] in_dp_li;
  logic [2:0][dp_rec_width_lp-1:0] in_rec_li;
  logic [2:0] in_sig_nan_li;

  // NaN boxing
  //
  localparam [dp_width_lp-1:0] dp_canonical = 64'h7ff80000_00000000;
  localparam [dp_width_lp-1:0] sp_canonical = 64'hffffffff_7fc00000;
  wire a_valid = (ipr_i == e_pr_double) | &a_i[sp_width_lp+:sp_width_lp];
  wire b_valid = (ipr_i == e_pr_double) | &b_i[sp_width_lp+:sp_width_lp];
  wire c_valid = (ipr_i == e_pr_double) | &c_i[sp_width_lp+:sp_width_lp];
  assign a_li = a_valid ? a_i : sp_canonical;
  assign b_li = b_valid ? b_i : sp_canonical;
  assign c_li = c_valid ? c_i : sp_canonical;
  assign in_sp_li = {c_li[0+:sp_width_lp], b_li[0+:sp_width_lp], a_li[0+:sp_width_lp]};
  assign in_dp_li = {c_li, b_li, a_li};
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

      logic is_sig_nan_sp;
      isSigNaNRecFN
       #(.expWidth(sp_exp_width_lp)
         ,.sigWidth(sp_sig_width_lp)
         )
       in_sp_sig_nan
        (.in(in_sp_rec_li)
         ,.isSigNaN(is_sig_nan_sp)
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

      logic is_sig_nan_dp;
      isSigNaNRecFN
       #(.expWidth(dp_exp_width_lp)
         ,.sigWidth(dp_sig_width_lp)
         )
       in_dp_sig_nan
        (.in(in_dp_rec_li)
         ,.isSigNaN(is_sig_nan_dp)
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
      assign in_sig_nan_li[i] = (ipr_i == e_pr_double) ? is_sig_nan_dp : is_sig_nan_sp;
    end
  assign {c_rec_li, b_rec_li, a_rec_li} = in_rec_li;
  assign {c_sig_nan_li, b_sig_nan_li, a_sig_nan_li} = in_sig_nan_li;

  // Generate auxiliary information
  //
 
  logic a_is_nan_lo, a_is_inf_lo, a_is_zero_lo, a_is_sub_lo;
  logic a_sgn_lo;
  logic [dp_exp_width_lp+1:0] a_exp_lo;
  logic [dp_sig_width_lp:0] a_sig_lo;

  recFNToRawFN
   #(.expWidth(dp_exp_width_lp)
     ,.sigWidth(dp_sig_width_lp)
     )
   aclass
    (.in(a_rec_li)
     ,.isNaN(a_is_nan_lo)
     ,.isInf(a_is_inf_lo)
     ,.isZero(a_is_zero_lo)
     ,.sign(a_sgn_lo)
     ,.sExp(a_exp_lo)
     ,.sig(a_sig_lo)
     );
  assign a_is_sub_lo = (ipr_i == e_pr_double)
                       ? (a_li[dp_width_lp-2-:dp_exp_width_lp] == '0) && (a_li[0+:dp_sig_width_lp])
                       : (a_li[sp_width_lp-2-:sp_exp_width_lp] == '0) && (a_li[0+:sp_sig_width_lp]);

  logic b_is_nan_lo, b_is_inf_lo, b_is_zero_lo, b_is_sub_lo;
  logic b_sgn_lo;
  logic [dp_exp_width_lp+1:0] b_exp_lo;
  logic [dp_sig_width_lp:0] b_sig_lo;

  recFNToRawFN
   #(.expWidth(dp_exp_width_lp)
     ,.sigWidth(dp_sig_width_lp)
     )
   bclass
    (.in(b_rec_li)
     ,.isNaN(b_is_nan_lo)
     ,.isInf(b_is_inf_lo)
     ,.isZero(b_is_zero_lo)
     ,.sign(b_sgn_lo)
     ,.sExp(b_exp_lo)
     ,.sig(b_sig_lo)
     );
  assign b_is_sub_lo = (ipr_i == e_pr_double)
                       ? (b_li[dp_width_lp-2-:dp_exp_width_lp] == '0) && (b_li[0+:dp_sig_width_lp])
                       : (b_li[sp_width_lp-2-:sp_exp_width_lp] == '0) && (b_li[0+:sp_sig_width_lp]);

  // FADD/FSUB
  //
  logic [dp_rec_width_lp-1:0] faddsub_lo;
  rv64_fflags_s faddsub_eflags_lo;

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
  rv64_fflags_s fmul_eflags_lo;

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
  rv64_fflags_s fcmp_eflags_lo, fcompare_eflags_lo, fminmax_eflags_lo;

  logic [dp_width_lp-1:0] fminmax_lo;
  rv64_fflags_s fcmp_nv_eflags_lo, fminmax_nv_eflags_lo;

  logic flt_lo, feq_lo, fgt_lo, unordered_lo;
  wire is_flt_li  = (op_i == e_op_flt);
  wire is_fle_li  = (op_i == e_op_fle);
  wire is_fmax_li = (op_i == e_op_fmax);
  wire is_fmin_li = (op_i == e_op_fmin);
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
     ,.exceptionFlags(fcmp_eflags_lo)
     );
  wire [dp_rec_width_lp-1:0] fle_lo  = ~fgt_lo;

  assign fminmax_nv_eflags_lo = '{nv : (a_sig_nan_li | b_sig_nan_li), default: '0};
  assign fminmax_eflags_lo  = fcmp_eflags_lo | fminmax_nv_eflags_lo;

  assign fcmp_nv_eflags_lo = '{nv : (a_sig_nan_li | b_sig_nan_li), default: '0};
  assign fcompare_eflags_lo = fcmp_eflags_lo | fcmp_nv_eflags_lo;

  always_comb
    begin
      fminmax_lo = '0;
      fcompare_lo = '0;
      unique case (op_i)
        e_op_fmin: fminmax_lo = (a_is_nan_lo & b_is_nan_lo)
                                 ? (opr_i == e_pr_single) ? sp_canonical : dp_canonical
                                 : (~a_is_nan_lo & b_is_nan_lo)
                                   ? a_li
                                   : (a_is_nan_lo & ~b_is_nan_lo)
                                     ? b_li
                                     : (flt_lo | (a_sgn_lo & ~b_sgn_lo))
                                       ? a_li
                                       : b_li;
        e_op_fmax: fminmax_lo = (a_is_nan_lo & b_is_nan_lo)
                                 ? (opr_i == e_pr_single) ? sp_canonical : dp_canonical
                                 : (~a_is_nan_lo & b_is_nan_lo)
                                   ? a_li
                                   : (a_is_nan_lo & ~b_is_nan_lo)
                                     ? b_li
                                     : (fgt_lo | (~a_sgn_lo & b_sgn_lo))
                                       ? a_li
                                       : b_li;
        e_op_feq : fcompare_lo = dp_rec_width_lp'(~unordered_lo & feq_lo);
        e_op_flt : fcompare_lo = dp_rec_width_lp'(~unordered_lo & flt_lo);
        e_op_fle : fcompare_lo = dp_rec_width_lp'(~unordered_lo & fle_lo);
        default : begin end
      endcase
    end

  // F[N]MADD/F[N]MSUB
  //
  logic [dp_rec_width_lp-1:0] fma_lo;
  rv64_fflags_s fma_eflags_lo;

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
  rv64_fflags_s f2i_eflags_lo;

  logic [dword_width_p-1:0] f2dw_lo;
  rv64_iflags_s f2dw_int_eflags_lo;
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
  rv64_iflags_s f2w_int_eflags_lo;
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
  rv64_fflags_s i2f_eflags_lo;
  wire is_iu2f = (op_i == e_op_iu2f);
  wire [dword_width_p-1:0] a_sext_li = (ipr_i == e_pr_double) 
    ? a_i
    : is_iu2f
      ? dword_width_p'($unsigned(a_i[0+:word_width_p]))
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
  logic [dp_width_lp-1:0] fsgn_lo;
  rv64_fflags_s fsgn_eflags_lo;

  logic sgn_li;
  always_comb
    if (opr_i == e_pr_double)
      unique case (op_i)
        e_op_fsgnj:  sgn_li =  b_li[dp_width_lp-1];
        e_op_fsgnjn: sgn_li = ~b_li[dp_width_lp-1];
        e_op_fsgnjx: sgn_li =  b_li[dp_width_lp-1] ^ a_li[dp_width_lp-1];
        default : sgn_li = '0;
      endcase
    else
      unique case (op_i)
        e_op_fsgnj:  sgn_li =  b_li[sp_width_lp-1];
        e_op_fsgnjn: sgn_li = ~b_li[sp_width_lp-1];
        e_op_fsgnjx: sgn_li =  b_li[sp_width_lp-1] ^ a_li[sp_width_lp-1];
        default : sgn_li = '0;
      endcase

  // Inject sign into double precision or valid single precision
  // Do not inject into auto-nonboxed
  assign fsgn_lo = (opr_i == e_pr_double) 
                   ? {sgn_li, a_li[0+:dp_width_lp-1]}
                   : {32'hffffffff, sgn_li, a_li[0+:sp_width_lp-1]};
  assign fsgn_eflags_lo = '0;
  

  // FCLASS
  //
  rv64_fclass_s fclass_lo;
  rv64_fflags_s fclass_eflags_lo;

  assign fclass_lo = '{padding : '0
                       ,q_nan  :  a_is_nan_lo & ~a_sig_nan_li
                       ,sig_nan:  a_is_nan_lo &  a_sig_nan_li
                       ,p_inf  : ~a_sgn_lo    &  a_is_inf_lo
                       ,p_norm : ~a_sgn_lo    & ~a_is_sub_lo & ~a_is_inf_lo & ~a_is_zero_lo & ~a_is_nan_lo
                       ,p_sub  : ~a_sgn_lo    &  a_is_sub_lo
                       ,p_zero : ~a_sgn_lo    &  a_is_zero_lo
                       ,n_zero :  a_sgn_lo    &  a_is_zero_lo
                       ,n_sub  :  a_sgn_lo    &  a_is_sub_lo
                       ,n_norm :  a_sgn_lo    & ~a_is_sub_lo & ~a_is_inf_lo & ~a_is_zero_lo & ~a_is_nan_lo
                       ,n_inf  :  a_sgn_lo    &  a_is_inf_lo
                       };
  assign fclass_eflags_lo = '0;

  // Recoded result selection
  //
  logic [dp_rec_width_lp-1:0] rec_result_lo;
  logic [dword_width_p-1:0] direct_result_lo;
  rv64_fflags_s rec_eflags_lo, direct_eflags_lo;
  always_comb
    begin
      rec_result_lo    = '0;
      direct_result_lo = '0;
      rec_eflags_lo    = '0;
      direct_eflags_lo = '0;
      unique case (op_i)
        e_op_fmadd, e_op_fmsub, e_op_fnmsub, e_op_fnmadd:
          begin
            rec_result_lo = fma_lo;
            rec_eflags_lo = fma_eflags_lo;
          end
        e_op_fadd, e_op_fsub:
          begin
            rec_result_lo = faddsub_lo;
            rec_eflags_lo = faddsub_eflags_lo;
          end
        e_op_fmul: 
          begin
            rec_result_lo = fmul_lo;
            rec_eflags_lo = fmul_eflags_lo;
          end
        e_op_f2f:
          begin
            rec_result_lo = a_rec_li;
            rec_eflags_lo = '0;
          end
        e_op_i2f, e_op_iu2f:
          begin
            rec_result_lo = i2f_lo;
            rec_eflags_lo = i2f_eflags_lo;
          end
        e_op_fsgnj, e_op_fsgnjn, e_op_fsgnjx:
          begin
            direct_result_lo = fsgn_lo;
            direct_eflags_lo = fsgn_eflags_lo;
          end
        e_op_fmin, e_op_fmax:
          begin
            direct_result_lo = fminmax_lo;
            direct_eflags_lo = fminmax_eflags_lo;
          end
        e_op_feq, e_op_flt, e_op_fle:
          begin
            direct_result_lo = fcompare_lo[0];
            direct_eflags_lo = fcompare_eflags_lo;
          end
        e_op_fclass:
          begin
            direct_result_lo = fclass_lo;
            direct_eflags_lo = '0;
          end
        e_op_f2i:
          begin
            direct_result_lo = (opr_i == e_pr_single)
                               ? dword_width_p'($signed(f2i_lo[0+:word_width_p]))
                               : f2i_lo;
            direct_eflags_lo = f2i_eflags_lo;
          end
        e_op_f2iu:
          begin
            direct_result_lo = (opr_i == e_pr_single)
                               ? dword_width_p'($signed(f2i_lo[0+:word_width_p]))
                               : f2i_lo;
            direct_eflags_lo = f2i_eflags_lo;
          end
        e_op_fmvi:
          begin
            direct_result_lo = (opr_i == e_pr_single)
                               ? dword_width_p'($signed(a_i[0+:word_width_p]))
                               : a_li;
            direct_eflags_lo = '0;
          end
        e_op_imvf:
          begin
            direct_result_lo = (opr_i == e_pr_single)
                               ? {32'hffffffff, a_sext_li[0+:word_width_p]}
                               : a_sext_li;
            direct_eflags_lo = '0;
          end
        default: begin end
      endcase
    end

  wire is_direct_result = 
      (op_i inside {e_op_f2i, e_op_f2iu, e_op_fsgnj, e_op_fsgnjn, e_op_fsgnjx, e_op_feq, e_op_flt, e_op_fle, e_op_fclass, e_op_fmin, e_op_fmax, e_op_fmvi, e_op_imvf});

  // Classify the result
  //
  logic result_nan_lo;
  recFNToRawFN
   #(.expWidth(dp_exp_width_lp)
     ,.sigWidth(dp_sig_width_lp)
     )
   fclass_result
    (.in(rec_result_lo)
     ,.isNaN(result_nan_lo)
     ,.isInf()
     ,.isZero()
     ,.sign()
     ,.sExp()
     ,.sig()
     );

  logic [sp_rec_width_lp-1:0] rec_result_dp2sp_lo;
  rv64_fflags_s sp_eflags_lo;
  recFNToRecFN
   #(.inExpWidth(dp_exp_width_lp)
     ,.inSigWidth(dp_sig_width_lp)
     ,.outExpWidth(sp_exp_width_lp)
     ,.outSigWidth(sp_sig_width_lp)
     )
   rec_dp_to_sp
    (.control(control_li)
     ,.in(rec_result_lo)
     ,.roundingMode(rm_i)
     ,.out(rec_result_dp2sp_lo)
     ,.exceptionFlags(sp_eflags_lo)
     );

  // Un-recode the results
  //
  logic [dword_width_p-1:0] raw_dp_result_lo;
  logic [dword_width_p-1:0] final_dp_result_lo;
  recFNToFN
   #(.expWidth(dp_exp_width_lp)
     ,.sigWidth(dp_sig_width_lp)
     )
   out_dp_rec
    (.in(rec_result_lo)
     ,.out(raw_dp_result_lo)
     );
  assign final_dp_result_lo = result_nan_lo ? dp_canonical : raw_dp_result_lo;

  logic [word_width_p-1:0] raw_sp_result_lo;
  logic [dword_width_p-1:0] final_sp_result_lo;
  recFNToFN
   #(.expWidth(sp_exp_width_lp)
     ,.sigWidth(sp_sig_width_lp)
     )
   out_sp_rec
    (.in(rec_result_dp2sp_lo)
     ,.out(raw_sp_result_lo)
     );
  assign final_sp_result_lo = result_nan_lo ? sp_canonical : {32'hffffffff, raw_sp_result_lo};

  rv64_fflags_s fp_eflags_lo, dir_eflags_lo, eflags_lo;

  wire [dp_width_lp-1:0] fp_result_lo = (opr_i == e_pr_double) ? final_dp_result_lo : final_sp_result_lo;
  assign fp_eflags_lo = (opr_i == e_pr_double) ? rec_eflags_lo : (rec_eflags_lo | sp_eflags_lo);
  assign dir_eflags_lo = (opr_i == e_pr_double) ? direct_eflags_lo : (direct_eflags_lo | sp_eflags_lo);

  wire [dword_width_p-1:0] result_lo = is_direct_result ? direct_result_lo : fp_result_lo;
  assign eflags_lo = is_direct_result ? dir_eflags_lo : fp_eflags_lo;
  bsg_dff_chain
   #(.width_p($bits(rv64_fflags_s)+dword_width_p)
     ,.num_stages_p(latency_p-1)
     )
   retimer_chain
    (.clk_i(clk_i)

     ,.data_i({eflags_lo, result_lo})
     ,.data_o({eflags_o, o})
     );

endmodule

