/**
 *
 * Name:
 *   bp_cce_src_sel.v
 *
 * Description:
 *   Source select module for inputs to ALU and Branch units.
 *
 *   The output width is parameterizable and should be the same as the input widths of
 *   both the ALU and Branch units.
 *
 */

// TODO: include both src_a and src_b in this module
// For example, need both sources available when selecting from sharers vectors

module bp_cce_src_sel
  import bp_common_pkg::*;
  import bp_common_aviary_pkg::*;
  import bp_cce_pkg::*;
  #(parameter bp_params_e bp_params_p = e_bp_inv_cfg
    `declare_bp_proc_params(bp_params_p)

    , parameter width_p = "inv"

    // Derived parameters
    , localparam mshr_width_lp = `bp_cce_mshr_width(lce_id_width_p, lce_assoc_p, paddr_width_p)
    , localparam lg_num_lce_lp = `BSG_SAFE_CLOG2(num_lce_p)
  )
  (input bp_cce_inst_src_sel_e                   src_sel_i
   , input bp_cce_inst_src_gpr_e                 gpr_sel_i
   , input bp_cce_inst_src_flag_e                flag_sel_i
   , input bp_cce_inst_src_special_e             special_sel_i

   // TODO: flags, and special registers as inputs
   , input [`bp_cce_inst_num_gpr-1:0][`bp_cce_inst_gpr_width-1:0]   gpr_i
   , input [mshr_width_lp-1:0]                                      mshr_i

   , input [`bp_cce_inst_gpr_width-1:0]          imm_i
   , output logic [`bp_cce_inst_gpr_width-1:0]   src_a_o
  );

  `declare_bp_cce_mshr_s(lce_id_width_p, lce_assoc_p, paddr_width_p);
  bp_cce_mshr_s mshr;
  assign mshr = mshr_i;

  logic [`bp_cce_inst_gpr_width-1:0] gpr, flag, special;

  always_comb begin : gpr_select
    gpr = '0;
    unique case (gpr_sel_i)
      e_src_r0: gpr = gpr_i[e_cce_gpr_r0];
      e_src_r1: gpr = gpr_i[e_cce_gpr_r1];
      e_src_r2: gpr = gpr_i[e_cce_gpr_r2];
      e_src_r3: gpr = gpr_i[e_cce_gpr_r3];
      e_src_r4: gpr = gpr_i[e_cce_gpr_r4];
      e_src_r5: gpr = gpr_i[e_cce_gpr_r5];
      e_src_r6: gpr = gpr_i[e_cce_gpr_r6];
      e_src_r7: gpr = gpr_i[e_cce_gpr_r7];
      default:  gpr = '0;
    endcase
  end

  always_comb begin : flag_select
    flag = '0;
    unique case (flag_sel_i)
      e_src_rqf:  flag[0] = mshr.flags[e_cce_flag_rqf];
      e_src_ucf:  flag[0] = mshr.flags[e_cce_flag_ucf];
      e_src_nerf: flag[0] = mshr.flags[e_cce_flag_nerf];
      e_src_ldf:  flag[0] = mshr.flags[e_cce_flag_ldf];
      e_src_pf:   flag[0] = mshr.flags[e_cce_flag_pf];
      e_src_lef:  flag[0] = mshr.flags[e_cce_flag_lef];
      e_src_cf:   flag[0] = mshr.flags[e_cce_flag_cf];
      e_src_cef:  flag[0] = mshr.flags[e_cce_flag_cef];
      e_src_cof:  flag[0] = mshr.flags[e_cce_flag_cof];
      e_src_cdf:  flag[0] = mshr.flags[e_cce_flag_cdf];
      e_src_tf:   flag[0] = mshr.flags[e_cce_flag_tf];
      e_src_rf:   flag[0] = mshr.flags[e_cce_flag_rf];
      e_src_uf:   flag[0] = mshr.flags[e_cce_flag_uf];
      e_src_if:   flag[0] = mshr.flags[e_cce_flag_if];
      e_src_nwbf: flag[0] = mshr.flags[e_cce_flag_nwbf];
      e_src_sf:   flag[0] = mshr.flags[e_cce_flag_sf];
      default:    flag = '0;
    endcase
  end

  always_comb begin : special_select
    special = '0;
    unique case (special_sel_i)

  e_src_flags                            = 4'b0000 // MSHR.flags

  // sharers vectors require src_b to provide GPR rX containing index to use
  ,e_src_sharers_hit                     = 4'b0001 // sharers_hits[rX]
  ,e_src_sharers_way                     = 4'b0010 // sharers_ways[rX]
  ,e_src_sharers_state                   = 4'b0011 // sharers_states[rX]

  ,e_src_cce_id                          = 4'b0100 // ID of this CCE
  ,e_src_num_lce                         = 4'b0101 // total number of LCE in system
  ,e_src_num_cce                         = 4'b0110 // total number of CCE in system
  ,e_src_num_wg                          = 4'b0111 // Number of WG managed by this CCE

  ,e_src_req_lce                         = 4'b1000 // MSHR.lce_id
  ,e_src_req_addr                        = 4'b1001 // MSHR.paddr
  ,e_src_req_way                         = 4'b1010 // MSHR.way_id
  ,e_src_next_coh_state                  = 4'b1011 // MSHR.next_coh_state
  ,e_src_lru_addr                        = 4'b1100 // MSHR.lru_paddr
  ,e_src_lru_way                         = 4'b1101 // MSHR.lru_way_id
  ,e_src_owner_lce                       = 4'b1110 // MSHR.owner_lce_id
  ,e_src_owner_way                       = 4'b1111 // MSHR.owner_way_id

      e_src_flags:   special[0+:`bp_cce_inst_num_flags] = mshr.flags;
      e_src_num_lce: special[0+:lg_num_lce_lp] = num_lce_p[0+:lg_num_lce_lp];
      default:       special = '0;
    endcase
  end

  always_comb begin : source_select
    unique case (src_sel_i)
      e_src_sel_gpr:     src_a_o = gpr;
      e_src_sel_flag:    src_a_o = flag;
      e_src_sel_special: src_a_o = special;
      e_src_sel_imm:     src_a_o = imm_i;
      default:           src_a_o = '0;
    endcase
  end

endmodule
