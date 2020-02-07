/**
 *
 * Name:
 *   bp_cce_src_sel.v
 *
 * Description:
 *   Source select module for inputs to ALU and Branch units.
 *
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

    // Derived parameters
    , localparam mshr_width_lp = `bp_cce_mshr_width(lce_id_width_p, lce_assoc_p, paddr_width_p)
    , localparam lce_assoc_width_lp = `BSG_SAFE_CLOG2(lce_max_assoc_p)
    , localparam cfg_bus_width_lp = `bp_cfg_bus_width(vaddr_width_p, core_id_width_p, cce_id_width_p, lce_id_width_p, cce_pc_width_p, cce_instr_width_p)
  )
  (// Select signals for src_a and src_b - from decoded instruction
   input bp_cce_inst_src_sel_e                   src_a_sel_i
   , input bp_cce_inst_src_u                     src_a_i
   , input bp_cce_inst_src_sel_e                 src_b_sel_i
   , input bp_cce_inst_src_u                     src_b_i

   // Select signals for functional unit inputs (directory, gad, pending, message, etc.)
   , input bp_cce_inst_mux_sel_addr_e            addr_sel
   , input bp_cce_inst_mux_sel_lce_e             lce_sel
   , input bp_cce_inst_mux_sel_way_e             way_sel
   , input bp_cce_inst_mux_sel_way_e             lru_way_sel
   , input bp_cce_inst_mux_sel_coh_state_e       coh_state_sel

   // Data sources - from registered data and functional units
   , input [cfg_bus_width_lp-1:0]                                   cfg_bus_i
   , input [mshr_width_lp-1:0]                                      mshr_i
   , input [`bp_cce_inst_num_gpr-1:0][`bp_cce_inst_gpr_width-1:0]   gpr_i
   , input [`bp_cce_inst_gpr_width-1:0]                             imm_i
   , input                                                          auto_fwd_msg_i
   , input bp_coh_states_e                                          coh_state_default_i
   , input [num_lce_p-1:0]                                          sharers_hits_i
   , input [num_lce_p-1:0][lce_assoc_width_lp-1:0]                  sharers_ways_i
   , input bp_coh_states_e [num_lce_p-1:0]                          sharers_coh_states_i
   , input                                                          mem_resp_v_i
   , input                                                          lce_resp_v_i
   , input                                                          lce_req_v_i
   , input bp_lce_cce_resp_type_e                                   lce_resp_type_i
   , input bp_cce_mem_cmd_type_e                                    mem_resp_type_i


   // TODO: should these be here, or in the register file?
   // Select signals for flags
   , input bp_cce_inst_rqf_sel_e                 rqf_sel
   , input bp_cce_inst_ucf_sel_e                 ucf_sel
   , input bp_cce_inst_nerf_sel_e                nerf_sel
   , input bp_cce_inst_ldf_sel_e                 ldf_sel
   , input bp_cce_inst_pf_sel_e                  pf_sel
   , input bp_cce_inst_lef_sel_e                 lef_sel
   , input bp_cce_inst_cf_sel_e                  cf_sel
   , input bp_cce_inst_cef_sel_e                 cef_sel
   , input bp_cce_inst_cof_sel_e                 cof_sel
   , input bp_cce_inst_cdf_sel_e                 cdf_sel
   , input bp_cce_inst_tf_sel_e                  tf_sel
   , input bp_cce_inst_rf_sel_e                  rf_sel
   , input bp_cce_inst_uf_sel_e                  uf_sel
   , input bp_cce_inst_if_sel_e                  if_sel
   , input bp_cce_inst_nwbf_sel_e                nwbf_sel
   , input bp_cce_inst_sf_sel_e                  sf_sel




   // Source A and B outputs
   , output logic [`bp_cce_inst_gpr_width-1:0]   src_a_o
   , output logic [`bp_cce_inst_gpr_width-1:0]   src_b_o

   // FU Select Outputs
   , output logic [paddr_width_p-1:0]            addr_o
   , output logic [lce_id_width_p-1:0]           lce_o
   , output logic [lce_assoc_width_lp-1:0]       way_o
   , output logic [lce_assoc_width_lp-1:0]       lru_way_o
   , output bp_coh_states_e                      state_o
  );

  `declare_bp_cfg_bus_s(vaddr_width_p, core_id_width_p, cce_id_width_p, lce_id_width_p, cce_pc_width_p, cce_instr_width_p);
  bp_cfg_bus_s cfg_bus_cast;
  assign cfg_bus_cast = cfg_bus_i;

  `declare_bp_cce_mshr_s(lce_id_width_p, lce_assoc_p, paddr_width_p);
  bp_cce_mshr_s mshr;
  assign mshr = mshr_i;

  always_comb begin:
    src_a_o = '0;
    unique case (src_a_sel_i)
      e_src_sel_gpr: begin
        unique case (src_a_i.gpr)
          e_opd_r0: src_a_o = gpr_i[e_opd_r0[0+:`bp_cce_inst_gpr_sel_width]];
          e_opd_r1: src_a_o = gpr_i[e_opd_r1[0+:`bp_cce_inst_gpr_sel_width]];
          e_opd_r2: src_a_o = gpr_i[e_opd_r2[0+:`bp_cce_inst_gpr_sel_width]];
          e_opd_r3: src_a_o = gpr_i[e_opd_r3[0+:`bp_cce_inst_gpr_sel_width]];
          e_opd_r4: src_a_o = gpr_i[e_opd_r4[0+:`bp_cce_inst_gpr_sel_width]];
          e_opd_r5: src_a_o = gpr_i[e_opd_r5[0+:`bp_cce_inst_gpr_sel_width]];
          e_opd_r6: src_a_o = gpr_i[e_opd_r6[0+:`bp_cce_inst_gpr_sel_width]];
          e_opd_r7: src_a_o = gpr_i[e_opd_r7[0+:`bp_cce_inst_gpr_sel_width]];
          default:  src_a_o = '0;
        endcase
      end
      e_src_sel_flag: begin
        unique case (src_a_i.flag)
          e_opd_rqf:  src_a_o[0] = mshr.flags[e_opd_rqf];
          e_opd_ucf:  src_a_o[0] = mshr.flags[e_opd_ucf];
          e_opd_nerf: src_a_o[0] = mshr.flags[e_opd_nerf];
          e_opd_ldf:  src_a_o[0] = mshr.flags[e_opd_ldf];
          e_opd_pf:   src_a_o[0] = mshr.flags[e_opd_pf];
          e_opd_lef:  src_a_o[0] = mshr.flags[e_opd_lef];
          e_opd_cf:   src_a_o[0] = mshr.flags[e_opd_cf];
          e_opd_cef:  src_a_o[0] = mshr.flags[e_opd_cef];
          e_opd_cof:  src_a_o[0] = mshr.flags[e_opd_cof];
          e_opd_cdf:  src_a_o[0] = mshr.flags[e_opd_cdf];
          e_opd_tf:   src_a_o[0] = mshr.flags[e_opd_tf];
          e_opd_rf:   src_a_o[0] = mshr.flags[e_opd_rf];
          e_opd_uf:   src_a_o[0] = mshr.flags[e_opd_uf];
          e_opd_if:   src_a_o[0] = mshr.flags[e_opd_if];
          e_opd_nwbf: src_a_o[0] = mshr.flags[e_opd_nwbf];
          e_opd_sf:   src_a_o[0] = mshr.flags[e_opd_sf];
          default:    src_a_o    = '0;
        endcase
      end
      e_src_sel_special: begin
        unique case (src_a_i.special)
          e_opd_req_lce:        src_a_o[0+:lce_id_width_p] = mshr.lce_id;
          e_opd_req_addr:       src_a_o[0+:paddr_width_p] = mshr.paddr;
          e_opd_req_way:        src_a_o[0+:lce_assoc_width_lp] = mshr.way_id;
          e_opd_lru_addr:       src_a_o[0+:paddr_width_p] = mshr.lru_paddr;
          e_opd_lru_way:        src_a_o[0+:lce_assoc_width_lp] = mshr.lru_way_id;
          e_opd_owner_lce:      src_a_o[0+:lce_id_width_p] = mshr.owner_lce_id;
          e_opd_owner_way:      src_a_o[0+:lce_assoc_width_lp] = mshr.owner_way_id;
          e_opd_next_coh_state: src_a_o[0+:$bits(bp_coh_states_e)] = mshr.next_coh_state;
          e_opd_flags:          src_a_o[0+:`bp_cce_inst_num_flags] = mshr.flags;
          e_opd_uc_req_size:    src_a_o[0+:$bits(bp_lce_cce_uc_req_size_e)] = mshr.uc_req_size;
          e_opd_data_length:    src_a_o[0+:$bits(bp_lce_cce_data_length_e)] = mshr.data_length;
          // TODO: correct? If sh_x[GPR] is the source, the index used comes from interpreting
          // src_b as a GPR
          e_opd_sharers_hit:    src_a_o[0] = sharers_hits_i[gpr_i[src_b_i.gpr[0+:`bp_cce_inst_gpr_sel_width]]];
          e_opd_sharers_way:    src_a_o[0+:lce_assoc_width_lp] = sharers_ways_i[gpr_i[src_b_i.gpr[0+:`bp_cce_inst_gpr_sel_width]]];
          e_opd_sharers_state:  src_a_o[0+:`bp_coh_bits] = sharers_coh_states_i;[gpr_i[src_b_i.gpr[0+:`bp_cce_inst_gpr_sel_width]]]
          default:              src_a_o = '0;
        endcase
      end
      e_src_sel_param: begin
        unique case (src_a_i.param)
          e_opd_cce_id:            src_a_o[0+:cce_id_width_p] = cfg_bus_cast.cce_id;
          e_opd_num_lce:           src_a_o[0+:lce_id_width_p] = num_lce_p[0+:lce_id_width_p+1];
          e_opd_num_cce:           src_a_o[0+:cce_id_width_p] = num_cce_p[0+:cce_id_width_p+1];
          e_opd_num_wg:            src_a_o[0+:lg_num_way_groups_lp] = num_way_groups_lp[0+:lg_num_way_groups_lp];
          e_opd_auto_fwd_msg:      src_a_o[0] = auto_fwd_msg_i;
          e_opd_coh_state_default: src_a_o[0+:$bits(bp_coh_states_e)] = coh_state_default_i;
          default:                 src_a_o = '0;
        endcase
      end
      e_src_sel_queue: begin
        unique case (src_a_i.queue)
          e_opd_mem_resp_v:    src_a_o[0] = mem_resp_v_i;
          e_opd_lce_resp_v:    src_a_o[0] = lce_resp_v_i;
          e_opd_pending_v:     src_a_o = '0;
          e_opd_lce_req_v:     src_a_o[0] = lce_req_v_i;
          e_opd_lce_resp_type: src_a_o[0+:$bits(bp_lce_cce_resp_type_e)] = lce_resp_type_i;
          e_opd_mem_resp_type: src_a_o[0+:$bits(bp_cce_mem_cmd_type_e)] = mem_resp_type_i;
          default:             src_a_o = '0;
        endcase
      end
      e_src_sel_imm: begin
        src_a_o = imm_i;
      end
      default: src_a_o = '0;
    end

    src_b_o = '0;
    unique case (src_b_sel_i)
      e_src_sel_gpr: begin
        unique case (src_b_i.gpr)
          e_opd_r0: src_b_o = gpr_i[e_opd_r0[0+:`bp_cce_inst_gpr_sel_width]];
          e_opd_r1: src_b_o = gpr_i[e_opd_r1[0+:`bp_cce_inst_gpr_sel_width]];
          e_opd_r2: src_b_o = gpr_i[e_opd_r2[0+:`bp_cce_inst_gpr_sel_width]];
          e_opd_r3: src_b_o = gpr_i[e_opd_r3[0+:`bp_cce_inst_gpr_sel_width]];
          e_opd_r4: src_b_o = gpr_i[e_opd_r4[0+:`bp_cce_inst_gpr_sel_width]];
          e_opd_r5: src_b_o = gpr_i[e_opd_r5[0+:`bp_cce_inst_gpr_sel_width]];
          e_opd_r6: src_b_o = gpr_i[e_opd_r6[0+:`bp_cce_inst_gpr_sel_width]];
          e_opd_r7: src_b_o = gpr_i[e_opd_r7[0+:`bp_cce_inst_gpr_sel_width]];
          default:  src_b_o = '0;
        endcase
      end
      e_src_sel_flag: begin
        unique case (src_b_i.flag)
          e_opd_rqf:  src_b_o[0] = mshr.flags[e_opd_rqf];
          e_opd_ucf:  src_b_o[0] = mshr.flags[e_opd_ucf];
          e_opd_nerf: src_b_o[0] = mshr.flags[e_opd_nerf];
          e_opd_ldf:  src_b_o[0] = mshr.flags[e_opd_ldf];
          e_opd_pf:   src_b_o[0] = mshr.flags[e_opd_pf];
          e_opd_lef:  src_b_o[0] = mshr.flags[e_opd_lef];
          e_opd_cf:   src_b_o[0] = mshr.flags[e_opd_cf];
          e_opd_cef:  src_b_o[0] = mshr.flags[e_opd_cef];
          e_opd_cof:  src_b_o[0] = mshr.flags[e_opd_cof];
          e_opd_cdf:  src_b_o[0] = mshr.flags[e_opd_cdf];
          e_opd_tf:   src_b_o[0] = mshr.flags[e_opd_tf];
          e_opd_rf:   src_b_o[0] = mshr.flags[e_opd_rf];
          e_opd_uf:   src_b_o[0] = mshr.flags[e_opd_uf];
          e_opd_if:   src_b_o[0] = mshr.flags[e_opd_if];
          e_opd_nwbf: src_b_o[0] = mshr.flags[e_opd_nwbf];
          e_opd_sf:   src_b_o[0] = mshr.flags[e_opd_sf];
          default:    src_b_o    = '0;
        endcase
      end
      e_src_sel_special: begin
        unique case (src_b_i.special)
          e_opd_req_lce:        src_b_o[0+:lce_id_width_p] = mshr.lce_id;
          e_opd_req_addr:       src_b_o[0+:paddr_width_p] = mshr.paddr;
          e_opd_req_way:        src_b_o[0+:lce_assoc_width_lp] = mshr.way_id;
          e_opd_lru_addr:       src_b_o[0+:paddr_width_p] = mshr.lru_paddr;
          e_opd_lru_way:        src_b_o[0+:lce_assoc_width_lp] = mshr.lru_way_id;
          e_opd_owner_lce:      src_b_o[0+:lce_id_width_p] = mshr.owner_lce_id;
          e_opd_owner_way:      src_b_o[0+:lce_assoc_width_lp] = mshr.owner_way_id;
          e_opd_next_coh_state: src_b_o[0+:$bits(bp_coh_states_e)] = mshr.next_coh_state;
          e_opd_flags:          src_b_o[0+:`bp_cce_inst_num_flags] = mshr.flags;
          e_opd_uc_req_size:    src_b_o[0+:$bits(bp_lce_cce_uc_req_size_e)] = mshr.uc_req_size;
          e_opd_data_length:    src_b_o[0+:$bits(bp_lce_cce_data_length_e)] = mshr.data_length;
          default:              src_b_o = '0;
        endcase
      end
      e_src_sel_param: begin
        unique case (src_b_i.param)
          e_opd_cce_id:            src_b_o[0+:cce_id_width_p] = cfg_bus_cast.cce_id;
          e_opd_num_lce:           src_b_o[0+:lce_id_width_p] = num_lce_p[0+:lce_id_width_p+1];
          e_opd_num_cce:           src_b_o[0+:cce_id_width_p] = num_cce_p[0+:cce_id_width_p+1];
          e_opd_num_wg:            src_b_o[0+:lg_num_way_groups_lp] = num_way_groups_lp[0+:lg_num_way_groups_lp];
          e_opd_auto_fwd_msg:      src_b_o[0] = auto_fwd_msg_i;
          e_opd_coh_state_default: src_b_o[0+:$bits(bp_coh_states_e)] = coh_state_default_i;
          default:                 src_b_o = '0;
        endcase
      end
      e_src_sel_queue: begin
        unique case (src_b_i.queue)
          e_opd_mem_resp_v:    src_b_o[0] = mem_resp_v_i;
          e_opd_lce_resp_v:    src_b_o[0] = lce_resp_v_i;
          e_opd_pending_v:     src_b_o = '0;
          e_opd_lce_req_v:     src_b_o[0] = lce_req_v_i;
          e_opd_lce_resp_type: src_b_o[0+:$bits(bp_lce_cce_resp_type_e)] = lce_resp_type_i;
          e_opd_mem_resp_type: src_b_o[0+:$bits(bp_cce_mem_cmd_type_e)] = mem_resp_type_i;
          default:             src_b_o = '0;
        endcase
      end
      e_src_sel_imm: begin
        src_b_o = imm_i;
      end
      default: src_b_o = '0;
    end

  end // src_a and src_b

// TODO: mux selects
  // addr_sel

  // lce_sel

  // way_sel

  // lru_way_sel

  // coh_state_sel



endmodule
