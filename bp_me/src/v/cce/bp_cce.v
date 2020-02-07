/**
 *
 * Name:
 *   bp_cce.v
 *
 * Description:
 *   This is the top level module for the CCE.
 *
 */

// Common TODO items for all of CCE
// 1. num_way_groups calculations
// 2. lce_assoc_p use and lg_lce_assoc_lp
// 3. lce_sets_p
// 4. tag_width_lp


module bp_cce
  import bp_common_pkg::*;
  import bp_common_aviary_pkg::*;
  import bp_cce_pkg::*;
  import bp_common_cfg_link_pkg::*;
  import bp_me_pkg::*;
  #(parameter bp_params_e bp_params_p = e_bp_inv_cfg
    `declare_bp_proc_params(bp_params_p)

    // Derived parameters
    , localparam block_size_in_bytes_lp    = (cce_block_width_p/8)
    , localparam lg_block_size_in_bytes_lp = `BSG_SAFE_CLOG2(block_size_in_bytes_lp)

    // bits for total number of way groups in the system
    , localparam lg_cce_way_groups_lp      = `BSG_SAFE_CLOG2(cce_way_groups_p)

    // number of way groups managed by this CCE
    , localparam num_way_groups_lp         = ((cce_way_groups_p % num_cce_p) == 0)
                                             ? (cce_way_groups_p/num_cce_p)
                                             : ((cce_way_groups_p/num_cce_p) + 1)
    , localparam lg_num_way_groups_lp      = `BSG_SAFE_CLOG2(num_way_groups_lp)

    , localparam cfg_bus_width_lp          = `bp_cfg_bus_width(vaddr_width_p, core_id_width_p, cce_id_width_p, lce_id_width_p, cce_pc_width_p, cce_instr_width_p)

    // interface widths
    `declare_bp_lce_cce_if_header_widths(cce_id_width_p, lce_id_width_p, lce_max_assoc_p, paddr_width_p)
    `declare_bp_lce_cce_if_widths(cce_id_width_p, lce_id_width_p, paddr_width_p, lce_max_assoc_p, dword_width_p, cce_block_width_p)
    `declare_bp_me_if_widths(paddr_width_p, cce_block_width_p, lce_id_width_p, lce_max_assoc_p)

    // TODO:
    // In most cases, use cce_id_width_p and lce_id_width_p instead of these.
    , localparam lg_num_lce_lp             = `BSG_SAFE_CLOG2(num_lce_p)
    , localparam lg_num_cce_lp             = `BSG_SAFE_CLOG2(num_cce_p)

    // TODO: these values will not be constant for all LCEs in new system with varying
    // organizations for I$, D$, A$
    , localparam lg_lce_assoc_lp           = `BSG_SAFE_CLOG2(lce_assoc_p)
    , localparam lg_lce_sets_lp            = `BSG_SAFE_CLOG2(lce_sets_p)
    , localparam tag_width_lp              = (paddr_width_p-lg_lce_sets_lp
                                              -lg_block_size_in_bytes_lp)


  )
  (input                                               clk_i
   , input                                             reset_i

   // Configuration Interface
   , input [cfg_bus_width_lp-1:0]                      cfg_bus_i
   , output [cce_instr_width_p-1:0]                    cfg_cce_ucode_data_o

   // LCE-CCE Interface
   // inbound: valid->ready (a.k.a., valid->yumi), demanding consumer (connects to FIFO)
   // outbound: ready&valid (connects directly to ME network)
   , input [lce_cce_req_header_width_lp-1:0]           lce_req_i
   , input                                             lce_req_v_i
   , output logic                                      lce_req_yumi_o
   , input [cce_lce_data_width_p-1:0]                  lce_req_data_i
   , input                                             lce_req_data_v_i
   , output logic                                      lce_req_data_yumi_o

   , input [lce_cce_resp_header_width_lp-1:0]          lce_resp_i
   , input                                             lce_resp_v_i
   , output logic                                      lce_resp_yumi_o
   , input [cce_lce_data_width_p-1:0]                  lce_resp_data_i
   , input                                             lce_resp_data_v_i
   , output logic                                      lce_resp_data_yumi_o

   , output logic [lce_cmd_header_width_lp-1:0]        lce_cmd_o
   , output logic                                      lce_cmd_v_o
   , input                                             lce_cmd_ready_i
   , input [cce_lce_data_width_p-1:0]                  lce_cmd_data_o
   , output logic                                      lce_cmd_data_v_o
   , input                                             lce_cmd_data_ready_i

   // CCE-MEM Interface
   // inbound: valid->ready (a.k.a., valid->yumi), demanding consumer (connects to FIFO)
   // outbound: ready&valid (connects to FIFO)
   , input [cce_mem_msg_header_width_lp-1:0]           mem_resp_i
   , input                                             mem_resp_v_i
   , output logic                                      mem_resp_yumi_o
   , input [cce_mem_data_width_p-1:0]                  mem_resp_data_i
   , input                                             mem_resp_data_v_i
   , output logic                                      mem_resp_data_yumi_o

   , output logic [cce_mem_msg_header_width_lp-1:0]    mem_cmd_o
   , output logic                                      mem_cmd_v_o
   , input                                             mem_cmd_ready_i
   , output logic [cce_mem_data_width_p-1:0]           mem_cmd_data_o
   , output logic                                      mem_cmd_data_v_o
   , input                                             mem_cmd_data_ready_i

  );

  // CCE Parameter Assertions
  //synopsys translate_off
  initial begin
    assert(`BSG_IS_POW2(cce_way_groups_p))
      else $error("Number of way groups must be a power of two");
    assert((cce_lce_data_width_p == 64) && (cce_mem_data_width_p == 64))
      else $error("CCE Data Width must be 64-bits");
    assert(`BSG_IS_POW2(dcache_assoc_p) && `BSG_IS_POW2(dcache_sets_p))
      else $error("D$ sets and assoc must be power of two");
    assert(`BSG_IS_POW2(icache_assoc_p) && `BSG_IS_POW2(icache_sets_p))
      else $error("I$ sets and assoc must be power of two");
    assert(`BSG_IS_POW2(acache_assoc_p) || acache_assoc_p == 0)
      else $error("A$ assoc must be power of two or 0");
    assert(`BSG_IS_POW2(acache_sets_p) || acache_sets_p == 0)
      else $error("A$ sets must be power of two or 0");
  end
  //synopsys translate_on

  // LCE-CCE and Mem-CCE Interface
  `declare_bp_me_if(paddr_width_p, cce_block_width_p, lce_id_width_p, lce_max_assoc_p);
  `declare_bp_lce_cce_if(cce_id_width_p, lce_id_width_p, paddr_width_p, lce_max_assoc_p, dword_width_p, cce_block_width_p);

  // Config Interface
  `declare_bp_cfg_bus_s(vaddr_width_p, core_id_width_p, cce_id_width_p, lce_id_width_p, cce_pc_width_p, cce_instr_width_p);
  bp_cfg_bus_s cfg_bus_cast_i;
  assign cfg_bus_cast_i = cfg_bus_i;

  // Message casting
  bp_lce_cce_req_header_s  lce_req;
  bp_lce_cce_resp_header_s lce_resp;
  bp_lce_cmd_header_s      lce_cmd;
  bp_cce_mem_msg_header_s  mem_cmd, mem_resp;
  assign lce_req   = lce_req_i;
  assign lce_resp  = lce_resp_i;
  assign lce_cmd_o = lce_cmd;
  assign mem_cmd_o = mem_cmd;
  assign mem_resp  = mem_resp_i;

  // MSHR register output signal
  `declare_bp_cce_mshr_s(lce_id_width_p, lce_max_assoc_p, paddr_width_p);
  bp_cce_mshr_s mshr;

  // Inter-module signals
  bp_cce_inst_s                        inst_lo;
  bp_cce_inst_decoded_s                decoded_inst_lo;
  logic [cce_pc_width_p-1:0]           ex_pc_lo;
  logic [cce_pc_width_p-1:0]           fetch_pc_lo, predicted_fetch_pc_lo, branch_resolution_pc_lo;
  logic                                stall_lo, mispredict_lo;
  //logic predict_taken_lo; // TODO: may be unused
  logic                                branch_res_lo, branch_mispredict_lo;
  logic [`bp_cce_inst_gpr_width-1:0]   src_a, src_b, alu_res_lo;
  logic [paddr_width_p-1:0]            addr_lo;
  logic [lce_id_width_p-1:0]           lce_lo;
  logic [lce_assoc_width_lp-1:0]       way_lo, lru_way_lo;
  bp_coh_states_e                      state_lo;

  logic dir_busy_lo;
  logic sharers_v_lo;
  logic [num_lce_p-1:0] sharers_hits_lo;
  logic [num_lce_p-1:0][lce_assoc_width_lp-1:0] sharers_ways_lo;
  bp_coh_states_e [num_lce_p-1:0] sharers_coh_states_lo;
  logic dir_addr_v_lo, dir_lru_v_lo;
  logic dir_lru_cached_excl_lo;
  logic [paddr_width_p-1:0] dir_addr_lo, dir_lru_addr_lo;

  logic [lce_assoc_widthlp-1:0] gad_req_addr_way_lo;
  logic gad_transfer_flag_lo;
  logic [lce_id_width_p-1:0] gad_owner_lce_lo;
  logic [lce_assoc_width_lp-1:0] gad_owner_way_lo;
  logic gad_replacement_flag_lo;
  logic gad_upgrade_flag_lo;
  logic gad_invalidate_flag_lo;
  logic gad_cached_flag_lo;
  logic gad_cached_exclusive_flag_lo;
  logic gad_cached_owned_flag_lo;
  logic gad_cached_dirty_flag_lo;

  logic [`bp_cce_inst_num_gpr-1:0][`bp_cce_inst_gpr_width-1:0] gpr_lo;
  bp_coh_states_e coh_state_default_lo;




  // Fetch Stage

  // Inst Fetch and microcode RAM
  bp_cce_inst_ram
    #(.bp_params_p(bp_params_p)
      )
    inst_ram
     (.clk_i(clk_i)
      ,.reset_i(reset_i)
      ,.cfg_bus_i(cfg_bus_i)
      ,.predicted_fetch_pc_i(predicted_fetch_pc_lo)
      ,.branch_resolution_pc_i(branch_resolution_pc_lo)
      ,.stall_i(stall_lo)
      ,.mispredict_i(mispredict_lo)
      ,.fetch_pc_o(fetch_pc_lo)
      ,.inst_o(inst_lo)
      );

  // Configuration Bus Microcode Data output
  assign cfg_cce_ucode_data_o = inst_lo;

  // Inst Pre-decode
  bp_cce_inst_predecode
    #(.width_p(cce_pc_width_p)
      )
    inst_predecode
     (.inst_i(inst_lo)
      ,.fetch_pc_i(fetch_pc_lo)
      //,.predict_taken_o(predict_taken_lo)
      ,.predicted_fetch_pc_o(predicted_fetch_pc_lo)
      );

  // Decode/Execute Stage


  // Performance monitor
  // TODO: define a set of signals that can be counted (will increment every cycle if condition set)
  // Define instructions that can set, clear, and read counters to GPR
  /*
  bp_cce_perfmon
    #()
    performance_monitor
     (.clk_i(clk_i)
      ,.reset_i(reset_i)
      // TODO: input signals
      // TODO: counter outputs
      );
  */

  // Instruction Decode
  // TODO: the decoded instruction output provides the intent (i.e., what the instruction would do if executed)
  // to the rest of the CCE. The inst_stall unit consumes the decoded instruction and then determines if the action
  // can take place depending on data or structural hazards, etc. The instruction only affects the architectural
  // state of the CCE and has any effect if no stall is detected.
  bp_cce_inst_decode
    #(.cce_pc_width_p(cce_pc_width_p)
      )
    inst_decode
     (.clk_i(clk_i)
      ,.reset_i(reset_i)
      ,.inst_i(inst_lo)
      ,.pc_i(fetch_pc_lo)
      // TODO: instruction valid signal - probably from inst_ram?
      ,.inst_v_i()
      ,.decoded_inst_o(decoded_inst_lo)
      ,.pc_o(ex_pc_lo)
      );

  // Instruction Stall Detection
  // TODO: this logic must be fast. No long paths should exist and a stall needs to be detected based on the 
  bp_cce_inst_stall
    #(.cnt_width_p(64)
      )
    inst_stall
     (.clk_i(clk_i)
      ,.reset_i(reset_i)

/*
      ,.pending_w_busy_i(pending_w_busy_from_msg)
      ,.lce_cmd_busy_i(lce_cmd_busy_from_msg)

      ,.lce_req_v_i(lce_req_v_i)
      ,.lce_resp_v_i(lce_resp_v_i)
      ,.lce_resp_type_i(lce_resp_li.msg_type)
      ,.mem_resp_v_i(mem_resp_v_i)
      ,.pending_v_i('0)

      ,.lce_cmd_ready_i(lce_cmd_ready_i)
      ,.mem_cmd_ready_i(mem_cmd_ready_i)

      ,.fence_zero_i(fence_zero_lo)

      ,.pc_stall_o(pc_stall_lo)
      ,.pc_branch_target_o(pc_branch_target_lo)

*/

      ,.stall_o(stall_lo)
      ,.stall_cnt_o(stall_cnt_lo)
      );

  // Directory
  // Pending Bits
  // GAD
  // Register File
  // Special Register File
  // Source Select Module
  // -- what does this select for and feed?
  // -- ALU, Branch?, Directory?, Pending Bits?
  // Message Tx/Rx
  // - Normal Mode Message Tx/Rx
  // - Uncached Only Mode Message Tx/Rx

  // ALU
  bp_cce_alu
    #(.width_p(`bp_cce_inst_gpr_width)
      )
    alu
     (.opd_a_i(src_a)
      ,.opd_b_i(src_b)
      ,.alu_op_i(decoded_inst_lo.alu_op)
      ,.res_o(alu_res_lo)
      );

  // Branch Unit
  bp_cce_branch
    #(.width_p(`bp_cce_inst_gpr_width)
      ,.cce_pc_width_p(cce_pc_width_p)
      )
    branch
     (.opd_a_i(src_a)
      ,.opd_b_i(src_b)
      ,.branch_i(decoded_inst_lo.branch)
      ,.predicted_taken_i(decoded_inst_lo.predict_taken)
      ,.branch_op_i(decoded_inst_lo.branch_op)
      ,.execute_pc_i(ex_pc_lo)
      ,.branch_target_i(decoded_inst_lo.branch_target)
      ,.branch_res_o(branch_res_lo)
      ,.mispredict_o(branch_mispredict_lo)
      ,.pc_o(branch_resolution_pc_lo)
      );

  // Pending Bits
  logic pending_li, pending_lo;
  bp_cce_pending
    #(.num_way_groups_p(num_way_groups_lp) // TODO: number of way groups managed by this CCE
      ,.num_cce_p(num_cce_p)
      ,.addr_width_p()
     )
    pending_bits
     (.clk_i(clk_i)
      ,.reset_i(reset_i)
      ,.w_v_i(pending_w_v_li)
      ,.w_addr_i()
      ,.w_addr_bypass_hash_i()
      ,.pending_i(pending_li)
      ,.clear_i(decoded_inst_lo.pending_clear)
      ,.r_v_i()
      ,.r_addr_i()
      ,.r_addr_bypass_hash_i()
      ,.pending_o(pending_lo)
      );

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

  // TODO: these should come from src_sel outputs
  logic [lg_lce_sets_lp-1:0] dir_set_li;
  logic [lg_num_lce_lp-1:0] dir_lce_li;
  logic [lg_lce_assoc_lp-1:0] dir_way_li;
  logic [tag_width_lp-1:0] dir_tag_li;
  logic [`bp_coh_bits-1:0] dir_coh_state_li;

  // Pending Bit Signals
  logic pending_lo;
  logic pending_v_lo;
  logic pending_li, pending_from_msg;
  logic pending_w_v_li, pending_w_v_from_msg;
  logic [lg_num_way_groups_lp-1:0] pending_w_way_group_li, pending_r_way_group_li, pending_w_way_group_from_msg;
  assign pending_r_way_group_li = dir_set_li;

  // WDP write is valid if instruction pending_w_v is set (WDP op) and decoder is not stalling
  // the instruction (due to mem data resp writing pending bit)
  logic wdp_pending_w_v;
  assign wdp_pending_w_v = decoded_inst_lo.pending_w_v & ~pc_stall_lo;
  assign pending_li = (wdp_pending_w_v) ? decoded_inst_lo.imm[0] : pending_from_msg;
  assign pending_w_v_li = (wdp_pending_w_v) ? decoded_inst_lo.pending_w_v : pending_w_v_from_msg;
  assign pending_w_way_group_li = (wdp_pending_w_v) ? dir_set_li : pending_w_way_group_from_msg;

  // Register signals
  logic null_wb_flag_li;
  assign null_wb_flag_li = (lce_resp_v_i & (lce_resp_li.msg_type == e_lce_cce_resp_null_wb));

  // Message Unit Signals
  logic                                          fence_zero_lo;
  logic                                          pending_w_busy_from_msg;
  logic                                          lce_cmd_busy_from_msg;
  logic                                          msg_inv_busy_lo;
  logic                                          msg_dir_w_v_lo;
  logic [lg_num_lce_lp-1:0]                      inv_dir_lce_lo;
  logic [lg_lce_assoc_lp-1:0]                    inv_dir_way_lo;


  // convert address (excluding block offset bits) into CCE local set index
  // TODO: for now, there is a 1:1 ratio of sets stored in directory and way groups
  // - when this changes, or when adding support for multiple LCE organizations, one bsg_hash_bank
  //   will be needed to map address to way group, and one per directory to map address to LCE set
  logic [lg_num_cce_lp-1:0] cce_dst_id_lo;
  localparam hash_index_width_lp=$clog2((2**lg_lce_sets_lp+num_cce_p-1)/num_cce_p);
  logic [hash_index_width_lp-1:0] cce_set_id_lo;
  logic [paddr_width_p-1:0] hash_addr_li;
  assign hash_addr_li =
    (decoded_inst_lo.dir_way_group_sel == e_dir_wg_sel_req_addr) ? mshr.paddr : mshr.lru_paddr;
  wire [lg_lce_sets_lp-1:0] hash_addr_rev =
    {<< {hash_addr_li[lg_block_size_in_bytes_lp+:lg_lce_sets_lp]}};

  bsg_hash_bank
    #(.banks_p(num_cce_p) // number of CCE's to spread way groups over
      ,.width_p(lg_lce_sets_lp) // width of address input
      )
    addr_to_cce_id
     (.i(hash_addr_rev)
      ,.bank_o(cce_dst_id_lo)
      ,.index_o(cce_set_id_lo)
      );

  // Directory
  bp_cce_dir
    #(.sets_p(num_way_groups_lp)
      ,.num_lce_p(num_lce_p)
      ,.lce_assoc_p(lce_assoc_p)
      ,.tag_width_p(tag_width_lp)
      )
    directory
     (.clk_i(clk_i)
      ,.reset_i(reset_i)

      ,.set_i(dir_set_li[0+:lg_num_way_groups_lp])
      ,.lce_i(dir_lce_li)
      ,.way_i(dir_way_li)
      ,.lru_way_i(mshr.lru_way_id)

      ,.r_cmd_i(decoded_inst_lo.dir_op)
      ,.r_v_i(decoded_inst_lo.dir_r_v)

      ,.tag_i(dir_tag_li)
      ,.coh_state_i(dir_coh_state_li)

      ,.w_cmd_i(decoded_inst_lo.dir_op)
      ,.w_v_i(decoded_inst_lo.dir_w_v | msg_dir_w_v_lo)
      ,.w_clr_row_i('0)

      ,.sharers_v_o(sharers_v_lo)
      ,.sharers_hits_o(sharers_hits_lo)
      ,.sharers_ways_o(sharers_ways_lo)
      ,.sharers_coh_states_o(sharers_coh_states_lo)

      ,.lru_v_o(dir_lru_v_lo)
      ,.lru_cached_excl_o(dir_lru_cached_excl_lo)
      ,.lru_tag_o(dir_lru_tag_lo)

      ,.busy_o(dir_busy_lo)

      ,.tag_o(dir_tag_lo)

      );

  // GAD logic - auxiliary directory information logic
  bp_cce_gad
    #(.num_lce_p(num_lce_p)
      ,.lce_assoc_p(lce_max_assoc_p)
      )
    gad
     (.clk_i(clk_i)
      ,.reset_i(reset_i)
      ,.gad_v_i(decoded_inst_lo.gad_v)

      ,.sharers_v_i(sharers_v_lo)
      ,.sharers_hits_i(sharers_hits_lo)
      ,.sharers_ways_i(sharers_ways_lo)
      ,.sharers_coh_states_i(sharers_coh_states_lo)

      ,.req_lce_i(lg_num_lce_lp'(mshr.lce_id))
      ,.req_type_flag_i(mshr.flags[e_flag_sel_rqf])
      ,.lru_dirty_flag_i(mshr.flags[e_flag_sel_ldf])
      ,.lru_cached_excl_flag_i(mshr.flags[e_flag_sel_lef])

      ,.req_addr_way_o(gad_req_addr_way_lo)

      ,.transfer_flag_o(gad_transfer_flag_lo)
      ,.transfer_lce_o(gad_transfer_lce_lo)
      ,.transfer_way_o(gad_transfer_lce_way_lo)
      ,.replacement_flag_o(gad_replacement_flag_lo)
      ,.upgrade_flag_o(gad_upgrade_flag_lo)
      ,.invalidate_flag_o(gad_invalidate_flag_lo)
      ,.cached_flag_o(gad_cached_flag_lo)
      ,.cached_exclusive_flag_o(gad_cached_exclusive_flag_lo)
      ,.cached_owned_flag_o(gad_cached_owned_flag_lo)
      ,.cached_dirty_flag_o(gad_cached_dirty_flag_lo)
      );

  // Registers
  bp_cce_reg
    #(.bp_params_p(bp_params_p))
    registers
     (.clk_i(clk_i)
      ,.reset_i(reset_i)
      ,.decoded_inst_i(decoded_inst_lo)
      ,.lce_req_i(lce_req_li)
      ,.null_wb_flag_i(null_wb_flag_li)
      ,.lce_resp_type_i(lce_resp_li.msg_type)
      ,.mem_resp_type_i(mem_resp_li.msg_type)
      ,.alu_res_i(alu_res_lo)
      ,.mov_src_i(src_a)

      ,.pending_o_i(pending_lo)
      ,.pending_v_o_i(pending_v_lo)
      ,.dir_lru_v_i(dir_lru_v_lo)
      ,.dir_lru_cached_excl_i(dir_lru_cached_excl_lo)
      ,.dir_lru_tag_i(dir_lru_tag_lo)
      ,.dir_tag_i(dir_tag_lo)

      ,.gad_req_addr_way_i(gad_req_addr_way_lo)
      ,.gad_transfer_lce_i(gad_transfer_lce_lo)
      ,.gad_transfer_lce_way_i(gad_transfer_lce_way_lo)
      ,.gad_transfer_flag_i(gad_transfer_flag_lo)
      ,.gad_replacement_flag_i(gad_replacement_flag_lo)
      ,.gad_upgrade_flag_i(gad_upgrade_flag_lo)
      ,.gad_invalidate_flag_i(gad_invalidate_flag_lo)
      ,.gad_cached_flag_i(gad_cached_flag_lo)
      ,.gad_cached_exclusive_flag_i(gad_cached_exclusive_flag_lo)
      ,.gad_cached_owned_flag_i(gad_cached_owned_flag_lo)
      ,.gad_cached_dirty_flag_i(gad_cached_dirty_flag_lo)

      // register state outputs
      ,.mshr_o(mshr)
      ,.gpr_o(gpr_r_lo)
      ,.nc_data_o(nc_data_r_lo)
      ,.coh_state_o(coh_state_r_lo)
      );

  // Message unit
  bp_cce_msg
    #(.bp_params_p(bp_params_p)
      )
    bp_cce_msg
     (.clk_i(clk_i)
      ,.reset_i(reset_i)

      ,.cfg_bus_i(cfg_bus_i)

      // To CCE
      ,.lce_req_i(lce_req_li)
      ,.lce_req_v_i(lce_req_v_i)
      ,.lce_req_yumi_o(lce_req_yumi_o)

      ,.lce_resp_i(lce_resp_li)
      ,.lce_resp_v_i(lce_resp_v_i)
      ,.lce_resp_yumi_o(lce_resp_yumi_o)

      // From CCE
      ,.lce_cmd_o(lce_cmd_lo)
      ,.lce_cmd_v_o(lce_cmd_v_o)
      ,.lce_cmd_ready_i(lce_cmd_ready_i)

      // To CCE
      ,.mem_resp_i(mem_resp_i)
      ,.mem_resp_v_i(mem_resp_v_i)
      ,.mem_resp_yumi_o(mem_resp_yumi_o)

      // From CCE
      ,.mem_cmd_o(mem_cmd_lo)
      ,.mem_cmd_v_o(mem_cmd_v_o)
      ,.mem_cmd_ready_i(mem_cmd_ready_i)

      ,.mshr_i(mshr)
      ,.decoded_inst_i(decoded_inst_lo)

      ,.pending_w_v_o(pending_w_v_from_msg)
      ,.pending_w_way_group_o(pending_w_way_group_from_msg)
      ,.pending_o(pending_from_msg)

      ,.pending_w_busy_o(pending_w_busy_from_msg)
      ,.lce_cmd_busy_o(lce_cmd_busy_from_msg)
      ,.msg_inv_busy_o(msg_inv_busy_lo)

      ,.gpr_i(gpr_r_lo)
      ,.sharers_hits_i(sharers_hits_lo)
      ,.sharers_ways_i(sharers_ways_lo)
      ,.nc_data_i(nc_data_r_lo)

      ,.fence_zero_o(fence_zero_lo)

      ,.lce_id_o(inv_dir_lce_lo)
      ,.lce_way_o(inv_dir_way_lo)

      ,.dir_w_v_o(msg_dir_w_v_lo)
      );



  // TODO: ensure branch flag operations are implemented/decoded properly
  // Especially setup of their sources

  // Branch multiple-flag operands
  // Apply bit-mask from instruction to MSHR flags register
  logic [`bp_cce_inst_num_flags-1:0] bf_opd;
  assign bf_opd = (mshr.flags & decoded_inst_lo.imm[0+:`bp_cce_inst_num_flags]);
  logic bf_and, bf_or;
  // All flags are set (AND) if the flags with mask applied is the same as the mask itself
  assign bf_and = (bf_opd == decoded_inst_lo.imm[0+:`bp_cce_inst_num_flags]);
  // Any flag is set (OR) if at least one bit is set in the masked flags
  assign bf_or = |bf_opd;

endmodule
