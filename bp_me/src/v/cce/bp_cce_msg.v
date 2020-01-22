/**
 *
 * Name:
 *   bp_cce_msg.v
 *
 * Description:
 *   This module handles sending and receiving of all messages in the CCE.
 *
 *   Processing of a Memory Data Response takes priority over processing of any other memory
 *   messages being sent or received. This arbitration is handled by the instruction decoder.
 *
 */

module bp_cce_msg
  import bp_common_pkg::*;
  import bp_common_aviary_pkg::*;
  import bp_cce_pkg::*;
  import bp_me_pkg::*;
  #(parameter bp_params_p                        = "inv"
    `declare_bp_proc_params(bp_params_p)

    // Derived parameters
    , localparam block_size_in_bytes_lp    = (cce_block_width_p/8)
    , localparam lg_num_lce_lp             = `BSG_SAFE_CLOG2(num_lce_p)
    , localparam lg_num_cce_lp             = `BSG_SAFE_CLOG2(num_cce_p)
    , localparam lg_block_size_in_bytes_lp = `BSG_SAFE_CLOG2(block_size_in_bytes_lp)
    , localparam lg_lce_assoc_lp           = `BSG_SAFE_CLOG2(lce_assoc_p)
    , localparam lg_lce_sets_lp            = `BSG_SAFE_CLOG2(lce_sets_p)
    , localparam num_way_groups_lp         = ((lce_sets_p % num_cce_p) == 0) ? (lce_sets_p/num_cce_p) : ((lce_sets_p/num_cce_p) + 1)
    , localparam lg_num_way_groups_lp      = `BSG_SAFE_CLOG2(num_way_groups_lp)
    , localparam mshr_width_lp = `bp_cce_mshr_width(lce_id_width_p, lce_assoc_p, paddr_width_p)

    , localparam cfg_bus_width_lp          = `bp_cfg_bus_width(vaddr_width_p, core_id_width_p, cce_id_width_p, lce_id_width_p, cce_pc_width_p, cce_instr_width_p)

    // interface widths
    `declare_bp_lce_cce_if_header_widths(cce_id_width_p, lce_id_width_p, lce_assoc_p, paddr_width_p)
    `declare_bp_lce_cce_if_widths(cce_id_width_p, lce_id_width_p, paddr_width_p, lce_assoc_p, dword_width_p, cce_block_width_p)
    `declare_bp_me_if_widths(paddr_width_p, cce_block_width_p, lce_id_width_p, lce_assoc_p)
  )
  (input                                               clk_i
   , input                                             reset_i

   // Configuration Interface
   , input [cfg_bus_width_lp-1:0]                      cfg_bus_i

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

   // Arbitration signals to stall unit
   // TODO: split busy signals into lce_req, lce_resp, mem_resp
   // TODO: split busy signals into lce_cmd, mem_cmd
   // TODO: add busy signal for invalidate commnad? or use msg_tx_busy
   , output logic                                      msg_tx_busy_o
   , output logic                                      msg_rx_busy_o
   , output logic                                      msg_spec_r_busy_o
   , output logic                                      msg_spec_w_busy_o

   // Pending bit write
   // TODO: busy signal differentiates between an auto-forwarding message writing
   // the bits, and an instruction writing the bits...
   // Do we need that?
   , output logic                                      pending_w_busy_o
   , output logic                                      pending_w_v_o
   , output logic [paddr_width_p-1:0]                  pending_w_addr_o
   , output logic                                      pending_w_addr_bypass_o
   , output logic                                      pending_o

   // Directory write interface
   // used when sending invalidates
   , output bp_cce_inst_minor_dir_op_e                 dir_w_cmd_o
   , output logic                                      dir_w_v_o
   , output logic [paddr_width_p-1:0]                  dir_w_addr_o
   , output logic                                      dir_w_addr_bypass_hash_o
   , output logic [lce_id_width_p-1:0]                 dir_lce_o
   , output logic [lce_assoc_width_lp-1:0]             dir_way_o
   , output bp_coh_states_e                            dir_coh_state_o

   // TODO: roll these into decoded instruction?
   // Input signals to feed output commands
   , input [lce_id_width_p-1:0]                        lce_i
   , input [paddr_width_p-1:0]                         addr_i
   , input [lg_assoc_lp-1:0]                           way_i
   , input [cce_lce_data_width_p-1:0]                  data_i
   , input bp_cce_mem_cmd_type_e                       mem_cmd_i
   , input bp_cce_mem_req_size_e                       mem_cmd_size_i
   , input bp_coh_states_e                             coh_state_i
   , input bp_lce_cmd_type_e                           lce_cmd_i
   , input bp_lce_cce_data_length_e                    lce_cmd_data_length_i

   // For invalidation command
   , input [num_lce_p-1:0]                             sharers_hits_i
   , input [num_lce_p-1:0][lg_lce_assoc_lp-1:0]        sharers_ways_i

   // Decoded Instruction
   , input bp_cce_inst_decoded_s                       decoded_inst_i


///////////////////////////// OLD //////////////////////////////


   // MSHR
   , input [mshr_width_lp-1:0]                         mshr_i


   // arbitration signals to instruction decode
   , output logic                                      lce_cmd_busy_o
   , output logic                                      msg_inv_busy_o

  );

  //synopsys translate_off
  initial begin
    assert(cce_lce_data_width_p == 64) else
      $error("CCE-LCE data width must be 64-bits");
    assert(cce_lce_data_width_p == cce_mem_data_width_p) else
      $error("CCE-LCE and CCE-MEM data widths must be the same");
  end
  //synopsys translate_on


  // Define structure variables for output queues
  `declare_bp_cfg_bus_s(vaddr_width_p, core_id_width_p, cce_id_width_p, lce_id_width_p, cce_pc_width_p, cce_instr_width_p);
  `declare_bp_me_if(paddr_width_p, cce_block_width_p, lce_id_width_p, lce_assoc_p);
  `declare_bp_lce_cce_if(cce_id_width_p, lce_id_width_p, paddr_width_p, lce_assoc_p, dword_width_p, cce_block_width_p);

  bp_cfg_bus_s cfg_bus_cast_i;

  assign cfg_bus_cast_i = cfg_bus_i;

  // Message Unit Signals
  bp_lce_cce_req_s                               lce_req_from_msg;
  logic                                          lce_req_v_from_msg, lce_req_yumi_from_msg;
  bp_lce_cce_resp_s                              lce_resp_from_msg;
  logic                                          lce_resp_v_from_msg, lce_resp_yumi_from_msg;
  bp_lce_cmd_s                                   lce_cmd_from_msg;
  logic                                          lce_cmd_v_from_msg, lce_cmd_ready_from_msg;
  bp_cce_mem_msg_s                               mem_cmd_from_msg;
  logic                                          mem_cmd_v_from_msg, mem_cmd_ready_from_msg;
  bp_cce_mem_msg_s                               mem_resp_from_msg;
  logic                                          mem_resp_v_from_msg, mem_resp_yumi_from_msg;

  // Uncached Module Signals
  bp_lce_cce_req_s                               lce_req_from_uc;
  logic                                          lce_req_v_from_uc, lce_req_yumi_from_uc;
  bp_lce_cmd_s                                   lce_cmd_from_uc;
  logic                                          lce_cmd_v_from_uc, lce_cmd_ready_from_uc;
  bp_cce_mem_msg_s                               mem_cmd_from_uc;
  logic                                          mem_cmd_v_from_uc, mem_cmd_ready_from_uc;
  bp_cce_mem_msg_s                               mem_resp_from_uc;
  logic                                          mem_resp_v_from_uc, mem_resp_yumi_from_uc;

  // Message unit
  bp_cce_msg_cached
    #(.bp_params_p(bp_params_p))
    bp_cce_msg_cached
     (.clk_i(clk_i)
      ,.reset_i(reset_i)

      ,.cce_id_i(cfg_bus_cast_i.cce_id)

      // To CCE
      ,.lce_req_i(lce_req_from_msg)
      ,.lce_req_v_i(lce_req_v_from_msg)
      ,.lce_req_yumi_o(lce_req_yumi_from_msg)

      ,.lce_resp_i(lce_resp_from_msg)
      ,.lce_resp_v_i(lce_resp_v_from_msg)
      ,.lce_resp_yumi_o(lce_resp_yumi_from_msg)

      // From CCE
      ,.lce_cmd_o(lce_cmd_from_msg)
      ,.lce_cmd_v_o(lce_cmd_v_from_msg)
      ,.lce_cmd_ready_i(lce_cmd_ready_from_msg)

      // To CCE
      ,.mem_resp_i(mem_resp_from_msg)
      ,.mem_resp_v_i(mem_resp_v_from_msg)
      ,.mem_resp_yumi_o(mem_resp_yumi_from_msg)

      // From CCE
      ,.mem_cmd_o(mem_cmd_from_msg)
      ,.mem_cmd_v_o(mem_cmd_v_from_msg)
      ,.mem_cmd_ready_i(mem_cmd_ready_from_msg)

      ,.mshr_i(mshr_i)
      ,.decoded_inst_i(decoded_inst_i)

      ,.pending_w_v_o(pending_w_v_o)
      ,.pending_w_way_group_o(pending_w_way_group_o)
      ,.pending_o(pending_o)

      ,.pending_w_busy_o(pending_w_busy_o)
      ,.lce_cmd_busy_o(lce_cmd_busy_o)
      ,.msg_inv_busy_o(msg_inv_busy_o)

      ,.gpr_i(gpr_i)
      ,.sharers_hits_i(sharers_hits_i)
      ,.sharers_ways_i(sharers_ways_i)
      ,.nc_data_i(nc_data_i)

      ,.fence_zero_o(fence_zero_o)

      ,.lce_id_o(lce_id_o)
      ,.lce_way_o(lce_way_o)

      ,.dir_w_v_o(dir_w_v_o)
      );

  // Uncached access module
  bp_cce_msg_uncached
    #(.bp_params_p(bp_params_p))
    bp_cce_msg_uncached
     (.clk_i(clk_i)
      ,.reset_i(reset_i)

      ,.cce_id_i(cfg_bus_cast_i.cce_id)

      // To CCE
      ,.lce_req_i(lce_req_from_uc)
      ,.lce_req_v_i(lce_req_v_from_uc)
      ,.lce_req_yumi_o(lce_req_yumi_from_uc)

      // From CCE
      ,.lce_cmd_o(lce_cmd_from_uc)
      ,.lce_cmd_v_o(lce_cmd_v_from_uc)
      ,.lce_cmd_ready_i(lce_cmd_ready_from_uc)

      // To CCE
      ,.mem_resp_i(mem_resp_from_uc)
      ,.mem_resp_v_i(mem_resp_v_from_uc)
      ,.mem_resp_yumi_o(mem_resp_yumi_from_uc)

      // From CCE
      ,.mem_cmd_o(mem_cmd_from_uc)
      ,.mem_cmd_v_o(mem_cmd_v_from_uc)
      ,.mem_cmd_ready_i(mem_cmd_ready_from_uc)
      );

  // Need to resolve the last outstanding msg during a mode switch
  logic uncached_outstanding;
  always_ff @(posedge clk_i)
    begin
      if (reset_i)
        uncached_outstanding <= '0;
      else if (mem_cmd_v_from_uc | mem_resp_yumi_from_uc)
        uncached_outstanding <= ~mem_resp_yumi_from_uc;
    end

  // Output Message Formation
  //
  // Input messages to the CCE are buffered by two element FIFOs in bp_cce_buffered.v, thus
  // the outbound signal is a yumi.
  //
  // Outbound queues all use ready&valid handshaking. Outbound messages going to LCEs are not
  // buffered by bp_cce_buffered.v, but messages to memory are.
  always_comb
  begin
    {lce_req_from_uc, lce_req_v_from_uc, lce_cmd_ready_from_uc} = '0;
    {mem_cmd_ready_from_uc, mem_resp_from_uc, mem_resp_v_from_uc} = '0;

    {lce_req_from_msg, lce_req_v_from_msg, lce_resp_from_msg, lce_resp_v_from_msg, lce_cmd_ready_from_msg} = '0;
    {mem_cmd_ready_from_msg, mem_resp_from_msg, mem_resp_v_from_msg} = '0;
    if (uncached_outstanding || (cfg_bus_cast_i.cce_mode == e_cce_mode_uncached)) begin
      lce_req_from_uc = lce_req_i;
      lce_req_v_from_uc = lce_req_v_i;
      lce_req_yumi_o = lce_req_yumi_from_uc;

      lce_cmd_o = lce_cmd_from_uc;
      lce_cmd_v_o = lce_cmd_v_from_uc;
      lce_cmd_ready_from_uc = lce_cmd_ready_i;

      lce_resp_yumi_o = '0;

      mem_cmd_o = mem_cmd_from_uc;
      mem_cmd_v_o = mem_cmd_v_from_uc;
      mem_cmd_ready_from_uc = mem_cmd_ready_i;

      mem_resp_from_uc = mem_resp_i;
      mem_resp_v_from_uc = mem_resp_v_i;
      mem_resp_yumi_o = mem_resp_yumi_from_uc;
    end else begin
      lce_req_from_msg = lce_req_i;
      lce_req_v_from_msg = lce_req_v_i;
      lce_req_yumi_o = lce_req_yumi_from_msg;

      lce_cmd_o = lce_cmd_from_msg;
      lce_cmd_v_o = lce_cmd_v_from_msg;
      lce_cmd_ready_from_msg = lce_cmd_ready_i;

      lce_resp_from_msg = lce_resp_i;
      lce_resp_v_from_msg = lce_resp_v_i;
      lce_resp_yumi_o = lce_resp_yumi_from_msg;

      mem_cmd_o = mem_cmd_from_msg;
      mem_cmd_v_o = mem_cmd_v_from_msg;
      mem_cmd_ready_from_msg = mem_cmd_ready_i;

      mem_resp_from_msg = mem_resp_i;
      mem_resp_v_from_msg = mem_resp_v_i;
      mem_resp_yumi_o = mem_resp_yumi_from_msg;
    end
  end

endmodule
