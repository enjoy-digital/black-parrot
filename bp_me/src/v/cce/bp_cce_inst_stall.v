/**
 *
 * Name:
 *   bp_cce_inst_stall.v
 *
 * Description:
 *   The stall unit collects the current instruction and status of other functional units to
 *   determine whether or not a stall must occur.
 *
 *   A stall prevents any changes to architectural state in the cycle asserted, and causes the
 *   current instruction to be replayed next cycle.
 */

module bp_cce_inst_stall
  import bp_cce_pkg::*;
  #(parameter cnt_width_p        = "inv")
  (input                                         clk_i
   , input                                       reset_i
   , input bp_cce_inst_decoded_s                 decoded_inst_i

   // TODO: other input signals for stall detection:
   // message Rx/Tx busy signals

   // input queue valid signals
   , input                                       lce_req_v_i
   , input                                       lce_resp_v_i
   , input                                       mem_resp_v_i
   , input                                       pending_v_i

   // output queue ready signals
   , input                                       lce_cmd_ready_i
   , input                                       mem_cmd_ready_i

   // Messague Unit resource busy signals
   , input                                       msg_pending_w_busy_i
   , input                                       msg_lce_cmd_busy_i
   , input                                       msg_lce_resp_busy_i
   // Directory busy (e.g., processing read)
   , input                                       dir_busy_i

   , output logic                                stall_o
   // TODO: move stall counter to bp_cce_perfmon?
   , output logic [cnt_width_p-1:0]              stall_count_o
  );

  logic [`BSG_SAFE_CLOG2((2**cnt_width_p)+1)-1:0] cnt_lo;
  bsg_counter_clear_up
    #(.max_val_p(2**cnt_width_p)
      ,.init_val_p(0)
      )
    stall_counter
     (.clk_i(clk_i)
      ,.reset_i(reset_i)
      ,.clear_i(decoded_inst_i.clr_stall_cnt)
      ,.up_i(stall_o)
      ,.count_o(cnt_lo)
      );
  assign stall_count_o = cnt_lo[0+:cnt_width_p];

  wire [`bp_cce_num_src_q-1:0] wfq_v_vec = {lce_req_v_i, lce_resp_v_i, mem_resp_v_i, pending_v_i};
  wire [`bp_cce_num_src_q-1:0] wfq_mask = decoded_inst_i.imm[0+:`bp_cce_num_src_q];

  always_comb begin
    stall_o = 1'b0;

    // Microcode instruction stalls - resource not ready

    // Message receive
    stall_o |= (decoded_inst_i.lce_req_yumi & ~lce_req_v_i);
    stall_o |= (decoded_inst_i.lce_resp_yumi & ~lce_resp_v_i);
    stall_o |= (decoded_inst_i.mem_resp_yumi & ~mem_resp_v_i);
    stall_o |= (decoded_inst_i.pending_yumi & ~pending_v_i);

    // Message send
    stall_o |= (decoded_inst_i.lce_cmd_v & ~lce_cmd_ready_i);
    stall_o |= (decoded_inst_i.mem_cmd_v & ~mem_cmd_ready_i);

    // Wait for queue operation
    stall_o |= (decoded_inst_i.wfq_v & ~(|(wfq_mask & wfq_v_vec)));


    // Functional Unit induced stalls

    // Directory is busy after a read - be safe and block execution until read is done
    stall_o |= dir_busy_i;

    // Message Unit Structural Hazards
    stall_o |= (decoded_inst_i.pending_w_v & msg_pending_w_busy_i);
    stall_o |= (decoded_inst_i.lce_cmd_v & msg_lce_cmd_busy_i);
    stall_o |= (decoded_inst_i.lce_resp_yumi & msg_lce_resp_busy_i);

    // Only stall if the current instruction is valid
    // TODO: not needed because decoder outputs decoded_inst == '0 if instruction not valid

  end

endmodule
