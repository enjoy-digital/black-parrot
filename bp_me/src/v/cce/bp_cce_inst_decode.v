/**
 *
 * Name:
 *   bp_cce_inst_decode.v
 *
 * Description:
 *   The decoder holds the decode+execute stage PC, instruction, and valid bit. The decoder also
 *   contains the instruction decode logic and outputs the decoded instruction used to control
 *   all of the other modules in the CCE.
 *
 *   The decoder does not check if the instruction can actually be executed, rather it outputs
 *   the instruction assuming it will be executed. The stall unit and other arbitration logic
 *   in the CCE will nullify the instruction if it cannot be executed this cycle. The decoder
 *   is informed of this through the stall_i signal, which causes the current PC, instruction,
 *   and valid bit to be retained at the end of the cycle. The current instruction is then
 *   replayed next cycle.
 *
 *   A mispredict event detected by the branch unit causes the next instruction to be invalid
 *   since the Fetch stage must re-direct instruction fetch. The instruction being produced
 *   by Fetch when a mispredict is detected becomes invalid and therefore there is a 1 cycle
 *   bubble/penalty for a mispredicted branch.
 *
 */

module bp_cce_inst_decode
  import bp_common_pkg::*;
  import bp_cce_pkg::*;
  import bp_me_pkg::*;
  #(parameter cce_pc_width_p = "inv"
  )
  (input                                         clk_i
   , input                                       reset_i

   // Instruction, PC, and valid bit from bp_cce_inst_ram
   , input bp_cce_inst_s                         inst_i
   , input [cce_pc_width_p-1:0]                  pc_i
   , input                                       inst_v_i

   // Stall signal from stall detection unit
   , input                                       stall_i

   // Mispredict signal from branch unit
   , input                                       mispredict_i

   // Decoded instruction
   , output bp_cce_inst_decoded_s                decoded_inst_o
   , output logic [cce_pc_width_p-1:0]           pc_o

  );


  // Execute Stage Instruction Register and PC
  bp_cce_inst_s inst_r, inst_n;
  logic [cce_pc_width_p-1:0] ex_pc_r, ex_pc_n;
  logic inst_v_r, inst_v_n;
  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      inst_r <= '0;
      ex_pc_r <= '0;
      inst_v_r <= '0;
    end else begin
      inst_r <= inst_n;
      ex_pc_r <= ex_pc_n;
      inst_v_r <= inst_v_n;
    end
  end

  always_comb begin

    // Next instruction determination

    // The next instruction and its PC that will be seen by the Execute stage comes from
    // the output of the instruction RAM and the fetch_pc register in bp_cce_inst_ram.
    inst_n = stall_i ? inst_r : inst_i;
    ex_pc_n = stall_i ? ex_pc_r : pc_i;
    // The next instruction is valid as long as there was not a mispredict detected in the
    // Execute stage. A mispredict squashes the next instruction, by setting the valid bit
    // to 0 for the next cycle, which then gates off the decoder next cycle.
    // If the current instruction stalls (which is detected after decoding due to interactions
    // between the current ucode instruction and resource conflicts with functional units),
    // the stall signal is sent back to the Fetch stage and the current instruction, PC, and
    // valid bit are retained for the next cycle. The stall signal also causes the valid
    // bit to hold its state.
    inst_v_n = stall_i
               ? inst_v_r
               : mispredict_i
                 ? 1'b0
                 : inst_v_i;


    // Current instruction decoding

    pc_o = ex_pc_r;

    decoded_inst_o = '0;
    decoded_inst_o.v = inst_v_r;

    // only finish decoding if current instruction is valid
    if (inst_v_r) begin
      decoded_inst_o.branch = inst_r.branch;
      decoded_inst_o.predict_taken = inst_r.predict_taken;

      decoded_inst_o.op = inst_r.op;
      decoded_inst_o.minor_op_u = inst_r.minor_op_u;

      // TODO: decoder
    end

  end


  // TODO: everything below here


  // Instruction Fields
  bp_cce_inst_op_e             op;
  bp_cce_inst_minor_op_u       minor_op_u;
  bp_cce_inst_type_u           op_type_u;
  // Instruction types
  bp_cce_inst_alu_op_s         alu_op_s;
  bp_cce_inst_branch_op_s      branch_op_s;
  bp_cce_inst_mov_op_s         mov_op_s;
  bp_cce_inst_flag_op_s        flag_op_s;
  bp_cce_inst_dir_op_s         dir_op_s;
  bp_cce_inst_misc_op_s        misc_op_s;
  bp_cce_inst_queue_op_s       queue_op_s;

  logic pushq_op, popq_op, poph_op;
  bp_cce_inst_dst_q_sel_e pushq_qsel;
  bp_cce_inst_src_q_sel_e popq_qsel;

  logic wfq_op;
  logic wfq_q_ready;
  logic stall_op;
  logic gpr_w_v;
  logic wdp_op;
  logic fence_op;

  // Control outputs
  always_comb
  begin
    gpr_w_v = '0;

    // reinterpret wires as instruction struct and fields
    inst = inst_i;
    op = inst.op;
    minor_op_u = inst.minor_op_u;
    op_type_u = inst.type_u;
    alu_op_s = op_type_u.alu_op_s;
    branch_op_s = op_type_u.branch_op_s;
    mov_op_s = op_type_u.mov_op_s;
    flag_op_s = op_type_u.flag_op_s;
    dir_op_s = op_type_u.dir_op_s;
    misc_op_s = op_type_u.misc_op_s;
    queue_op_s = op_type_u.queue_op_s;

    // Defaults for outputs
    decoded_inst_v_o = '0;
    decoded_inst_o = '0;
    pc_stall_o = '0;
    pc_branch_target_o = '0;

    // Pushq and Popq operation details - used for decoding and fetch control
    pushq_op = (op == e_op_queue) & (minor_op_u == e_pushq_op);
    popq_op = (op == e_op_queue) & (minor_op_u == e_popq_op);
    poph_op = (op == e_op_queue) & (minor_op_u == e_poph_op);
    pushq_qsel = queue_op_s.op.pushq.dst_q;
    popq_qsel = queue_op_s.op.popq.src_q;

    decoded_inst_v_o = inst_v_i;
    decoded_inst_o.op = op;
    decoded_inst_o.minor_op_u = minor_op_u;

    case (op)
      e_op_alu: begin
        decoded_inst_o.alu_v = 1'b1;
        // All ALU arithmetic operations write a GPR destination
        decoded_inst_o.alu_dst_w_v = 1'b1;
        decoded_inst_o.imm[0+:`bp_cce_inst_imm16_width] = alu_op_s.imm;
        // Dst and Src fields are GPRs or immediate (src only)
        decoded_inst_o.dst.gpr = alu_op_s.dst;
        decoded_inst_o.src_a.gpr = alu_op_s.src_a;
        decoded_inst_o.src_b.gpr = alu_op_s.src_b;

        decoded_inst_o.dst_sel = e_dst_sel_gpr;
        decoded_inst_o.src_a_sel = e_src_sel_gpr;
        decoded_inst_o.src_b_sel = e_src_sel_gpr;

      end
      e_op_branch: begin

        decoded_inst_o.branch_v = 1'b1;
        // Next PC computation
        decoded_inst_o.imm[0+:`bp_cce_inst_imm16_width] = branch_op_s.imm;
        pc_branch_target_o = branch_op_s.target[0+:cce_pc_width_p];

        // Default to GPR sources
        decoded_inst_o.src_a.gpr = branch_op_s.src_a.gpr;
        decoded_inst_o.src_b.gpr = branch_op_s.src_b.gpr;
        decoded_inst_o.src_a_sel = e_src_sel_gpr;
        decoded_inst_o.src_b_sel = e_src_sel_gpr;

        // Flag ops use flag as source A, immediate of 1 or 0 as source B
        if (minor_op_u.branch_minor_op == e_bf_op) begin
          decoded_inst_o.src_a.flag = branch_op_s.src_a.flag;
          decoded_inst_o.src_a_sel = e_src_sel_flag;

          if (branch_op_s.src_a.flag == e_src_flag_and
              | branch_op_s.src_a.flag == e_src_flag_nand
              | branch_op_s.src_a.flag == e_src_flag_or
              | branch_op_s.src_a.flag == e_src_flag_nor) begin
            decoded_inst_o.src_b.special = branch_op_s.src_b.special;
            decoded_inst_o.src_b_sel = e_src_sel_special;
          end

        // Branch if queue.ready set, source B is immediate set to 1
        end else if (minor_op_u.branch_minor_op == e_bqv_op) begin
          decoded_inst_o.src_a.special = branch_op_s.src_a.special;
          decoded_inst_o.src_a_sel = e_src_sel_special;

        // Branch if special register equal to GPR/imm encoded src_b
        end else if (minor_op_u.branch_minor_op == e_bs_op) begin
          decoded_inst_o.src_a.special = branch_op_s.src_a.special;
          decoded_inst_o.src_a_sel = e_src_sel_special;

        end

      end
      e_op_move: begin
        decoded_inst_o.mov_dst_w_v = 1'b1;

        if (minor_op_u.mov_minor_op == e_movi_op) begin
          decoded_inst_o.dst.gpr = mov_op_s.dst.gpr;
          decoded_inst_o.dst_sel = e_dst_sel_gpr;
          decoded_inst_o.src_a.gpr = e_src_gpr_imm;
          decoded_inst_o.src_a_sel = e_src_sel_gpr;
          decoded_inst_o.imm[0+:`bp_cce_inst_imm32_width] = mov_op_s.op.movi.imm;

        end else if (minor_op_u.mov_minor_op == e_movis_op) begin
          decoded_inst_o.dst.special = mov_op_s.dst.special;
          decoded_inst_o.dst_sel = e_dst_sel_special;
          decoded_inst_o.src_a.gpr = e_src_gpr_imm;
          decoded_inst_o.src_a_sel = e_src_sel_gpr;
          decoded_inst_o.imm[0+:`bp_cce_inst_imm32_width] = mov_op_s.op.movi.imm;

        end else if (minor_op_u.mov_minor_op == e_mov_op) begin
          // Dst and Src fields are GPRs
          decoded_inst_o.dst.gpr = mov_op_s.dst.gpr;
          decoded_inst_o.dst_sel = e_dst_sel_gpr;
          decoded_inst_o.src_a.gpr = mov_op_s.op.mov.src.gpr;
          decoded_inst_o.src_a_sel = e_src_sel_gpr;

        end else if (minor_op_u.mov_minor_op == e_movf_op) begin
          // move flag to gpr - src_a is a flag
          decoded_inst_o.dst.gpr = mov_op_s.dst.gpr;
          decoded_inst_o.dst_sel = e_dst_sel_gpr;
          decoded_inst_o.src_a.flag = mov_op_s.op.mov.src.flag;
          decoded_inst_o.src_a_sel = e_src_sel_flag;

        end else if (minor_op_u.mov_minor_op == e_movsg_op) begin
          decoded_inst_o.dst.gpr = mov_op_s.dst.gpr;
          decoded_inst_o.dst_sel = e_dst_sel_gpr;
          decoded_inst_o.src_a.special = mov_op_s.op.mov.src.special;
          decoded_inst_o.src_a_sel = e_src_sel_special;

        end else if (minor_op_u.mov_minor_op == e_movgs_op) begin
          decoded_inst_o.dst.special = mov_op_s.dst.special;
          decoded_inst_o.dst_sel = e_dst_sel_special;
          decoded_inst_o.src_a.gpr = mov_op_s.op.mov.src.gpr;
          decoded_inst_o.src_a_sel = e_src_sel_gpr;

        end else begin
          decoded_inst_o.mov_dst_w_v = 1'b0;
        end

      end
      e_op_flag: begin

        // Flag ops always use flag for src_a and src_b
        decoded_inst_o.src_a.flag = flag_op_s.src_a;
        decoded_inst_o.src_a_sel = e_src_sel_flag;

        decoded_inst_o.src_b.flag = flag_op_s.src_b;
        decoded_inst_o.src_b_sel = e_src_sel_flag;

        // immediate bit 0 always from instruction immediate
        decoded_inst_o.imm[0] = flag_op_s.val;

        // destination - by default, destination is a flag register
        decoded_inst_o.dst.flag = flag_op_s.dst.flag;
        decoded_inst_o.dst_sel = e_dst_sel_flag;

        if (minor_op_u.flag_minor_op == e_andf_op) begin
          decoded_inst_o.minor_op_u.alu_minor_op = e_and_op;
          decoded_inst_o.dst.gpr = flag_op_s.dst.gpr;
          decoded_inst_o.dst_sel = e_dst_sel_gpr;
          decoded_inst_o.alu_v = 1'b1;
          decoded_inst_o.alu_dst_w_v = 1'b1;
        end else if (minor_op_u.flag_minor_op == e_orf_op) begin
          decoded_inst_o.minor_op_u.alu_minor_op = e_or_op;
          decoded_inst_o.dst.gpr = flag_op_s.dst.gpr;
          decoded_inst_o.dst_sel = e_dst_sel_gpr;
          decoded_inst_o.alu_v = 1'b1;
          decoded_inst_o.alu_dst_w_v = 1'b1;
        end else begin
          if (flag_op_s.dst == e_dst_rqf) begin
            decoded_inst_o.rqf_sel = e_rqf_imm0;
            decoded_inst_o.flag_mask_w_v = e_flag_rqf;
          end else if (flag_op_s.dst == e_dst_ucf) begin
            decoded_inst_o.rqf_sel = e_rqf_imm0;
            decoded_inst_o.flag_mask_w_v = e_flag_ucf;
          end else if (flag_op_s.dst == e_dst_nerf) begin
            decoded_inst_o.nerf_sel = e_nerf_imm0;
            decoded_inst_o.flag_mask_w_v = e_flag_nerf;
          end else if (flag_op_s.dst == e_dst_ldf) begin
            decoded_inst_o.ldf_sel = e_ldf_imm0;
            decoded_inst_o.flag_mask_w_v = e_flag_ldf;
          end else if (flag_op_s.dst == e_dst_nwbf) begin
            decoded_inst_o.nwbf_sel = e_nwbf_imm0;
            decoded_inst_o.flag_mask_w_v = e_flag_nwbf;
          end else if (flag_op_s.dst == e_dst_tf) begin
            decoded_inst_o.tf_sel = e_tf_imm0;
            decoded_inst_o.flag_mask_w_v = e_flag_tf;
          end else if (flag_op_s.dst == e_dst_rf) begin
            decoded_inst_o.rf_sel = e_rf_imm0;
            decoded_inst_o.flag_mask_w_v = e_flag_rf;
          end else if (flag_op_s.dst == e_dst_pf) begin
            decoded_inst_o.pf_sel = e_pf_imm0;
            decoded_inst_o.flag_mask_w_v = e_flag_pf;
          end else if (flag_op_s.dst == e_dst_uf) begin
            decoded_inst_o.uf_sel = e_uf_imm0;
            decoded_inst_o.flag_mask_w_v = e_flag_uf;
          end else if (flag_op_s.dst == e_dst_if) begin
            decoded_inst_o.if_sel = e_if_imm0;
            decoded_inst_o.flag_mask_w_v = e_flag_if;
          end else if (flag_op_s.dst == e_dst_cf) begin
            decoded_inst_o.cf_sel = e_cf_imm0;
            decoded_inst_o.flag_mask_w_v = e_flag_cf;
          end else if (flag_op_s.dst == e_dst_cef) begin
            decoded_inst_o.cef_sel = e_cef_imm0;
            decoded_inst_o.flag_mask_w_v = e_flag_cef;
          end else if (flag_op_s.dst == e_dst_cof) begin
            decoded_inst_o.cof_sel = e_cof_imm0;
            decoded_inst_o.flag_mask_w_v = e_flag_cof;
          end else if (flag_op_s.dst == e_dst_cdf) begin
            decoded_inst_o.cdf_sel = e_cdf_imm0;
            decoded_inst_o.flag_mask_w_v = e_flag_cdf;
          end else if (flag_op_s.dst == e_dst_ucf) begin
            decoded_inst_o.ucf_sel = e_ucf_imm0;
            decoded_inst_o.flag_mask_w_v = e_flag_ucf;
          end else if (flag_op_s.dst == e_dst_sf) begin
            decoded_inst_o.sf_sel = e_sf_imm0;
            decoded_inst_o.flag_mask_w_v = e_flag_sf;
          end
        end

      end
      e_op_dir: begin
        // Directory input mux selects
        decoded_inst_o.dir_way_group_sel = dir_op_s.dir_way_group_sel;
        decoded_inst_o.dir_lce_sel = dir_op_s.dir_lce_sel;
        decoded_inst_o.dir_way_sel = dir_op_s.dir_way_sel;
        decoded_inst_o.dir_coh_state_sel = dir_op_s.dir_coh_state_sel;
        decoded_inst_o.dir_tag_sel = dir_op_s.dir_tag_sel;

        if (minor_op_u.dir_minor_op == e_rdp_op) begin
          decoded_inst_o.pending_r_v = 1'b1;
          decoded_inst_o.flag_mask_w_v = e_flag_pf;
          decoded_inst_o.dir_op = minor_op_u.dir_minor_op;
        end
        if (minor_op_u.dir_minor_op == e_rdw_op) begin
          decoded_inst_o.dir_r_v = 1'b1;
          decoded_inst_o.dir_op = minor_op_u.dir_minor_op;
        end
        if (minor_op_u.dir_minor_op == e_rde_op) begin
          decoded_inst_o.dir_r_v = 1'b1;
          decoded_inst_o.dst.gpr = dir_op_s.dst;
          decoded_inst_o.dst_sel = e_dst_sel_gpr;
          decoded_inst_o.rde_w_v = 1'b1;
          decoded_inst_o.dir_op = minor_op_u.dir_minor_op;
        end
        if (minor_op_u.dir_minor_op == e_wdp_op) begin
          decoded_inst_o.dir_w_v = 1'b1;
          decoded_inst_o.pending_w_v = 1'b1;
          decoded_inst_o.imm[0] = dir_op_s.pending;
          decoded_inst_o.dir_op = minor_op_u.dir_minor_op;
        end
        if (minor_op_u.dir_minor_op == e_wde_op) begin
          decoded_inst_o.dir_w_v = 1'b1;
          decoded_inst_o.imm[0+:`bp_coh_bits] = dir_op_s.state;
          decoded_inst_o.dir_op = minor_op_u.dir_minor_op;
        end
        if (minor_op_u.dir_minor_op == e_wds_op) begin
          decoded_inst_o.dir_w_v = 1'b1;
          decoded_inst_o.imm[0+:`bp_coh_bits] = dir_op_s.state;
          decoded_inst_o.dir_op = minor_op_u.dir_minor_op;
        end
        if (minor_op_u.dir_minor_op == e_gad_op) begin
          decoded_inst_o.gad_v = 1'b1;
          decoded_inst_o.transfer_lce_w_v = 1'b1; // transfer_lce, transfer_lce_way
          decoded_inst_o.req_addr_way_w_v = 1'b1; // req_addr_way
          decoded_inst_o.transfer_lce_w_v = 1'b1; // transfer_lce, transfer_lce_way
          decoded_inst_o.tf_sel = e_tf_logic;
          decoded_inst_o.rf_sel = e_rf_logic;
          decoded_inst_o.uf_sel = e_uf_logic;
          decoded_inst_o.if_sel = e_if_logic;
          decoded_inst_o.cf_sel = e_cf_logic;
          decoded_inst_o.cef_sel = e_cef_logic;
          decoded_inst_o.cof_sel = e_cof_logic;
          decoded_inst_o.cdf_sel = e_cdf_logic;
          decoded_inst_o.flag_mask_w_v =
            (e_flag_tf | e_flag_rf | e_flag_uf | e_flag_if | e_flag_cf | e_flag_cef
             | e_flag_cof | e_flag_cdf);
        end

      end
      e_op_misc: begin
        if (minor_op_u.misc_minor_op == e_clm_op) begin
          decoded_inst_o.mshr_clear = 1'b1;
        end
        else if (minor_op_u.misc_minor_op == e_fence_op) begin
          // do nothing
        end
        else if (minor_op_u.misc_minor_op == e_stall_op) begin
          // do nothing
        end
      end
      e_op_queue: begin
        if (minor_op_u.queue_minor_op == e_specq_op) begin
          decoded_inst_o.spec_cmd = queue_op_s.op.specq.cmd;
          decoded_inst_o.spec_w_v = 1'b1;
          case (queue_op_s.op.specq.cmd)
            e_spec_cmd_set: begin
              decoded_inst_o.spec_bits.spec = 1'b1;
              decoded_inst_o.spec_bits.squash = 1'b0;
              decoded_inst_o.spec_bits.fwd_mod = 1'b0;
              decoded_inst_o.spec_bits.state = '0;
            end
            e_spec_cmd_unset: begin
              decoded_inst_o.spec_bits.spec = 1'b0;
              decoded_inst_o.spec_bits.squash = 1'b0;
              decoded_inst_o.spec_bits.fwd_mod = 1'b0;
              decoded_inst_o.spec_bits.state = '0;
            end
            e_spec_cmd_squash: begin
              decoded_inst_o.spec_bits.spec = 1'b0;
              decoded_inst_o.spec_bits.squash = 1'b1;
              decoded_inst_o.spec_bits.fwd_mod = 1'b0;
              decoded_inst_o.spec_bits.state = '0;
            end
            e_spec_cmd_fwd_mod: begin
              decoded_inst_o.spec_bits.spec = 1'b0;
              decoded_inst_o.spec_bits.squash = 1'b0;
              decoded_inst_o.spec_bits.fwd_mod = 1'b1;
              decoded_inst_o.spec_bits.state = queue_op_s.op.specq.state;
            end
            e_spec_cmd_clear: begin
              // set all fields to 0
              decoded_inst_o.spec_bits.spec = 1'b0;
              decoded_inst_o.spec_bits.squash = 1'b0;
              decoded_inst_o.spec_bits.fwd_mod = 1'b0;
              decoded_inst_o.spec_bits.state = '0;
            end
            default: begin
              // shouldn't reach here unless bad instruction
              // be safe and make it a nop
              decoded_inst_o.spec_w_v = 1'b0;
            end
          endcase
        end
        if (minor_op_u.queue_minor_op == e_pushq_op) begin
          // lce cmd
          decoded_inst_o.lce_cmd = queue_op_s.op.pushq.cmd.lce_cmd;
          // cce_lce_cmd_queue inputs
          decoded_inst_o.lce_cmd_lce_sel = queue_op_s.op.pushq.lce_cmd_lce_sel;
          decoded_inst_o.lce_cmd_addr_sel = queue_op_s.op.pushq.lce_cmd_addr_sel;
          decoded_inst_o.lce_cmd_way_sel = queue_op_s.op.pushq.lce_cmd_way_sel;
          // mem cmd
          decoded_inst_o.mem_cmd = queue_op_s.op.pushq.cmd.mem_cmd;
          // mem_cmd_queue inputs
          decoded_inst_o.mem_cmd_addr_sel = queue_op_s.op.pushq.mem_cmd_addr_sel;
          // mem_resp
          decoded_inst_o.mem_resp = queue_op_s.op.pushq.cmd.mem_resp;

          // Output queue data valid signals
          // Output to LCE (ready&valid)
          decoded_inst_o.lce_cmd_v = (pushq_qsel == e_dst_q_lce_cmd);
          // Output to Mem (ready&valid), connects to FIFO buffer
          decoded_inst_o.mem_cmd_v = (pushq_qsel == e_dst_q_mem_cmd);

          if ((pushq_qsel == e_dst_q_mem_cmd) & queue_op_s.op.pushq.speculative) begin
            decoded_inst_o.spec_w_v = 1'b1;
            decoded_inst_o.spec_cmd = e_spec_cmd_clear;
            decoded_inst_o.spec_bits.spec = 1'b1;
            // rest of spec_bits = 0
            // write speculative flag
            decoded_inst_o.flag_mask_w_v = e_flag_sf;
            decoded_inst_o.sf_sel = e_sf_logic;
          end

        end
        if (minor_op_u.queue_minor_op == e_inv_op) begin
          decoded_inst_o.inv_cmd_v = 1'b1;

          // invalidation op performs a write directory state operation
          // WG = request address
          // LCE = from INV unit
          // WAY = from INV unit
          // Coherence State = Invalid (immediate)

          decoded_inst_o.dir_op = e_wds_op;

          // Directory input mux selects
          decoded_inst_o.dir_way_group_sel = e_dir_wg_sel_req_addr;
          decoded_inst_o.dir_lce_sel = e_dir_lce_sel_inv;
          decoded_inst_o.dir_way_sel = e_dir_way_sel_inv;
          decoded_inst_o.dir_coh_state_sel = e_dir_coh_sel_inst_imm;
          decoded_inst_o.imm[0+:`bp_coh_bits] = e_COH_I;

          decoded_inst_o.lce_cmd = e_lce_cmd_invalidate_tag;
          decoded_inst_o.lce_cmd_addr_sel = e_lce_cmd_addr_req_addr;

        end
        if ((minor_op_u.queue_minor_op == e_popq_op)
            | (minor_op_u.queue_minor_op == e_poph_op)) begin

          // only dequeue if actually a POPQ op, not a POPH op
          if (minor_op_u.queue_minor_op == e_popq_op) begin
            // Input queue yumi signals (to FIFOs)
            // Input messages are buffered by FIFOs, and dequeueing uses valid->yumi protocol
            // Input from LCE (valid->yumi)
            decoded_inst_o.lce_req_yumi = lce_req_v_i & (popq_qsel == e_src_q_sel_lce_req);
            decoded_inst_o.lce_resp_yumi = lce_resp_v_i & (popq_qsel == e_src_q_sel_lce_resp);
            // Input from Mem (valid->yumi)
            decoded_inst_o.mem_resp_yumi = mem_resp_v_i & (popq_qsel == e_src_q_sel_mem_resp);
          end

          if (queue_op_s.op.popq.src_q == e_src_q_sel_lce_resp) begin
            decoded_inst_o.nwbf_sel = e_nwbf_lce_resp;
            decoded_inst_o.flag_mask_w_v = e_flag_nwbf;
            decoded_inst_o.dst.gpr = queue_op_s.op.popq.dst;
            decoded_inst_o.dst_sel = e_dst_sel_gpr;
            decoded_inst_o.resp_type_w_v = 1'b1;

          end else if (queue_op_s.op.popq.src_q == e_src_q_sel_mem_resp) begin
            // pop the response type into a GPR
            decoded_inst_o.dst.gpr = queue_op_s.op.popq.dst;
            decoded_inst_o.dst_sel = e_dst_sel_gpr;
            decoded_inst_o.mem_resp_type_w_v = 1'b1;

          end else if (queue_op_s.op.popq.src_q == e_src_q_sel_lce_req) begin
            decoded_inst_o.req_sel = e_req_sel_lce_req;
            decoded_inst_o.lru_way_sel = e_lru_way_sel_lce_req;
            decoded_inst_o.req_w_v = 1'b1; // req_lce, req_addr
            decoded_inst_o.lru_way_w_v = 1'b1;
            decoded_inst_o.nerf_sel = e_nerf_lce_req;
            decoded_inst_o.ldf_sel = e_ldf_lce_req;
            decoded_inst_o.rqf_sel = e_rqf_lce_req;
            decoded_inst_o.ucf_sel = e_ucf_lce_req;
            decoded_inst_o.uc_req_size_w_v = 1'b1;
            decoded_inst_o.flag_mask_w_v = (e_flag_rqf | e_flag_nerf | e_flag_ldf | e_flag_ucf);
          end
        end
      end
      default: begin
      end
    endcase

    // Write enables

    // GPR writes occur for mov op, alu op, and LCE Response type pop - only if dst_sel is GPR
    gpr_w_v = (decoded_inst_o.mov_dst_w_v | decoded_inst_o.alu_dst_w_v
               | decoded_inst_o.resp_type_w_v | decoded_inst_o.rde_w_v
               | decoded_inst_o.mem_resp_type_w_v)
              & (decoded_inst_o.dst_sel == e_dst_sel_gpr);
    decoded_inst_o.gpr_w_mask =
      {
      (decoded_inst_o.dst.gpr == e_dst_r7) & (gpr_w_v)
      ,(decoded_inst_o.dst.gpr == e_dst_r6) & (gpr_w_v)
      ,(decoded_inst_o.dst.gpr == e_dst_r5) & (gpr_w_v)
      ,(decoded_inst_o.dst.gpr == e_dst_r4) & (gpr_w_v)
      ,(decoded_inst_o.dst.gpr == e_dst_r3) & (gpr_w_v)
      ,(decoded_inst_o.dst.gpr == e_dst_r2) & (gpr_w_v)
      ,(decoded_inst_o.dst.gpr == e_dst_r1) & (gpr_w_v)
      ,(decoded_inst_o.dst.gpr == e_dst_r0) & (gpr_w_v)
      };

  end

endmodule
