/**
 *
 * Name:
 *   bp_cce_dir_nonsynth_tracer.v
 *
 * Description:
 *
 */

module bp_cce_dir_nonsynth_tracer
  import bp_common_pkg::*;
  import bp_cce_pkg::*;
  #(parameter sets_p                      = "inv" // number of LCE sets tracked by this directory
    , parameter lce_assoc_p               = "inv" // associativity of each set
    , parameter num_lce_p                 = "inv" // number of LCEs
    , parameter tag_width_p               = "inv" // address tag width

    // Default parameters

    // For even numbers of LCEs, all rows are fully utilized
    // For odd numbers of LCEs, last row for a way group will only have 1 tag set in use
    // TODO: this is set as a constant based on prior physical design work showing 2 tag sets
    // per row gives good PPA, assuming 64-set, 8-way associative LCEs
    , parameter tag_sets_per_row_lp       = 2

    // Derived parameters

    // Directory information widths
    , localparam entry_width_lp           = (tag_width_p+$bits(bp_coh_states_e))
    , localparam tag_set_width_lp         = (entry_width_lp*lce_assoc_p)
    , localparam row_width_lp             = (tag_set_width_lp*tag_sets_per_row_lp)

    // Number of rows to hold one set from all LCEs
    // TODO: this wastes space if there is an odd number of LCEs in the system
    , localparam rows_per_set_lp          = (num_lce_p == 1) ? 1
                                            :((num_lce_p % tag_sets_per_row_lp) == 0)
                                              ? (num_lce_p / tag_sets_per_row_lp)
                                              : ((num_lce_p / tag_sets_per_row_lp) + 1)

    // Total number of rows in the directory RAM
    , localparam rows_lp                  = (rows_per_set_lp*sets_p)

    // Is the last directory row for each set fully utilized?
    // yes (1) for even number of LCEs, no (0) for odd number of LCEs
    , localparam last_row_full_lp         = ((num_lce_p % 2) == 0)

    , localparam counter_max_lp           = (rows_lp+1)

    // Derived parameters - widths
    , localparam lg_sets_lp               = `BSG_SAFE_CLOG2(sets_p)
    , localparam lg_num_lce_lp            = `BSG_SAFE_CLOG2(num_lce_p)
    , localparam lg_lce_assoc_lp          = `BSG_SAFE_CLOG2(lce_assoc_p)
    , localparam lg_tag_sets_per_row_lp   = `BSG_SAFE_CLOG2(tag_sets_per_row_lp)
    , localparam lg_rows_per_set_lp       = `BSG_SAFE_CLOG2(rows_per_set_lp)
    , localparam lg_rows_lp               = `BSG_SAFE_CLOG2(rows_lp)

    , localparam addr_offset_shift_lp     = 1

    // TODO: DEBUG only
    , parameter cce_id_width_p            = "inv"

    , localparam cce_dir_trace_file_p = "cce_dir"
  )
  (input                                                          clk_i
   , input                                                        reset_i
   , input                                                        freeze_i

   , input [lg_sets_lp-1:0]                                       set_i
   , input [lg_num_lce_lp-1:0]                                    lce_i
   , input [lg_lce_assoc_lp-1:0]                                  way_i
   , input [lg_lce_assoc_lp-1:0]                                  lru_way_i
   , input [`bp_cce_inst_minor_op_width-1:0]                      r_cmd_i
   , input                                                        r_v_i

   , input [tag_width_p-1:0]                                      tag_i
   , input [$bits(bp_coh_states_e)-1:0]                           coh_state_i
   , input [`bp_cce_inst_minor_op_width-1:0]                      w_cmd_i
   , input                                                        w_v_i
   // TODO: fix this input functionality?
   , input                                                        w_clr_row_i

   , input                                                        busy_i

   , input                                                        sharers_v_i
   , input [num_lce_p-1:0]                                        sharers_hits_i
   , input [num_lce_p-1:0][lg_lce_assoc_lp-1:0]                   sharers_ways_i
   , input [num_lce_p-1:0][$bits(bp_coh_states_e)-1:0]            sharers_coh_states_i

   , input                                                        lru_v_i
   , input                                                        lru_cached_excl_i
   , input [tag_width_p-1:0]                                      lru_tag_i

   , input [tag_width_p-1:0]                                      tag_o_i

   // TODO: debug only, remove
   , input [cce_id_width_p-1:0]                                   cce_id_i
  );

  integer file;
  string file_name;

  logic sh_v_r, lru_v_r;
  logic freeze_r;
  always_ff @(posedge clk_i) begin
    freeze_r <= freeze_i;
    sh_v_r <= sharers_v_i;
    lru_v_r <= lru_v_i;
  end


  always_ff @(negedge clk_i)
    if (freeze_r & ~freeze_i)
      begin
        file_name = $sformatf("%s_%x.trace", cce_dir_trace_file_p, cce_id_i);
        file      = $fopen(file_name, "w");
      end

  // Tracer
  always_ff @(negedge clk_i) begin
    if (~reset_i) begin
      if (r_v_i & w_v_i) begin
        $fdisplay(file, "[%t]: CCE[%0d] write set[%0d] lce[%0d] way[%0d] tag[%0d] state[%0d]"
                 , $time, cce_id_i, set_i, lce_i, way_i, tag_i, coh_state_i
                 );
      end
      else if (r_v_i) begin
        $fdisplay(file, "[%t]: CCE[%0d] read set[%0d] lce[%0d] lru_way[%0d]"
                 , $time, cce_id_i, set_i, lce_i, lru_way_i
                 );
      end

      if (~sh_v_r & sharers_v_i) begin
        $fwrite(file, "[%t]: CCE[%0d] sharers hits[%b]"
                 , $time, cce_id_i, sharers_hits_i
                 );
        for (int i = 0; i < num_lce_p; i++) begin
          $fwrite(file, " [%0d]: way[%0d] state[%0d]"
                  , i, sharers_ways_i[i], sharers_coh_states_i[i]
                 );
        end
        $fwrite(file, "\n");
      end
      if (~lru_v_r & lru_v_i) begin
        $fdisplay(file, "[%t]: CCE[%0d] lru_excl[%b] lru_tag[%H]"
                 , $time, cce_id_i, lru_cached_excl_i, lru_tag_i
                 );
      end
    end // reset
  end // always_ff

endmodule
