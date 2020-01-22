/**
 *
 * Name:
 *   bp_cce_spec_bits.v
 *
 * Description:
 *   This module contains the metadata required for speculative memory accesses.
 *
 *   These bits are stored in flops and may be read asynchronously.
 *
 *   Write to Read forwarding is supported.
 *
 */

module bp_cce_spec_bits
  import bp_common_pkg::*;
  import bp_cce_pkg::*;
  #(parameter num_way_groups_p            = "inv"
    , parameter num_cce_p                 = "inv"
    , parameter addr_width_p              = "inv"

    // Derived parameters
    , localparam lg_num_way_groups_lp     = `BSG_SAFE_CLOG2(num_way_groups_p)

  )
  (input                                                          clk_i
   , input                                                        reset_i

   // Write port
   , input                                                        w_v_i
   , input [addr_width_p-1:0]                                     w_addr_i
   , input                                                        w_addr_bypass_hash_i

   , input                                                        spec_v_i
   , input                                                        squash_v_i
   , input                                                        fwd_mod_v_i
   , input                                                        state_v_i
   , input bp_cce_spec_s                                          spec_i

   // Read port
   , input                                                        r_v_i
   , input [addr_width_p-1:0]                                     r_addr_i
   , input                                                        r_addr_bypass_hash_i

   , output bp_cce_spec_s                                         spec_o
   , output logic                                                 v_o

  );

  // Address to way group hashing
  logic [lg_num_way_groups_lp-1:0] r_wg_lo, w_wg_lo;
  wire [addr_width_p-1:0] r_addr_rev = {<< {r_addr_i}};
  wire [addr_width_p-1:0] w_addr_rev = {<< {w_addr_i}};
  logic [lg_num_way_groups_lp-1:0] r_wg, w_wg;

  bsg_hash_bank
    #(.banks_p(num_cce_p) // number of CCE's to spread way groups over
      ,.width_p(addr_width_p) // width of address input
      )
    r_addr_hash
     (.i(r_addr_rev)
      ,.bank_o()
      ,.index_o(r_wg_lo)
      );

  bsg_hash_bank
    #(.banks_p(num_cce_p) // number of CCE's to spread way groups over
      ,.width_p(addr_width_p) // width of address input
      )
    w_addr_hash
     (.i(w_addr_rev)
      ,.bank_o()
      ,.index_o(w_wg_lo)
      );

  assign r_wg = (r_addr_bypass_hash_i) ? r_addr_i[0+:lg_num_way_groups_lp] : r_wg_lo;
  assign w_wg = (w_addr_bypass_hash_i) ? w_addr_i[0+:lg_num_way_groups_lp] : w_wg_lo;

  // speculation metadata bits
  bp_cce_spec_s [num_way_groups_p-1:0] spec_bits_r, spec_bits_n;

  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      spec_bits_r <= '0;
    end else begin
      spec_bits_r <= spec_bits_n;
    end
  end

  always_comb begin
    if (reset_i) begin
      spec_bits_n = '0;
    end else begin
      spec_bits_n = spec_bits_r;
      if (w_v_i) begin
        if (spec_v_i) begin
          spec_bits_n[w_wg].spec = spec_i.spec;
        end
        if (squash_v_i) begin
          spec_bits_n[w_wg].squash = spec_i.squash;
        end
        if (fwd_mod_v_i) begin
          spec_bits_n[w_wg].fwd_mod = spec_i.fwd_mod;
        end
        if (state_v_i) begin
          spec_bits_n[w_wg].state = spec_i.state;
        end
      end
    end
  end

  // Output
  // Normally, the output is determined by the read way group and comes from the flopped values
  // If reading from the same way group that is being written, output the next value
  assign spec_o = (r_v_i & w_v_i & (w_wg == r_wg))
    ? spec_bits_n[r_wg]
    : spec_bits_r[r_wg];

  // Output is valid if read signal is asserted
  assign v_o = r_v_i;

endmodule
