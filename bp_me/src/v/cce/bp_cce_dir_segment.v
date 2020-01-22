/**
 *
 * Name:
 *   bp_cce_dir_segment.v
 *
 * Description:
 *   A directory segment stores the coherence state and tags for a set of cache blocks tracked
 *   by the CCE. Each segment tracks coherence info for a single type of LCE in the system.
 *   Different LCE types include data caches, instruction caches, and accelerator caches.
 *
 *   The physical directory SRAM is a synchronous read 1RW memory.
 *
 *   All write operations take 1 cycle, but read operations take multiple cycles. The busy_o
 *   signal will be asserted until the read operation is completed.
 *
 */

module bp_cce_dir_segment
  import bp_common_pkg::*;
  import bp_cce_pkg::*;
  #(parameter sets_p                      = "inv" // number of sets tracked by this directory
    , parameter assoc_p                   = "inv" // associativity of each set
    , parameter paddr_width_p             = "inv" // physical address width
    , parameter num_cce_p                 = "inv" // number of CCEs that blocks are banked across
    , parameter num_lce_p                 = "inv" // number of LCEs tracked in this directory
    , parameter block_size_in_bytes_p     = "inv" // size of cache blocks in bytes
    , parameter block_addr_width_p        = "inv" // width of address to use for hashing

    // Default parameters

    // This is set as a constant based on prior physical design work showing 2 tag sets
    // per row gives good PPA, assuming 64-set, 8-way associative LCEs.
    // For even numbers of LCEs, all rows are fully utilized
    // For odd numbers of LCEs, last row for a way group will only have 1 tag set in use
    , parameter tag_sets_per_row_lp       = 2

    // Derived parameters
    , localparam lg_tag_sets_per_row_lp   = `BSG_SAFE_CLOG2(tag_sets_per_row_lp)

    , localparam lg_sets_lp                = `BSG_SAFE_CLOG2(sets_p)
    , localparam lg_num_cce_lp             = `BSG_SAFE_CLOG2(num_cce_p)
    , localparam lg_num_lce_lp             = `BSG_SAFE_CLOG2(num_lce_p)
    , localparam lg_assoc_lp               = `BSG_SAFE_CLOG2(assoc_p)
    , localparam lg_block_size_in_bytes_lp = `BSG_SAFE_CLOG2(block_size_in_bytes_p)

    , localparam tag_offset_lp             = (block_addr_width_p+lg_block_size_in_bytes_lp)
    , localparam tag_width_lp              = (paddr_width_p-tag_offset_width_lp)

    , localparam hash_index_width_lp      = $clog2((2**block_addr_width_p+num_cce_p-1)/num_cce_p)

    // Directory information widths
    , localparam entry_width_lp           = (tag_width_lp+$bits(bp_coh_states_e))
    , localparam tag_set_width_lp         = (entry_width_lp*assoc_p)
    , localparam row_width_lp             = (tag_set_width_lp*tag_sets_per_row_lp)

    // Number of rows to hold one set from all LCEs
    // Note: If there is an odd number of LCEs managed by this directory segment, the
    // last row per set will only utilize one of the 2 entries.
    , localparam rows_per_set_lp          = (num_lce_p == 1) ? 1
                                            :((num_lce_p % tag_sets_per_row_lp) == 0)
                                              ? (num_lce_p / tag_sets_per_row_lp)
                                              : ((num_lce_p / tag_sets_per_row_lp) + 1)
    , localparam lg_rows_per_set_lp       = `BSG_SAFE_CLOG2(rows_per_set_lp)

    // Total number of rows in the directory RAM
    , localparam rows_lp                  = (rows_per_set_lp*sets_p)
    , localparam lg_rows_lp               = `BSG_SAFE_CLOG2(rows_lp)

    // Is the last directory row for each set fully utilized?
    , localparam last_row_full_lp         = ((num_lce_p % tag_sets_per_row_lp) == 0)

    , localparam counter_max_lp           = (rows_lp+1)
    , localparam counter_width_lp         = `BSG_SAFE_CLOG2(counter_max_lp)

  )
  (input                                                          clk_i
   , input                                                        reset_i

   // input address, fed to bsg_hash_bank
   , input [paddr_width_p-1:0]                                    addr_i
   // bypass signal to use low bits of raw address instead of hashed address
   , input                                                        addr_bypass_i

   , input [lg_num_lce_lp-1:0]                                    lce_i
   , input [lg_assoc_lp-1:0]                                      way_i
   , input [lg_assoc_lp-1:0]                                      lru_way_i
   , input bp_coh_states_e                                        coh_state_i

   , input bp_cce_inst_minor_dir_op_e                             cmd_i
   , input                                                        r_v_i
   , input                                                        w_v_i

   , output logic                                                 busy_o

   , output logic                                                 sharers_v_o
   , output logic [num_lce_p-1:0]                                 sharers_hits_o
   , output logic [num_lce_p-1:0][lg_assoc_lp-1:0]                sharers_ways_o
   , output bp_coh_states_e [num_lce_p-1:0]                       sharers_coh_states_o

   , output logic                                                 lru_v_o
   , output logic                                                 lru_cached_excl_o
   , output logic [paddr_width_p-1:0]                             lru_addr_o

   , output logic                                                 addr_v_o
   , output logic [paddr_width_p-1:0]                             addr_o
  );

  // If value of tag_sets_per_row_lp changes (is no longer 2) the directory logic
  // needs to be re-written.
  initial begin
    assert(tag_sets_per_row_lp == 2) else
      $error("Unsupported configuration: number of sets per row must equal 2");
    assert(sets_p > 1) else
      $error("Number of cache sets must be greater than 1; direct-mapped caches not supported");
  end

  // input address hashing
  logic [lg_num_cce_lp-1:0] cce_id_lo;
  logic [hash_index_width_lp-1:0] set_id_lo;
  // NOTE: reverse the address to use the low order bits for striping cache blocks across CCEs
  wire [block_addr_width_p-1:0] hash_addr_rev = { <<{addr_i[lg_block_size_in_bytes_lp+:block_addr_width_p]}};

  bsg_hash_bank
    #(.banks_p(num_cce_p) // number of CCE's to spread way groups over
      ,.width_p(block_addr_width_p) // width of address input
      )
    addr_to_cce_id
     (.i(hash_addr_rev)
      ,.bank_o(cce_id_lo)
      ,.index_o(set_id_lo)
      );

  // Bypass hashing if input wants to use raw address input
  logic [lg_sets_lp-1:0] set_id;
  assign set_id = addr_bypass_i ? addr_i[0+:lg_sets_lp] : set_id_lo[0+:lg_sets_lp];

  // address offset table
  // lookup: lce_i[1+:], assuming tag_sets_per_row_lp == 2
  logic [rows_per_set_lp-1:0][lg_rows_lp-1:0] addr_offset_table;
  genvar i;
  generate
    for (i = 0; i < rows_per_set_lp; i++) begin
      assign addr_offset_table[i] = (i * sets_p);
    end
  endgenerate
  wire [lg_rows_per_set_lp-1:0] addr_lce = lce_i[1+:lg_rows_per_set_lp];
  wire [lg_rows_lp-1:0] addr_offset = addr_offset_table[addr_lce];

  // directory address for single entry operations
  wire [lg_rows_lp-1:0] entry_row_addr = addr_offset + set_id;

  // Struct for directory entries
  `declare_bp_cce_dir_entry_s(tag_width_lp);

  // Directory signals
  // read / write valid signals
  logic dir_ram_w_v;
  logic dir_ram_v;
  // address input and address register
  logic [lg_rows_lp-1:0] dir_ram_addr;
  logic [lg_rows_lp-1:0] dir_ram_addr_r, dir_ram_addr_n;
  // write mask and data in
  dir_entry_s [tag_sets_per_row_lp-1:0][assoc_p-1:0] dir_ram_w_mask, dir_ram_w_data;
  // data out
  dir_entry_s [tag_sets_per_row_lp-1:0][assoc_p-1:0] dir_row_entries;

  // Counter
  logic cnt_clr, cnt_inc;
  logic [counter_width_lp-1:0] cnt;

  // State machine
  typedef enum logic [2:0] {
    RESET
    ,INIT
    ,READY
    ,READ_FULL
    ,READ_ENTRY
  } dir_state_e;

  dir_state_e state_r, state_n;

  // Registers
  // TODO: set_r and set_n are unused. Remove?
  logic [lg_sets_lp-1:0]          set_r, set_n;
  logic [lg_num_lce_lp-1:0]       lce_r, lce_n;
  logic [lg_assoc_lp-1:0]         way_r, way_n;
  logic [lg_assoc_lp-1:0]         lru_way_r, lru_way_n;
  logic [tag_width_lp-1:0]        tag_r, tag_n;
  logic [tag_sets_per_row_lp-1:0] dir_data_o_v_r, dir_data_o_v_n;


  // Sharers registers
  logic                                             sharers_v_r, sharers_v_n;
  logic [num_lce_p-1:0]                             sharers_hits_r, sharers_hits_n;
  logic [num_lce_p-1:0][lg_assoc_lp-1:0]            sharers_ways_r, sharers_ways_n;
  bp_coh_states_e [num_lce_p-1:0]                   sharers_coh_states_r, sharers_coh_states_n;

  assign sharers_v_o = sharers_v_r;
  assign sharers_hits_o = sharers_hits_r;
  assign sharers_ways_o = sharers_ways_r;
  assign sharers_coh_states_o = sharers_coh_states_r;

  logic [tag_sets_per_row_lp-1:0]                                 sharers_hits;
  logic [tag_sets_per_row_lp-1:0][lg_assoc_lp-1:0]                sharers_ways;
  bp_coh_states_e [tag_sets_per_row_lp-1:0]                       sharers_coh_states;

  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      state_r <= RESET;
      set_r <= '0;
      lce_r <= '0;
      way_r <= '0;
      lru_way_r <= '0;
      tag_r <= '0;
      dir_data_o_v_r <= '0;
      dir_ram_addr_r <= '0;

      sharers_v_r <= '0;
      sharers_hits_r <= '0;
      sharers_ways_r <= '0;
      sharers_coh_states_r <= '0;

    end else begin
      state_r <= state_n;
      set_r <= set_n;
      lce_r <= lce_n;
      way_r <= way_n;
      lru_way_r <= lru_way_n;
      tag_r <= tag_n;
      dir_data_o_v_r <= dir_data_o_v_n;
      dir_ram_addr_r <= dir_ram_addr_n;

      sharers_v_r <= sharers_v_n;
      sharers_hits_r <= sharers_hits_n;
      sharers_ways_r <= sharers_ways_n;
      sharers_coh_states_r <= sharers_coh_states_n;

    end
  end

  // Directory State Machine logic
  always_comb begin
    // state - hold by default
    state_n = state_r;

    // counter inputs
    cnt_clr = 1'b0;
    cnt_inc = 1'b0;

    // directory inputs
    dir_ram_w_mask = '0;
    dir_ram_w_data = '0;
    dir_ram_v = '0;
    dir_ram_w_v = '0;
    dir_ram_addr = '0;

    // registers
    set_n = set_r;
    lce_n = lce_r;
    way_n = way_r;
    lru_way_n = lru_way_r;
    tag_n = tag_r;
    dir_data_o_v_n = '0;
    dir_ram_addr_n = dir_ram_addr_r;

    sharers_v_n = sharers_v_r;
    sharers_hits_n = sharers_hits_r;
    sharers_ways_n = sharers_ways_r;
    sharers_coh_states_n = sharers_coh_states_r;

    // outputs
    busy_o = '0;
    addr_v_o = '0;
    addr_o = '0;

    case (state_r)
      RESET: begin
        state_n = INIT;
        cnt_clr = 1'b1;
      end
      INIT: begin
        // clear every row in directory after reset
        dir_ram_v = 1'b1;
        dir_ram_w_v = 1'b1;
        dir_ram_addr = cnt[0+:lg_rows_lp];
        dir_ram_w_mask = '1;
        dir_ram_w_data = '0;
        state_n = (cnt == (rows_lp-1)) ? READY : INIT;
        cnt_clr = (state_n == READY);
        cnt_inc = ~cnt_clr;
        // directory is busy and cannot accept commands
        busy_o = 1'b1;
      end
      READY: begin

        if (r_v_i) begin

          // reset the sharers vectors for the new read; new values will be prepared for writing
          // starting in the next cycle, when the first read data is valid
          sharers_v_n = '0;
          sharers_hits_n = '0;
          sharers_ways_n = '0;
          sharers_coh_states_n = '0;

          // capture inputs into registers
          set_n     = set_id;
          lce_n     = lce_i;
          way_n     = way_i;
          lru_way_n = lru_way_i;
          tag_n     = addr_i[tag_offset_lp+:tag_width_lp];

          // ensure counter is reset to 0
          cnt_clr = 1'b1;

          // setup the read
          dir_ram_v = 1'b1;
          dir_ram_addr_n = '0;

          // initiate directory read of first row of way group
          // first row will be valid on output of directory next cycle (in READ)
          if (cmd_i == e_rdw_op) begin
            state_n = READ_FULL;

            // setup directory ram inputs
            dir_ram_addr[0+:lg_sets_lp] = set_id;

            // next address to read from directory
            dir_ram_addr_n = dir_ram_addr + sets_p;

            // next cycle, the data coming out of the RAM will be valid
            dir_data_o_v_n =
              (num_lce_p < tag_sets_per_row_lp)
              ? {'0, num_lce_p('1)}
              : '1;

          // read entry
          end else if (cmd_i == e_rde_op) begin
            state_n = READ_ENTRY;

            // entry read does not use LRU way, override register next value to 0
            lru_way_n = '0;

            // The address to read depends on how many rows per way group there are.
            // If there is only one row per wg, then the input way group is the address.
            // If there is more than one row per wg, then the input way group is the high bits
            // and the rd_wg_row_select is the low bits since RDE op takes only one read (read a
            // single entry from a single tag set)
            dir_ram_addr = entry_row_addr;

          end

        // directory write
        end else if (w_v_i) begin
          // mark sharers info as invalid after a write, since it is possible the write
          // changes data in the way-group that generated the sharers vectors
          sharers_v_n = '0;

          tag_n = '0;

          state_n = READY;
          dir_ram_v = 1'b1;
          dir_ram_w_v = 1'b1;

          dir_ram_addr = entry_row_addr;

          if (cmd_i == e_clr_op) begin
            dir_ram_w_data = '0;
            dir_ram_w_mask = '1;
          end else if (cmd_i == e_wde_op) begin
            dir_ram_w_mask[lce_i[0]][way_i] = '1;
            dir_ram_w_data[lce_i[0]][way_i].tag = addr_i[tag_offset_lp+:tag_width_lp];
            dir_ram_w_data[lce_i[0]][way_i].state = coh_state_i;
          end else if (cmd_i == e_wds_op) begin
            dir_ram_w_mask[lce_i[0]][way_i].state = bp_coh_states_e'('1);
            dir_ram_w_data[lce_i[0]][way_i].state = coh_state_i;
          end
        end

      end
      READ_FULL: begin
        // directory is busy
        busy_o = 1'b1;

        // cnt should be shifted based on LOG2(tag_sets_per_row_lp)
        // would require tag_sets_per_row_lp to be a power of two
        for(int i = 0; i < tag_sets_per_row_lp; i++) begin
          sharers_hits_n[(cnt << 1) + i] = sharers_hits[i];
          sharers_ways_n[(cnt << 1) + i] = sharers_ways[i];
          sharers_coh_states_n[(cnt << 1) + i] = sharers_coh_states[i];
        end

        // do another read if required (num_lce_p > 2 and rows_per_set_lp >= 2)
        if (cnt < (rows_per_set_lp-1)) begin
          dir_ram_v = 1'b1;
          dir_ram_addr = dir_ram_addr_r;
          dir_ram_addr_n = dir_ram_addr_r + sets_p;
          dir_data_o_v_n = (cnt == (rows_per_set_lp-2))
                           ? (last_row_full_lp)
                             ? '1
                             : 2'b01
                           : '1;
        end else begin
          state_n = READY;
          cnt_clr = 1'b1;
          // sharers will be valid next cycle
          sharers_v_n = 1'b1;
        end
        cnt_inc = ~cnt_clr;
      end
      READ_ENTRY: begin
        busy_o = 1'b1;
        sharers_hits_n[0] = (tag_r == dir_row_entries[lce_r[0]][way_r].tag)
                            & |dir_row_entries[lce_r[0]][way_r].state;
        sharers_ways_n[0] = way_r;
        sharers_coh_states_n[0] = dir_row_entries[lce_r[0]][way_r].state;
        sharers_v_n = 1'b1;
        // output the tag in the entry, and store it to the register
        // override output of addr_o with the tag from the directory read
        addr_v_o = 1'b1;
        addr_o = {dir_row_entries[lce_r[0]][way_r].tag, tag_offset_lp'(0)};
        tag_n = dir_row_entries[lce_r[0]][way_r].tag;
        state_n = READY;
      end
      default: begin
        state_n = RESET;
      end
    endcase
  end

  // Instantiated modules

  // Reads are synchronous, with the address latched in the current cycle, and data available next
  // Writes take 1 cycle
  bsg_mem_1rw_sync_mask_write_bit
    #(.width_p(row_width_lp)
      ,.els_p(rows_lp)
      )
    directory
     (.clk_i(clk_i)
      ,.reset_i(reset_i)
      ,.w_i(dir_ram_w_v)
      ,.w_mask_i(dir_ram_w_mask)
      ,.addr_i(dir_ram_addr)
      ,.data_i(dir_ram_w_data)
      ,.v_i(dir_ram_v)
      ,.data_o(dir_row_entries)
      );

  // counter
  bsg_counter_clear_up
    #(.max_val_p(counter_max_lp)
      ,.init_val_p(0)
     )
    counter
     (.clk_i(clk_i)
      ,.reset_i(reset_i)
      ,.clear_i(cnt_clr)
      ,.up_i(cnt_inc)
      ,.count_o(cnt)
      );

  // combinational logic to determine hit, way, and state for current directory row output
  bp_cce_dir_tag_checker
    #(.tag_sets_per_row_p(tag_sets_per_row_lp)
      ,.row_width_p(row_width_lp)
      ,.assoc_p(assoc_p)
      ,.tag_width_p(tag_width_lp)
     )
    tag_checker
     (.row_i(dir_row_entries)
      ,.row_v_i(dir_data_o_v_r)
      ,.tag_i(tag_r)
      ,.sharers_hits_o(sharers_hits)
      ,.sharers_ways_o(sharers_ways)
      ,.sharers_coh_states_o(sharers_coh_states)
     );

  logic [tag_width_lp-1:0] lru_tag_lo;
  bp_cce_dir_lru_extract
    #(.tag_sets_per_row_p(tag_sets_per_row_lp)
      ,.rows_per_set_p(rows_per_set_lp)
      ,.row_width_p(row_width_lp)
      ,.assoc_p(assoc_p)
      ,.num_lce_p(num_lce_p)
      ,.tag_width_p(tag_width_lp)
     )
    lru_extract
     (.row_i(dir_row_entries)
      ,.row_v_i(dir_data_o_v_r)
      ,.row_num_i(cnt[0+:lg_rows_per_set_lp])
      ,.lce_i(lce_r)
      ,.lru_way_i(lru_way_r)
      ,.lru_v_o(lru_v_o)
      ,.lru_cached_excl_o(lru_cached_excl_o)
      ,.lru_tag_o(lru_tag_lo)
     );
  assign lru_addr_o = {lru_tag_lo, tag_offset_lp'(0)};

endmodule
