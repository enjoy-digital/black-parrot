
package bp_be_hardfloat_pkg;

localparam dword_width_gp  = 64;
localparam word_width_gp   = 32;

localparam float_width_gp  = 32;
localparam double_width_gp = 64;

typedef enum logic [4:0]
{
  e_op_fadd    = 5'b00000
  ,e_op_fsub   = 5'b00001
  ,e_op_fmul   = 5'b00010
  ,e_op_fmin   = 5'b00011
  ,e_op_fmax   = 5'b00100
  ,e_op_fmadd  = 5'b00101
  ,e_op_fmsub  = 5'b00110
  ,e_op_fnmsub = 5'b00111
  ,e_op_fnmadd = 5'b01000
  ,e_op_i2f    = 5'b01001
  ,e_op_iu2f   = 5'b01010
  ,e_op_fsgnj  = 5'b01011
  ,e_op_fsgnjn = 5'b01100
  ,e_op_fsgnjx = 5'b01101
  ,e_op_feq    = 5'b01110
  ,e_op_flt    = 5'b01111
  ,e_op_fle    = 5'b10000
  ,e_op_fclass = 5'b10001
  ,e_op_f2i    = 5'b10010
  ,e_op_f2iu   = 5'b10011
  ,e_op_pass   = 5'b11111
} bsg_fp_op_e;

typedef enum logic
{
  e_pr_single  = 1'b0
  ,e_pr_double = 1'b1
} bsg_fp_pr_e;

typedef enum logic [2:0]
{
  e_rne   = 3'b000
  ,e_rtz  = 3'b001
  ,e_rdn  = 3'b010
  ,e_rup  = 3'b011
  ,e_rmm  = 3'b100
  // 3'b101, 3'b110 reserved
  ,e_dyn  = 3'b111
} bsg_fp_rm_e;

typedef struct packed
{
  // Invalid operation
  logic nv;
  // Divide by zero
  logic dz;
  // Overflow
  logic of;
  // Underflow
  logic uf;
  // Inexact
  logic nx;
}  bsg_fp_eflags_s;

typedef struct packed
{
  // Invalid operation
  logic nv;
  // Overflow
  logic of;
  // Inexact
  logic nx;
}  bsg_int_eflags_s;

typedef struct packed
{
  logic [53:0] padding;
  logic        q_nan;
  logic        sig_nan;
  logic        p_inf;
  logic        p_norm;
  logic        p_sub;
  logic        p_zero;
  logic        n_zero;
  logic        n_sub;
  logic        n_norm;
  logic        n_inf;
}  rv64_fclass_s;

endpackage

