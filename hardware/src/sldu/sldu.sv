// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Matheus Cavalcante <matheusd@iis.ee.ethz.ch>
// Description:
// This is Ara's slide unit. It is responsible for running the vector slide (up/down)
// instructions, which need access to the whole Vector Register File.

module sldu import ara_pkg::*; import rvv_pkg::*; #(
    parameter  int  unsigned NrLanes   = 0,
    parameter  type          vaddr_t = logic, // Type used to address vector register file elements
    // Dependant parameters. DO NOT CHANGE!
    localparam int  unsigned DataWidth = $bits(elen_t), // Width of the lane datapath
    localparam int  unsigned StrbWidth = DataWidth/8,
    localparam type          strb_t    = logic [StrbWidth-1:0], // Byte-strobe type
    localparam type          cluster_reduction_rx_cnt_t = logic [idx_width($clog2(MaxNrClusters)+1):0]
  ) (
    input  logic                   clk_i,
    input  logic                   rst_ni,
    // Id
    input  id_cluster_t            cluster_id_i,
    input num_cluster_t            num_clusters_i,
    // Interface with the main sequencer
    input  pe_req_t                pe_req_i,
    input  logic                   pe_req_valid_i,
    input  logic     [NrVInsn-1:0] pe_vinsn_running_i,
    output logic                   pe_req_ready_o,
    output pe_resp_t               pe_resp_o,
    // Interface with the lanes
    input  elen_t    [NrLanes-1:0] sldu_operand_i,
    input  target_fu_e [NrLanes-1:0] sldu_operand_target_fu_i,
    input  logic     [NrLanes-1:0] sldu_operand_valid_i,
    output logic     [NrLanes-1:0] sldu_operand_ready_o,
    output logic     [NrLanes-1:0] sldu_result_req_o,
    output vid_t     [NrLanes-1:0] sldu_result_id_o,
    output vaddr_t   [NrLanes-1:0] sldu_result_addr_o,
    output elen_t    [NrLanes-1:0] sldu_result_wdata_o,
    output strb_t    [NrLanes-1:0] sldu_result_be_o,
    input  logic     [NrLanes-1:0] sldu_result_gnt_i,
    input  logic     [NrLanes-1:0] sldu_result_final_gnt_i,
    // Support for reductions
    output sldu_mux_e              sldu_mux_sel_o,
    output logic     [NrLanes-1:0] sldu_red_valid_o,
    // Interface with the Mask Unit
    input  strb_t    [NrLanes-1:0] mask_i,
    input  logic     [NrLanes-1:0] mask_valid_i,
    output logic                   mask_ready_o,
    // Interface with Ring Interconnect
    output elen_t sldu_ring_o,
    output logic  sldu_ring_valid_o, 
    input  logic  sldu_ring_ready_i, 

    input  elen_t sldu_ring_i,
    input  logic  sldu_ring_valid_i,
    output logic  sldu_ring_ready_o,

    output logic sldu_dir_o,
    output logic sldu_bypass_o,
    output logic sldu_config_valid_o

  );

  `include "common_cells/registers.svh"

  import cf_math_pkg::idx_width;

  ////////////////////////////////
  //  Vector instruction queue  //
  ////////////////////////////////

  // We store a certain number of in-flight vector instructions
  localparam VInsnQueueDepth = SlduInsnQueueDepth;

  struct packed {
    pe_req_t [VInsnQueueDepth-1:0] vinsn;

    // Each instruction can be in one of the three execution phases.
    // - Being accepted (i.e., it is being stored for future execution in this
    //   vector functional unit).
    // - Being issued (i.e., its micro-operations are currently being issued
    //   to the corresponding functional units).
    // - Waiting for data from the ring before it can be committed (e.g. for sliding or 
    //   reduction operations)
    // - Being committed (i.e., its results are being written to the vector
    //   register file).
    // We need pointers to index which instruction is at each execution phase
    // between the VInsnQueueDepth instructions in memory.
    logic [idx_width(VInsnQueueDepth)-1:0] accept_pnt;
    logic [idx_width(VInsnQueueDepth)-1:0] issue_pnt;
    logic [idx_width(VInsnQueueDepth)-1:0] ring_pnt;
    logic [idx_width(VInsnQueueDepth)-1:0] commit_pnt;

    // We also need to count how many instructions are queueing to be
    // issued/committed, to avoid accepting more instructions than
    // we can handle.
    logic [idx_width(VInsnQueueDepth):0] issue_cnt;
    logic [idx_width(VInsnQueueDepth):0] ring_cnt;
    logic [idx_width(VInsnQueueDepth):0] commit_cnt;
  } vinsn_queue_d, vinsn_queue_q;

  pe_req_t vinsn_issue_q;
  logic vinsn_issue_valid_q;
  // Is the vector instruction queue full?
  logic vinsn_queue_full;
  assign vinsn_queue_full = (vinsn_queue_q.commit_cnt == VInsnQueueDepth);

  // Do we have a vector instruction ready to be issued?
  `FF(vinsn_issue_q, vinsn_queue_d.vinsn[vinsn_queue_d.issue_pnt], '0)
  `FF(vinsn_issue_valid_q, vinsn_queue_d.issue_cnt != '0, 1'b0)

  // Do we have a vector instruction with results being committed?
  pe_req_t vinsn_commit;
  logic    vinsn_commit_valid;
  assign vinsn_commit       = vinsn_queue_q.vinsn[vinsn_queue_q.commit_pnt];
  assign vinsn_commit_valid = (vinsn_queue_q.commit_cnt != '0);

  // Do we have a vector instruction waiting for ring packets?
  pe_req_t vinsn_ring;
  logic    vinsn_ring_valid;
  assign vinsn_ring       = vinsn_queue_q.vinsn[vinsn_queue_q.ring_pnt];
  assign vinsn_ring_valid = (vinsn_queue_q.ring_cnt != '0);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      vinsn_queue_q <= '0;
    end else begin
      vinsn_queue_q <= vinsn_queue_d;
    end
  end

  /////////////////////
  //  Result queues  //
  /////////////////////

  localparam int unsigned ResultQueueDepth = 3;

  // There is a result queue per lane, holding the results that were not
  // yet accepted by the corresponding lane.
  typedef struct packed {
    vid_t id;
    vaddr_t addr;
    elen_t wdata;
    strb_t be;
  } payload_t;

  // Result queue
  payload_t [ResultQueueDepth-1:0][NrLanes-1:0] result_queue_d, result_queue_q;
  logic     [ResultQueueDepth-1:0][NrLanes-1:0] result_queue_valid_d, result_queue_valid_q;
  // We need two pointers in the result queue. One pointer to
  // indicate with `payload_t` we are currently writing into (write_pnt),
  // and one pointer to indicate which `payload_t` we are currently
  // reading from and writing into the lanes (read_pnt).
  logic     [idx_width(ResultQueueDepth)-1:0]   result_queue_write_pnt_d, result_queue_write_pnt_q, result_queue_write_pnt_q1, result_queue_write_pnt_q2;
  logic     [idx_width(ResultQueueDepth)-1:0]   result_queue_read_pnt_d, result_queue_read_pnt_q;
  // We need to count how many valid elements are there in this result queue.
  logic     [idx_width(ResultQueueDepth):0]     result_queue_cnt_d, result_queue_cnt_q;
  // Vector to register the final grants from the operand requesters, which indicate
  // that the result was actually written in the VRF (while the normal grant just says
  // that the result was accepted by the operand requester stage
  logic     [NrLanes-1:0]                       result_final_gnt_d, result_final_gnt_q;

  // Is the result queue full?
  logic result_queue_full;
  assign result_queue_full = (result_queue_cnt_q == ResultQueueDepth);
  // Is the result queue empty?
  logic result_queue_empty;
  assign result_queue_empty = (result_queue_cnt_q == '0);

  always_ff @(posedge clk_i or negedge rst_ni) begin: p_result_queue_ff
    if (!rst_ni) begin
      result_queue_q           <= '0;
      result_queue_valid_q     <= '0;
      result_queue_write_pnt_q <= '0;
      result_queue_write_pnt_q2 <= '0;
      result_queue_read_pnt_q  <= '0;
      result_queue_cnt_q       <= '0;
    end else begin
      result_queue_q           <= result_queue_d;
      result_queue_valid_q     <= result_queue_valid_d;
      result_queue_write_pnt_q <= result_queue_write_pnt_d;
      result_queue_write_pnt_q2 <= result_queue_write_pnt_q1;
      result_queue_read_pnt_q  <= result_queue_read_pnt_d;
      result_queue_cnt_q       <= result_queue_cnt_d;
    end
  end

  ////////////////////////////////
  //  Spill-reg from the lanes  //
  ////////////////////////////////

  elen_t [NrLanes-1:0] sldu_operand;
  logic  [NrLanes-1:0] sldu_operand_valid;
  logic  [NrLanes-1:0] sldu_operand_ready;
  target_fu_e [NrLanes-1:0] sldu_operand_target_fu_d, sldu_operand_target_fu_q;

  // Don't handshake if the operands target the addrgen!
  // Moreover, when computing NP2 slides, loop over the same data!
  elen_t [NrLanes-1:0] sldu_operand_d;
  logic [NrLanes-1:0]  sldu_operand_valid_d;
  logic [NrLanes-1:0]  sldu_operand_ready_q;

  typedef enum logic {
    NP2_BUFFER_PNT,
    NP2_RESULT_PNT
  } np2_result_queue_pnt_e;

  typedef enum logic {
    NP2_EXT_SEL,
    NP2_LOOP_SEL
  } np2_loop_mux_e;
  np2_loop_mux_e np2_loop_mux_sel_d, np2_loop_mux_sel_q;

  logic slide_np2_buf_valid_d, slide_np2_buf_valid_q;


  for (genvar l = 0; l < NrLanes; l++) begin
    spill_register #(
      .T(elen_t)
    ) i_sldu_spill_register (
      .clk_i  (clk_i                      ),
      .rst_ni (rst_ni                     ),
      .valid_i(sldu_operand_valid_d[l]    ),
      .ready_o(sldu_operand_ready_q[l]    ),
      .data_i (sldu_operand_d[l]          ),
      .valid_o(sldu_operand_valid[l]      ),
      .ready_i(sldu_operand_ready[l]      ),
      .data_o (sldu_operand[l]            )
    );
  end

  always_comb begin
    for (int l = 0; l < NrLanes; l++) begin
      sldu_operand_d[l] = np2_loop_mux_sel_q == NP2_EXT_SEL
                                        ? sldu_operand_i[l]
                                        : result_queue_q[NP2_BUFFER_PNT][l].wdata;
      sldu_operand_valid_d[l] = (sldu_operand_target_fu_q[l] == ALU_SLDU)
                                        ? (np2_loop_mux_sel_q == NP2_EXT_SEL
                                        ? sldu_operand_valid_i[l]
                                        : slide_np2_buf_valid_q)
                                        : 1'b0;
      sldu_operand_ready_o[l] = (sldu_operand_target_fu_q[l] == ALU_SLDU)
                                        ? (np2_loop_mux_sel_q == NP2_EXT_SEL
                                        ? sldu_operand_ready_q[l]
                                        : 1'b0)
                                        : 1'b0;
    end
  end

  assign sldu_operand_target_fu_d = sldu_operand_target_fu_i;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)
      sldu_operand_target_fu_q <= target_fu_e'(1'b0);
    else
      sldu_operand_target_fu_q <= sldu_operand_target_fu_d;
  end

  //////////////////////////
  //  Cut from the masku  //
  //////////////////////////

  strb_t [NrLanes-1:0] mask_q;
  logic  [NrLanes-1:0] mask_valid_d, mask_valid_q;
  logic                mask_ready_d;
  logic  [NrLanes-1:0] mask_ready_q;

  for (genvar l = 0; l < NrLanes; l++) begin
    stream_register #(
      .T(strb_t)
    ) i_mask_operand_register (
      .clk_i     (clk_i           ),
      .rst_ni    (rst_ni          ),
      .clr_i     (1'b0            ),
      .testmode_i(1'b0            ),
      .data_o    (mask_q[l]       ),
      .valid_o   (mask_valid_q[l] ),
      .ready_i   (mask_ready_d    ),
      .data_i    (mask_i[l]       ),
      .valid_i   (mask_valid_d[l] ),
      .ready_o   (mask_ready_q[l] )
    );

    // Sample only SLDU mask valid
    assign mask_valid_d[l] = mask_valid_i[l] & ~vinsn_issue_q.vm & vinsn_issue_valid_q;
  end


  // Don't upset the masku with a spurious ready
  assign mask_ready_o = mask_ready_q[0] & mask_valid_i[0] & ~vinsn_issue_q.vm & vinsn_issue_valid_q & !(vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu});

  ///////////////////
  //  NP2 Support  //
  ///////////////////

  // The SLDU only supports powers of two (p2) strides
  // Decompose the non-power-of-two (np2) slide in multiple p2 slides

  // We implement the np2 support here and fully process every input packet
  // singularly to comply with the undisturbed policy. We cannot use the VRF
  // as intermediate buffer; each VRF write is a commit.

  typedef logic [idx_width(8*NrLanes)-1:0] stride_t;

  stride_t                                  p2_stride_gen_stride_d;
  logic                                     p2_stride_gen_valid_d;
  logic                                     p2_stride_gen_update_d;
  logic [idx_width(idx_width(8*NrLanes)):0] p2_stride_gen_popc_q;
  stride_t                                  p2_stride_gen_stride_q;
  logic                                     p2_stride_gen_valid_q;

  p2_stride_gen #(
    .NrLanes (NrLanes)
  ) i_p2_stride_gen (
    .clk_i       (clk_i                 ),
    .rst_ni      (rst_ni                ),
    .stride_i    (p2_stride_gen_stride_d),
    .valid_i     (p2_stride_gen_valid_d ),
    .update_i    (p2_stride_gen_update_d),
    .popc_o      (p2_stride_gen_popc_q  ),
    .stride_p2_o (p2_stride_gen_stride_q),
    .valid_o     (p2_stride_gen_valid_q )
  );

  //////////////////
  //  Reductions  //
  //////////////////

  // Inter-lane reductions are performed with a logarithmic tree, and the result is
  // accumulated in the last Lane. Then, in the end, the result is passed to the first
  // lane for SIMD reduction
  logic [idx_width(NrLanes)-1:0] red_stride_cnt_d, red_stride_cnt_q;
  logic [idx_width(NrLanes):0] red_stride_cnt_d_wide;

  logic is_issue_reduction, is_issue_alu_reduction, is_issue_vmfpu_reduction;

  assign is_issue_alu_reduction   = vinsn_issue_valid_q & (vinsn_issue_q.vfu == VFU_Alu);
  assign is_issue_vmfpu_reduction = vinsn_issue_valid_q & (vinsn_issue_q.vfu == VFU_MFpu);
  assign is_issue_reduction       = is_issue_alu_reduction | is_issue_vmfpu_reduction;

  always_comb begin
    sldu_mux_sel_o = NO_RED;
    if ((is_issue_alu_reduction && !(vinsn_commit_valid && vinsn_commit.vfu != VFU_Alu)) || (vinsn_commit_valid && vinsn_commit.vfu == VFU_Alu)) begin
      sldu_mux_sel_o = ALU_RED;
    end else if ((is_issue_vmfpu_reduction && !(vinsn_commit_valid && vinsn_commit.vfu != VFU_MFpu)) || (vinsn_commit_valid && vinsn_commit.vfu == VFU_MFpu)) begin
      sldu_mux_sel_o = MFPU_RED;
    end
  end

  /////////////////////
  //  SLDU DataPath  //
  /////////////////////

  // Input/output non-flat operands
  elen_t [NrLanes-1:0] sld_op_src;
  elen_t [NrLanes-1:0] sld_op_dst;

  // Input and output eew for reshuffling
  rvv_pkg::vew_e sld_eew_src;
  rvv_pkg::vew_e sld_eew_dst;

  // 0: slidedown, 1: slideup
  logic sld_dir;

  // The SLDU slides by powers of two
  logic [idx_width(4*NrLanes):0] sld_slamt;

  sldu_op_dp #(
    .NrLanes  (NrLanes    )
  ) i_sldu_op_dp (
    .op_i     (sld_op_src ),
    .slamt_i  (sld_slamt  ),
    .eew_src_i(sld_eew_src),
    .eew_dst_i(sld_eew_dst),
    .dir_i    (sld_dir    ),
    .op_o     (sld_op_dst )
  );

  //////////////////
  //  Slide unit  //
  //////////////////

  // Vector instructions currently running
  logic [NrVInsn-1:0] vinsn_running_d, vinsn_running_q;

  // Interface with the main sequencer
  pe_resp_t pe_resp;

  // State of the slide FSM
  typedef enum logic [3:0] {
    SLIDE_IDLE,
    SLIDE_RUN,
    SLIDE_RUN_VSLIDE1UP_FIRST_WORD,
    SLIDE_RUN_OSUM,
    SLIDE_WAIT_OSUM,
    SLIDE_NP2_SETUP,
    SLIDE_NP2_RUN,
    SLIDE_NP2_COMMIT,
    SLIDE_NP2_WAIT
  } slide_state_e;
  slide_state_e state_d, state_q;

  logic  [8*NrLanes-1:0] out_en_flat, out_en_seq;
  strb_t [NrLanes-1:0]   out_en;

  // Pointers in the input operand and the output result
  logic   [idx_width(NrLanes*StrbWidth):0] in_pnt_d, in_pnt_q;
  logic   [idx_width(NrLanes*StrbWidth):0] out_pnt_d, out_pnt_q;
  vaddr_t                                  vrf_pnt_d, vrf_pnt_q;

  // Remaining bytes of the current instruction in the issue phase
  vlen_t issue_cnt_d, issue_cnt_q;
  // Respected by default: input_limit_d  = 8*NrLanes + out_pnt_d - in_pnt_d;
  // To enforce: output_limit_d = out_pnt_d + issue_cnt_d;
  logic [idx_width(MAXVL+1):0] output_limit_d, output_limit_q;

  // Remaining bytes of the current instruction in the commit phase
  vlen_t commit_cnt_d, commit_cnt_q;

  vlen_t ring_cnt_d, ring_cnt_q;

  /////////////////////////////
  //     Ring Interconnect   //
  /////////////////////////////

  elen_t fifo_ring_out, fifo_ring_inp;
  logic fifo_ring_ready_inp, fifo_ring_ready_out;
  logic fifo_ring_valid_out, fifo_ring_valid_inp;

  //// Stream FIFOs to handle ring data
  // Fifo to interact with input from ring
  localparam int RingFifoDepth = 4;
  // stream_fifo #(
  //   .FALL_THROUGH(1'b1),
  //   .DATA_WIDTH(64),
  //   .DEPTH(RingFifoDepth)
  // ) i_ring_fifo_input (
  //   .clk_i     (clk_i),
  //   .rst_ni    (rst_ni),

  //   .flush_i   (1'b0),
  //   .testmode_i (1'b0),
  //   .usage_o (/*unused*/),

  //   .data_i    (sldu_ring_i),
  //   .valid_i   (sldu_ring_valid_i),
  //   .ready_o   (sldu_ring_ready_o),

  //   .data_o    (fifo_ring_inp),
  //   .valid_o   (fifo_ring_valid_inp),
  //   .ready_i   (fifo_ring_ready_out)
  // );

  // // Fifo to interact with output to ring
  // stream_fifo #(
  //   .FALL_THROUGH(1'b1),
  //   .DATA_WIDTH(64),
  //   .DEPTH(RingFifoDepth)
  // ) i_ring_fifo_output (
  //   .clk_i     (clk_i),
  //   .rst_ni    (rst_ni),

  //   .flush_i   (1'b0),
  //   .testmode_i (1'b0),
  //   .usage_o (/*unused*/),

  //   .data_i    (fifo_ring_out),
  //   .valid_i   (fifo_ring_valid_out),
  //   .ready_o   (fifo_ring_ready_inp),

  //   .data_o    (sldu_ring_o),
  //   .valid_o   (sldu_ring_valid_o),
  //   .ready_i   (sldu_ring_ready_i)
  // );

  // stream_register #(
  //   .T(elen_t)
  // ) i_stream_inp (
  //   .clk_i (clk_i),
  //   .rst_ni (rst_ni), 
  //   .clr_i     (1'b0), 
  //   .testmode_i (1'b0),

  //   .data_i    (sldu_ring_i),
  //   .valid_i   (sldu_ring_valid_i),
  //   .ready_o   (sldu_ring_ready_o),

  //   .data_o    (fifo_ring_inp),
  //   .valid_o   (fifo_ring_valid_inp),
  //   .ready_i   (fifo_ring_ready_out)
  // );

  // stream_register #(
  //   .T(elen_t)
  // ) i_stream_out (
  //   .clk_i (clk_i),
  //   .rst_ni (rst_ni), 
  //   .clr_i     (1'b0), 
  //   .testmode_i (1'b0),

  //   .data_i    (fifo_ring_out),
  //   .valid_i   (fifo_ring_valid_out),
  //   .ready_o   (fifo_ring_ready_inp),

  //   .data_o    (sldu_ring_o),
  //   .valid_o   (sldu_ring_valid_o),
  //   .ready_i   (sldu_ring_ready_i)
  // );
  
  // Assigning Interface
  assign fifo_ring_inp = sldu_ring_i;
  assign fifo_ring_valid_inp = sldu_ring_valid_i; 
  assign sldu_ring_ready_o = fifo_ring_ready_out; 

  assign sldu_ring_o = fifo_ring_out; 
  assign sldu_ring_valid_o = fifo_ring_valid_out;
  assign fifo_ring_ready_inp = sldu_ring_ready_i; 

  elen_t ring_data_prev_d, ring_data_prev_q;
  logic ring_data_prev_valid_d, ring_data_prev_valid_q;
  logic slide_data_valid;
  logic [$clog2(NrLanes)-1:0] edge_lane_id;
  logic [NrLanes-1:0] slide_data_accepted;
  logic update_inp_op_pnt;

  // For inter-cluster reductions
  cluster_reduction_rx_cnt_t cluster_red_cnt_d, cluster_red_cnt_q;
  logic bypass;
  logic sld_dir_ring;

  logic is_edge_cluster, use_fifo_inp;
  id_cluster_t max_cluster_id; 
  assign max_cluster_id = (1 << num_clusters_i) - 1;

  always_ff @(posedge clk_i or negedge rst_ni) begin : proc_ring
    if(~rst_ni) begin
      ring_data_prev_q       <= '0;
      ring_data_prev_valid_q <= 1'b0;
      cluster_red_cnt_q      <= '0;
      ring_cnt_q             <= '0;
    end else begin
      ring_data_prev_q       <= ring_data_prev_d;
      ring_data_prev_valid_q <= ring_data_prev_valid_d;
      cluster_red_cnt_q      <= cluster_red_cnt_d;
      ring_cnt_q             <= ring_cnt_d;
    end
  end
  
  always_comb begin: p_sldu
    // Maintain state
    vinsn_queue_d = vinsn_queue_q;
    issue_cnt_d   = issue_cnt_q;
    ring_cnt_d    = ring_cnt_q;
    commit_cnt_d  = commit_cnt_q;
    in_pnt_d      = in_pnt_q;
    out_pnt_d     = out_pnt_q;
    vrf_pnt_d     = vrf_pnt_q;
    state_d       = state_q;

    result_queue_d           = result_queue_q;
    result_queue_valid_d     = result_queue_valid_q;
    result_queue_read_pnt_d  = result_queue_read_pnt_q;
    result_queue_write_pnt_d = result_queue_write_pnt_q;
    result_queue_cnt_d       = result_queue_cnt_q;

    result_final_gnt_d = result_final_gnt_q;

    // Vector instructions currently running
    vinsn_running_d = vinsn_running_q & pe_vinsn_running_i;

    out_en_flat    = '0;
    out_en_seq     = '0;
    out_en         = '0;
    output_limit_d = output_limit_q;

    // We are not ready, by default
    pe_resp            = '0;
    mask_ready_d       = 1'b0;
    sldu_operand_ready = '0;


    red_stride_cnt_d = red_stride_cnt_q;

    p2_stride_gen_stride_d = '0;
    p2_stride_gen_valid_d  = 1'b0;
    p2_stride_gen_update_d = 1'b0;

    np2_loop_mux_sel_d    = np2_loop_mux_sel_q;
    slide_np2_buf_valid_d = slide_np2_buf_valid_q;

    red_stride_cnt_d_wide = {red_stride_cnt_q, red_stride_cnt_q[idx_width(NrLanes)-1]};

    // Inform the main sequencer if we are idle
    pe_req_ready_o = !vinsn_queue_full;

    // Slide Unit DP
    sld_op_src  = sldu_operand;
    sld_eew_src = (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})
                ? vinsn_issue_q.vtype.vsew
                : vinsn_issue_q.eew_vs2;
    sld_eew_dst = vinsn_issue_q.vtype.vsew;
    sld_dir     = (vinsn_issue_q.op == VSLIDEUP) || (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu});
    sld_slamt   = (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})
                ? red_stride_cnt_q
                : stride_t'(vinsn_issue_q.stride >> vinsn_issue_q.vtype.vsew);

    // Ring Interconnect states
    sld_dir_ring = (vinsn_issue_q.op == VSLIDEDOWN) ? 1'b0 : 1'b1;
    sldu_dir_o = sld_dir_ring;
    sldu_bypass_o = 1'b0;
    sldu_config_valid_o = 1'b0;
    
    fifo_ring_out = '0; 
    fifo_ring_valid_out = 1'b0;
    fifo_ring_ready_out = 1'b0;

    ring_data_prev_d = ring_data_prev_q;
    ring_data_prev_valid_d = ring_data_prev_valid_q;

    result_queue_write_pnt_q1 = result_queue_write_pnt_q2;

    cluster_red_cnt_d = cluster_red_cnt_q;
    slide_data_valid = 1'b0;

    /////////////////
    //  Slide FSM  //
    /////////////////

    unique case (state_q)
      SLIDE_IDLE: begin
        // To check if ara's in sync to avoid data getting lost in the ring.
        // E.g. for slideup -> slidedown instr., ring direction changes
        // and so if all ring packets not handled properly, it stalls.
        automatic logic inSync = vinsn_issue_valid_q && 
                                   (vinsn_queue_d.issue_pnt == vinsn_queue_d.ring_pnt); 
        // if (vinsn_issue_valid_q) begin
        if (inSync) begin
          state_d   = vinsn_issue_q.is_stride_np2 ? SLIDE_NP2_SETUP : SLIDE_RUN;
          vrf_pnt_d = '0;

          unique case (vinsn_issue_q.op)
            VSLIDEUP: begin
              // -> Supporting only slides by 1 for now.
              // vslideup starts reading the source operand from its beginning
              in_pnt_d  = '0;
              // vslideup starts writing the destination vector at the slide offset
              out_pnt_d = '0; //vinsn_issue_q.stride[idx_width(8*NrLanes)-1:0];

              // Initialize counters
              issue_cnt_d = vinsn_issue_q.vl << int'(vinsn_issue_q.vtype.vsew);

              // Initialize be-enable-generation ancillary signals
              output_limit_d = vinsn_issue_q.use_scalar_op ? out_pnt_d + issue_cnt_d : issue_cnt_d;

              // Trim vector elements which are not touched by the slide unit
              ////issue_cnt_d -= vinsn_issue_q.stride[$bits(issue_cnt_d)-1:0];

              // Start writing at the middle of the destination vector
              vrf_pnt_d = vinsn_issue_q.stride >> $clog2(8*NrLanes);

              // Go to SLIDE_RUN_VSLIDE1UP_FIRST_WORD if this is a vslide1up instruction
              if (vinsn_issue_q.use_scalar_op)
                state_d = SLIDE_RUN_VSLIDE1UP_FIRST_WORD;
            end
            VSLIDEDOWN: begin
              // -> Supporting only slides by 1 for now.
              // vslidedown starts reading the source operand from the slide offset
              in_pnt_d  = 0; // vinsn_issue_q.stride[idx_width(8*NrLanes)-1:0];
              // vslidedown starts writing the destination vector at its beginning
              out_pnt_d = '0;

              // Initialize counters
              issue_cnt_d = vinsn_issue_q.vl << int'(vinsn_issue_q.vtype.vsew);
              output_limit_d = issue_cnt_d;

            end
            // Ordered sum reductions
            VFREDOSUM, VFWREDOSUM: begin
              // Ordered redsum instructions doesn't need in/out_pnt
              in_pnt_d  = '0;
              out_pnt_d = '0;

              // The total number of transactions is vl - 1, but the last data is sent
              // to lane 0
              issue_cnt_d  = vinsn_issue_q.vl;

              state_d = SLIDE_RUN_OSUM;
            end
            // Unordered reductions
            default: begin
              // Unordered redsum instructions doesn't need in/out_pnt
              in_pnt_d  = '0;
              out_pnt_d = '0;

              // Initialize issue cnt. Pretend to move NrLanes 64-bit elements for (clog2(NrLanes) + 1) times.
              issue_cnt_d  = NrLanes * ($clog2(NrLanes) + num_clusters_i + 1 ) << EW64;
            end
          endcase
        end
      end

      SLIDE_RUN, SLIDE_RUN_VSLIDE1UP_FIRST_WORD, SLIDE_NP2_COMMIT: begin
        // Are we ready?
        // During a reduction (vinsn_issue_q.vfu == VFU_Alu/VFU_MFPU) don't wait for mask bits
        if ((&sldu_operand_valid ||
           (((vinsn_issue_q.stride[$bits(vinsn_issue_q.vl)-1:0] >> vinsn_issue_q.vtype.vsew) >= vinsn_issue_q.vl) &&
           (state_q == SLIDE_RUN_VSLIDE1UP_FIRST_WORD))) &&
           !result_queue_full && (vinsn_issue_q.vm || vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} || (|mask_valid_q)))
        begin

          // How many bytes are we copying from the operand to the destination, in this cycle?
          automatic int in_byte_count = NrLanes * 8 - in_pnt_q;
          automatic int out_byte_count = NrLanes * 8 - out_pnt_q;
          automatic int byte_count = in_byte_count < out_byte_count ? in_byte_count : out_byte_count;

          // Build the sequential byte-output-enable
          for (int unsigned b = 0; b < 8*NrLanes; b++)
            if ((b >= out_pnt_q && b < output_limit_q) || vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})
              out_en_seq[b] = 1'b1;

          // Shuffle the output enable
          for (int unsigned b = 0; b < 8*NrLanes; b++)
            out_en_flat[shuffle_index(b, NrLanes, vinsn_issue_q.vtype.vsew)] = out_en_seq[b];

          // Mask the output enable with the mask vector
          out_en = out_en_flat & ({8*NrLanes{vinsn_issue_q.vm | (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})}} | mask_q);

          // Write in the correct bytes
          for (int lane = 0; lane < NrLanes; lane++)
            for (int b = 0; b < 8; b++)
              if (out_en[lane][b]) begin
                result_queue_d[result_queue_write_pnt_q][lane].wdata[8*b +: 8] = sld_op_dst[lane][8*b +: 8];
                result_queue_d[result_queue_write_pnt_q][lane].be[b]           = 1'b1;
              end

          // Initialize id and addr fields of the result queue requests
          for (int lane = 0; lane < NrLanes; lane++) begin
            result_queue_d[result_queue_write_pnt_q][lane].id   = vinsn_issue_q.id;
            result_queue_d[result_queue_write_pnt_q][lane].addr =
              vaddr(vinsn_issue_q.vd, NrLanes) + vrf_pnt_q;
          end

          // Depending on the operation being performed, decide whether to update pointers
          // Also decide whether to use the ring or not for reductions.
          update_inp_op_pnt = 1'b0;
          if (vinsn_issue_q.op inside {VSLIDEUP, VSLIDEDOWN}) begin
            sldu_config_valid_o = 1'b1;
            // Slide operation
            if (fifo_ring_ready_inp) begin
              fifo_ring_out = vinsn_issue_q.op==VSLIDEDOWN ? sldu_operand[0] : sldu_operand[NrLanes-1];
              fifo_ring_valid_out = 1'b1;
              update_inp_op_pnt = 1'b1;
            end
          end else begin
            // Reduction operation
            if (issue_cnt_q <= NrLanes * (num_clusters_i + 1) << EW64) begin
              // Now we have Inter Cluster reduction, where ring is used.
              
              // Put clusters other than 0 into bypass mode, if it has received all reduction packets.
              // This is because cluster 0 has to receive the last packet after all reductions are done.
              bypass = (cluster_id_i !=0) && (cluster_red_cnt_q == cluster_reduction_rx_cnt_init(cluster_id_i));
              sldu_config_valid_o = 1'b1;
              sldu_bypass_o = bypass;
              
              if (!bypass && fifo_ring_ready_inp) begin
                fifo_ring_out = sldu_operand[NrLanes-1];
                fifo_ring_valid_out = 1'b1;

                update_inp_op_pnt = 1'b1;
                cluster_red_cnt_d = cluster_red_cnt_q + 1;
              end else if (bypass) begin
                update_inp_op_pnt = 1'b1;
                slide_data_valid = 1'b1;
              end
              
            end else begin
              // Inter lane reduction still in progress, so don't use ring yet
              update_inp_op_pnt = 1'b1;
              slide_data_valid = 1'b1;
            end
          end

          // Sldu can receive next packet from Lanes
          if (update_inp_op_pnt) begin 
            in_pnt_d    = NrLanes * 8;
            out_pnt_d   = NrLanes * 8;
          end

          // In Jump to SLIDE_RUN if stride is P2
          if (state_q != SLIDE_NP2_COMMIT)
            state_d = SLIDE_RUN;

          // If this is a vslide1up instruction, copy the scalar operand to the first word
          if (state_q == SLIDE_RUN_VSLIDE1UP_FIRST_WORD && cluster_id_i==0) begin
            unique case (vinsn_issue_q.vtype.vsew)
              EW8: begin
                result_queue_d[result_queue_write_pnt_q][0].wdata[7:0] = vinsn_issue_q.scalar_op[7:0];
                result_queue_d[result_queue_write_pnt_q][0].be[0:0] = vinsn_issue_q.vm || mask_q[0][0];
              end
              EW16: begin
                result_queue_d[result_queue_write_pnt_q][0].wdata[15:0] = vinsn_issue_q.scalar_op[15:0];
                result_queue_d[result_queue_write_pnt_q][0].be[1:0] = {2{vinsn_issue_q.vm || mask_q[0][0]}};
              end
              EW32: begin
                result_queue_d[result_queue_write_pnt_q][0].wdata[31:0] = vinsn_issue_q.scalar_op[31:0];
                result_queue_d[result_queue_write_pnt_q][0].be[3:0] = {4{vinsn_issue_q.vm || mask_q[0][0]}};
              end
              EW64: begin
                result_queue_d[result_queue_write_pnt_q][0].wdata[63:0] = vinsn_issue_q.scalar_op[63:0];
                result_queue_d[result_queue_write_pnt_q][0].be[7:0] = {8{vinsn_issue_q.vm || mask_q[0][0]}};
              end
            endcase

          end

          // Read a full word from the VRF or finished the instruction
          if (in_pnt_d == NrLanes * 8) begin
            // Reset the pointer and ask for a new operand
            in_pnt_d           = '0;
            sldu_operand_ready = '1;
            
            // Left-rotate the logarithmic counter. Hacky way to write it, but it's to
            // deal with the 2-lanes design without complaints from Verilator...
            // wide signal to please the tool
            red_stride_cnt_d_wide = {red_stride_cnt_q, red_stride_cnt_q[idx_width(NrLanes)-1]};
            red_stride_cnt_d      = red_stride_cnt_d_wide[idx_width(NrLanes)-1:0];

            // Reset the input data mux
            np2_loop_mux_sel_d = NP2_EXT_SEL;
            if (state_q == SLIDE_NP2_COMMIT) begin
              // Jump to NP2 setup again
              state_d = SLIDE_NP2_SETUP;
            end
          end

          // Filled up a word to the VRF or finished the instruction
          if (out_pnt_d == NrLanes * 8) begin
            // Reset the pointer
            out_pnt_d = vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} ? {'0, red_stride_cnt_d, 3'b0} : '0;
            // We used all the bits of the mask
            if (vinsn_issue_q.op inside {VSLIDEUP, VSLIDEDOWN})
              mask_ready_d = !vinsn_issue_q.vm;

            // Increment VRF address
            vrf_pnt_d = vrf_pnt_q + 1;

            issue_cnt_d = issue_cnt_q - 8*NrLanes;
            if (issue_cnt_q <= 8*NrLanes) begin
                state_d = SLIDE_IDLE;
                
                // Reset the logarighmic counter
                red_stride_cnt_d = 1;

                // Go to the next instruction to be issued
                vinsn_queue_d.issue_pnt += 1;
                if (vinsn_queue_d.issue_pnt == VInsnQueueDepth) begin 
                  vinsn_queue_d.issue_pnt = '0; 
                end
                vinsn_queue_d.issue_cnt -= 1;
            end

            // Send result to the VRF
            result_queue_cnt_d += 1;

            result_queue_write_pnt_d = result_queue_write_pnt_q+1;
            if (result_queue_write_pnt_q == ResultQueueDepth-1)
              result_queue_write_pnt_d = '0;

            if (state_q == SLIDE_NP2_COMMIT) state_d = SLIDE_NP2_WAIT;
          end

        end
        if (state_q == SLIDE_NP2_COMMIT)
          if (slide_np2_buf_valid_q && sldu_operand_ready_q[0])
            // Reset the buffer-valid if the buffer is read, by default
            slide_np2_buf_valid_d = 1'b0;

      end
      SLIDE_RUN_OSUM: begin
        // Short Note: For ordered sum reduction instruction, only one lane has a valid data, and it is sent to the next lane
        // Don't wait for mask bits
        if (!result_queue_full) begin
          for (int lane = 0; lane < NrLanes; lane++) begin
            if (sldu_operand_valid[lane]) begin
              automatic int tgt_lane = (lane == NrLanes - 1) ? 0 : lane + 1;
              // Send result to lane 0
              if (issue_cnt_q == 1) tgt_lane = 0;

              // Acknowledge the received operand
              sldu_operand_ready[lane] = 1'b1;

              // Send result to next lane
              result_queue_d[result_queue_write_pnt_q][tgt_lane].wdata =
                sldu_operand[lane];
              result_queue_d[result_queue_write_pnt_q][tgt_lane].be =
                {8{vinsn_issue_q.vm}} | mask_q[tgt_lane];
              result_queue_valid_d[result_queue_write_pnt_q][tgt_lane] = '1;

              issue_cnt_d = issue_cnt_q - 1;
            end
          end
        end

        // Finish the operation
        if (issue_cnt_d == '0) begin
          state_d      = SLIDE_WAIT_OSUM;
          // Increment vector instruction queue pointers and counters
          vinsn_queue_d.issue_pnt += 1;
          vinsn_queue_d.issue_cnt -= 1;
        end
      end
      SLIDE_WAIT_OSUM: begin
        // Wait one cycle for the last result processing
        commit_cnt_d = 1'b0;
        state_d      = SLIDE_IDLE;
      end
      SLIDE_NP2_SETUP: begin
        // Prepare the write pointer
        result_queue_write_pnt_d = NP2_BUFFER_PNT;
        // Prepare the read pointer
        result_queue_read_pnt_d = NP2_RESULT_PNT;
        // Setup the mux sel as soon as we get one operand
        if (sldu_operand_valid_i[0])
          np2_loop_mux_sel_d = NP2_LOOP_SEL;
        // Setup the p2-stride generator
        p2_stride_gen_stride_d = stride_t'(vinsn_issue_q.stride >> vinsn_issue_q.vtype.vsew);
        p2_stride_gen_valid_d  = 1'b1;
        // Start processing the first VRF chunk as soon as the result queue is completely empty
        if (np2_loop_mux_sel_q == NP2_LOOP_SEL && result_queue_empty) begin
          state_d = SLIDE_NP2_RUN;
        end
      end
      SLIDE_NP2_RUN: begin
        // Reset the buffer-valid if the buffer is read, by default
        if (slide_np2_buf_valid_q && sldu_operand_ready_q[0])
          slide_np2_buf_valid_d = 1'b0;
        // Setup the current p2 stride
        sld_slamt = p2_stride_gen_stride_q;
        // Slide the operands as soon as valid
        if (&sldu_operand_valid) begin
          for (int unsigned l = 0; l < NrLanes; l++)
            result_queue_d[result_queue_write_pnt_q][l].wdata = sld_op_dst[l];
          slide_np2_buf_valid_d = 1'b1;
          // Operands correctly read
          sldu_operand_ready     = '1;
          // Update the p2 stride
          p2_stride_gen_update_d = 1'b1;
          // Commit the final result
          if (p2_stride_gen_popc_q == {'0, 1'b1} && result_queue_empty) begin
            state_d = SLIDE_NP2_COMMIT;
            // Prepare the write pointer
            result_queue_write_pnt_d = NP2_RESULT_PNT;
          end
        end
      end
      SLIDE_NP2_WAIT: begin
        if (result_queue_empty) begin
          result_queue_read_pnt_d  = NP2_RESULT_PNT;
          result_queue_write_pnt_d = NP2_RESULT_PNT;
          state_d = SLIDE_NP2_COMMIT;
        end
      end
      default:;
    endcase

    //////////////////////////////////
    //  Write results into the VRF  //
    //////////////////////////////////
    slide_data_accepted = '0;
    for (int lane = 0; lane < NrLanes; lane++) begin: result_write
      sldu_result_req_o[lane]   = result_queue_valid_q[result_queue_read_pnt_q][lane] & (~(vinsn_commit.vfu inside {VFU_Alu, VFU_MFpu}));
      sldu_red_valid_o[lane]    = result_queue_valid_q[result_queue_read_pnt_q][lane] & (vinsn_commit.vfu inside {VFU_Alu, VFU_MFpu});
      sldu_result_addr_o[lane]  = result_queue_q[result_queue_read_pnt_q][lane].addr;
      sldu_result_id_o[lane]    = result_queue_q[result_queue_read_pnt_q][lane].id;
      sldu_result_wdata_o[lane] = result_queue_q[result_queue_read_pnt_q][lane].wdata;
      sldu_result_be_o[lane]    = result_queue_q[result_queue_read_pnt_q][lane].be;

      // Update the final gnt vector
      result_final_gnt_d[lane] |= sldu_result_final_gnt_i[lane];

      // Received a grant from the VRF (slide) or from the FUs (reduction).
      // Deactivate the request, but do not bump the pointers for now.
      if (((vinsn_commit.vfu inside {VFU_Alu, VFU_MFpu} && sldu_red_valid_o[lane]) || sldu_result_req_o[lane]) && sldu_result_gnt_i[lane]) begin
        result_queue_valid_d[result_queue_read_pnt_q][lane] = 1'b0;
        result_queue_d[result_queue_read_pnt_q][lane]       = '0;
        // Reset the final gnt vector since we are now waiting for another final gnt
        result_final_gnt_d[lane] = 1'b0;
        slide_data_accepted[lane] = 1'b1;
      end
    end: result_write

    // All lanes accepted the VRF request
    // If this was the last request, wait for all the final grants!
    // If this is a reduction, no need for the final grants
    // if (!(|result_queue_valid_d[result_queue_read_pnt_q]) &&
    //   (vinsn_commit.vfu inside {VFU_Alu, VFU_MFpu} || (&result_final_gnt_d || commit_cnt_q > (NrLanes * 8))))
    if (|slide_data_accepted && (vinsn_commit.vfu inside {VFU_Alu, VFU_MFpu} || (&result_final_gnt_d || commit_cnt_q >= (NrLanes * 8))))
      // There is something waiting to be written
      if (!result_queue_empty) begin
        if (state_q != SLIDE_NP2_SETUP)
          // Increment the read pointer
          if (result_queue_read_pnt_q == ResultQueueDepth-1)
            result_queue_read_pnt_d = 0;
          else
            result_queue_read_pnt_d = result_queue_read_pnt_q + 1;

        // Decrement the counter of results waiting to be written
        result_queue_cnt_d -= 1;

        // Decrement the counter of remaining vector elements waiting to be written
        commit_cnt_d = commit_cnt_q - NrLanes * 8;
        if (commit_cnt_q <= (NrLanes * 8)) begin
          commit_cnt_d = '0;
        end
      end

    // Finished committing the results of a vector instruction
    if (vinsn_commit_valid && commit_cnt_d == '0) begin
      // Mark the vector instruction as being done
      pe_resp.vinsn_done[vinsn_commit.id] = 1'b1;

      // Update the commit counters and pointers
      vinsn_queue_d.commit_cnt -= 1;
      if (vinsn_queue_d.commit_pnt == VInsnQueueDepth-1)
        vinsn_queue_d.commit_pnt = '0;
      else
        vinsn_queue_d.commit_pnt += 1;

      // Update the commit counter for the next instruction
      if (vinsn_queue_d.commit_cnt != '0) begin
        commit_cnt_d = vinsn_queue_q.vinsn[vinsn_queue_d.commit_pnt].op inside {VSLIDEUP, VSLIDEDOWN}
                     ? vinsn_queue_q.vinsn[vinsn_queue_d.commit_pnt].vl << int'(vinsn_queue_q.vinsn[vinsn_queue_d.commit_pnt].vtype.vsew)
                     : (NrLanes * $clog2(NrLanes)) << EW64;

        // Add packets for inter cluster reduction
        if (vinsn_queue_q.vinsn[vinsn_queue_d.commit_pnt].vfu inside {VFU_Alu, VFU_MFpu}) begin
          commit_cnt_d += (NrLanes * (num_clusters_i + 1)) << EW64;
        end

        // Trim vector elements which are not written by the slide unit
        if (vinsn_queue_q.vinsn[vinsn_queue_d.commit_pnt].op == VSLIDEUP)
          commit_cnt_d -= vinsn_queue_q.vinsn[vinsn_queue_d.commit_pnt].stride[$bits(commit_cnt_d)-1:0];

      end
    end

    //////////////////////////////////
    //    Handle data from Ring     //
    //////////////////////////////////

    // Check if we have a valid data from the input fifo from the ring
    // and set the ring data to the desired lane
    // For the clusters at the edge (Cluster-0 or NrClusters-1), use the current ring data and previous ring data.
    // For the other clusters, set the input ring data to the edge lane of the current cluster.
    // If this is inter lane reduction, where ring is not needed, we just set data to valid.
    
    edge_lane_id = (vinsn_ring.op==VSLIDEDOWN || vinsn_ring.vfu inside {VFU_Alu, VFU_MFpu}) ? NrLanes-1 : 0;
    is_edge_cluster = (cluster_id_i == max_cluster_id && vinsn_ring.op==VSLIDEDOWN) || (cluster_id_i==0 && vinsn_ring.op==VSLIDEUP);
    
    // For the edge cluster alone, for slidedown, the last 8*NrLanes bytes should not be ring but from previous ring packet and scalar operand.
    use_fifo_inp = 1'b1;
    if (is_edge_cluster && vinsn_ring.op==VSLIDEDOWN && (ring_cnt_q <= 8*NrLanes) && ring_data_prev_valid_q) begin
      use_fifo_inp = 1'b0;
    end
    
    if (result_queue_cnt_d && 
        ((vinsn_queue_d.issue_pnt != vinsn_queue_d.ring_pnt) || ((issue_cnt_d < ring_cnt_d) && (vinsn_queue_d.issue_pnt == vinsn_queue_d.ring_pnt)))) begin
      // If we are in the same instruction and we expect a fifo packet, only then process a fifo packet
      // Otherwise issue has already gone to the next instruction in which case fifo packets are to be used.
      if (fifo_ring_valid_inp && use_fifo_inp) begin
            // We have a valid ring packet
            ring_data_prev_d = fifo_ring_inp;
            ring_data_prev_valid_d = 1'b1;
            
            // For the edge clusters, We need to have the current ring packet and the previous ring packet
            if (is_edge_cluster) begin
              
              if (ring_data_prev_valid_q) begin
                if (vinsn_ring.op==VSLIDEDOWN) begin 
                  unique case (vinsn_ring.vtype.vsew)
                    EW64: result_queue_d[result_queue_write_pnt_q2][edge_lane_id].wdata = {fifo_ring_inp};
                    EW32: result_queue_d[result_queue_write_pnt_q2][edge_lane_id].wdata = {fifo_ring_inp[31:0],
                                                                                           ring_data_prev_q[63:32]};
                    EW16: result_queue_d[result_queue_write_pnt_q2][edge_lane_id].wdata = {fifo_ring_inp[15:0],
                                                                                           ring_data_prev_q[31:16],
                                                                                           ring_data_prev_q[63:48],
                                                                                           ring_data_prev_q[47:32]};
                    EW8 : result_queue_d[result_queue_write_pnt_q2][edge_lane_id].wdata = {fifo_ring_inp[7:0], 
                                                                                           ring_data_prev_q[15:8],
                                                                                           ring_data_prev_q[31:24], 
                                                                                           ring_data_prev_q[23:16],
                                                                                           ring_data_prev_q[63:32]};
                  endcase
                end else begin 
                  unique case (vinsn_ring.vtype.vsew)
                    EW64: result_queue_d[result_queue_write_pnt_q2][edge_lane_id].wdata = {ring_data_prev_q};
                    EW32: result_queue_d[result_queue_write_pnt_q2][edge_lane_id].wdata = {fifo_ring_inp[31:0],
                                                                                           ring_data_prev_q[63:32]};
                    EW16: result_queue_d[result_queue_write_pnt_q2][edge_lane_id].wdata = {fifo_ring_inp[31:16],
                                                                                           fifo_ring_inp[15:0],
                                                                                           fifo_ring_inp[47:32],
                                                                                           ring_data_prev_q[63:48]};
                    EW8 : result_queue_d[result_queue_write_pnt_q2][edge_lane_id].wdata = {fifo_ring_inp[31:0],
                                                                                           fifo_ring_inp[15:8],
                                                                                           fifo_ring_inp[7:0], 
                                                                                           fifo_ring_inp[23:16],
                                                                                           ring_data_prev_q[63:56]};
                  endcase
                end
                slide_data_valid = 1'b1;
              end else begin
                if (vinsn_ring.op==VSLIDEDOWN) begin
                  // Slidedown needs to wait for 1 more ring packet, so don't update pnt
                  slide_data_valid = 1'b0;
                end else if (vinsn_ring.op==VSLIDEUP) begin
                  // For the 1st packet, we can write only the current ring input and the first 32-bits is filled with scalar op later
                  unique case (vinsn_ring.vtype.vsew)
                    EW64: result_queue_d[result_queue_write_pnt_q2][0].wdata[63:0]  = {vinsn_ring.scalar_op};
                    EW32: result_queue_d[result_queue_write_pnt_q2][0].wdata[63:32] = {fifo_ring_inp[31:0]};
                    EW16: result_queue_d[result_queue_write_pnt_q2][0].wdata[63:16] = {fifo_ring_inp[31:16],
                                                                                       fifo_ring_inp[15:0],
                                                                                       fifo_ring_inp[47:32]};
                    EW8 : result_queue_d[result_queue_write_pnt_q2][0].wdata[63:56] = {fifo_ring_inp[31:0],
                                                                                       fifo_ring_inp[15:8],
                                                                                       fifo_ring_inp[7:0], 
                                                                                       fifo_ring_inp[23:16]};
                  endcase
                  slide_data_valid = 1'b1;
                end
              end
            
            end else begin
              // If cluster received a ring packet for reduction, update counter
              if (vinsn_ring.vfu inside {VFU_Alu, VFU_MFpu}) begin
                
                // If this is the last transfer, Cluster-0 should receive data into Lane-0 from Lane-(NrLanes-1) of Cluster-(NrCluster-1)
                if (cluster_id_i==0 && ring_cnt_q==8*NrLanes)
                  result_queue_d[result_queue_write_pnt_q2][0].wdata = fifo_ring_inp;
                else 
                  result_queue_d[result_queue_write_pnt_q2][edge_lane_id].wdata = fifo_ring_inp;
              end else begin
                // Slides for non-edge clusters
                result_queue_d[result_queue_write_pnt_q2][edge_lane_id].wdata = fifo_ring_inp;
              end
              slide_data_valid = 1'b1;
            end
            fifo_ring_ready_out = 1'b1;  // Acknowledge fifo that data has been accepted.
      end else begin
          // For the last data for slide1down use the scalar op
          if (cluster_id_i == max_cluster_id && vinsn_ring.op==VSLIDEDOWN && (ring_cnt_q <= 8*NrLanes) && ring_data_prev_valid_q) begin
            unique case (vinsn_ring.vtype.vsew)
              EW64: result_queue_d[result_queue_write_pnt_q2][edge_lane_id].wdata = {vinsn_ring.scalar_op};
              EW32: result_queue_d[result_queue_write_pnt_q2][edge_lane_id].wdata = {vinsn_ring.scalar_op[31:0] , ring_data_prev_q[63:32]};
              EW16: result_queue_d[result_queue_write_pnt_q2][edge_lane_id].wdata = {vinsn_ring.scalar_op[15:0] , ring_data_prev_q[31:16], ring_data_prev_q[63:48], ring_data_prev_q[47:32]};
              EW8 : result_queue_d[result_queue_write_pnt_q2][edge_lane_id].wdata = {vinsn_ring.scalar_op[7:0]  , ring_data_prev_q[15:8], ring_data_prev_q[31:24], 
                                                                                     ring_data_prev_q[23:16], ring_data_prev_q[63:32]};
            endcase
            slide_data_valid = 1'b1;
            ring_data_prev_d = '0; 
            ring_data_prev_valid_d = 1'b0;
          end
      end
    end

    // Set the data to valid to be sent to the VRF
    // Update the write_pnt_q2
    if (slide_data_valid) begin
      result_queue_valid_d[result_queue_write_pnt_q2] = '1;

      result_queue_write_pnt_q1 = result_queue_write_pnt_q2 + 1;
      if (result_queue_write_pnt_q1 == ResultQueueDepth)
        result_queue_write_pnt_q1 = '0;
      
      ring_cnt_d = ring_cnt_q - 8*NrLanes;
      
      // Update counters and pointers
      if (vinsn_ring_valid && ring_cnt_d == '0) begin
        ring_data_prev_d = '0; 
        ring_data_prev_valid_d = 1'b0;
            
        vinsn_queue_d.ring_cnt -= 1;
        vinsn_queue_d.ring_pnt += 1;
        if (vinsn_queue_d.ring_pnt == VInsnQueueDepth) begin
          vinsn_queue_d.ring_pnt = '0; 
        end 
        
        // Update the ring counter for the next instruction
        if (vinsn_queue_d.ring_cnt != '0) begin
          ring_cnt_d = vinsn_queue_q.vinsn[vinsn_queue_d.ring_pnt].op inside {VSLIDEUP, VSLIDEDOWN}
                        ? vinsn_queue_q.vinsn[vinsn_queue_d.ring_pnt].vl << int'(vinsn_queue_q.vinsn[vinsn_queue_d.ring_pnt].vtype.vsew)
                        : (NrLanes * ($clog2(NrLanes))) << EW64;

          // Add packets for inter cluster reduction
          if (vinsn_queue_q.vinsn[vinsn_queue_d.ring_pnt].vfu inside {VFU_Alu, VFU_MFpu}) begin
            ring_cnt_d += (NrLanes * (num_clusters_i + 1)) << EW64;
          end

          // Trim vector elements which are not written by the slide unit
          if (vinsn_queue_q.vinsn[vinsn_queue_d.ring_pnt].op == VSLIDEUP)
            ring_cnt_d -= vinsn_queue_q.vinsn[vinsn_queue_d.ring_pnt].stride[$bits(ring_cnt_d)-1:0];
        end
      end
    end

    //////////////////////////////
    //  Accept new instruction  //
    //////////////////////////////

    if (!vinsn_queue_full && pe_req_valid_i && !vinsn_running_q[pe_req_i.id] &&
      (pe_req_i.vfu == VFU_SlideUnit || pe_req_i.op inside {[VREDSUM:VWREDSUM], [VFREDUSUM:VFWREDOSUM]})) begin
      vinsn_queue_d.vinsn[vinsn_queue_q.accept_pnt] = pe_req_i;
      vinsn_running_d[pe_req_i.id]                  = 1'b1;

      // Calculate the slide offset inside the vector register
      if (pe_req_i.op inside {VSLIDEUP, VSLIDEDOWN})
        vinsn_queue_d.vinsn[vinsn_queue_q.accept_pnt].stride = pe_req_i.stride <<
          int'(pe_req_i.vtype.vsew);
      // Always move 64-bit packs of data from one lane to the other
      if (pe_req_i.vfu inside {VFU_Alu, VFU_MFpu})
        vinsn_queue_d.vinsn[vinsn_queue_q.accept_pnt].vtype.vsew = EW64;

      if (vinsn_queue_d.commit_cnt == '0) begin
        commit_cnt_d = pe_req_i.op inside {VSLIDEUP, VSLIDEDOWN}
                     ? pe_req_i.vl << int'(pe_req_i.vtype.vsew)
                     : (NrLanes * ($clog2(NrLanes))) << EW64;

        // Add packets for inter cluster reduction
        if (pe_req_i.vfu inside {VFU_Alu, VFU_MFpu}) begin 
          commit_cnt_d += (NrLanes * (num_clusters_i + 1)) << EW64;
        end

        // Trim vector elements which are not written by the slide unit
        // VSLIDE1UP always writes at least 1 element
        if (pe_req_i.op == VSLIDEUP && !pe_req_i.use_scalar_op) begin
          commit_cnt_d -= vinsn_queue_d.vinsn[vinsn_queue_q.accept_pnt].stride[$bits(commit_cnt_d)-1:0];
        end
      end

      // Set expected ring counter packets for the instruction
      if (vinsn_queue_d.ring_cnt == '0) begin
        ring_cnt_d = pe_req_i.op inside {VSLIDEUP, VSLIDEDOWN}
                     ? pe_req_i.vl << int'(pe_req_i.vtype.vsew)
                     : (NrLanes * ($clog2(NrLanes))) << EW64;

        // Add packets for inter cluster reduction
        if (pe_req_i.vfu inside {VFU_Alu, VFU_MFpu}) begin 
          ring_cnt_d += (NrLanes * (num_clusters_i + 1)) << EW64;
        end

        // Trim vector elements which are not written by the slide unit
        // VSLIDE1UP always writes at least 1 element
        if (pe_req_i.op == VSLIDEUP && !pe_req_i.use_scalar_op) begin
          ring_cnt_d -= vinsn_queue_d.vinsn[vinsn_queue_q.accept_pnt].stride[$bits(ring_cnt_d)-1:0];
        end
      end

      // Bump pointers and counters of the vector instruction queue
      if (vinsn_queue_q.accept_pnt == VInsnQueueDepth-1) begin
        vinsn_queue_d.accept_pnt = '0;
      end else begin
        vinsn_queue_d.accept_pnt += 1;
      end
      vinsn_queue_d.issue_cnt += 1;
      vinsn_queue_d.ring_cnt += 1;
      vinsn_queue_d.commit_cnt += 1;
    end
  end: p_sldu
  
  // Function to track how many packets should be pushed into ring by each cluster for inter cluster reduction.
  // E.g. for 4 clusters
  // Step-1: In the first step 0->1 1->2 2->3 and 3->1 - all clusters send a single packet to all other clusters. So atleast 1 present.
  // Step-2: 2nd step 1->3, put 2 in bypass mode (i.e. even numbered clusters other than 0). 
  //         Since 0 is not in bypass 3->0, 0->1 also happens. But the FPU in last lane of cluster-0 and 1, receiving data from SLDU ignores it.
  //         This is tracked by the reduction_rx_cnt_d in FPU. 
  // Last-Step: Finally 3 sends result to 1.
  function automatic cluster_reduction_rx_cnt_t cluster_reduction_rx_cnt_init(id_cluster_t clusterId);
    case (clusterId)
      0:  cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      1:  cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(2);
      2:  cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      3:  cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(3);
      4:  cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      5:  cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(2);
      6:  cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      7:  cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(4);
      8:  cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      9:  cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(2);
      10: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      11: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(3);
      12: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      13: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(2);
      14: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      15: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(5);
      16: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      17: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(2);
      18: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      19: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(3);
      20: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      21: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(2);
      22: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      23: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(4);
      24: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      25: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(2);
      26: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      27: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(3);
      28: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      29: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(2);
      30: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      31: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(6);
    endcase
  endfunction: cluster_reduction_rx_cnt_init

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      vinsn_running_q       <= '0;
      issue_cnt_q           <= '0;
      commit_cnt_q          <= '0;
      in_pnt_q              <= '0;
      out_pnt_q             <= '0;
      vrf_pnt_q             <= '0;
      output_limit_q        <= '0;
      state_q               <= SLIDE_IDLE;
      pe_resp_o             <= '0;
      result_final_gnt_q    <= '0;
      red_stride_cnt_q      <= 1;
      np2_loop_mux_sel_q    <= NP2_EXT_SEL;
      slide_np2_buf_valid_q <= 1'b0;
    end else begin
      vinsn_running_q       <= vinsn_running_d;
      issue_cnt_q           <= issue_cnt_d;
      commit_cnt_q          <= commit_cnt_d;
      in_pnt_q              <= in_pnt_d;
      out_pnt_q             <= out_pnt_d;
      vrf_pnt_q             <= vrf_pnt_d;
      output_limit_q        <= output_limit_d;
      state_q               <= state_d;
      pe_resp_o             <= pe_resp;
      result_final_gnt_q    <= result_final_gnt_d;
      red_stride_cnt_q      <= red_stride_cnt_d;
      np2_loop_mux_sel_q    <= np2_loop_mux_sel_d;
      slide_np2_buf_valid_q <= slide_np2_buf_valid_d;      
    end
  end

endmodule: sldu
