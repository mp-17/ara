// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Matheus Cavalcante <matheusd@iis.ee.ethz.ch>
// Description:
// This is the sequencer of one lane. It controls the execution of one vector
// instruction within one lane, interfacing with the internal functional units
// and with the main sequencer.

module lane_sequencer import ara_pkg::*; import rvv_pkg::*; import cf_math_pkg::idx_width; #(
    parameter int unsigned NrLanes = 0
  ) (
    input  logic                                          clk_i,
    input  logic                                          rst_ni,
    // Lane ID
    input  logic                 [idx_width(NrLanes)-1:0] lane_id_i,
    // Interface with the main sequencer
    input  pe_req_t                                       pe_req_i,
    input  logic                                          pe_req_valid_i,
    input  logic                 [NrVInsn-1:0]            pe_vinsn_running_i,
    output logic                                          pe_req_ready_o,
    output pe_resp_t                                      pe_resp_o,
    // Interface with the operand requester
    output operand_request_cmd_t [NrOperandQueues-1:0]    operand_request_o,
    output logic                 [NrOperandQueues-1:0]    operand_request_valid_o,
    input  logic                 [NrOperandQueues-1:0]    operand_request_ready_i,
    output logic                                          alu_vinsn_done_o,
    output logic                                          mfpu_vinsn_done_o,
    // Interface with the lane's VFUs
    output vfu_operation_t                                vfu_operation_o,
    output logic                                          vfu_operation_valid_o,
    input  logic                                          alu_ready_i,
    input  logic                 [NrVInsn-1:0]            alu_vinsn_done_i,
    input  logic                                          mfpu_ready_i,
    input  logic                 [NrVInsn-1:0]            mfpu_vinsn_done_i
  );

  // Find the first lane that will fetch a VRF word with at least a potentially valid element inside
  function automatic logic [$clog2(NrLanes)-1:0] first_active_lane(rvv_pkg::vew_e eew, vlen_t vstart);
    // Start lane
    // Number of elements in a single L*64-bit fetch: (NrLanes << (64 - pe_req_d.vtype.vsew)).
    // vstart / (NrLanes << (64 - pe_req_d.vtype.vsew)) -> don't care.
    // vstart % NrLanes -> our starting lane if:
    // (vstart % (NrLanes << (64 - pe_req_d.vtype.vsew))) / NrLanes.
    // Otherwise, the starting lane continues to be the 0th.

    // Work on the correct number of bits
    logic [$clog2(MaxNrLanes)-1:0] start_lane;

    unique case (eew)
      rvv_pkg::EW8: begin
        start_lane = &vstart[$clog2(8*NrLanes)-1:$clog2(NrLanes)]
                   ? vstart[$clog2(NrLanes)-1:0]
                   : '0;
      end
      rvv_pkg::EW16: begin
        start_lane = &vstart[$clog2(4*NrLanes)-1:$clog2(NrLanes)]
                   ? vstart[$clog2(NrLanes)-1:0]
                   : '0;
      end
      rvv_pkg::EW32: begin
        start_lane = &vstart[$clog2(2*NrLanes)-1:$clog2(NrLanes)]
                   ? vstart[$clog2(NrLanes)-1:0]
                   : '0;
      end
      // EW64, default
      default: begin
        start_lane = vstart[$clog2(NrLanes)-1:0];
      end
    endcase

    return start_lane;
  endfunction

  // Find the last lane that will fetch a VRF word with at least a potentially valid element inside
  function automatic logic [$clog2(NrLanes)-1:0] last_active_lane(rvv_pkg::vew_e eew, vlen_t vl);
    // End lane
    // Number of elements in a single L*64-bit fetch: (NrLanes << (64 - vtype.vsew)).
    // vl / (NrLanes << (64 - vtype.vsew)) -> don't care.
    // (vl % NrLanes) - 1 -> our end lane if:
    // (vl % (NrLanes << (64 - vtype.vsew)) - 1) / NrLanes.
    // With the end lane we should subtract 1 since vl represents a number of
    // elements and NOT an index.

    // Work on the correct number of bits
    logic [$clog2(NrLanes)-1:0] end_lane;

    // Buffers to simplify the code reading
    logic [$clog2(8*NrLanes)-1:0] buf8;
    logic [$clog2(4*NrLanes)-1:0] buf16;
    logic [$clog2(2*NrLanes)-1:0] buf32;

    unique case (eew)
      rvv_pkg::EW8: begin
        buf8       = vl[$clog2(8*NrLanes)-1:0] - 1;
        end_lane   = !(|buf8[$clog2(8*NrLanes)-1:$clog2(NrLanes)])
                   ? vl[$clog2(NrLanes)-1:0] - 1
                   : '1;
      end
      rvv_pkg::EW16: begin
        buf16      = vl[$clog2(4*NrLanes)-1:0] - 1;
        end_lane   = !(|buf16[$clog2(4*NrLanes)-1:$clog2(NrLanes)])
                   ? vl[$clog2(NrLanes)-1:0] - 1
                   : '1;
      end
      rvv_pkg::EW32: begin
        buf32      = vl[$clog2(2*NrLanes)-1:0] - 1;
        end_lane   = !(|buf32[$clog2(2*NrLanes)-1:$clog2(NrLanes)])
                   ? vl[$clog2(NrLanes)-1:0] - 1
                   : '1;
      end
      // EW64, default
      default: begin
        end_lane   = vl[$clog2(NrLanes)-1:0] - 1;
      end
    endcase

    return end_lane;
  endfunction

  // Number of ELEN * #Lane large words
  // Standard case: LMUL_MAX*VLEN/(NrLanes*ELEN) == 1024 / 8 == 128
  typedef logic [$clog2(MAX_LMUL*VLEN/(NrLanes*ELEN))-1:0] wide_vrf_word_t;
  typedef logic [$clog2(VLEN/NrLanes):0] lane_vlen_t;
  logic vrf_words_t balanced_words_vs1, balanced_words_vs2, balanced_words_vd, balanced_words_vm;
  logic vrf_words_t unbalanced_words_vs1, unbalanced_words_vs2, unbalanced_wordsy_vd;

  // How many (ELENB * NrLanes)-wide words we need to fetch from the VRF.
  // This corresponds to the number of (ELENB)-wide words to be fetched from each VRF chunk.
  function vrf_words_t balanced_words(vew_e eew, vlen_t vl, vlen_t vstart, vlen_t stride, int unsigned NrLanes)
    // Slides can reduce the effective vl with their strides
    vl_eff = vl - stride;
    // ceil(vl_eff_byte / wide_word_byte) - floor(vstart_byte / wide_word_byte)
    balanced_words = (vl_eff << eew) / (ELENB * NrLanes) - (vstart << eew) / (ELENB * NrLanes);
    if ((vl_eff << eew) % (ELENB * NrLanes))
      balanced_words += 1;
  endfunction

  // How many (ELEN * NrLanes)-wide words we need to fetch from the VRF.
  // This corresponds to the number of (ELEN)-wide words to be fetched from each VRF chunk.
  // vl is in [bit] here (words for the mask unit)
  function vrf_words_t balanced_words_vm(vlen_t vl, vlen_t vstart, vlen_t stride, int unsigned NrLanes)
    // Slides can reduce the effective vl with their strides
    vl_eff = vl - stride;
    // ceil(vl_eff_bit / wide_word_bit) - floor(vstart_bit / wide_word_bit)
    balanced_words_vm = (vl_eff / (ELEN * NrLanes)) - (vstart / (ELEN * NrLanes));
    if (vl_eff % (ELEN * NrLanes))
      balanced_words_vm += 1;
  endfunction

  // ELEN-wide words to be fetched by this lane
  // The slide unit does not need unbalanced_words, so we can avoid resizing vl because of the stride
  function vrf_words_t unbalanced_words([idx_width(NrLanes)-1:0] lane_id, vew_e eew, vlen_t vl, vlen_t vstart, vlen_t stride, int unsigned NrLanes)
    // Find start and end lane for VRF words for this specific eew
    start_lane_id = start_lane(eew, vstart);
    end_lane_id = end_lane(eew, vl);
    // floor(vl_byte / wide_word_byte) - floor(vstart_byte - wide_word_byte)
    unbalanced_words = (vl << eew) / (ELENB * NrLanes) - (vstart << eew) / (ELENB * NrLanes);
    // Account for the lane index
    unique case ({lane_id <= end_lane_id, lane id < start_lane_id})
      2'b01: unbalanced_words -= 1;
      2'b10: unbalanced_words += 1;
      default:;
    endcase
  endfunction

  // #Elements to be fetched by this lane
  function lane_vlen_t unbalanced_vl([idx_width(NrLanes)-1:0] lane_id, vlen_t vl, vlen_t vstart, int unsigned NrLanes)
    // floor(vl / Nrlanes) - floor(vstart / NrLanes) ; avoid dependency on lane index
    unbalanced_vl = vl / NrLanes - vstart / NrLanes;
    // Account for the lane index
    unique case ({lane_id < pe_req.vl[idx_width(NrLanes)-1:0], lane_id < pe_req.vstart[idx_width(NrLanes)-1:0]})
      2'b01: unbalanced_vl -= 1;
      2'b10: unbalanced_vl += 1;
      default:;
    endcase
    return unbalanced_vl;
  endfunction

  // #Elements to be fetched by this lane if the load needs to be balanced
  function lane_vlen_t balanced_vl(vlen_t vl, vlen_t vstart, int unsigned NrLanes)
    // ceil(vl / Nrlanes) - floor(vstart / NrLanes)
    balanced_vl = vl / NrLanes - vstart / NrLanes;
    if (vl % NrLanes)
      balanced_vl += 1;
    return balanced_vl;
  endfunction

  ////////////////////////////
  //  Register the request  //
  ////////////////////////////

  // Don't accept the same request more than once!
  // The main sequencer keeps the valid high and broadcast
  // a certain instruction with ID == X to all the lanes
  // until every lane has sampled it.

  // Every time a lane handshakes the main sequencer, it also
  // saves the insn ID, not to re-sample the same instruction.
  vid_t last_id_d, last_id_q;
  logic pe_req_valid_i_msk;
  logic en_sync_mask_d, en_sync_mask_q;

  pe_req_t pe_req;
  logic    pe_req_valid;
  logic    pe_req_ready;

  fall_through_register #(
    .T(pe_req_t)
  ) i_pe_req_register (
    .clk_i     (clk_i             ),
    .rst_ni    (rst_ni            ),
    .clr_i     (1'b0              ),
    .testmode_i(1'b0              ),
    .data_i    (pe_req_i          ),
    .valid_i   (pe_req_valid_i_msk),
    .ready_o   (pe_req_ready_o    ),
    .data_o    (pe_req            ),
    .valid_o   (pe_req_valid      ),
    .ready_i   (pe_req_ready      )
  );

  always_comb begin
    // Default assignment
    last_id_d      = last_id_q;
    en_sync_mask_d = en_sync_mask_q;

    // If the sync mask is enabled and the ID is the same
    // as before, avoid to re-sample the same instruction
    // more than once.
    if (en_sync_mask_q && (pe_req_i.id == last_id_q))
      pe_req_valid_i_msk = 1'b0;
    else
      pe_req_valid_i_msk = pe_req_valid_i;

    // Enable the sync mask when a handshake happens,
    // and save the insn ID
    if (pe_req_valid_i_msk && pe_req_ready_o) begin
      last_id_d      = pe_req_i.id;
      en_sync_mask_d = 1'b1;
    end

    // Disable the block if the sequencer valid goes down
    if (!pe_req_valid_i && en_sync_mask_q)
      en_sync_mask_d = 1'b0;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      last_id_q      <= '0;
      en_sync_mask_q <= 1'b0;
    end else begin
      last_id_q      <= last_id_d;
      en_sync_mask_q <= en_sync_mask_d;
    end
  end

  //////////////////////////////////////
  //  Operand Request Command Queues  //
  //////////////////////////////////////

  // We cannot use a simple FIFO because the operand request commands include
  // bits that indicate whether there is a hazard between different vector
  // instructions. Such hazards must be continuously cleared based on the
  // value of the currently running loops from the main sequencer.
  operand_request_cmd_t [NrOperandQueues-1:0] operand_request;
  logic                 [NrOperandQueues-1:0] operand_request_push;

  operand_request_cmd_t [NrOperandQueues-1:0] operand_request_d;
  logic                 [NrOperandQueues-1:0] operand_request_valid_d;

  always_comb begin: p_operand_request
    for (int queue = 0; queue < NrOperandQueues; queue++) begin
      // Maintain state
      operand_request_d[queue]       = operand_request_o[queue];
      operand_request_valid_d[queue] = operand_request_valid_o[queue];

      // Clear the request
      if (operand_request_ready_i[queue]) begin
        operand_request_d[queue]       = '0;
        operand_request_valid_d[queue] = 1'b0;
      end

      // Got a new request
      if (operand_request_push[queue]) begin
        operand_request_d[queue]       = operand_request[queue];
        operand_request_valid_d[queue] = 1'b1;
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin: p_operand_request_ff
    if (!rst_ni) begin
      operand_request_o       <= '0;
      operand_request_valid_o <= '0;
    end else begin
      operand_request_o       <= operand_request_d;
      operand_request_valid_o <= operand_request_valid_d;
    end
  end

  /////////////////////////////
  //  VFU Operation control  //
  /////////////////////////////

  // Running instructions
  logic [NrVInsn-1:0] vinsn_done_d, vinsn_done_q;
  logic [NrVInsn-1:0] vinsn_running_d, vinsn_running_q;

  // VFU operation
  vfu_operation_t vfu_operation_d;
  logic           vfu_operation_valid_d;

  // Cut the path
  logic alu_vinsn_done_d, mfpu_vinsn_done_d;

  // Returns true if the corresponding lane VFU is ready.
  function automatic logic vfu_ready(vfu_e vfu, logic alu_ready_i, logic mfpu_ready_i);
    vfu_ready = 1'b1;
    unique case (vfu)
      VFU_Alu,
      VFU_MaskUnit: vfu_ready = alu_ready_i;
      VFU_MFpu    : vfu_ready = mfpu_ready_i;
      default:;
    endcase
  endfunction : vfu_ready

  always_comb begin: sequencer
    // Running loops
    vinsn_running_d = vinsn_running_q & pe_vinsn_running_i;

    // Ready to accept a new request, by default
    pe_req_ready = 1'b1;

    // Loops that finished execution
    vinsn_done_d         = alu_vinsn_done_i | mfpu_vinsn_done_i;
    alu_vinsn_done_d     = |alu_vinsn_done_i;
    mfpu_vinsn_done_d    = |mfpu_vinsn_done_i;
    pe_resp_o.vinsn_done = vinsn_done_q;

    // Make no requests to the operand requester
    operand_request    = '0;
    operand_request_push = '0;

    // Make no requests to the lane's VFUs
    vfu_operation_d       = '0;
    vfu_operation_valid_d = 1'b0;

    // Find the number of VRF balanced words
    balanced_words_vs1 = balanced_words(lane_id_i, end_lane_id, eew_vs1, vl, vstart, 0, NrLanes);
    balanced_words_vs2 = balanced_words(lane_id_i, end_lane_id, eew_vs2, vl, vstart, 0, NrLanes);
    balanced_words_vd  = balanced_words(lane_id_i, end_lane_id, eew_vd,  vl, vstart, 0, NrLanes);
    balanced_words_vm  = balanced_words_vm(lane_id_i, vl, vstart, 0, NrLanes);

    // Find the number of VRF unbalanced words
    unbalanced_words_vs1 = unbalanced_words(lane_id_i, start_lane_id, end_lane_id, eew_vs1, vl, vstart, NrLanes);
    unbalanced_words_vs2 = unbalanced_words(lane_id_i, start_lane_id, end_lane_id, eew_vs2, vl, vstart, NrLanes);
    unbalanced_words_vd  = unbalanced_words(lane_id_i, start_lane_id, end_lane_id, eew_vd,  vl, vstart, NrLanes);

    // Find the number of VRF unbalanced vl
    unbalanced_vl = unbalanced_vl(lane_id_i, pe_req.vl, pe_req.vstart, NrLanes);
    // Find the number of VRF balanced vl
    balanced_vl = balanced_vl(pe_req.vl, pe_req.vstart, NrLanes);

    // Is this a reduction?
    is_reduct = (pe_req.op inside {[VREDSUM:VWREDSUM, [VFREDUSUM:VFWREDOSUM]]});

    // If the operand requesters are busy, abort the request and wait for another cycle.
    if (pe_req_valid) begin : stall_op_req_busy
      unique case (pe_req.vfu)
        VFU_Alu : begin
          pe_req_ready = !(operand_request_valid_o[AluA] ||
            operand_request_valid_o[AluB ] ||
            operand_request_valid_o[MaskM]);
        end
        VFU_MFpu : begin
          pe_req_ready = !(operand_request_valid_o[MulFPUA] ||
            operand_request_valid_o[MulFPUB] ||
            operand_request_valid_o[MulFPUC] ||
            operand_request_valid_o[MaskM]);
        end
        VFU_LoadUnit : pe_req_ready = !(operand_request_valid_o[MaskM] ||
            (pe_req_i.op == VLXE && operand_request_valid_o[SlideAddrGenA]));
        VFU_SlideUnit: pe_req_ready = !(operand_request_valid_o[SlideAddrGenA]);
        VFU_StoreUnit: begin
          pe_req_ready = !(operand_request_valid_o[StA] ||
            operand_request_valid_o[MaskM] ||
            (pe_req_i.op == VSXE && operand_request_valid_o[SlideAddrGenA]));
        end
        VFU_MaskUnit : begin
          pe_req_ready = !(operand_request_valid_o[AluA] ||
            operand_request_valid_o [AluB] ||
            operand_request_valid_o [MulFPUA] ||
            operand_request_valid_o [MulFPUB] ||
            operand_request_valid_o[MaskB] ||
            operand_request_valid_o[MaskM]);
        end
        VFU_None : begin
          pe_req_ready = !(operand_request_valid_o[MaskB]);
        end
        default:;
      endcase // stall_op_req_busy
    end

    // We received a new vector instruction
    if (pe_req_valid && pe_req_ready && !vinsn_running_d[pe_req.id]) begin : if_pe_req_valid
      // Populate the VFU request
      vfu_operation_d = '{
        id             : pe_req.id,
        op             : pe_req.op,
        vm             : pe_req.vm,
        vfu            : pe_req.vfu,
        use_vs1        : pe_req.use_vs1,
        use_vs2        : pe_req.use_vs2,
        use_vd_op      : pe_req.use_vd_op,
        scalar_op      : pe_req.scalar_op,
        use_scalar_op  : pe_req.use_scalar_op,
        vd             : pe_req.vd,
        use_vd         : pe_req.use_vd,
        swap_vs2_vd_op : pe_req.swap_vs2_vd_op,
        fp_rm          : pe_req.fp_rm,
        wide_fp_imm    : pe_req.wide_fp_imm,
        cvt_resize     : pe_req.cvt_resize,
        vtype          : pe_req.vtype,
        default        : '0
      };
      vfu_operation_valid_d = (vfu_operation_d.vfu != VFU_None) ? 1'b1 : 1'b0;

      vfu_operation_d.vl = pe_req.unbalanced ? unbalanced_vl : balanced_vl;
      vfu_operation_d.words_vreg = pe_req.unbalanced ? unbalanced_words : balanced_words;

      // Calculate the start element for Lane[i]. This will be forwarded to both opqueues
      // and operand requesters, with some light modification in the case of a vslide.
      // Regardless of the EW, the start element of Lane[i] is "vstart / NrLanes".
      // If vstart deos not divide NrLanes perfectly, some low-index lanes will send
      // mock data to balance the payload.
      vstart_balanced = pe_req.vstart / NrLanes;
      vstart_unbalanced = pe_req.vstart / NrLanes;
      if (lane_id_i < (pe_req.vstart % NrLanes))
        vstart_unbalanced += 1;
      vfu_operation_d.vstart = vstart_balanced;

      // Mark the vector instruction as running
      vinsn_running_d[pe_req.id] = (vfu_operation_d.vfu != VFU_None) ? 1'b1 : 1'b0;

      // Mute request if the instruction runs in the lane, is unbalanced, and vl is greater than vstart.
      // Example 1 of balanced insn: insn on mask vectors, as MASKU has to receive something from all lanes
      // and the partial results come from VALU and VMFPU.
      // Example 2 of balanced insn: during a reduction, all the lanes must cooperate in the inter-lane phase.
      if (vfu_operation_d.vl > vfu_operation_d.vstart && is_unbalanced) begin
        vfu_operation_valid_d = 1'b0;
        // We are already done with this instruction
        vinsn_done_d[pe_req.id] |= 1'b1;
        vinsn_running_d[pe_req.id] = 1'b0;
      end

      ////////////////////////
      //  Operand requests  //
      ////////////////////////

      unique case (pe_req.vfu)
        VFU_Alu: begin
          operand_request[AluA] = '{
            id         : pe_req.id,
            vs         : pe_req.vs1,
            eew        : pe_req.eew_vs1,
            conv       : pe_req.conversion_vs1,
            // If reductions and vl == 0, we must replace with neutral values
            neutral_val: OpQueueReductionZExt
            unbalanced : pe_req.unbalanced,
            cvt_resize : pe_req.cvt_resize,
            vtype      : pe_req.vtype,
            // In case of reduction, AluA opqueue will keep the scalar element
            words_vreg : is_reduct ? 1 : unbalanced_words_vs1,
            vl         : is_reduct ? 1 : unbalanced_vl,
            vstart     : vstart,
            hazard     : pe_req.hazard_vs1 | pe_req.hazard_vd,
            is_reduct  : is_reduct,
            target_fu  : ALU_SLDU,
            default    : '0
          };
          operand_request_push[AluA] = pe_req.use_vs1;

          operand_request[AluB] = '{
            id         : pe_req.id,
            vs         : pe_req.vs2,
            eew        : pe_req.eew_vs2,
            conv       : pe_req.conversion_vs2,
            // If reductions and vl == 0, we must replace with neutral values
            neutral_val: (vfu_operation_d.vl == '0) ? OpQueueReductionZExt
            scale_vl   : pe_req.scale_vl,
            cvt_resize : pe_req.cvt_resize,
            vtype      : pe_req.vtype,
            // If reductions and vl == 0, we must replace the operands with neutral
            // values in the opqueues. So, vl must be 1 at least
            words_vreg : is_reduct ? 1 : unbalanced_words_vs2,
            vl         : (is_reduct && vfu_operation_d.vl == '0)
                         ? 1 : unbalanced_vl,
            vstart     : vfu_operation_d.vstart,
            hazard     : pe_req.hazard_vs2 | pe_req.hazard_vd,
            is_reduct  : is_reduct ? 1'b1 : 0,
            target_fu  : ALU_SLDU,
            default    : '0
          };
          operand_request_push[AluB] = pe_req.use_vs2;

          // This vector instruction uses masks
          operand_request[MaskM] = '{
            id     : pe_req.id,
            vs     : VMASK,
            eew    : pe_req.vtype.vsew,
            vtype  : pe_req.vtype,
            // The payload to the masku is always balanced
		    words_vreg: balanced_words_vm,
            vl     : pe_req.vl,
            vstart : vfu_operation_d.vstart,
            hazard : pe_req.hazard_vm | pe_req.hazard_vd,
            default: '0
          };
          operand_request_push[MaskM] = !pe_req.vm;
        end
        VFU_MFpu: begin
          operand_request[MulFPUA] = '{
            id         : pe_req.id,
            vs         : pe_req.vs1,
            eew        : pe_req.eew_vs1,
            // If reductions and vl == 0, we must replace with neutral values
            conv       : pe_req.conversion_vs1,
            scale_vl   : pe_req.scale_vl,
            cvt_resize : pe_req.cvt_resize,
            vtype      : pe_req.vtype,
            // If reductions and vl == 0, we must replace the operands with neutral
            // values in the opqueues. So, vl must be 1 at least
            words_vreg : is_reduct ? 1 : unbalanced_words_vreg,
            vl         : is_reduct ? 1 : unbalanced_vl,
            vstart     : vfu_operation_d.vstart,
            hazard     : pe_req.hazard_vs1 | pe_req.hazard_vd,
            is_reduct  : is_reduct ? 1'b1 : 0,
            target_fu  : MFPU_ADDRGEN,
            default    : '0
          };
          operand_request_push[MulFPUA] = pe_req.use_vs1;

          operand_request[MulFPUB] = '{
            id         : pe_req.id,
            vs         : pe_req.swap_vs2_vd_op ? pe_req.vd        : pe_req.vs2,
            eew        : pe_req.swap_vs2_vd_op ? pe_req.eew_vd_op : pe_req.eew_vs2,
            // If reductions and vl == 0, we must replace with neutral values
            conv       : pe_req.conversion_vs2,
            scale_vl   : pe_req.scale_vl,
            cvt_resize : pe_req.cvt_resize,
            vtype      : pe_req.vtype,
            // If reductions and vl == 0, we must replace the operands with neutral
            // values in the opqueues. So, vl must be 1 at least
            words_vreg : is_reduct ? 1 : unbalanced_words_vreg,
            vl         : (is_reduct && vfu_operation_d.vl == '0)
                        ? 1 : unbalanced_vl,
            vstart     : vfu_operation_d.vstart,
            hazard     : (pe_req.swap_vs2_vd_op ?
            pe_req.hazard_vd : (pe_req.hazard_vs2 | pe_req.hazard_vd)),
            is_reduct  : is_reduct ? 1'b1 : 0,
            target_fu  : MFPU_ADDRGEN,
            default: '0
          };
          operand_request_push[MulFPUB] = pe_req.swap_vs2_vd_op ?
          pe_req.use_vd_op : pe_req.use_vs2;

          operand_request[MulFPUC] = '{
            id         : pe_req.id,
            vs         : pe_req.swap_vs2_vd_op ? pe_req.vs2            : pe_req.vd,
            eew        : pe_req.swap_vs2_vd_op ? pe_req.eew_vs2        : pe_req.eew_vd_op,
            conv       : pe_req.swap_vs2_vd_op ? pe_req.conversion_vs2 : OpQueueConversionNone,
            scale_vl   : pe_req.scale_vl,
            cvt_resize : pe_req.cvt_resize,
            // If reductions and vl == 0, we must replace the operands with neutral
            // values in the opqueues. So, vl must be 1 at least
            vl         : (is_reduct && vfu_operation_d.vl == '0)
                        ? 1 : vfu_operation_d.vl,
            vstart     : vfu_operation_d.vstart,
            vtype      : pe_req.vtype,
            hazard     : pe_req.swap_vs2_vd_op ?
            (pe_req.hazard_vs2 | pe_req.hazard_vd) : pe_req.hazard_vd,
            is_reduct  : is_reduct ? 1'b1 : 0,
            target_fu  : MFPU_ADDRGEN,
            default : '0
          };
          operand_request_push[MulFPUC] = pe_req.swap_vs2_vd_op ?
          pe_req.use_vs2 : pe_req.use_vd_op;

          // This vector instruction uses masks
          operand_request[MaskM] = '{
            id     : pe_req.id,
            vs     : VMASK,
            eew    : pe_req.vtype.vsew,
            vtype  : pe_req.vtype,
            // Since this request goes outside of the lane, we might need to request an
            // extra operand regardless of whether it is valid in this lane or not.
		    words_vreg: balanced_words_vm,
            vl     : vl, // do we need it? don't think so (opqueue and opreq do not need it)
            vstart : vfu_operation_d.vstart,
            hazard : pe_req.hazard_vm | pe_req.hazard_vd,
            default: '0
          };
          operand_request_push[MaskM] = !pe_req.vm;
        end
        VFU_LoadUnit : begin
          // This vector instruction uses masks
          operand_request[MaskM] = '{
            id     : pe_req.id,
            vs     : VMASK,
            eew    : pe_req.vtype.vsew,
            vtype  : pe_req.vtype,
            // Since this request goes outside of the lane, we might need to request an
            // extra operand regardless of whether it is valid in this lane or not.
		    words_vreg: balanced_words_vm,
            vl     : vl,
            vstart : vfu_operation_d.vstart,
            hazard : pe_req.hazard_vm | pe_req.hazard_vd,
            default: '0
          };
          operand_request_push[MaskM] = !pe_req.vm;

          // Load indexed
          operand_request[SlideAddrGenA] = '{
            id       : pe_req_i.id,
            vs       : pe_req_i.vs2,
            eew      : pe_req_i.eew_vs2,
            conv     : pe_req_i.conversion_vs2,
            target_fu: MFPU_ADDRGEN,
		    words_vreg: balanced_words_vs2,
            vl       : vl,
            scale_vl : pe_req_i.scale_vl,
            vstart   : vfu_operation_d.vstart,
            vtype    : pe_req_i.vtype,
            hazard   : pe_req_i.hazard_vs2 | pe_req_i.hazard_vd,
            default  : '0
          };
          operand_request_push[SlideAddrGenA] = pe_req_i.op == VLXE;
        end

        VFU_StoreUnit : begin
          // vstart is supported here
          operand_request[StA] = '{
            id      : pe_req.id,
            vs      : pe_req.vs1,
            eew     : pe_req.old_eew_vs1,
            conv    : pe_req.conversion_vs1,
            scale_vl: pe_req.scale_vl,
            vtype   : pe_req.vtype,
		    words_vreg: balanced_words_vs1,
            vl      : vfu_operation_d.vl,
            vstart  : vfu_operation_d.vstart,
            hazard  : pe_req.hazard_vs1 | pe_req.hazard_vd,
            default : '0
          };
          operand_request_push[StA] = pe_req.use_vs1;

          // This vector instruction uses masks
          // TODO: add vstart support here
          operand_request[MaskM] = '{
            id     : pe_req.id,
            vs     : VMASK,
            eew    : pe_req.vtype.vsew,
            vtype  : pe_req.vtype,
            // Since this request goes outside of the lane, we might need to request an
            // extra operand regardless of whether it is valid in this lane or not.
		    words_vreg: balanced_words_vm,
            vl     : vl,
            vstart : vfu_operation_d.vstart,
            hazard : pe_req.hazard_vm | pe_req.hazard_vd,
            default: '0
          };
          operand_request_push[MaskM] = !pe_req.vm;

          // Store indexed
          operand_request[SlideAddrGenA] = '{
            id       : pe_req_i.id,
            vs       : pe_req_i.vs2,
            eew      : pe_req_i.eew_vs2,
            conv     : pe_req_i.conversion_vs2,
            target_fu: MFPU_ADDRGEN,
		    words_vreg: balanced_words_vs2,
            vl       : vl,
            scale_vl : pe_req_i.scale_vl,
            vstart   : vfu_operation_d.vstart,
            vtype    : pe_req_i.vtype,
            hazard   : pe_req_i.hazard_vs2 | pe_req_i.hazard_vd,
            default  : '0
          };
          operand_request_push[SlideAddrGenA] = pe_req_i.op == VSXE;
        end

        VFU_SlideUnit: begin
          operand_request[SlideAddrGenA] = '{
            id       : pe_req.id,
            vs       : pe_req.vs2,
            eew      : pe_req.eew_vs2,
            conv     : pe_req.conversion_vs2,
            target_fu: ALU_SLDU,
            is_slide : 1'b1,
            scale_vl : pe_req.scale_vl,
            vtype    : pe_req.vtype,
            vstart   : vfu_operation_d.vstart,
            hazard   : pe_req.hazard_vs2 | pe_req.hazard_vd,
            default  : '0
          };
          operand_request_push[SlideAddrGenA] = pe_req.use_vs2;

          unique case (pe_req.op)
            VSLIDEUP: begin
              // Slideup fetches the first vl - stride elements at least.
              // If vstart > stride, then the initial vstart - vstride elements of this series are not fetched.
              vslideup_vstart = vstart > stride ? vstart - vstride : '0;
              operand_request[SlideAddrGenA].words_vreg = balanced_words(lane_id_i, end_lane_id, eew_vs2,  vl, vslideup_vstart, stride, NrLanes);
            end
            VSLIDEDOWN: begin
              // Source elements behavior for vslidedown sees "stride" behaving as a vstart
              // [0 <= i+OFFSET < VLMAX]  src[i] = vs2[i+OFFSET]
              // Morever, the last fetched source element is at index (vl + vstride - 1)
              operand_request[SlideAddrGenA].words_vreg = balanced_words(lane_id_i, end_lane_id, eew_vs2,  vl + vstride, stride, 0, NrLanes);
            end
            default:;
          endcase

          // This vector instruction uses masks
          operand_request[MaskM] = '{
            id      : pe_req.id,
            vs      : VMASK,
            eew     : pe_req.vtype.vsew,
            is_slide: 1'b1,
            vtype   : pe_req.vtype,
            vstart  : vfu_operation_d.vstart,
            hazard  : pe_req.hazard_vm | pe_req.hazard_vd,
            default : '0
          };
          operand_request_push[MaskM] = !pe_req.vm;

          unique case (pe_req.op)
            VSLIDEUP: begin
              vslideup_vstart = vstart > stride ? vstart - vstride : '0;
              operand_request[MaskM].words_vreg = balanced_words_vm(lane_id_i,  vl, vslideup_vstart, stride, NrLanes);
            end
            VSLIDEDOWN: begin
              operand_request[MaskM].words_vreg = balanced_words_vm(lane_id_i,  vl + vstride, stride, 0, NrLanes);
            end
            default:;
          endcase
        end
        VFU_MaskUnit: begin
          operand_request[AluA] = '{
            id      : pe_req.id,
            vs      : pe_req.vs1,
            eew     : pe_req.eew_vs1,
            scale_vl: pe_req.scale_vl,
            vtype   : pe_req.vtype,
            vstart  : vfu_operation_d.vstart,
            hazard  : pe_req.hazard_vs1 | pe_req.hazard_vd,
            default : '0
          };

          // This is an operation that runs normally on the ALU, and then gets *condensed* and
          // reshuffled at the Mask Unit.
          if (pe_req.op inside {[VMSEQ:VMSBC]}) begin
            // Comparisons are done in the ALU and then passed to the MASKU
            // The vreg encoding has already to be correct
            operand_request[AluA].words_vreg = balanced_words(lane_id_i, end_lane_id, eew_vs1, vl, vstart, 0, NrLanes);
          end else begin
            // Mask bitwise logical operations are performed in the ALU and then forwarded to the MASKU
            // The vreg encoding is not important here since the operations are bitwise
            operand_request[AluA].words_vreg = balanced_words_vm(lane_id_i,  vl, vstart, 0, NrLanes);
          end
          operand_request_push[AluA] = pe_req.use_vs1 && !(pe_req.op inside {[VMFEQ:VMFGE], VCPOP, VMSIF, VMSOF, VMSBF});

          operand_request[AluB] = '{
            id      : pe_req.id,
            vs      : pe_req.vs2,
            eew     : pe_req.eew_vs2,
            scale_vl: pe_req.scale_vl,
            vtype   : pe_req.vtype,
            vstart  : vfu_operation_d.vstart,
            hazard  : pe_req.hazard_vs2 | pe_req.hazard_vd,
            default : '0
          };
          if (pe_req.op inside {[VMSEQ:VMSBC]}) begin
            // Comparisons are done in the ALU and then passed to the MASKU
            // The vreg encoding has already to be correct
            operand_request[AluB].words_vreg = balanced_words(lane_id_i, end_lane_id, eew_vs1, vl, vstart, 0, NrLanes);
          end else begin
            // Mask bitwise logical operations are performed in the ALU and then forwarded to the MASKU
            // The vreg encoding is not important here since the operations are bitwise
            operand_request[AluB].words_vreg = balanced_words_vm(lane_id_i,  vl, vstart, 0, NrLanes);
          end
          operand_request_push[AluB] = pe_req.use_vs2 && !(pe_req.op inside {[VMFEQ:VMFGE], VCPOP, VMSIF, VMSOF, VMSBF, VFIRST});

          operand_request[MulFPUA] = '{
            id      : pe_req.id,
            vs      : pe_req.vs1,
            eew     : pe_req.eew_vs1,
            scale_vl: pe_req.scale_vl,
            vtype   : pe_req.vtype,
            vstart  : vfu_operation_d.vstart,
            hazard  : pe_req.hazard_vs1 | pe_req.hazard_vd,
            default : '0
          };

          // Comparisons are done in the ALU and then passed to the MASKU
          // The vreg encoding has already to be correct
          operand_request[MulFPUA].words_vreg = balanced_words(lane_id_i, end_lane_id, eew_vs1, vl, vstart, 0, NrLanes);
          operand_request_push[MulFPUA] = pe_req.use_vs1 && pe_req.op inside {[VMFEQ:VMFGE]};

          operand_request[MulFPUB] = '{
            id      : pe_req.id,
            vs      : pe_req.vs2,
            eew     : pe_req.eew_vs2,
            scale_vl: pe_req.scale_vl,
            vtype   : pe_req.vtype,
            vstart  : vfu_operation_d.vstart,
            hazard  : pe_req.hazard_vs2 | pe_req.hazard_vd,
            default : '0
          };
          // Comparisons are done in the ALU and then passed to the MASKU
          // The vreg encoding has already to be correct
          operand_request[MulFPUB].words_vreg = balanced_words(lane_id_i, end_lane_id, eew_vs1, vl, vstart, 0, NrLanes);
          operand_request_push[MulFPUB] = pe_req.use_vs2 && pe_req.op inside {[VMFEQ:VMFGE]};

          operand_request[MaskB] = '{
            id      : pe_req.id,
            vs      : pe_req.vd,
            eew     : pe_req.eew_vd_op,
            scale_vl: pe_req.scale_vl,
            vtype   : pe_req.vtype,
            vstart  : vfu_operation_d.vstart,
            hazard  : pe_req.hazard_vd,
            default : '0
          };
          operand_request[MaskM].words_vreg = balanced_words_vm(lane_id_i,  vl, vstart, 0, NrLanes);
          operand_request_push[MaskB] = pe_req.use_vd_op;

          operand_request[MaskM] = '{
            id     : pe_req.id,
            vs     : VMASK,
            eew    : pe_req.vtype.vsew,
            vtype  : pe_req.vtype,
            vstart : vfu_operation_d.vstart,
            hazard : pe_req.hazard_vm,
            default: '0
          };
          operand_request[MaskM].words_vreg = balanced_words_vm(lane_id_i,  vl, vstart, 0, NrLanes);
          operand_request_push[MaskM] = !pe_req.vm;
        end
        VFU_None: begin
          operand_request[MaskB] = '{
            id         : pe_req.id,
            vs         : pe_req.vs2,
            eew        : pe_req.eew_vs2,
            conv       : pe_req.conversion_vs2,
            scale_vl   : pe_req.scale_vl,
            cvt_resize : pe_req.cvt_resize,
            vtype      : pe_req.vtype,
            vl         : vfu_operation_d.vl,
            vstart     : vfu_operation_d.vstart,
            hazard     : pe_req.hazard_vs2,
            default    : '0
          };
          operand_request_push[MaskB] = 1'b1;
        end
        default:;
      endcase // pe_req.vfu
    end : if_pe_req_valid
  end: sequencer

  always_ff @(posedge clk_i or negedge rst_ni) begin: p_sequencer_ff
    if (!rst_ni) begin
      vinsn_done_q    <= '0;
      vinsn_running_q <= '0;

      vfu_operation_o       <= '0;
      vfu_operation_valid_o <= 1'b0;

      alu_vinsn_done_o  <= 1'b0;
      mfpu_vinsn_done_o <= 1'b0;
    end else begin
      vinsn_done_q    <= vinsn_done_d;
      vinsn_running_q <= vinsn_running_d;

      vfu_operation_o       <= vfu_operation_d;
      vfu_operation_valid_o <= vfu_operation_valid_d;

      alu_vinsn_done_o  <= alu_vinsn_done_d;
      mfpu_vinsn_done_o <= mfpu_vinsn_done_d;
    end
  end

endmodule : lane_sequencer
