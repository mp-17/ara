// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Navaneeth Kunhi Purayil <nkunhi@student.ethz.ch>
// Description:
// This module does the alignment of data coming from System in stages (each stage shifting by power of 2 bytes)
// This means the alignment in the Load store unit can be removed.

module align_stage import ara_pkg::*; import rvv_pkg::*;  #(
  parameter  int           unsigned AxiDataWidth        = 0,
  parameter  int           unsigned AxiAddrWidth        = 0,
  parameter  type                   axi_ar_t            = logic,
  parameter  type                   axi_r_t             = logic,
  parameter  type                   axi_aw_t            = logic,
  parameter  type                   axi_w_t             = logic,
  parameter  type                   axi_b_t             = logic,
  parameter  type                   axi_req_t           = logic,
  parameter  type                   axi_resp_t          = logic,
  parameter  type                   axi_addr_t          = logic [AxiAddrWidth-1:0],
  parameter  type                   axi_data_t          = logic [AxiDataWidth-1:0],
  localparam int           unsigned NumStages           = $clog2(AxiDataWidth/8)

) (
  // Clock and Reset
  input  logic              clk_i,
  input  logic              rst_ni,
  
  input  axi_req_t axi_req_i,
  output axi_req_t axi_req_o, 

  input  axi_resp_t axi_resp_i, 
  output axi_resp_t axi_resp_o
);

typedef struct packed {
  axi_addr_t addr;
  vlen_t len;
  elen_t stride;
  vew_e vew;
  logic is_load;
  logic is_burst;
  logic [NumStages-1:0] shift_en;
  logic valid;
} req_track_t;

localparam int unsigned NumTrackers=8;

req_track_t [NumTrackers-1:0] tracker_d, tracker_q;
logic [$clog2(NumTrackers)-1:0] w_pnt_tracker_d, w_pnt_tracker_q;
logic [NumStages-1:0] [$clog2(NumTrackers)-1:0] r_pnt_tracker_d, r_pnt_tracker_q;
logic [$clog2(NumTrackers):0] cnt_tracker_d, cnt_tracker_q;

axi_req_t  [NumStages+1:0] axi_req_cut;
axi_resp_t [NumStages:0] axi_resp_i_cut, axi_resp_o_cut;

axi_data_t data_prev_d, data_prev_q;
logic data_prev_valid_d, data_prev_valid_q;

logic last_q, last_d;


typedef logic [AxiDataWidth/8-1:0] be_data_t;
be_data_t [NumStages:0] be_d, be_q;
be_data_t be_final_d, be_final_q;

always_ff @(posedge clk_i or negedge rst_ni) begin
  if(~rst_ni) begin
    tracker_q         <= '0;
    w_pnt_tracker_q   <= '0;
    r_pnt_tracker_q   <= '0;
    cnt_tracker_q     <= '0;
    data_prev_q       <= '0;
    data_prev_valid_q <= 1'b0;
    last_q            <= 1'b0;
    be_q              <= '0;
    be_final_q        <= '0; 
  end else begin
    tracker_q         <= tracker_d;
    w_pnt_tracker_q   <= w_pnt_tracker_d;
    r_pnt_tracker_q   <= r_pnt_tracker_d;
    cnt_tracker_q     <= cnt_tracker_d;
    data_prev_q       <= data_prev_d;
    data_prev_valid_q <= data_prev_valid_d;
    last_q            <= last_d;
    be_q              <= be_d;
    be_final_q        <= be_final_d;
  end
end

for (genvar s=0; s <= NumStages; s++) begin 
  axi_cut #(
    .ar_chan_t   (axi_ar_t     ),
    .aw_chan_t   (axi_aw_t     ),
    .b_chan_t    (axi_b_t      ),
    .r_chan_t    (axi_r_t      ),
    .w_chan_t    (axi_w_t      ),
    .axi_req_t   (axi_req_t    ),
    .axi_resp_t  (axi_resp_t   )  
  ) i_align_axi_cut (
    .clk_i       (clk_i),
    .rst_ni      (rst_ni),
    .slv_req_i   (axi_req_cut [s]),
    .slv_resp_o  (axi_resp_o_cut[s]),
    .mst_req_o   (axi_req_cut [s+1]),
    .mst_resp_i  (axi_resp_i_cut[s])
  );
  
  if (s < NumStages) begin
    shift #(
        .AxiDataWidth(AxiDataWidth), 
        .axi_data_t(axi_resp_t),
        .ShiftVal (1<<(s))
      ) i_shift (
        .data_i (axi_resp_o_cut[s+1]),
        .data_o (axi_resp_i_cut[s]),
        .sld_valid(tracker_q[r_pnt_tracker_q[s]].shift_en[s])
      );
  end
end

assign axi_req_o  = axi_req_cut[NumStages+1];
assign axi_req_cut[0] = axi_req_i;

logic tracker_full, tracker_empty;
assign tracker_full = (cnt_tracker_q==NumTrackers);
assign tracker_empty = (cnt_tracker_q==0);

always_comb begin

  // Initialize state
  w_pnt_tracker_d = w_pnt_tracker_q;
  cnt_tracker_d = cnt_tracker_q;
  tracker_d = tracker_q;
  r_pnt_tracker_d = r_pnt_tracker_q;
  data_prev_d = data_prev_q;
  data_prev_valid_d = data_prev_valid_q;

  // Set input and output responses
  axi_resp_i_cut[NumStages] = axi_resp_i;
  axi_resp_o = axi_resp_o_cut[0];
  axi_resp_o.ar_ready = !tracker_full & axi_resp_o_cut[0].ar_ready;
 
  // If a request arrives, add to tracker.
  // Assign shift enable for different stages
  if (axi_req_i.ar_valid && !tracker_full) begin
    tracker_d[w_pnt_tracker_q].addr = axi_req_i.ar.addr;
    for (int i=0; i<NumStages; i++) begin 
      if (axi_req_i.ar.addr & (1<<i)) begin 
        tracker_d[w_pnt_tracker_q].shift_en[i] = 1'b1;
      end
    end
    // Incr pointer and counter
    w_pnt_tracker_d = w_pnt_tracker_q + 1;
    if (w_pnt_tracker_q == NumTrackers-1) begin 
      w_pnt_tracker_d = 0;
    end
    cnt_tracker_d = cnt_tracker_q + 1;
  end

  // Update read pointer of each stage
  // Once last packet is received by each stage, point to the next tracker.
  for (int s=0; s < NumStages; s++) begin
    if (axi_resp_o_cut[s+1].r.last && axi_resp_o_cut[s+1].r_valid && axi_req_cut[s+1].r_ready) begin
      r_pnt_tracker_d[s] = r_pnt_tracker_q[s] + 1;
      if (r_pnt_tracker_q[s] == NumTrackers-1) begin
        r_pnt_tracker_d[s] = 0;
      end
      // In the last stage, reset the shift enable for all stages
      if (s==0) begin
        tracker_d[r_pnt_tracker_q[s]].shift_en = '0;
        cnt_tracker_d = cnt_tracker_q - 1;
      end
    end
  end

  // Handling unaligned data using byte enable
  be_d = be_q;
  be_q[NumStages]='1;
  // If a stage receives a valid packet, shift the byte enable
  for (int s=0; s < NumStages; s++) begin
    if (axi_resp_o_cut[s+1].r_valid) begin
      be_d[s] = tracker_q[r_pnt_tracker_q[s]].shift_en[s] ? be_q[s+1] >> (1 << s) : be_q[s+1];
    end
  end
  be_final_d = be_q[0];

  // Track the previous data packet and along with the byte enable
  // combine the current packet and the previous packet.
  last_d = last_q;
  axi_resp_o.r_valid    = 1'b0;
  axi_resp_o.r.last     = 1'b0;
  
  // Combine the previous data and the current data packets
  if (data_prev_valid_q) begin
    for (int b=0; b<AxiDataWidth/8; b++) begin
      axi_resp_o.r.data[b*8 +: 8] = be_final_q[b] ? data_prev_q[b*8 +: 8] : axi_resp_o_cut[0].r.data[b*8 +: 8];
    end
    axi_resp_o.r_valid  = 1'b1;
    if (last_q) begin
      axi_resp_o.r.last = 1'b1;
      data_prev_d       = '0;
      data_prev_valid_d = 1'b0;
      last_d = 1'b0;
    end
  end

  if (axi_resp_o_cut[0].r_valid && axi_req_cut[0].r_ready) begin
    data_prev_d = axi_resp_o_cut[0].r.data;
    data_prev_valid_d     = 1'b1;
    if (axi_resp_o_cut[0].r.last) begin
      if (be_final_q != '1) begin
        // For unaligned data, this is the last packet.
        axi_resp_o.r.last = 1'b1;
        data_prev_d       = '0;
        data_prev_valid_d = 1'b0;
      end else begin
        // Otherwise for aligned data the next cycle is the last packet.
        last_d = 1'b1;
      end
    end
  end

end
endmodule

module shift #(
  parameter  int           unsigned AxiDataWidth        = 0,
  parameter  type                   axi_data_t          = logic,
  parameter  int           unsigned ShiftVal            = 0
) (
  input axi_data_t data_i, 
  output axi_data_t data_o,

  input logic sld_valid
);

  always_comb begin 
    data_o = data_i;
    if (sld_valid)
      data_o.r.data = {data_i.r.data[ShiftVal*8-1:0], data_i.r.data[AxiDataWidth-1:ShiftVal*8]};
  end

endmodule
