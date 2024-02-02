// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Navaneeth Kunhi Purayil <nkunhi@student.ethz.ch>
// Description:
// This module does the shuffling of data coming from Memory in stages to achieve the required element mapping.
// The shuffling should be such that the first N elements (where N is no of lanes) go to Cluster-0 for all data types, 
// Next N elements go to Cluster-1 and so on.

module shuffle_stage import ara_pkg::*; import rvv_pkg::*;  #(
  parameter  int           unsigned NrLanes             = 0,   // Number of Lanes in each ARA
  parameter  int           unsigned NrClusters          = 0,   // Number of Ara instances
  parameter  int           unsigned ClusterAxiDataWidth        = 0,   // Axi Data width of one cluster
  parameter  int           unsigned AxiAddrWidth        = 0,
  parameter  type                   axi_r_t             = logic,
  parameter  type                   axi_w_t             = logic,
  parameter  type                   axi_req_t           = logic,
  parameter  type                   axi_resp_t          = logic,
  
  parameter  type                   axi_addr_t          = logic [AxiAddrWidth-1:0],
  parameter  type                   axi_data_t          = logic [NrClusters*ClusterAxiDataWidth-1:0],
  localparam int           unsigned NumStages           = $clog2(ClusterAxiDataWidth/(8*NrLanes))
) (
  // Clock and Reset
  input  logic              clk_i,
  input  logic              rst_ni,
  
  input   axi_req_t  [NrClusters-1:0] axi_req_i,
  output  axi_req_t  [NrClusters-1:0] axi_req_o,

  input   axi_resp_t [NrClusters-1:0] axi_resp_i,
  output  axi_resp_t [NrClusters-1:0] axi_resp_o,

  input vew_e vew_ar_i,
  input vew_e vew_aw_i
);

typedef struct packed {
  axi_addr_t addr;
  vlen_t len;
  elen_t stride;
  vew_e vew;
  logic is_load;
  logic is_burst;
  logic [NumStages-1:0] shuffle_en;
  logic valid;
} req_track_t;

localparam int unsigned NumTrackers=8;
typedef logic [$clog2(NumTrackers)-1:0] pnt_t; 
typedef logic [$clog2(NumTrackers):0] cnt_t; 

req_track_t [NumTrackers-1:0] rd_tracker_d, rd_tracker_q;
pnt_t rd_accept_pnt_d, rd_accept_pnt_q;
pnt_t [NumStages-1:0] rd_issue_pnt_d, rd_issue_pnt_q;
cnt_t rd_cnt_d, rd_cnt_q;

req_track_t [NumTrackers-1:0] wr_tracker_d, wr_tracker_q;
pnt_t wr_accept_pnt_d, wr_accept_pnt_q;
pnt_t [NumStages-1:0] wr_issue_pnt_d, wr_issue_pnt_q;
cnt_t wr_cnt_d, wr_cnt_q;

typedef axi_r_t [NrClusters-1:0] stage_r_t;
stage_r_t [NumStages-1:0] r_data_in, r_data_out;

typedef axi_w_t [NrClusters-1:0] stage_w_t; 
stage_w_t [NumStages-1:0] w_data_in, w_data_out;

logic [NumStages-1:0] r_valid, r_ready, w_valid, w_ready;
logic [NumStages-1:0] r_shuffle_en, w_shuffle_en;

logic [NrClusters-1:0] r_ready_i, r_valid_o;
logic [NrClusters-1:0] w_valid_i, w_ready_o;

logic rd_full, wr_full;
assign rd_full = (rd_cnt_q == NumTrackers);
assign wr_full = (wr_cnt_q == NumTrackers);

// To handle cases where vlsu of each cluster is ready to 
// receive read resp or not.
stream_fork #(
  .N_OUP(NrClusters)
) i_cluster_stream_fork (
  .clk_i  (clk_i), 
  .rst_ni (rst_ni),
  .valid_o(r_valid_o            ),
  .valid_i(r_valid[NumStages-1] ), 
  .ready_i(r_ready_i            ),
  .ready_o(r_ready[NumStages-1] )
);

// To handle cases where write data does not come simultaneously 
// from all the clusters
stream_join #(
  .N_INP(NrClusters)
) i_cluster_stream_join (
  .inp_ready_o(w_ready_o),
  .inp_valid_i(w_valid_i),
  .oup_ready_i(w_ready[0]),
  .oup_valid_o(w_valid[0])
);

for (genvar s=0; s<NumStages; s++) begin 

  // Shuffling read data
  shuffle #(
    .NrLanes             (NrLanes),
    .NrClusters          (NrClusters                  ),  
    .ClusterAxiDataWidth (ClusterAxiDataWidth         ),
    .T                   (stage_r_t                   ),
    .scale               (s)
  ) i_shuffle_rd (
    .data_i       ( r_data_in  [s]  ),
    .data_o       ( r_data_out [s]  ),
    .enable_i     ( r_shuffle_en [s]  )
  );

  if (s >= 1) begin
    stream_register #(
      .T       (stage_r_t)
    ) i_align_reg_r  (
      .clk_i      ( clk_i                     ),
      .rst_ni     ( rst_ni                    ),
      .clr_i      ( 1'b0                      ),
      .testmode_i ( 1'b0                      ),
      // Input
      .valid_i    ( r_valid    [s-1]          ),
      .ready_o    ( r_ready    [s-1]          ),
      .data_i     ( r_data_out [s-1]          ),
      // Output
      .valid_o    ( r_valid    [s]            ),
      .ready_i    ( r_ready    [s]            ),
      .data_o     ( r_data_in  [s]            )
    );
  end

  // Shuffling write data
  shuffle #(
    .NrLanes             (NrLanes),
    .NrClusters          (NrClusters                  ),  
    .ClusterAxiDataWidth (ClusterAxiDataWidth         ),
    .T                   (stage_w_t                   ),
    .scale               (s)
  ) i_shuffle_wr (
    .data_i       ( w_data_in  [s]    ),
    .data_o       ( w_data_out [s]    ),
    .enable_i     ( w_shuffle_en [s]  )
  );

  if (s >= 1) begin
    stream_register #(
      .T       (stage_w_t)
    ) i_align_reg_w  (
      .clk_i      ( clk_i                     ),
      .rst_ni     ( rst_ni                    ),
      .clr_i      ( 1'b0                      ),
      .testmode_i ( 1'b0                      ),
      // Input
      .valid_i    ( w_valid    [s-1]          ),
      .ready_o    ( w_ready    [s-1]          ),
      .data_i     ( w_data_out [s-1]          ),
      // Output
      .valid_o    ( w_valid    [s]            ),
      .ready_i    ( w_ready    [s]            ),
      .data_o     ( w_data_in  [s]            )
    );
  end
end

// Handle Response path
for (genvar c=0; c < NrClusters; c++) begin
  // Bypass the registers for signals other than R channel
  assign axi_resp_o[c].aw_ready = axi_resp_i[c].aw_ready && !wr_full;
  assign axi_resp_o[c].ar_ready = axi_resp_i[c].ar_ready && !rd_full; 
  
  assign axi_resp_o[c].b_valid = axi_resp_i[c].b_valid;
  assign axi_resp_o[c].b = axi_resp_i[c].b;

  // Reads  
  assign r_data_in[0][c] = axi_resp_i[c].r;             // Copy input resp to first stage
  assign axi_resp_o[c].r = r_data_out[NumStages-1][c];  // Copy output resp from last stage
  assign axi_resp_o[c].r_valid = r_valid_o[c];          // Copy valid from stream fork to output

  // Writes
  assign axi_resp_o[c].w_ready = w_ready_o[c];          // Copy ready from stream join output to response

end
assign r_valid[0]   = axi_resp_i[0].r_valid;            // From the Global Ld-St for now all clusters receive data together. 
                                                        // Hence using only cluster-0 's indication of r_valid.

// Handle Request path
for (genvar c=0; c < NrClusters; c++) begin
  assign axi_req_o[c].aw = axi_req_i[c].aw;
  assign axi_req_o[c].aw_valid = axi_req_i[c].aw_valid;
  assign axi_req_o[c].ar = axi_req_i[c].ar;
  assign axi_req_o[c].ar_valid = axi_req_i[c].ar_valid;
  assign axi_req_o[c].b_ready = axi_req_i[c].b_ready;
  
  // Reads
  assign axi_req_o[c].r_ready = r_ready[0];             
  assign r_ready_i[c] = axi_req_i[c].r_ready;           // From input request, get ready inputs to stream fork

  // Writes
  assign w_data_in[0][c] = axi_req_i[c].w;              // Copy input write data to first shuffle stage
  assign w_valid_i[c] = axi_req_i[c].w_valid;           // Copy valid signals to stream join
   
  assign axi_req_o[c].w  = w_data_out[NumStages-1][c];  // Copy last stage data to req output
  assign axi_req_o[c].w_valid = w_valid[NumStages-1];   // valid signal is the output valid of stream join

end
assign w_ready[NumStages-1] = axi_resp_i[0].w_ready; // The Global Ld-St is ready to receive write packets together. Hence using only cluster-0 's w_ready.

// Set status of shuffle stage to the current pointers
for (genvar s=0; s<NumStages; s++) begin
  assign r_shuffle_en[s] = rd_tracker_q[rd_issue_pnt_q[s]].shuffle_en[s];
  assign w_shuffle_en[s] = wr_tracker_q[wr_issue_pnt_q[s]].shuffle_en[s];
end

always_ff @(posedge clk_i or negedge rst_ni) begin
  if(~rst_ni) begin
    rd_tracker_q <= '0;
    rd_accept_pnt_q <= '0;
    rd_issue_pnt_q <= '0;
    rd_cnt_q <= '0;

    wr_tracker_q <= '0;
    wr_accept_pnt_q <= '0;
    wr_issue_pnt_q <= '0;
    wr_cnt_q <= '0;
  end else begin
    rd_tracker_q <= rd_tracker_d;
    rd_accept_pnt_q <= rd_accept_pnt_d;
    rd_issue_pnt_q <= rd_issue_pnt_d;
    rd_cnt_q <= rd_cnt_d;

    wr_tracker_q <= wr_tracker_d;
    wr_accept_pnt_q <= wr_accept_pnt_d;
    wr_issue_pnt_q <= wr_issue_pnt_d;
    wr_cnt_q <= wr_cnt_d;

  end
end

always_comb begin

  rd_tracker_d = rd_tracker_q;
  rd_accept_pnt_d = rd_accept_pnt_q;
  rd_issue_pnt_d = rd_issue_pnt_q;
  rd_cnt_d = rd_cnt_q;

  wr_tracker_d = wr_tracker_q;
  wr_accept_pnt_d = wr_accept_pnt_q;
  wr_issue_pnt_d = wr_issue_pnt_q;
  wr_cnt_d = wr_cnt_q;

  // If a request arrives, add to tracker.
  if (axi_req_i[0].ar_valid & axi_resp_o[0].ar_ready) begin
    rd_tracker_d[rd_accept_pnt_q].vew = vew_ar_i;
    rd_tracker_d[rd_accept_pnt_q].len = axi_req_i[0].ar.len;
    rd_accept_pnt_d = (rd_accept_pnt_q == NumTrackers-1) ? '0 : rd_accept_pnt_q + 1;
    rd_cnt_d += 1;

    for (int s=0; s<NumStages; s++) begin
      rd_tracker_d[rd_accept_pnt_q].shuffle_en[s] = (s >= vew_ar_i) ? 1'b1 : 1'b0;
    end
  end

  if (axi_req_i[0].aw_valid & axi_resp_o[0].aw_ready) begin
    wr_tracker_d[wr_accept_pnt_q].vew = vew_aw_i;
    wr_tracker_d[wr_accept_pnt_q].len = axi_req_i[0].aw.len;
    wr_accept_pnt_d = (wr_accept_pnt_q == NumTrackers-1) ? '0 : wr_accept_pnt_q + 1; 
    wr_cnt_d += 1;

    for (int s=0; s<NumStages; s++) begin
      wr_tracker_d[wr_accept_pnt_q].shuffle_en[s] = (s >= vew_aw_i) ? 1'b1 : 1'b0;
    end
  end

  // Update counters
  // Update issue pointer of each stage
  // Once last packet is received by each stage, point to the next tracker.
  for (int s=0; s < NumStages; s++) begin
    
    // Reset shuffle config for the read shuffle stages
    // If the last stage sends the last packet, we need to go to the vew of the next request
    if (r_data_out[s][0].last & r_valid[s] & r_ready[s]) begin
      rd_issue_pnt_d[s] = (rd_issue_pnt_q[s] == NumTrackers-1) ? '0 : rd_issue_pnt_q[s] + 1;
      // In the last stage, reset the shift enable for the tracker instance
      if (s==NumStages-1) begin
        rd_tracker_d[rd_issue_pnt_q[s]].shuffle_en = '0;
        rd_cnt_d -= 1'b1;
      end
    end
    
    // Reset shuffle config for the write shuffle stages
    if (w_data_out[s][0].last & w_valid[s] & w_ready[s]) begin
      wr_issue_pnt_d[s] = (wr_issue_pnt_q[s] == NumTrackers-1) ? '0 : wr_issue_pnt_q[s] + 1;
      // In the last stage, reset the shift enable for the tracker instance
      if (s==NumStages-1) begin
        wr_tracker_d[wr_issue_pnt_q[s]].shuffle_en = '0;
        wr_cnt_d -= 1'b1;
      end
    end
  end
end

endmodule

module shuffle import rvv_pkg::*; #(
  parameter  int           unsigned NrLanes             = 0,
  parameter  int           unsigned NrClusters          = 0,
  parameter  int           unsigned ClusterAxiDataWidth = 0,
  parameter  type                   T                   = logic,
  parameter  int           unsigned scale               = 0, // In bytes
  localparam int           unsigned TotalDataWidth      = ClusterAxiDataWidth * NrClusters,
  localparam int           unsigned TotalLanes          = NrClusters * NrLanes,
  localparam int           unsigned BlockSize           = (8*NrLanes) << scale,            // Sizes to move together (8*N) bits is the base. Doubles every stage.
  localparam int           unsigned Iterations          = TotalDataWidth / BlockSize / 2    // Number of 8N bits in Total Axi Data Width (8 since starting the shuffle stage with 8-bit)          

) (
  input  T  data_i,
  output T  data_o,

  input logic enable_i
);

  logic [TotalDataWidth-1:0] data_in, data_out;

  always_comb begin
    data_o = data_i;

    if (enable_i) begin  
      for (int c=0; c<NrClusters; c++) begin 
        data_in[c*ClusterAxiDataWidth +: ClusterAxiDataWidth] = data_i[c].data;
      end
      
      for (int i=0; i < 2; i++) begin                           // Shuffle between 2 adjacent blocks
        for (int j=0; j < Iterations; j++) begin                // Iterations depend on the stage we are shuffling
          data_out[(j * Iterations + i)*BlockSize +: BlockSize] = data_in[(i * Iterations + j)*BlockSize  +: BlockSize];
        end
      end

      for (int c=0; c<NrClusters; c++) begin 
        data_o[c].data = data_out[c*ClusterAxiDataWidth +: ClusterAxiDataWidth];
      end
    end
  end

  if (ClusterAxiDataWidth > 64*NrLanes)
    $error("Cluster BW should not be large than datapath width");

endmodule
