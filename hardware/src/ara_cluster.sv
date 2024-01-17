// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Navaneeth Kunhi Purayil <nkunhi@student.ethz.ch>
// Description:
// Ara's System, containing Ara instances and Global Units connecting them.

module ara_cluster import ara_pkg::*; #(
    // RVV Parameters
    parameter  int           unsigned NrLanes      = 0,   // Number of parallel vector lanes per Ara instance
    parameter  int           unsigned NrClusters   = 0,   // Number of Ara instances

    // Support for floating-point data types
    parameter  fpu_support_e          FPUSupport   = FPUSupportHalfSingleDouble,
    // External support for vfrec7, vfrsqrt7
    parameter  fpext_support_e        FPExtSupport = FPExtSupportEnable,
    // Support for fixed-point data types
    parameter  fixpt_support_e        FixPtSupport = FixedPointEnable,
    // AXI Interface
    parameter  int           unsigned AxiDataWidth        = 0,
    parameter  int           unsigned AxiAddrWidth        = 0,
    parameter  int           unsigned ClusterAxiDataWidth = 0,
    parameter  int           unsigned NrAxiCuts           = 2,

    parameter  type                   axi_ar_t     = logic,
    parameter  type                   axi_r_t      = logic,
    parameter  type                   axi_aw_t     = logic,
    parameter  type                   axi_w_t      = logic,
    parameter  type                   axi_b_t      = logic,
    parameter  type                   axi_req_t    = logic,
    parameter  type                   axi_resp_t   = logic,

    parameter  type                   cluster_axi_ar_t     = logic,
    parameter  type                   cluster_axi_r_t      = logic,
    parameter  type                   cluster_axi_aw_t     = logic,
    parameter  type                   cluster_axi_w_t      = logic,
    parameter  type                   cluster_axi_b_t      = logic,
    parameter  type                   cluster_axi_req_t    = logic,
    parameter  type                   cluster_axi_resp_t   = logic,
  
    localparam int  unsigned DataWidth = $bits(elen_t),
    localparam type remote_data_t = logic [DataWidth-1:0],

    // Dependant parameters. DO NOT CHANGE!
    // Ara has NrLanes + 3 processing elements: each one of the lanes, the vector load unit, the
    // vector store unit, the slide unit, and the mask unit.
    localparam int           unsigned NrPEs        = NrLanes + 4
  ) (
    // Clock and Reset
    input  logic              clk_i,
    input  logic              rst_ni,
    // Scan chain
    input  logic              scan_enable_i,
    input  logic              scan_data_i,
    output logic              scan_data_o,
    // Interface with Ariane
    input  accelerator_req_t  acc_req_i,
    output accelerator_resp_t acc_resp_o,
    // AXI interface
    output axi_req_t          axi_req_o,
    input  axi_resp_t         axi_resp_i
  );

  accelerator_resp_t     [NrClusters-1:0] acc_resp;
  cluster_axi_req_t      [NrClusters-1:0] ara_axi_req, ara_axi_req_cut;
  cluster_axi_resp_t     [NrClusters-1:0] ara_axi_resp, ara_axi_resp_cut, ara_axi_resp_delay;

  axi_req_t  axi_req_cut;
  axi_resp_t axi_resp_cut;

  // Ring
  remote_data_t [NrClusters-1:0] ring_data_l, ring_data_r;
  logic         [NrClusters-1:0] ring_data_valid_l, ring_data_valid_r; 
  logic         [NrClusters-1:0] ring_data_ready_l, ring_data_ready_r;

  remote_data_t [NrClusters-1:0] sldu_i, sldu_o; 
  logic         [NrClusters-1:0] sldu_valid_i, sldu_valid_o; 
  logic         [NrClusters-1:0] sldu_ready_i, sldu_ready_o;

  logic         [NrClusters-1:0] sldu_dir, sldu_bypass, sldu_conf_valid;

  for (genvar cluster=0; cluster < NrClusters; cluster++) begin : p_cluster 
    ara #(
      .NrLanes     (NrLanes             ),
      .NrClusters  (NrClusters          ),
      .ClusterId   (cluster             ),
      .FPUSupport  (FPUSupport          ),
      .FPExtSupport(FPExtSupport        ),
      .FixPtSupport(FixPtSupport        ),
      .AxiDataWidth(ClusterAxiDataWidth ),
      .AxiAddrWidth(AxiAddrWidth        ),
      .axi_ar_t    (cluster_axi_ar_t    ),
      .axi_r_t     (cluster_axi_r_t     ),
      .axi_aw_t    (cluster_axi_aw_t    ),
      .axi_w_t     (cluster_axi_w_t     ),
      .axi_b_t     (cluster_axi_b_t     ),
      .axi_req_t   (cluster_axi_req_t   ),
      .axi_resp_t  (cluster_axi_resp_t  )
    ) i_ara (
      .clk_i           (clk_i            ),
      .rst_ni          (rst_ni           ),
      .scan_enable_i   (scan_enable_i    ),
      .scan_data_i     (1'b0             ),
      .scan_data_o     (/* Unused */     ),
      .acc_req_i       (acc_req_i        ),
      .acc_resp_o      (acc_resp[cluster]    ),
      .axi_req_o       (ara_axi_req[cluster] ),
      .axi_resp_i      (ara_axi_resp[cluster]),
      
      // To Ring Routers
      .ring_data_o   (sldu_o      [cluster]), 
      .ring_valid_o  (sldu_valid_o[cluster]),
      .ring_ready_i  (sldu_ready_i[cluster]),

      .ring_data_i   (sldu_i      [cluster]), 
      .ring_valid_i  (sldu_valid_i[cluster]), 
      .ring_ready_o  (sldu_ready_o[cluster]),

      .sldu_dir_o        (sldu_dir[cluster]),
      .sldu_bypass_o     (sldu_bypass[cluster]),
      .sldu_config_valid_o (sldu_conf_valid[cluster])
    );

    ring_router i_ring_router(
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      
      // From SLDU in ARA
      .sldu_i       (sldu_o      [cluster]),
      .sldu_valid_i (sldu_valid_o[cluster]),
      .sldu_ready_o (sldu_ready_i[cluster]),  

      .sldu_o       (sldu_i      [cluster]),
      .sldu_valid_o (sldu_valid_i[cluster]),
      .sldu_ready_i (sldu_ready_o[cluster]),

      // Ring configuration
      .dir        (sldu_dir[cluster]),                  // 0-slidedown(left) 1-slideup(right)
      .bypass     (sldu_bypass[cluster]),
      .conf_valid (sldu_conf_valid[cluster]),

      // From other ring routers
      .ring_right_i       (ring_data_l      [cluster == (NrClusters-1) ? 0 : cluster + 1]),
      .ring_right_valid_i (ring_data_valid_l[cluster == (NrClusters-1) ? 0 : cluster + 1]),
      .ring_right_ready_o (ring_data_ready_l[cluster == (NrClusters-1) ? 0 : cluster + 1]),

      .ring_left_i        (ring_data_r      [cluster == 0 ? (NrClusters-1) : cluster - 1]),
      .ring_left_valid_i  (ring_data_valid_r[cluster == 0 ? (NrClusters-1) : cluster - 1]),
      .ring_left_ready_o  (ring_data_ready_r[cluster == 0 ? (NrClusters-1) : cluster - 1]),

      .ring_right_o       (ring_data_r      [cluster]),
      .ring_right_valid_o (ring_data_valid_r[cluster]),
      .ring_right_ready_i (ring_data_ready_r[cluster]),

      .ring_left_o        (ring_data_l      [cluster]),
      .ring_left_valid_o  (ring_data_valid_l[cluster]),
      .ring_left_ready_i  (ring_data_ready_l[cluster])

    );

    // Axi Cuts to ARA
    axi_cut #(
      .ar_chan_t   (cluster_axi_ar_t     ),
      .aw_chan_t   (cluster_axi_aw_t     ),
      .b_chan_t    (cluster_axi_b_t      ),
      .r_chan_t    (cluster_axi_r_t      ),
      .w_chan_t    (cluster_axi_w_t      ),
      .axi_req_t   (cluster_axi_req_t    ),
      .axi_resp_t  (cluster_axi_resp_t   )
    ) i_global_ldst_ara_axi_cut (
      .clk_i       (clk_i),
      .rst_ni      (rst_ni),
      .slv_req_i   (ara_axi_req[cluster]),
      .slv_resp_o  (ara_axi_resp[cluster]),
      .mst_req_o   (ara_axi_req_cut[cluster]),
      .mst_resp_i  (ara_axi_resp_cut[cluster])
    );

  end

  // Global Ld/St Unit
  global_ldst #(
    .NrLanes            (NrLanes            ),
    .NrClusters         (NrClusters         ),
    .AxiDataWidth       (AxiDataWidth       ),
    .ClusterAxiDataWidth(ClusterAxiDataWidth),
    .AxiAddrWidth       (AxiAddrWidth       ),
    .cluster_axi_req_t  (cluster_axi_req_t  ),
    .cluster_axi_resp_t (cluster_axi_resp_t ),
    .axi_req_t          (axi_req_t          ),
    .axi_resp_t         (axi_resp_t         )
  ) i_global_ldst (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    // To Ara
    .axi_req_i (ara_axi_req_cut  ),
    .axi_resp_o(ara_axi_resp_cut ),
    // To System
    .axi_resp_i(axi_resp_cut),
    .axi_req_o (axi_req_cut)
  );
  
  /*
  // Axi Cuts
  for (genvar i=0; i < NrAxiCuts; i++) begin : p_cuts
    axi_cut #(
    .ar_chan_t   (axi_ar_t     ),
    .aw_chan_t   (axi_aw_t     ),
    .b_chan_t    (axi_b_t      ),
    .r_chan_t    (axi_r_t      ),
    .w_chan_t    (axi_w_t      ),
    .axi_req_t   (axi_req_t    ),
    .axi_resp_t  (axi_resp_t   )
  ) i_global_ldst_system_axi_cut (
    .clk_i       (clk_i),
    .rst_ni      (rst_ni),
    .slv_req_i   (axi_req_cut [i]),
    .slv_resp_o  (axi_resp_cut[i]),
    .mst_req_o   (axi_req_cut [i+1]),
    .mst_resp_i  (axi_resp_cut[i+1])
  );
  end

  assign axi_req_o  = axi_req_cut[NrAxiCuts]; 
  assign axi_resp_cut[NrAxiCuts] = axi_resp_i;*/

  // Align stage

  align_stage #(
      .AxiDataWidth(AxiDataWidth),
      .AxiAddrWidth(AxiAddrWidth),
      .axi_ar_t   (axi_ar_t     ),
      .axi_aw_t   (axi_aw_t     ),
      .axi_b_t    (axi_b_t      ),
      .axi_r_t    (axi_r_t      ),
      .axi_w_t    (axi_w_t      ),
      .axi_req_t(axi_req_t), 
      .axi_resp_t(axi_resp_t)
    ) i_align_stage (
      .clk_i (clk_i), 
      .rst_ni(rst_ni), 

      .axi_req_i(axi_req_cut),
      .axi_resp_o(axi_resp_cut),

      .axi_req_o(axi_req_o),
      .axi_resp_i(axi_resp_i)
  );

  assign acc_resp_o = acc_resp[0];

endmodule : ara_cluster
