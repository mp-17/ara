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
    parameter  int           unsigned NrClusters     = 0,   // Number of Ara instances

    // Support for floating-point data types
    parameter  fpu_support_e          FPUSupport   = FPUSupportHalfSingleDouble,
    // External support for vfrec7, vfrsqrt7
    parameter  fpext_support_e        FPExtSupport = FPExtSupportEnable,
    // Support for fixed-point data types
    parameter  fixpt_support_e        FixPtSupport = FixedPointEnable,
    // AXI Interface
    parameter  int           unsigned AxiDataWidth = 0,
    parameter  int           unsigned AxiAddrWidth = 0,
    parameter  int           unsigned ClusterAxiDataWidth = 0,

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

  for (genvar cluster=0; cluster < NrClusters; cluster++) begin : p_cluster 
    ara #(
      .NrLanes     (NrLanes             ),
      .NrClusters  (NrClusters          ),
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
      .axi_resp_i      (ara_axi_resp[cluster])
	);
  
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
    .axi_resp_i(axi_resp_i),
    .axi_req_o (axi_req_o )
  );

  //   axi_cut #(
  //   .ar_chan_t   (axi_ar_t     ),
  //   .aw_chan_t   (axi_aw_t     ),
  //   .b_chan_t    (axi_b_t      ),
  //   .r_chan_t    (axi_r_t      ),
  //   .w_chan_t    (axi_w_t      ),
  //   .axi_req_t   (axi_req_t    ),
  //   .axi_resp_t  (axi_resp_t   )
  // ) i_global_ldst_system_axi_cut (
  //   .clk_i       (clk_i),
  //   .rst_ni      (rst_ni),
  //   .slv_req_i   (ara_axi_req[cluster]),
  //   .slv_resp_o  (ara_axi_resp[cluster]),
  //   .mst_req_o   (ara_axi_req_cut[cluster]),
  //   .mst_resp_i  (ara_axi_resp_cut[cluster])
  // );

  assign acc_resp_o = acc_resp[0];

  // assign ara_axi_resp_delay = ara_axi_resp;
  // assign #(RespDelay*1ns) ara_axi_resp_delay = ara_axi_resp;

endmodule : ara_cluster
