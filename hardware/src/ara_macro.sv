// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Navaneeth Kunhi Purayil <nkunhi@student.ethz.ch>
// Description:
// A Repeatable ARA macro, containing ARA and ring router

module ara_macro import ara_pkg::*; #(
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

    // Id
    input  logic     [cf_math_pkg::idx_width(NrClusters)-1:0] cluster_id_i,

    // Interface with Ariane
    input  accelerator_req_t  acc_req_i,
    output accelerator_resp_t acc_resp_o,

    // AXI interface
    output cluster_axi_req_t          axi_req_o,
    input  cluster_axi_resp_t         axi_resp_i,

    // Ring
    input remote_data_t  ring_data_r_i,
    input logic          ring_data_r_valid_i,
    output logic         ring_data_r_ready_o, 

    input remote_data_t  ring_data_l_i,
    input logic          ring_data_l_valid_i,
    output logic         ring_data_l_ready_o, 

    output remote_data_t ring_data_r_o,
    output logic         ring_data_r_valid_o,
    input logic          ring_data_r_ready_i, 

    output remote_data_t ring_data_l_o,
    output logic         ring_data_l_valid_o,
    input logic          ring_data_l_ready_i

  );

  // To System AXI
  cluster_axi_req_t     ara_axi_req;
  cluster_axi_resp_t    ara_axi_resp;

  // Sldu to Ariane
  remote_data_t sldu_i, sldu_o; 
  logic         sldu_valid_i, sldu_valid_o; 
  logic         sldu_ready_i, sldu_ready_o;

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
    
    .cluster_id_i    (cluster_id_i     ),
    .acc_req_i       (acc_req_i        ),
    .acc_resp_o      (acc_resp_o       ),
    .axi_req_o       (ara_axi_req      ),
    .axi_resp_i      (ara_axi_resp     ),
    
    // To Ring Routers
    .ring_data_o         (sldu_o             ), 
    .ring_valid_o        (sldu_valid_o       ),
    .ring_ready_i        (sldu_ready_i       ),

    .ring_data_i         (sldu_i             ), 
    .ring_valid_i        (sldu_valid_i       ), 
    .ring_ready_o        (sldu_ready_o       ),

    .sldu_dir_o          (sldu_dir           ),
    .sldu_bypass_o       (sldu_bypass        ),
    .sldu_config_valid_o (sldu_conf_valid    )

  );

  ring_router i_ring_router (
    .clk_i             (clk_i),
    .rst_ni            (rst_ni),
    
    // From SLDU in ARA
    .sldu_i       (sldu_o        ),
    .sldu_valid_i (sldu_valid_o  ),
    .sldu_ready_o (sldu_ready_i  ),  

    .sldu_o       (sldu_i        ),
    .sldu_valid_o (sldu_valid_i  ),
    .sldu_ready_i (sldu_ready_o  ),

    // Ring configuration
    .dir        (sldu_dir        ),                  // 0-slidedown(left) 1-slideup(right)
    .bypass     (sldu_bypass     ),
    .conf_valid (sldu_conf_valid ),

    // From other ring routers
    .ring_right_i       (ring_data_r_i      ),
    .ring_right_valid_i (ring_data_r_valid_i),
    .ring_right_ready_o (ring_data_r_ready_o),

    .ring_left_i        (ring_data_l_i      ),
    .ring_left_valid_i  (ring_data_l_valid_i),
    .ring_left_ready_o  (ring_data_l_ready_o),

    .ring_right_o       (ring_data_r_o      ),
    .ring_right_valid_o (ring_data_r_valid_o),
    .ring_right_ready_i (ring_data_r_ready_i),

    .ring_left_o        (ring_data_l_o      ),
    .ring_left_valid_o  (ring_data_l_valid_o),
    .ring_left_ready_i  (ring_data_l_ready_i)
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
    .slv_req_i   (ara_axi_req),
    .slv_resp_o  (ara_axi_resp),
    .mst_req_o   (axi_req_o),
    .mst_resp_i  (axi_resp_i)
  );

endmodule