// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Navaneeth Kunhi Purayil <nkunhi@student.ethz.ch>
// Description:
// Global load store unit that receives LD-ST AXI request from Ara instances
// and generates an AXI request to the System XBAR.

module global_ldst #(
  parameter  int           unsigned NrLanes = 0,
  parameter  int           unsigned NrGroups = 0,
  parameter  int           unsigned AxiDataWidth = 0,
  parameter  int           unsigned GrpAxiDataWidth = 0,
  parameter type grp_axi_req_t = logic,
  parameter type grp_axi_resp_t = logic,
  parameter type axi_req_t = logic,
  parameter type axi_resp_t = logic,
  localparam int size_axi = $clog2(AxiDataWidth)-3
  ) (
  // To ARA
  input  grp_axi_req_t   [NrGroups-1:0] axi_req_i,
  output grp_axi_resp_t  [NrGroups-1:0] axi_resp_o,
  
  // To System AXI 
  input  axi_resp_t                     axi_resp_i,
  output axi_req_t                      axi_req_o
);

always_comb begin : p_global_ldst

  // Copy data between ARA<->System
  // Combine Request from Lane Groups
  // Here using Group0 as the request and ignoring the other requests.
  // aw
  axi_req_o.aw = axi_req_i[0].aw;
  axi_req_o.aw.len = (((axi_req_i[0].aw.len + 1) << axi_req_i[0].aw.size) << $clog2(NrGroups) >> size_axi) - 1;
  axi_req_o.aw.size = size_axi;
  axi_req_o.aw_valid = axi_req_i[0].aw_valid;
  // ar
  axi_req_o.ar = axi_req_i[0].ar;
  axi_req_o.ar.len = (((axi_req_i[0].ar.len + 1) << axi_req_i[0].ar.size) << $clog2(NrGroups) >> size_axi) - 1;
  axi_req_o.ar.size = size_axi;
  axi_req_o.ar_valid = axi_req_i[0].ar_valid;
  // w
  for (int i=0; i<NrGroups; i++) begin : write_copy
    axi_req_o.w.data[i*GrpAxiDataWidth +: GrpAxiDataWidth] = axi_req_i[i].w.data;
    axi_req_o.w.strb[i*GrpAxiDataWidth/8 +: GrpAxiDataWidth/8] = axi_req_i[i].w.strb;
  end
  
  axi_req_o.w.last = axi_req_i[0].w.last;
  axi_req_o.w.user = axi_req_i[0].w.user;
  axi_req_o.w_valid = axi_req_i[0].w_valid;
  axi_req_o.b_ready = axi_req_i[0].b_ready;                                            
  axi_req_o.r_ready = axi_req_i[0].r_ready;

  // Distribute response to Lane Groups
  for (int i=0; i<NrGroups; i++) begin : read_copy
    axi_resp_o[i].aw_ready = axi_resp_i.aw_ready;
    axi_resp_o[i].ar_ready = axi_resp_i.ar_ready;
    axi_resp_o[i].w_ready = axi_resp_i.w_ready;
    axi_resp_o[i].b_valid = axi_resp_i.b_valid;
    axi_resp_o[i].b = axi_resp_i.b;
    axi_resp_o[i].r_valid = axi_resp_i.r_valid;
    // r
    axi_resp_o[i].r.id   = axi_resp_i.r.id;
    axi_resp_o[i].r.resp = axi_resp_i.r.resp;
    axi_resp_o[i].r.last = axi_resp_i.r.last;
    axi_resp_o[i].r.user = axi_resp_i.r.user;
    axi_resp_o[i].r.data = axi_resp_i.r.data[i*GrpAxiDataWidth +: GrpAxiDataWidth];
  end
end : p_global_ldst

endmodule : global_ldst