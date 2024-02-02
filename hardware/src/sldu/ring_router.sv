// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Navaneeth Kunhi Purayil <nkunhi@student.ethz.ch>
// Description:
// Router to move data between different ARA clusters for
// sliding and reduction operations.
// 
// Data received from Cluster on the left is send to the cluster on the right for bypass mode. 
// Otherwise data is send to the Slide Unit of the Cluster.

module ring_router import ara_pkg::*; #(
	localparam int  unsigned DataWidth = $bits(elen_t),
	localparam type remote_data_t = logic [DataWidth-1:0]
	) (

	input logic clk_i, 
	input logic rst_ni, 
	
	// From SLDU in ARA
	input remote_data_t sldu_i,
	input logic sldu_valid_i,
	output logic sldu_ready_o,  

	output remote_data_t sldu_o,
	output logic sldu_valid_o,
	input logic sldu_ready_i,

	input logic dir,                  // 0-move data left (slidedown) 1- move data right
	input logic bypass,
	input logic conf_valid,

	// From other ring routers
	input remote_data_t ring_right_i,
	input logic         ring_right_valid_i,
	output logic        ring_right_ready_o,
	
	input remote_data_t ring_left_i,
	input logic         ring_left_valid_i,
	output logic        ring_left_ready_o,

	output remote_data_t ring_right_o,
	output logic         ring_right_valid_o,
	input logic          ring_right_ready_i,

	output remote_data_t ring_left_o,
	output logic         ring_left_valid_o,
	input logic          ring_left_ready_i

	);

	remote_data_t ring_right_inp, ring_left_inp; 
	logic ring_right_ready_out, ring_left_ready_out;
	logic ring_right_valid_inp, ring_left_valid_inp;

	remote_data_t ring_right_out, ring_left_out; 
	logic ring_right_ready_inp, ring_left_ready_inp;
	logic ring_right_valid_out, ring_left_valid_out;

	logic dir_d, dir_q;
	logic bypass_d, bypass_q;

	always_ff @(posedge clk_i or negedge rst_ni) begin
		if(~rst_ni) begin
			dir_q <= 1'b0;
			bypass_q <= 1'b0;
		end else begin 
			dir_q <= dir_d;
			bypass_q <= bypass_d;
		end
	end

	spill_register #(
		.T(elen_t)
	) i_ring_right_spill_register (
		.clk_i  (clk_i                        ),
		.rst_ni (rst_ni                       ),
		
		.valid_i(ring_right_valid_i           ),
		.ready_o(ring_right_ready_o           ),
		.data_i (ring_right_i                 ),

		.valid_o(ring_right_valid_inp         ),
		.ready_i(ring_right_ready_out         ),
		.data_o (ring_right_inp               )
	);

	spill_register #(
		.T(elen_t)
	) i_ring_left_spill_register (
		.clk_i  (clk_i                        ),
		.rst_ni (rst_ni                       ),
		
		.valid_i(ring_left_valid_i           ),
		.ready_o(ring_left_ready_o           ),
		.data_i (ring_left_i                 ),
		
		.valid_o(ring_left_valid_inp         ),
		.ready_i(ring_left_ready_out         ),
		.data_o (ring_left_inp               )
	);

	spill_register #(
		.T(elen_t)
	) i_ring_right_spill_register_o (
		.clk_i  (clk_i                        ),
		.rst_ni (rst_ni                       ),
		
		.valid_i(ring_right_valid_out           ),
		.ready_o(ring_right_ready_inp           ),
		.data_i (ring_right_out                 ),
		
		.valid_o(ring_right_valid_o         ),
		.ready_i(ring_right_ready_i         ),
		.data_o (ring_right_o               )
	);

	spill_register #(
		.T(elen_t)
	) i_ring_left_spill_register_o (
		.clk_i  (clk_i                        ),
		.rst_ni (rst_ni                       ),
		
		.valid_i(ring_left_valid_out          ),
		.ready_o(ring_left_ready_inp          ),
		.data_i (ring_left_out                ),
		
		.valid_o(ring_left_valid_o            ),
		.ready_i(ring_left_ready_i            ),
		.data_o (ring_left_o                  )
	);

	always_comb begin
		ring_left_valid_out  = 1'b0; 
		ring_right_valid_out = 1'b0;
		sldu_valid_o       = 1'b0;

		ring_left_ready_out  = 1'b0; 
		ring_right_ready_out = 1'b0; 
		sldu_ready_o       = 1'b0;

		bypass_d = conf_valid ? bypass : bypass_q;
		dir_d    = conf_valid ? dir    : dir_q;

		if (~bypass_d) begin
			if (dir_d==0) begin
				ring_left_out        = sldu_i;
				ring_left_valid_out  = sldu_valid_i;
				sldu_ready_o       = ring_left_ready_inp;
				  
				sldu_o             = ring_right_inp;
				sldu_valid_o       = ring_right_valid_inp;
				ring_right_ready_out = sldu_ready_i;
			end else begin
				ring_right_out        = sldu_i;
				ring_right_valid_out  = sldu_valid_i;
				sldu_ready_o       = ring_right_ready_inp;

				sldu_o             = ring_left_inp;
				sldu_valid_o       = ring_left_valid_inp;
				ring_left_ready_out = sldu_ready_i;
			end
		end else begin
			if (dir_d==0) begin
				ring_left_out        = ring_right_inp;
				ring_left_valid_out  = ring_right_valid_inp;
				ring_right_ready_out = ring_left_ready_inp;
			end else begin
				ring_right_out        = ring_left_inp;
				ring_right_valid_out  = ring_left_valid_inp;
				ring_left_ready_out   = ring_right_ready_inp;
			end
		end
	end

endmodule
