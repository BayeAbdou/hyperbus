// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Luca Valente <luca.valente@unibo.it>

module hyperbus_phy_if import hyperbus_pkg::*; #(
    parameter int unsigned IsClockODelayed = 1,
    parameter int unsigned NumChips = 2,
    parameter int unsigned NumPhys = 2,
    parameter int unsigned TimerWidth = 16,
    parameter int unsigned RxFifoLogDepth = 3,
    parameter int unsigned StartupCycles = 60000, /*MHz*/ // Conservative maximum frequency estimate
    parameter int unsigned  SyncStages  = 2,
    parameter type hyper_tx_t = logic,
    parameter type hyper_rx_t = logic
)(
    input  logic                clk_i,
    input  logic                clk_i_90,
    input  logic                rst_ni,
    input  logic                test_mode_i,
    // Config registers
    input  hyper_cfg_t          cfg_i,
    // Transactions
    input  logic                trans_valid_i,
    output logic                trans_ready_o,
    input  hyper_tf_t           trans_i,            // TODO: increase burst width!
    input  logic [NumChips-1:0] trans_cs_i,
    // Transmitting channel
    input  logic                tx_valid_i,
    output logic                tx_ready_o,
    input  hyper_tx_t           tx_i,
    // Receiving channel
    output logic                rx_valid_o,
    input  logic                rx_ready_i,
    output hyper_rx_t           rx_o,
    // B response
    output logic                b_valid_o,
    input  logic                b_ready_i,
    output logic                b_error_o,

    // Physical interace: facing HyperBus
    output logic [NumPhys-1:0][NumChips-1:0] hyper_cs_no,
    output logic [NumPhys-1:0]               hyper_ck_o,
    output logic [NumPhys-1:0]               hyper_ck_no,
    output logic [NumPhys-1:0]               hyper_rwds_o,
    input  logic [NumPhys-1:0]               hyper_rwds_i,
    output logic [NumPhys-1:0]               hyper_rwds_oe_o,
    input  logic [NumPhys-1:0][7:0]          hyper_dq_i,
    output logic [NumPhys-1:0][7:0]          hyper_dq_o,
    output logic [NumPhys-1:0]               hyper_dq_oe_o,
    output logic [NumPhys-1:0]               hyper_reset_no
);

      phy_rx_t [NumPhys-1:0]       phy_fifo_rx;
      phy_rx_t [NumPhys-1:0]       fifo_axi_rx;
      logic [NumPhys-1:0]          phy_fifo_valid;
      logic [NumPhys-1:0]          phy_fifo_ready;
      logic [NumPhys-1:0]          fifo_axi_valid;
      logic                        fifo_axi_ready;

      logic [NumPhys-1:0][1:0]     fifo_axi_usage;

      logic                        rx_both_valid;

      genvar                          i;
      generate

         if (NumPhys==2) begin : phy_wrap

                  // Both PHYs are always simultaneously active in the dual configuration.
            // A single shared FSM (hyperbus_phy_dual) drives both transceivers
            // to prevent FSM desynchronization.

            logic [1:0][15:0] phy_dual_rx_data;
            logic [1:0]       phy_dual_rx_error;
            logic [1:0]       phy_dual_rx_last;
            logic             phy_dual_tx_ready;
            logic             phy_dual_trans_ready;
            logic             phy_dual_b_valid;
            logic             phy_dual_b_error;

            // Both FIFOs must be valid before presenting data to upstream
            assign rx_both_valid  = & fifo_axi_valid;
            assign rx_valid_o     = rx_both_valid;
            assign fifo_axi_ready = rx_ready_i && rx_both_valid;

            assign rx_o.error    = fifo_axi_rx[0].error | fifo_axi_rx[1].error;
            assign rx_o.last     = fifo_axi_rx[0].last & fifo_axi_rx[1].last;

            assign tx_ready_o    = phy_dual_tx_ready;
            assign trans_ready_o = phy_dual_trans_ready;
            assign b_valid_o     = phy_dual_b_valid;
            assign b_error_o     = phy_dual_b_error;

            for ( i=0; i<NumPhys;i++) begin : phy_unroll
               assign rx_o.data[i*16 +:16] = fifo_axi_rx[i].data;
               assign phy_fifo_rx[i] = '{data:  phy_dual_rx_data[i],
                                         error: phy_dual_rx_error[i],
                                         last:  phy_dual_rx_last[i]};

               stream_fifo #(
                   .FALL_THROUGH ( 1'b0        ),
                   .DEPTH        ( 4           ),
                   .T            ( phy_rx_t    )
               ) rx_fifo (
                   .clk_i          ( clk_i             ),
                   .rst_ni         ( rst_ni            ),
                   .flush_i        ( 1'b0              ),
                   .testmode_i     ( 1'b0              ),
                   .usage_o        ( fifo_axi_usage[i] ),
                   .data_i         ( phy_fifo_rx[i]    ),
                   .valid_i        ( phy_fifo_valid[i] ),
                   .ready_o        ( phy_fifo_ready[i] ),
                   .data_o         ( fifo_axi_rx[i]    ),
                   .valid_o        ( fifo_axi_valid[i] ),
                   .ready_i        ( fifo_axi_ready    )
               );
            end // for ( i=0; i<NumPhys;i++)

            hyperbus_phy_dual #(
                .IsClockODelayed ( IsClockODelayed ),
                .NumChips        ( NumChips        ),
                .StartupCycles   ( StartupCycles   ),
                .SyncStages      ( SyncStages      )
            ) i_phy_dual (
                .clk_i,
                .clk_i_90,
                .rst_ni,
                .test_mode_i,

                .cfg_i,

                .trans_valid_i  ( trans_valid_i         ),
                .trans_ready_o  ( phy_dual_trans_ready  ),
                .trans_i        ( trans_i               ),
                .trans_cs_i     ( trans_cs_i            ),

                .tx_valid_i     ( tx_valid_i            ),
                .tx_ready_o     ( phy_dual_tx_ready     ),
                .tx_data_i      ( tx_i.data             ),
                .tx_strb_i      ( tx_i.strb             ),
                .tx_last_i      ( tx_i.last             ),

                .rx_valid_o     ( phy_fifo_valid        ),
                .rx_ready_i     ( phy_fifo_ready        ),
                .rx_data_o      ( phy_dual_rx_data      ),
                .rx_error_o     ( phy_dual_rx_error     ),
                .rx_last_o      ( phy_dual_rx_last      ),

                .b_valid_o      ( phy_dual_b_valid      ),
                .b_ready_i      ( b_ready_i             ),
                .b_error_o      ( phy_dual_b_error      ),

                .hyper_cs_no,
                .hyper_ck_o,
                .hyper_ck_no,
                .hyper_rwds_o,
                .hyper_rwds_i,
                .hyper_rwds_oe_o,
                .hyper_dq_i,
                .hyper_dq_o,
                .hyper_dq_oe_o,
                .hyper_reset_no
            );
         end else begin // if (NumPhys==2)

            hyperbus_phy #(
                 .IsClockODelayed( IsClockODelayed   ),
                 .NumChips       ( NumChips          ),
                 .StartupCycles  ( StartupCycles     ),
                 .NumPhys        ( NumPhys           ),
                 .SyncStages     ( SyncStages        )
             ) i_phy (
                 .clk_i          ( clk_i           ),
                 .clk_i_90       ( clk_i_90        ),
                 .rst_ni         ( rst_ni          ),
                 .test_mode_i    ( test_mode_i     ),

                 .cfg_i          ( cfg_i           ),

                 .busy_o         (                 ),

                 .rx_data_o      ( rx_o.data       ),
                 .rx_last_o      ( rx_o.last       ),
                 .rx_error_o     ( rx_o.error      ),
                 .rx_valid_o     ( rx_valid_o      ),
                 .rx_ready_i     ( rx_ready_i      ),

                 .tx_data_i      ( tx_i.data       ),
                 .tx_strb_i      ( tx_i.strb       ),
                 .tx_last_i      ( tx_i.last       ),
                 .tx_valid_i     ( tx_valid_i      ),
                 .tx_ready_o     ( tx_ready_o      ),

                 .b_error_o      ( b_error_o       ),
                 .b_valid_o      ( b_valid_o       ),
                 .b_ready_i      ( b_ready_i       ),

                 .trans_i        ( trans_i         ),
                 .trans_cs_i     ( trans_cs_i      ),
                 .trans_valid_i  ( trans_valid_i   ),
                 .trans_ready_o  ( trans_ready_o   ),

                 .hyper_cs_no    ( hyper_cs_no     ),
                 .hyper_ck_o     ( hyper_ck_o      ),
                 .hyper_ck_no    ( hyper_ck_no     ),
                 .hyper_rwds_o   ( hyper_rwds_o    ),
                 .hyper_rwds_i   ( hyper_rwds_i    ),
                 .hyper_rwds_oe_o( hyper_rwds_oe_o ),
                 .hyper_dq_i     ( hyper_dq_i      ),
                 .hyper_dq_o     ( hyper_dq_o      ),
                 .hyper_dq_oe_o  ( hyper_dq_oe_o   ),
                 .hyper_reset_no ( hyper_reset_no  )
            );
         end // else: !if(NumPhys==2)
     endgenerate


endmodule // hyperbus_wrap_0
