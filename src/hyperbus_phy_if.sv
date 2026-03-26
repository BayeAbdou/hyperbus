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

      generate

         if (NumPhys==2) begin : phy_wrap

            hyperbus_phy_dual #(
                .IsClockODelayed ( IsClockODelayed ),
                .NumChips        ( NumChips        ),
                .NumPhys         ( NumPhys         ),
                .StartupCycles   ( StartupCycles   ),
                .SyncStages      ( SyncStages      ),
                .hyper_tx_t      ( hyper_tx_t      ),
                .hyper_rx_t      ( hyper_rx_t      )
            ) i_phy_dual (
                .clk_i           ( clk_i          ),
                .clk_i_90        ( clk_i_90       ),
                .rst_ni          ( rst_ni         ),
                .test_mode_i     ( test_mode_i    ),
                .cfg_i           ( cfg_i          ),
                .trans_valid_i   ( trans_valid_i  ),
                .trans_ready_o   ( trans_ready_o  ),
                .trans_i         ( trans_i        ),
                .trans_cs_i      ( trans_cs_i     ),
                .tx_valid_i      ( tx_valid_i     ),
                .tx_ready_o      ( tx_ready_o     ),
                .tx_i            ( tx_i           ),
                .rx_valid_o      ( rx_valid_o     ),
                .rx_ready_i      ( rx_ready_i     ),
                .rx_o            ( rx_o           ),
                .b_valid_o       ( b_valid_o      ),
                .b_ready_i       ( b_ready_i      ),
                .b_error_o       ( b_error_o      ),
                .hyper_cs_no     ( hyper_cs_no    ),
                .hyper_ck_o      ( hyper_ck_o     ),
                .hyper_ck_no     ( hyper_ck_no    ),
                .hyper_rwds_o    ( hyper_rwds_o   ),
                .hyper_rwds_i    ( hyper_rwds_i   ),
                .hyper_rwds_oe_o ( hyper_rwds_oe_o),
                .hyper_dq_i      ( hyper_dq_i     ),
                .hyper_dq_o      ( hyper_dq_o     ),
                .hyper_dq_oe_o   ( hyper_dq_oe_o  ),
                .hyper_reset_no  ( hyper_reset_no )
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
