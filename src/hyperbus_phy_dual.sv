// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// hyperbus_phy_dual: Single-FSM dual-PHY HyperBus controller.
//
// Replaces two independent hyperbus_phy instances in the NumPhys=2 path to
// prevent FSM desynchronization. A single shared FSM drives both HyperBus
// transceivers simultaneously so that state transitions and timing are
// identical for both PHYs at all times.

module hyperbus_phy_dual import hyperbus_pkg::*; #(
    parameter int unsigned IsClockODelayed = -1,
    parameter int unsigned NumChips        = 2,
    parameter int unsigned TimerWidth      = 16,
    parameter int unsigned RxFifoLogDepth  = 3,
    parameter int unsigned SyncStages      = 2,
    parameter int unsigned StartupCycles   = 300 /*us*/ * 200 /*MHz*/
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
    input  hyper_tf_t           trans_i,
    input  logic [NumChips-1:0] trans_cs_i,
    // TX channel: [15:0] = PHY0, [31:16] = PHY1
    input  logic                tx_valid_i,
    output logic                tx_ready_o,
    input  logic [31:0]         tx_data_i,
    input  logic [3:0]          tx_strb_i,
    input  logic                tx_last_i,
    // RX channel, one stream per PHY (feeds external FIFOs)
    output logic [1:0]          rx_valid_o,
    input  logic [1:0]          rx_ready_i,
    output logic [1:0][15:0]    rx_data_o,
    output logic [1:0]          rx_error_o,
    output logic [1:0]          rx_last_o,
    // B response (single, for both PHYs)
    output logic                b_valid_o,
    input  logic                b_ready_i,
    output logic                b_error_o,
    // Physical interfaces, indexed by PHY [1:0]
    output logic [1:0][NumChips-1:0] hyper_cs_no,
    output logic [1:0]               hyper_ck_o,
    output logic [1:0]               hyper_ck_no,
    output logic [1:0]               hyper_rwds_o,
    input  logic [1:0]               hyper_rwds_i,
    output logic [1:0]               hyper_rwds_oe_o,
    input  logic [1:0][7:0]          hyper_dq_i,
    output logic [1:0][7:0]          hyper_dq_o,
    output logic [1:0]               hyper_dq_oe_o,
    output logic [1:0]               hyper_reset_no
);

    // Both PHYs are always simultaneously active
    localparam int unsigned NumPhys = 2;

    // ========================
    //   Single shared FSM state
    // ========================
    hyper_phy_state_t       state_d,    state_q;
    logic [TimerWidth-1:0]  timer_d,    timer_q;
    hyper_tf_t              tf_d,       tf_q;
    logic [NumChips-1:0]    cs_d,       cs_q;

    // B response tracking
    logic b_pending_q;
    logic b_pending_set;
    logic b_pending_clear;

    // Outstanding read words counter (tracks word-pairs: one per PHY per cycle)
    logic [RxFifoLogDepth:0]    r_outstand_q;
    logic                       r_outstand_inc;
    logic                       r_outstand_dec;

    // Auxiliary control signals
    logic ctl_write_zero_lat;
    logic ctl_add_latency;
    logic ctl_tf_burst_last;
    logic ctl_tf_burst_done;
    logic ctl_timer_two;
    logic ctl_timer_one;
    logic ctl_timer_zero;
    logic ctl_timer_rwr_done;
    logic ctl_rclk_ena;
    logic ctl_wclk_ena;

    // Command-address (same word sent to both PHYs)
    hyper_phy_ca_t ca;

    // TRX control signals (shared: both PHYs receive identical signals)
    logic             trx_clk_ena;
    logic             trx_cs_ena;
    logic [1:0]       trx_rwds_sample;
    logic             trx_rwds_sample_ena;
    logic [1:0][15:0] trx_tx_data;
    logic             trx_tx_data_oe;
    logic [1:0][1:0]  trx_tx_rwds;
    logic             trx_tx_rwds_oe;
    logic             trx_rx_clk_set;
    logic             trx_rx_clk_reset;
    logic [1:0][15:0] trx_rx_data;
    logic [1:0]       trx_rx_valid;
    logic [1:0]       trx_rx_ready;

    // ========================
    //   Two Transceiver instances (shared control, per-PHY data)
    // ========================
    for (genvar p = 0; p < NumPhys; p++) begin : gen_trx
        hyperbus_trx #(
            .IsClockODelayed ( IsClockODelayed ),
            .NumChips        ( NumChips        ),
            .RxFifoLogDepth  ( RxFifoLogDepth  ),
            .SyncStages      ( SyncStages      )
        ) i_trx (
            .clk_i,
            .clk_i_90,
            .rst_ni,
            .test_mode_i,
            // Shared control signals
            .cs_i               ( cs_q                  ),
            .cs_ena_i           ( trx_cs_ena            ),
            .rwds_sample_o      ( trx_rwds_sample[p]    ),
            .rwds_sample_ena_i  ( trx_rwds_sample_ena   ),
            .tx_clk_delay_i     ( cfg_i.t_tx_clk_delay  ),
            .tx_clk_ena_i       ( trx_clk_ena           ),
            .tx_data_oe_i       ( trx_tx_data_oe        ),
            .tx_rwds_oe_i       ( trx_tx_rwds_oe        ),
            .rx_clk_delay_i     ( cfg_i.t_rx_clk_delay  ),
            .rx_clk_set_i       ( trx_rx_clk_set        ),
            .rx_clk_reset_i     ( trx_rx_clk_reset      ),
            // Per-PHY data signals
            .tx_data_i          ( trx_tx_data[p]        ),
            .tx_rwds_i          ( trx_tx_rwds[p]        ),
            .rx_data_o          ( trx_rx_data[p]        ),
            .rx_valid_o         ( trx_rx_valid[p]       ),
            .rx_ready_i         ( trx_rx_ready[p]       ),
            // Physical interface
            .hyper_cs_no        ( hyper_cs_no[p]        ),
            .hyper_ck_o         ( hyper_ck_o[p]         ),
            .hyper_ck_no        ( hyper_ck_no[p]        ),
            .hyper_rwds_o       ( hyper_rwds_o[p]       ),
            .hyper_rwds_i       ( hyper_rwds_i[p]       ),
            .hyper_rwds_oe_o    ( hyper_rwds_oe_o[p]    ),
            .hyper_dq_i         ( hyper_dq_i[p]         ),
            .hyper_dq_o         ( hyper_dq_o[p]         ),
            .hyper_dq_oe_o      ( hyper_dq_oe_o[p]      ),
            .hyper_reset_no     ( hyper_reset_no[p]     )
        );
    end

    // ========================
    //   Dataflow
    // ========================

    // Command-address: same 48-bit CA word sent to both PHYs
    assign ca = hyper_phy_ca_t '{
        write:      ~tf_q.write,
        addr_space: tf_q.address_space,
        burst_type: tf_q.burst_type,
        addr_upper: tf_q.address[31:3],
        reserved:   '0,
        addr_lower: tf_q.address[2:0]
    };

    // Write dataflow: PHY0 gets tx_data_i[15:0], PHY1 gets tx_data_i[31:16]
    always_comb begin : proc_comb_tx
        trx_tx_data[0] = '0;
        trx_tx_data[1] = '0;
        trx_tx_rwds[0] = '0;
        trx_tx_rwds[1] = '0;
        tx_ready_o     = 1'b0;
        ctl_wclk_ena   = 1'b0;
        if (state_q == SendCA) begin
            // Both PHYs transmit the same CA word (CA is identical for both chips)
            trx_tx_data[0] = ca[(8'(timer_q) << 4) +: 16];
            trx_tx_data[1] = ca[(8'(timer_q) << 4) +: 16];
        end else if (state_q == Write) begin
            trx_tx_data[0] = tx_data_i[15:0];   // PHY0 lower half
            trx_tx_data[1] = tx_data_i[31:16];  // PHY1 upper half
            trx_tx_rwds[0] = ~tx_strb_i[1:0];
            trx_tx_rwds[1] = ~tx_strb_i[3:2];
            tx_ready_o     = 1'b1;  // Memory always ready within HyperBus burst
            ctl_wclk_ena   = tx_valid_i;
        end
    end

    // Write response
    assign b_valid_o       = b_pending_q;
    assign b_error_o       = 1'b0;
    assign b_pending_clear = b_valid_o & b_ready_i;

    always_ff @(posedge clk_i or negedge rst_ni) begin : proc_ff_b_pending
        if      (~rst_ni)           b_pending_q <= 1'b0;
        else if (b_pending_set)     b_pending_q <= 1'b1;
        else if (b_pending_clear)   b_pending_q <= 1'b0;
    end

    // Read response dataflow (per PHY, synchronized outputs)
    assign rx_data_o[0]  = trx_rx_data[0];
    assign rx_data_o[1]  = trx_rx_data[1];
    assign rx_error_o    = '0;

    // last: asserted for the final outstanding word-pair when the burst is done.
    // Both PHYs fire simultaneously so last is identical for both.
    assign rx_last_o[0]  = (state_q != Read) & ctl_tf_burst_done & (r_outstand_q == 1);
    assign rx_last_o[1]  = rx_last_o[0];

    // Connect TRX ready inputs to downstream FIFO readies
    assign trx_rx_ready[0] = rx_ready_i[0];
    assign trx_rx_ready[1] = rx_ready_i[1];

    // Present valid to downstream only while there are outstanding words
    assign rx_valid_o[0]   = trx_rx_valid[0] & (r_outstand_q != '0);
    assign rx_valid_o[1]   = trx_rx_valid[1] & (r_outstand_q != '0);

    // RX clock backpressure: suspend HyperBus clock if either downstream FIFO stalls.
    // This prevents the CDC FIFO from overflowing when upstream cannot accept data.
    assign ctl_rclk_ena    = rx_ready_i[0] & rx_ready_i[1];

    // Disable RWDS-domain clock once all outstanding words have been consumed
    assign trx_rx_clk_reset = b_pending_clear;

    // Outstanding word-pair counter:
    //   increment when a clock cycle fires (one word delivered to each PHY's CDC FIFO)
    //   decrement when PHY0's data is consumed (both PHYs fire in lockstep)
    assign r_outstand_dec = rx_valid_o[0] & rx_ready_i[0];

    always_ff @(posedge clk_i or negedge rst_ni) begin : proc_ff_r_outstand
        if      (~rst_ni)                           r_outstand_q <= '0;
        else if (r_outstand_inc & ~r_outstand_dec)  r_outstand_q <= r_outstand_q + 1;
        else if (r_outstand_dec & ~r_outstand_inc)  r_outstand_q <= r_outstand_q - 1;
    end

    // ========================
    //   Control
    // ========================

    assign ctl_write_zero_lat = tf_q.address_space & tf_q.write;
    // Use OR of both PHY RWDS samples: either PHY needing extra latency triggers it
    assign ctl_add_latency    = (trx_rwds_sample[0] | trx_rwds_sample[1]) |
                                 cfg_i.en_latency_additional;

    // Burst is "last" when <= NumPhys words remain (burst=2 or burst=1 for odd lengths)
    assign ctl_tf_burst_last  = (tf_q.burst > 0) &&
                                (tf_q.burst <= hyper_blen_t'(NumPhys));
    assign ctl_tf_burst_done  = (tf_q.burst == 0);

    assign ctl_timer_rwr_done = (timer_q <= 3);
    assign ctl_timer_two      = (timer_q == 2);
    assign ctl_timer_one      = (timer_q == 1);
    assign ctl_timer_zero     = (timer_q == 0);

    // ========================
    //   Single shared FSM
    // ========================
    always_comb begin : proc_comb_phy_fsm
        // Default outputs
        trans_ready_o       = 1'b0;
        r_outstand_inc      = 1'b0;
        b_pending_set       = 1'b0;
        trx_cs_ena          = 1'b1;
        trx_clk_ena         = 1'b0;
        trx_rx_clk_set      = 1'b0;
        trx_rwds_sample_ena = 1'b0;
        // Default next state
        state_d = state_q;
        timer_d = timer_q - 1;
        tf_d    = tf_q;
        cs_d    = cs_q;
        // Tri-state control of DQ and RWDS
        trx_tx_rwds_oe = 1'b0;
        trx_tx_data_oe = 1'b0;

        case (state_q)
            Startup: begin
                trx_cs_ena = 1'b0;
                if (ctl_timer_one) begin
                    state_d = Idle;
                end
            end
            Idle: begin
                trx_cs_ena    = 1'b0;
                timer_d       = timer_q;
                trans_ready_o = 1'b1;
                if (trans_valid_i & ~b_pending_q & r_outstand_q == '0) begin
                    tf_d    = trans_i;
                    cs_d    = trans_cs_i;
                    // Send 3 CA words (t_CSS respected through clock delay)
                    timer_d = 2;
                    state_d = SendCA;
                    // Enable output driver one cycle early (tri-state IO pads are slow)
                    trx_tx_data_oe = 1'b1;
                end
            end
            SendCA: begin
                trx_clk_ena         = 1'b1;
                trx_tx_data_oe      = 1'b1;
                trx_rwds_sample_ena = ~ctl_write_zero_lat;
                if (ctl_timer_zero) begin
                    if (ctl_write_zero_lat) begin
                        timer_d = cfg_i.t_burst_max;
                        state_d = Write;
                    end else begin
                        timer_d = TimerWidth'(cfg_i.t_latency_access) << ctl_add_latency;
                        state_d = WaitLatAccess;
                    end
                end
            end
            WaitLatAccess: begin
                trx_clk_ena    = 1'b1;
                trx_tx_data_oe = 1'b1;
                // Subtract one cycle for last CA, one for state transition delay
                if (ctl_timer_two) begin
                    timer_d = cfg_i.t_burst_max;
                    if (tf_q.write) begin
                        state_d        = Write;
                        trx_tx_data_oe = 1'b1;
                        trx_tx_rwds_oe = ~ctl_write_zero_lat;
                    end else begin
                        state_d        = Read;
                        trx_tx_data_oe = 1'b0;
                        trx_tx_rwds_oe = 1'b0;
                    end
                end
            end
            Read: begin
                trx_rx_clk_set = 1'b1;
                // Only fire clock if both downstream FIFOs can accept data
                if (ctl_rclk_ena) begin
                    trx_clk_ena    = 1'b1;
                    r_outstand_inc = 1'b1;
                    // Each cycle both PHYs consume one word → decrement burst by 2.
                    // Address advances by 1 per clock cycle (one HyperBus address
                    // unit = one clock cycle row, containing NumPhys×16 bits).
                    // This preserves the same addressing semantics as hyperbus_phy.sv.
                    tf_d.burst   = tf_q.burst - hyper_blen_t'(NumPhys);
                    tf_d.address = tf_q.address + 1;
                    if (ctl_tf_burst_last) begin
                        timer_d = cfg_i.t_csh_cycles;
                        state_d = WaitXfer;
                    end
                end
                // Force-terminate access on burst time limit
                if (ctl_timer_one) begin
                    timer_d = cfg_i.t_csh_cycles;
                    state_d = WaitXfer;
                end
            end
            Write: begin
                // Drive DQ and RWDS lines
                trx_tx_data_oe = 1'b1;
                trx_tx_rwds_oe = ~ctl_write_zero_lat;
                if (ctl_wclk_ena) begin
                    trx_clk_ena  = 1'b1;
                    // Each cycle both PHYs consume one word → decrement burst by 2.
                    // Address advances by 1 per clock cycle (same semantics as hyperbus_phy.sv).
                    tf_d.burst   = tf_q.burst - hyper_blen_t'(NumPhys);
                    tf_d.address = tf_q.address + 1;
                    if (ctl_tf_burst_last) begin
                        b_pending_set = 1'b1;
                        timer_d       = cfg_i.t_csh_cycles;
                        state_d       = WaitXfer;
                    end
                end
                // Force-terminate access on burst time limit
                if (ctl_timer_one) begin
                    timer_d = cfg_i.t_csh_cycles;
                    state_d = WaitXfer;
                end
            end
            WaitXfer: begin
                // Wait for FFed clock output to stop
                if (ctl_timer_zero) begin
                    timer_d = cfg_i.t_read_write_recovery;
                    state_d = WaitRWR;
                end
            end
            WaitRWR: begin
                trx_cs_ena = 1'b0;
                if (ctl_timer_rwr_done) begin
                    if (ctl_tf_burst_done) begin
                        state_d = Idle;
                    end else begin
                        state_d        = SendCA;
                        trx_tx_data_oe = 1'b1;
                    end
                end
            end
        endcase
    end

    // ========================
    //   State registers
    // ========================
    always_ff @(posedge clk_i or negedge rst_ni) begin : proc_ff_phy
        if (~rst_ni) begin
            state_q <= Startup;
            timer_q <= StartupCycles;
            tf_q    <= hyper_tf_t'{burst_type: 1'b1, default:'0};
            cs_q    <= '0;
        end else begin
            state_q <= state_d;
            timer_q <= timer_d;
            tf_q    <= tf_d;
            cs_q    <= cs_d;
        end
    end

endmodule
