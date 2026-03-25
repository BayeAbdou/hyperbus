// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Armin Berger <bergerar@ethz.ch>
// Stephan Keck <kecks@ethz.ch>
// Thomas Benz <tbenz@iis.ee.ethz.ch>
// Paul Scheffler <paulsc@iis.ee.ethz.ch>
// Luca Valente <luca.valente@unibo.it>

// Dual-PHY HyperBus controller with a single shared FSM.
//
// A single state machine drives both hyperbus_trx instances in lockstep,
// eliminating the FSM-divergence that arises when two independent controllers
// each make a local accept/reject decision on the same incoming transaction.
//
// Per-PHY stream FIFOs sit between each TRX's RWDS-domain CDC FIFO and the
// combined output.  They absorb the ±1-cycle RWDS timing difference that can
// occur between two physically separate HyperBus devices, allowing the
// combined 32-bit output word to be presented simultaneously.
//
// Active-PHY tracking: cfg_i.phys_in_use selects whether one PHY or both are
// in use.  cfg_i.which_phy selects the single PHY when phys_in_use == 0.
// The phy_active_q register transitions safely: only when the FSM is in Idle,
// no B-response is pending, and all outstanding read words have been consumed.

module hyperbus_phy_dual import hyperbus_pkg::*; #(
    parameter int unsigned IsClockODelayed = -1,
    parameter int unsigned NumChips        = 2,
    parameter int unsigned TimerWidth      = 16,
    parameter int unsigned RxFifoLogDepth  = 3,
    parameter int unsigned SyncStages      = 2,
    parameter int unsigned StartupCycles   = 300 /*us*/ * 200 /*MHz*/
)(
    input  logic                     clk_i,
    input  logic                     clk_i_90,
    input  logic                     rst_ni,
    input  logic                     test_mode_i,
    // Config registers
    input  hyper_cfg_t               cfg_i,
    // PHY control status
    output logic                     busy_o,
    // Transactions
    input  logic                     trans_valid_i,
    output logic                     trans_ready_o,
    input  hyper_tf_t                trans_i,
    input  logic [NumChips-1:0]      trans_cs_i,
    // Transmitting channel
    // PHY 0 occupies bits [15: 0], PHY 1 occupies bits [31:16]
    input  logic                     tx_valid_i,
    output logic                     tx_ready_o,
    input  logic [31:0]              tx_data_i,
    input  logic [3:0]               tx_strb_i,
    input  logic                     tx_last_i,
    // Receiving channel
    // PHY 0 occupies bits [15: 0], PHY 1 occupies bits [31:16]
    output logic                     rx_valid_o,
    input  logic                     rx_ready_i,
    output logic [31:0]              rx_data_o,
    output logic                     rx_last_o,
    output logic                     rx_error_o,
    // B response
    output logic                     b_valid_o,
    input  logic                     b_ready_i,
    output logic                     b_error_o,
    // Physical interface (2 PHYs)
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

    localparam int unsigned NumPhys = 2;

    // =========================================================================
    //  Active-PHY tracking
    //  phy_active_q[i] == 1 means PHY i participates in the current/next
    //  transaction.  Transitions are gated to safe points only.
    // =========================================================================

    logic [NumPhys-1:0] phy_enable;
    logic [NumPhys-1:0] phy_active_q, phy_active_d;
    logic               change_phy_active;

    // Desired active set as determined by configuration
    assign phy_enable        = cfg_i.phys_in_use ? 2'b11
                                                  : (2'b01 << cfg_i.which_phy);
    assign change_phy_active = (phy_active_q != phy_enable);

    // =========================================================================
    //  FSM state
    // =========================================================================

    hyper_phy_state_t       state_d, state_q;
    logic [TimerWidth-1:0]  timer_d, timer_q;
    hyper_tf_t              tf_d,    tf_q;
    logic [NumChips-1:0]    cs_d,    cs_q;

    // B response
    logic b_pending_q;
    logic b_pending_set;
    logic b_pending_clear;

    // Outstanding RX word-pairs (one increment per Read clock, one decrement
    // per combined consumer read at the stream-FIFO output)
    logic [RxFifoLogDepth:0] r_outstand_q;
    logic                    r_outstand_inc;
    logic                    r_outstand_dec;

    // Transition phy_active_q only when the module is fully idle so that no
    // in-flight transaction is disrupted.
    assign phy_active_d = (change_phy_active &&
                           (state_q == Idle)  &&
                           ~b_pending_q       &&
                           (r_outstand_q == '0)) ? phy_enable : phy_active_q;

    always_ff @(posedge clk_i or negedge rst_ni) begin : proc_ff_phy_active
        if (!rst_ni) phy_active_q <= 2'b11;
        else         phy_active_q <= phy_active_d;
    end

    // Number of active PHYs derived from phy_active_q (not directly from cfg)
    // so that burst accounting matches which PHYs are actually running.
    logic [1:0] phys_in_use;
    assign phys_in_use = (&phy_active_q) ? 2'd2 : 2'd1;

    // =========================================================================
    //  Auxiliary control signals
    // =========================================================================

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

    // Command-address (identical for both PHYs – they address the same device)
    hyper_phy_ca_t ca;

    // =========================================================================
    //  TRX control buses (broadcast from single FSM to all active PHYs)
    // =========================================================================

    logic        trx_clk_ena;
    logic        trx_cs_ena;
    logic        trx_rwds_sample_ena;
    logic [15:0] trx_tx_data   [NumPhys];
    logic        trx_tx_data_oe;
    logic [1:0]  trx_tx_rwds   [NumPhys];
    logic        trx_tx_rwds_oe;
    logic        trx_rx_clk_set;
    logic        trx_rx_clk_reset;

    // TRX outputs
    logic [15:0] trx_rx_data    [NumPhys];
    logic        trx_rx_valid   [NumPhys];
    logic        trx_rx_ready   [NumPhys];
    logic        trx_rwds_sample[NumPhys];

    // =========================================================================
    //  Transceivers — one per PHY, driven by the single FSM
    // =========================================================================

    for (genvar i = 0; i < NumPhys; i++) begin : gen_trx
        hyperbus_trx #(
            .IsClockODelayed( IsClockODelayed ),
            .NumChips       ( NumChips        ),
            .RxFifoLogDepth ( RxFifoLogDepth  ),
            .SyncStages     ( SyncStages      )
        ) i_trx (
            .clk_i,
            .clk_i_90,
            .rst_ni,
            .test_mode_i,
            .cs_i               ( cs_q                          ),
            // CS is only asserted when this PHY is in the active set
            .cs_ena_i           ( trx_cs_ena & phy_active_q[i]  ),
            .rwds_sample_o      ( trx_rwds_sample[i]            ),
            .rwds_sample_ena_i  ( trx_rwds_sample_ena           ),
            .tx_clk_delay_i     ( cfg_i.t_tx_clk_delay          ),
            .tx_clk_ena_i       ( trx_clk_ena                   ),
            .tx_data_i          ( trx_tx_data[i]                ),
            .tx_data_oe_i       ( trx_tx_data_oe                ),
            .tx_rwds_i          ( trx_tx_rwds[i]                ),
            .tx_rwds_oe_i       ( trx_tx_rwds_oe                ),
            .rx_clk_delay_i     ( cfg_i.t_rx_clk_delay          ),
            .rx_clk_set_i       ( trx_rx_clk_set                ),
            .rx_clk_reset_i     ( trx_rx_clk_reset              ),
            .rx_data_o          ( trx_rx_data[i]                ),
            .rx_valid_o         ( trx_rx_valid[i]               ),
            .rx_ready_i         ( trx_rx_ready[i]               ),
            .hyper_cs_no        ( hyper_cs_no[i]                ),
            .hyper_ck_o         ( hyper_ck_o[i]                 ),
            .hyper_ck_no        ( hyper_ck_no[i]                ),
            .hyper_rwds_o       ( hyper_rwds_o[i]               ),
            .hyper_rwds_i       ( hyper_rwds_i[i]               ),
            .hyper_rwds_oe_o    ( hyper_rwds_oe_o[i]            ),
            .hyper_dq_i         ( hyper_dq_i[i]                 ),
            .hyper_dq_o         ( hyper_dq_o[i]                 ),
            .hyper_dq_oe_o      ( hyper_dq_oe_o[i]              ),
            .hyper_reset_no     ( hyper_reset_no[i]             )
        );
    end

    // =========================================================================
    //  Per-PHY stream FIFOs
    //  These absorb ±1-cycle RWDS timing differences between the two PHYs so
    //  that a combined 32-bit word can always be presented simultaneously.
    //  The FIFO input valid is gated by r_outstand_q to reject any residual
    //  output from the TRX CDC FIFO after a transaction has completed.
    // =========================================================================

    phy_rx_t [NumPhys-1:0] fifo_in_data;
    phy_rx_t [NumPhys-1:0] fifo_out_data;
    logic [NumPhys-1:0]    fifo_in_valid;
    logic [NumPhys-1:0]    fifo_out_valid;
    logic [NumPhys-1:0]    fifo_in_ready;
    logic [NumPhys-1:0]    fifo_out_ready;
    logic [NumPhys-1:0][1:0] fifo_usage;  // unused externally, required by stream_fifo port

    for (genvar i = 0; i < NumPhys; i++) begin : gen_rx_fifo
        // Feed TRX output straight into the per-PHY FIFO.
        // Gate with r_outstand_q to prevent spurious CDC-FIFO residue.
        // Note: hyperbus_trx has no error output; error stays 0 for now.
        assign fifo_in_data[i].data  = trx_rx_data[i];
        assign fifo_in_data[i].last  = 1'b0;   // last is computed at FIFO output
        assign fifo_in_data[i].error = 1'b0;   // TODO: propagate when TRX exposes errors
        assign fifo_in_valid[i]      = trx_rx_valid[i] & (r_outstand_q != '0);
        assign trx_rx_ready[i]       = fifo_in_ready[i];

        stream_fifo #(
            .FALL_THROUGH ( 1'b0     ),
            .DEPTH        ( 4        ),
            .T            ( phy_rx_t )
        ) i_rx_fifo (
            .clk_i      ( clk_i             ),
            .rst_ni     ( rst_ni            ),
            .flush_i    ( 1'b0              ),
            .testmode_i ( 1'b0              ),
            .usage_o    ( fifo_usage[i]     ),
            .data_i     ( fifo_in_data[i]   ),
            .valid_i    ( fifo_in_valid[i]  ),
            .ready_o    ( fifo_in_ready[i]  ),
            .data_o     ( fifo_out_data[i]  ),
            .valid_o    ( fifo_out_valid[i] ),
            .ready_i    ( fifo_out_ready[i] )
        );
    end

    // =========================================================================
    //  RX data path
    // =========================================================================

    // Combined valid: every active PHY's FIFO must have a word ready
    logic rx_both_valid;
    assign rx_both_valid = &(fifo_out_valid | ~phy_active_q);

    assign rx_valid_o = rx_both_valid;
    assign rx_data_o  = {fifo_out_data[1].data, fifo_out_data[0].data};
    // OR the error flags from all active PHYs (mirrors original hyperbus_phy_if behaviour)
    assign rx_error_o = | ({fifo_out_data[1].error, fifo_out_data[0].error} & phy_active_q);
    // rx_last_o is derived directly from FSM state so it is correctly aligned
    // with the combined stream-FIFO output (no need to store it in the FIFOs).
    assign rx_last_o  = (state_q != Read) & ctl_tf_burst_done & (r_outstand_q == 1);

    // Drain all active PHYs' stream FIFOs simultaneously when the consumer reads
    always_comb begin : proc_comb_rx_ready
        fifo_out_ready = '0;
        if (rx_both_valid && rx_ready_i) begin
            fifo_out_ready[0] = phy_active_q[0];
            fifo_out_ready[1] = phy_active_q[1];
        end
    end

    // Backpressure: hold the RWDS clock when the consumer stalls, which
    // prevents the TRX CDC FIFO from overflowing (same mechanism as in
    // the single-PHY hyperbus_phy).
    assign ctl_rclk_ena = ~(rx_valid_o & ~rx_ready_i);

    // Disable the incoming RWDS clock enable once all expected words received
    assign trx_rx_clk_reset = b_pending_clear;

    // r_outstand_q counts combined word-pairs outstanding at the consumer
    // output (incremented by the FSM in the Read state, decremented when the
    // consumer reads the combined stream-FIFO output).
    assign r_outstand_dec = rx_valid_o & rx_ready_i;
    always_ff @(posedge clk_i or negedge rst_ni) begin : proc_ff_r_outstand
        if      (~rst_ni)                           r_outstand_q <= '0;
        else if (r_outstand_inc & ~r_outstand_dec)  r_outstand_q <= r_outstand_q + 1;
        else if (r_outstand_dec & ~r_outstand_inc)  r_outstand_q <= r_outstand_q - 1;
    end

    // =========================================================================
    //  B response
    // =========================================================================

    assign b_valid_o       = b_pending_q;
    assign b_error_o       = 1'b0;    // TODO
    assign b_pending_clear = b_valid_o & b_ready_i;

    always_ff @(posedge clk_i or negedge rst_ni) begin : proc_ff_b_pending
        if      (~rst_ni)           b_pending_q <= 1'b0;
        else if (b_pending_set)     b_pending_q <= 1'b1;
        else if (b_pending_clear)   b_pending_q <= 1'b0;
    end

    // =========================================================================
    //  TX data path
    // =========================================================================

    // CA is identical for both PHYs — they address the same memory location
    assign ca = hyper_phy_ca_t '{
        write:      ~tf_q.write,
        addr_space: tf_q.address_space,
        burst_type: tf_q.burst_type,
        addr_upper: tf_q.address[31:3],
        reserved:   '0,
        addr_lower: tf_q.address[2:0]
    };

    always_comb begin : proc_comb_tx
        trx_tx_data[0] = '0;
        trx_tx_data[1] = '0;
        trx_tx_rwds[0] = '0;
        trx_tx_rwds[1] = '0;
        tx_ready_o     = 1'b0;
        ctl_wclk_ena   = 1'b0;
        if (state_q == SendCA) begin
            // CA words are the same on both physical buses
            trx_tx_data[0] = ca[(8'(timer_q) << 4) +: 16];
            trx_tx_data[1] = ca[(8'(timer_q) << 4) +: 16];
        end else if (state_q == Write) begin
            // PHY 0 carries bits [15:0], PHY 1 carries bits [31:16]
            trx_tx_data[0] = tx_data_i[15: 0];
            trx_tx_data[1] = tx_data_i[31:16];
            trx_tx_rwds[0] = ~tx_strb_i[1:0];
            trx_tx_rwds[1] = ~tx_strb_i[3:2];
            tx_ready_o     = 1'b1;   // HyperBus burst always accepts data
            ctl_wclk_ena   = tx_valid_i;
        end
    end

    // =========================================================================
    //  Auxiliary control signal assignments
    // =========================================================================

    assign ctl_write_zero_lat = tf_q.address_space & tf_q.write;
    // Use the OR of all active PHYs' RWDS samples: if either device requires
    // additional latency we honour it.  cfg_i.en_latency_additional is the
    // software-controlled override.
    assign ctl_add_latency    = (trx_rwds_sample[0] & phy_active_q[0])
                              | (trx_rwds_sample[1] & phy_active_q[1])
                              | cfg_i.en_latency_additional;

    assign ctl_tf_burst_last  = (tf_q.burst == 1) || (tf_q.burst == phys_in_use);
    assign ctl_tf_burst_done  = (tf_q.burst == 0);

    assign ctl_timer_rwr_done = (timer_q <= 3);
    assign ctl_timer_two      = (timer_q == 2);
    assign ctl_timer_one      = (timer_q == 1);
    assign ctl_timer_zero     = (timer_q == 0);

    assign busy_o = (state_q != Idle);

    // =========================================================================
    //  Single shared FSM
    //  Logic is intentionally identical to hyperbus_phy with the following
    //  differences:
    //   • trans_ready_o is also blocked during a phy_active transition
    //   • tf_d.burst decrements by phys_in_use (1 or 2) per clock
    // =========================================================================

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
        // State-dependent logic
        case (state_q)
            Startup: begin
                trx_cs_ena = 1'b0;
                if (ctl_timer_one) begin
                    state_d = Idle;
                end
            end
            Idle: begin
                trx_cs_ena = 1'b0;
                timer_d    = timer_q;
                // Block new transactions when a B-response is still pending,
                // when read data has not been fully consumed, or while the
                // active-PHY set is being changed.
                trans_ready_o = ~b_pending_q & (r_outstand_q == '0) & ~change_phy_active;
                if (trans_valid_i & ~b_pending_q & (r_outstand_q == '0) & ~change_phy_active) begin
                    tf_d           = trans_i;
                    cs_d           = trans_cs_i;
                    timer_d        = 2;
                    state_d        = SendCA;
                    // Enable output driver one cycle early (tri-state turn-
                    // around of IO pads is slow relative to the data pins)
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
                // Subtract one cycle for the last CA word and one for the
                // state-register pipeline stage
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
                if (ctl_rclk_ena) begin
                    trx_clk_ena    = 1'b1;
                    r_outstand_inc = 1'b1;
                    tf_d.burst     = tf_q.burst - phys_in_use;
                    tf_d.address   = tf_q.address + 1;
                    if (ctl_tf_burst_last) begin
                        timer_d = cfg_i.t_csh_cycles;
                        state_d = WaitXfer;
                    end
                end
                // Force-terminate on burst time limit
                if (ctl_timer_one) begin
                    timer_d = cfg_i.t_csh_cycles;
                    state_d = WaitXfer;
                end
            end
            Write: begin
                trx_tx_data_oe = 1'b1;
                trx_tx_rwds_oe = ~ctl_write_zero_lat;
                if (ctl_wclk_ena) begin
                    trx_clk_ena  = 1'b1;
                    tf_d.burst   = tf_q.burst - phys_in_use;
                    tf_d.address = tf_q.address + 1;
                    if (ctl_tf_burst_last) begin
                        b_pending_set = 1'b1;
                        timer_d       = cfg_i.t_csh_cycles;
                        state_d       = WaitXfer;
                    end
                end
                // Force-terminate on burst time limit
                if (ctl_timer_one) begin
                    timer_d = cfg_i.t_csh_cycles;
                    state_d = WaitXfer;
                end
            end
            WaitXfer: begin
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

    // =========================================================================
    //  PHY state registers
    // =========================================================================

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
