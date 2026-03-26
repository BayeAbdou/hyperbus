// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Armin Berger <bergerar@ethz.ch>

// Single-FSM dual-PHY HyperBus controller.
//
// Replaces two independent hyperbus_phy instances in the NumPhys=2 path.
// A single FSM drives all NumPhys TRX blocks simultaneously so they stay
// cycle-accurate with each other.  Per-PHY stream FIFOs absorb CDC jitter
// between the RWDS-domain RX data and the system clock domain.
//
// Fix 1 – ctl_rclk_ena is gated with FIFO *input* readiness so the FSM
//          immediately stops issuing HyperBus clocks when any active FIFO
//          is full, preventing r_outstand_q from running ahead.
//
// Fix 2 – The Idle state will not accept a new transaction until all
//          active FIFOs have been drained (fifos_empty), so stale FIFO
//          contents from the previous transaction cannot bleed into the
//          next one.
//
// Fix 3 – Critical debug signals are annotated with (* syn_keep *) /
//          (* preserve *) to prevent Libero from optimising them away.
//
// Fix 4 – Both TRX instances are generated with a `for (genvar i …)`
//          loop; no mirror-test single-TRX shortcut.

module hyperbus_phy_dual import hyperbus_pkg::*; #(
    parameter int unsigned IsClockODelayed = 1,
    parameter int unsigned NumChips        = 2,
    parameter int unsigned NumPhys         = 2,
    parameter int unsigned TimerWidth      = 16,
    parameter int unsigned RxFifoLogDepth  = 3,
    parameter int unsigned StartupCycles   = 60000,
    parameter int unsigned SyncStages      = 2,
    parameter type         hyper_tx_t      = logic,
    parameter type         hyper_rx_t      = logic
)(
    input  logic                             clk_i,
    input  logic                             clk_i_90,
    input  logic                             rst_ni,
    input  logic                             test_mode_i,
    // Configuration registers
    input  hyper_cfg_t                       cfg_i,
    // Transaction channel
    input  logic                             trans_valid_i,
    output logic                             trans_ready_o,
    input  hyper_tf_t                        trans_i,
    input  logic [NumChips-1:0]              trans_cs_i,
    // TX data channel
    input  logic                             tx_valid_i,
    output logic                             tx_ready_o,
    input  hyper_tx_t                        tx_i,
    // RX data channel
    output logic                             rx_valid_o,
    input  logic                             rx_ready_i,
    output hyper_rx_t                        rx_o,
    // Write-response channel
    output logic                             b_valid_o,
    input  logic                             b_ready_i,
    output logic                             b_error_o,
    // Physical interfaces (one set per PHY)
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

    // =========================================================================
    //  Active-PHY tracking
    // =========================================================================

    // Fix 3: preserve for Libero debug
    (* syn_keep = "true" *) (* preserve *)
    logic [NumPhys-1:0]  phy_active_q, phy_active_d;

    (* syn_keep = "true" *) (* preserve *)
    logic [1:0]          phys_in_use;

    logic [NumPhys-1:0]  phy_enable;
    logic                change_phy_active;

    assign phy_enable        = cfg_i.phys_in_use ? '1 : (NumPhys'(1) << cfg_i.which_phy);
    assign change_phy_active = (phy_active_q != phy_enable);
    assign phys_in_use       = cfg_i.phys_in_use ? 2'd2 : 2'd1;

    // =========================================================================
    //  FSM state
    // =========================================================================

    (* syn_keep = "true" *) (* preserve *)
    hyper_phy_state_t        state_d, state_q;

    (* syn_keep = "true" *) (* preserve *)
    logic [TimerWidth-1:0]   timer_d, timer_q;

    hyper_tf_t               tf_d, tf_q;
    logic [NumChips-1:0]     cs_d,  cs_q;

    // =========================================================================
    //  Write-response pending flag
    // =========================================================================

    (* syn_keep = "true" *) (* preserve *)
    logic b_pending_q;
    logic b_pending_set;
    logic b_pending_clear;

    // =========================================================================
    //  Outstanding-read counter
    // =========================================================================

    (* syn_keep = "true" *) (* preserve *)
    logic [RxFifoLogDepth:0] r_outstand_q;
    logic                    r_outstand_inc;
    logic                    r_outstand_dec;

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

    (* syn_keep = "true" *) (* preserve *)
    logic ctl_rclk_ena;
    logic ctl_wclk_ena;

    // =========================================================================
    //  FIFO control busses  (Fix 1 / Fix 2 / Fix 3)
    // =========================================================================

    (* syn_keep = "true" *) (* preserve *)
    logic [NumPhys-1:0]        fifo_in_valid;

    (* syn_keep = "true" *) (* preserve *)
    logic [NumPhys-1:0]        fifo_in_ready;

    (* syn_keep = "true" *) (* preserve *)
    logic [NumPhys-1:0]        fifo_out_valid;

    logic [NumPhys-1:0]        fifo_out_ready;
    logic [NumPhys-1:0][15:0]  fifo_out_data;

    // Fix 1: stop issuing clocks as soon as any active FIFO can no longer
    //        accept data (input side full).
    (* syn_keep = "true" *) (* preserve *)
    logic fifo_can_accept;

    assign fifo_can_accept = &(fifo_in_ready | ~phy_active_q);
    assign ctl_rclk_ena    = fifo_can_accept & ~(rx_valid_o & ~rx_ready_i);

    // Fix 2: gate Idle→SendCA transition on all active FIFOs being empty.
    (* syn_keep = "true" *) (* preserve *)
    logic fifos_empty;

    assign fifos_empty = &(~fifo_out_valid | ~phy_active_q);

    // =========================================================================
    //  Combined RX output
    // =========================================================================

    // All active FIFOs must hold data before we present a word to the consumer.
    assign rx_valid_o     = &(fifo_out_valid | ~phy_active_q) & (r_outstand_q != '0);
    assign fifo_out_ready = {NumPhys{rx_valid_o & rx_ready_i}};
    assign r_outstand_dec = rx_valid_o & rx_ready_i;

    for (genvar i = 0; i < NumPhys; i++) begin : gen_rx_data
        assign rx_o.data[i*16 +:16] = fifo_out_data[i];
    end
    assign rx_o.error = 1'b0;
    assign rx_o.last  = (state_q != Read) & ctl_tf_burst_done & (r_outstand_q == 1);

    // =========================================================================
    //  Write-response channel
    // =========================================================================

    assign b_valid_o       = b_pending_q;
    assign b_error_o       = 1'b0;
    assign b_pending_clear = b_valid_o & b_ready_i;

    always_ff @(posedge clk_i or negedge rst_ni) begin : proc_ff_b_pending
        if      (~rst_ni)          b_pending_q <= 1'b0;
        else if (b_pending_set)    b_pending_q <= 1'b1;
        else if (b_pending_clear)  b_pending_q <= 1'b0;
    end

    // =========================================================================
    //  Outstanding-read counter
    // =========================================================================

    always_ff @(posedge clk_i or negedge rst_ni) begin : proc_ff_r_outstand
        if      (~rst_ni)                           r_outstand_q <= '0;
        else if (r_outstand_inc & ~r_outstand_dec)  r_outstand_q <= r_outstand_q + 1;
        else if (r_outstand_dec & ~r_outstand_inc)  r_outstand_q <= r_outstand_q - 1;
    end

    // =========================================================================
    //  phy_active_q  – switch only when idle and FIFOs drained
    // =========================================================================

    assign phy_active_d = (change_phy_active && (state_q == Idle) && fifos_empty) ?
                          phy_enable : phy_active_q;

    always_ff @(posedge clk_i or negedge rst_ni) begin : proc_ff_phy_active
        if (~rst_ni)  phy_active_q <= '1;
        else          phy_active_q <= phy_active_d;
    end

    // =========================================================================
    //  Auxiliary control
    // =========================================================================

    assign ctl_write_zero_lat = tf_q.address_space & tf_q.write;
    // Extra latency if any active PHY samples RWDS high, or if forced by cfg.
    assign ctl_add_latency    = |(trx_rwds_sample & phy_active_q) |
                                cfg_i.en_latency_additional;

    assign ctl_tf_burst_last  = (tf_q.burst == 1) || (tf_q.burst == phys_in_use);
    assign ctl_tf_burst_done  = (tf_q.burst == 0);

    assign ctl_timer_rwr_done = (timer_q <= 3);
    assign ctl_timer_two      = (timer_q == 2);
    assign ctl_timer_one      = (timer_q == 1);
    assign ctl_timer_zero     = (timer_q == 0);

    // =========================================================================
    //  Command-address word
    // =========================================================================

    hyper_phy_ca_t ca;

    assign ca = hyper_phy_ca_t '{
        write:      ~tf_q.write,
        addr_space: tf_q.address_space,
        burst_type: tf_q.burst_type,
        addr_upper: tf_q.address[31:3],
        reserved:   '0,
        addr_lower: tf_q.address[2:0]
    };

    // =========================================================================
    //  Per-PHY TRX wiring
    // =========================================================================

    logic [NumPhys-1:0]        trx_clk_ena;
    logic [NumPhys-1:0]        trx_cs_ena;
    logic [NumPhys-1:0]        trx_rwds_sample;
    logic [NumPhys-1:0]        trx_rwds_sample_ena;
    logic [NumPhys-1:0][15:0]  trx_tx_data;
    logic [NumPhys-1:0]        trx_tx_data_oe;
    logic [NumPhys-1:0][1:0]   trx_tx_rwds;
    logic [NumPhys-1:0]        trx_tx_rwds_oe;
    logic [NumPhys-1:0]        trx_rx_clk_set;
    logic [NumPhys-1:0]        trx_rx_clk_reset;
    logic [NumPhys-1:0][15:0]  trx_rx_data;
    logic [NumPhys-1:0]        trx_rx_valid;
    logic [NumPhys-1:0]        trx_rx_ready;

    // =========================================================================
    //  TX dataflow
    // =========================================================================

    always_comb begin : proc_comb_tx
        trx_tx_data  = '0;
        trx_tx_rwds  = '0;
        tx_ready_o   = 1'b0;
        ctl_wclk_ena = 1'b0;
        if (state_q == SendCA) begin
            // Timer counts 2→1→0; each step selects one 16-bit CA word:
            //   timer=2 → ca[47:32], timer=1 → ca[31:16], timer=0 → ca[15:0]
            for (int i = 0; i < NumPhys; i++)
                trx_tx_data[i] = ca[(8'(timer_q) << 4) +: 16];
        end else if (state_q == Write) begin
            for (int i = 0; i < NumPhys; i++) begin
                trx_tx_data[i] = tx_i.data[16*i +:16];
                trx_tx_rwds[i] = ~tx_i.strb[2*i +:2];
            end
            tx_ready_o   = 1'b1;
            ctl_wclk_ena = tx_valid_i;
        end
    end

    // =========================================================================
    //  FSM (single instance controlling all NumPhys TRX blocks)
    // =========================================================================

    always_comb begin : proc_comb_phy_fsm
        // Default outputs
        trans_ready_o       = 1'b0;
        r_outstand_inc      = 1'b0;
        b_pending_set       = 1'b0;
        // TRX control defaults
        trx_cs_ena          = {NumPhys{1'b1}};
        trx_clk_ena         = '0;
        trx_rx_clk_set      = '0;
        trx_rwds_sample_ena = '0;
        trx_tx_data_oe      = '0;
        trx_tx_rwds_oe      = '0;
        // Default next-state
        state_d = state_q;
        timer_d = timer_q - 1;
        tf_d    = tf_q;
        cs_d    = cs_q;

        case (state_q)

            Startup: begin
                trx_cs_ena = '0;
                if (ctl_timer_one) state_d = Idle;
            end

            Idle: begin
                trx_cs_ena    = '0;
                timer_d       = timer_q;
                // Fix 2: do not advertise readiness until FIFOs are empty
                trans_ready_o = fifos_empty;
                if (trans_valid_i & ~b_pending_q &
                        (r_outstand_q == '0) & fifos_empty) begin
                    tf_d           = trans_i;
                    cs_d           = trans_cs_i;
                    timer_d        = 2;
                    state_d        = SendCA;
                    trx_tx_data_oe = phy_active_q;
                end
            end

            SendCA: begin
                trx_clk_ena         = phy_active_q;
                trx_tx_data_oe      = phy_active_q;
                trx_rwds_sample_ena = phy_active_q & {NumPhys{~ctl_write_zero_lat}};
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
                trx_clk_ena    = phy_active_q;
                trx_tx_data_oe = phy_active_q;
                // Two-cycle pipeline offset before switching to data phase
                if (ctl_timer_two) begin
                    timer_d = cfg_i.t_burst_max;
                    if (tf_q.write) begin
                        state_d        = Write;
                        trx_tx_data_oe = phy_active_q;
                        trx_tx_rwds_oe = phy_active_q & {NumPhys{~ctl_write_zero_lat}};
                    end else begin
                        state_d        = Read;
                        trx_tx_data_oe = '0;
                        trx_tx_rwds_oe = '0;
                    end
                end
            end

            Read: begin
                trx_rx_clk_set = phy_active_q;
                // Fix 1: ctl_rclk_ena now checks FIFO input readiness
                if (ctl_rclk_ena) begin
                    trx_clk_ena    = phy_active_q;
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
                trx_tx_data_oe = phy_active_q;
                trx_tx_rwds_oe = phy_active_q & {NumPhys{~ctl_write_zero_lat}};
                if (ctl_wclk_ena) begin
                    trx_clk_ena  = phy_active_q;
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
                trx_cs_ena = '0;
                if (ctl_timer_rwr_done) begin
                    if (ctl_tf_burst_done) begin
                        state_d = Idle;
                    end else begin
                        state_d        = SendCA;
                        trx_tx_data_oe = phy_active_q;
                    end
                end
            end

            default: ;

        endcase
    end

    // =========================================================================
    //  State registers
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

    // =========================================================================
    //  Fix 4: generate loop – one TRX + one RX FIFO per PHY
    // =========================================================================

    for (genvar i = 0; i < NumPhys; i++) begin : gen_phy

        // Connect TRX output into FIFO input
        assign fifo_in_valid[i]    = trx_rx_valid[i];
        assign trx_rx_ready[i]     = fifo_in_ready[i];
        // Reset the RX-clock enable once the write response is acknowledged
        assign trx_rx_clk_reset[i] = b_pending_clear;

        // RX FIFO: buffers CDC jitter from RWDS domain to system clock domain
        stream_fifo #(
            .FALL_THROUGH ( 1'b0         ),
            .DEPTH        ( 4            ),
            .T            ( logic [15:0] )
        ) i_rx_fifo (
            .clk_i      ( clk_i             ),
            .rst_ni     ( rst_ni            ),
            .flush_i    ( 1'b0              ),
            .testmode_i ( 1'b0              ),
            .usage_o    (                   ),
            .data_i     ( trx_rx_data[i]    ),
            .valid_i    ( fifo_in_valid[i]  ),
            .ready_o    ( fifo_in_ready[i]  ),
            .data_o     ( fifo_out_data[i]  ),
            .valid_o    ( fifo_out_valid[i] ),
            .ready_i    ( fifo_out_ready[i] )
        );

        // Transceiver: DDR I/O + CDC FIFO from RWDS clock to system clock
        hyperbus_trx #(
            .IsClockODelayed ( IsClockODelayed ),
            .NumChips        ( NumChips        ),
            .RxFifoLogDepth  ( RxFifoLogDepth  ),
            .SyncStages      ( SyncStages      )
        ) i_trx (
            .clk_i              ( clk_i                  ),
            .clk_i_90           ( clk_i_90               ),
            .rst_ni             ( rst_ni                  ),
            .test_mode_i        ( test_mode_i             ),
            .cs_i               ( cs_q                   ),
            .cs_ena_i           ( trx_cs_ena[i]          ),
            .rwds_sample_o      ( trx_rwds_sample[i]     ),
            .rwds_sample_ena_i  ( trx_rwds_sample_ena[i] ),
            .tx_clk_delay_i     ( cfg_i.t_tx_clk_delay   ),
            .tx_clk_ena_i       ( trx_clk_ena[i]         ),
            .tx_data_i          ( trx_tx_data[i]         ),
            .tx_data_oe_i       ( trx_tx_data_oe[i]      ),
            .tx_rwds_i          ( trx_tx_rwds[i]         ),
            .tx_rwds_oe_i       ( trx_tx_rwds_oe[i]      ),
            .rx_clk_delay_i     ( cfg_i.t_rx_clk_delay   ),
            .rx_clk_set_i       ( trx_rx_clk_set[i]      ),
            .rx_clk_reset_i     ( trx_rx_clk_reset[i]    ),
            .rx_data_o          ( trx_rx_data[i]         ),
            .rx_valid_o         ( trx_rx_valid[i]        ),
            .rx_ready_i         ( trx_rx_ready[i]        ),
            .hyper_cs_no        ( hyper_cs_no[i]         ),
            .hyper_ck_o         ( hyper_ck_o[i]          ),
            .hyper_ck_no        ( hyper_ck_no[i]         ),
            .hyper_rwds_o       ( hyper_rwds_o[i]        ),
            .hyper_rwds_i       ( hyper_rwds_i[i]        ),
            .hyper_rwds_oe_o    ( hyper_rwds_oe_o[i]     ),
            .hyper_dq_i         ( hyper_dq_i[i]          ),
            .hyper_dq_o         ( hyper_dq_o[i]          ),
            .hyper_dq_oe_o      ( hyper_dq_oe_o[i]       ),
            .hyper_reset_no     ( hyper_reset_no[i]      )
        );

    end // for (genvar i = 0; i < NumPhys; i++)

endmodule
