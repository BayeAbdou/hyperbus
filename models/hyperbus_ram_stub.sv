// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

`timescale 1ns/1ps

// Behavioral HyperBus device stub for PHY 1 in dual-PHY simulations.
//
// Purpose
// -------
// Drop-in replacement for the s27ks0641 on PHY 1.  The stub implements the
// HyperBus protocol (6-byte CA, latency wait, DDR read/write data phase with
// correct RWDS strobe timing) backed by an associative memory, so writes are
// stored and re-played on reads, allowing the testbench data-integrity checks
// to pass.  Every transaction start and every write word is announced via
// $display for easy debugging.
//
// Interface
// ---------
// Matches the s27ks0641 pinout: individual DQ7..DQ0 inout pins, single RWDS
// inout wire, CSNeg, CK, CKNeg, RESETNeg inputs.
//
// Timing
// ------
// The HyperBus PHY (hyperbus_phy_dual.sv) with t_latency_access=6 and
// en_latency_additional=1 (as set by the testbench) takes:
//
//   3  CK cycles for CA (SendCA state, timer 2→0)
//  10  CK cycles for latency (WaitLatAccess state, timer 12→2)
//  +1  CK cycle  for rx_rwds_clk_ena FF registration (Read case)
//  = 14 CK cycles = 28 CK half-cycles from the first CK edge after CS# to
//    when the controller's RWDS capture gate opens.
//
//  CK (tx_clk_90) is delayed 1.5 ns from PHY clk_i and PHY 1 adds
//  t_rx_clk_delay_phy1 = 8 steps × 0.25 ns/step = 2 ns of RWDS pipeline
//  delay.  For the stub's first RWDS rising edge to arrive at the controller
//  AFTER the gate opens, the stub must start driving at half_cnt ≥ 29.
//
//  HalfLatencyRead  (default 29): first rising RWDS edge at this half-cycle.
//  HalfLatencyWrite (default 27): first write-data CK edge at this half-cycle.
//    The controller starts driving actual write data at PHY cycle 15
//    (t = 84 ns from SendCA), which maps to CK rising half_cnt = 27.
//
//  NOTE: Both default values assume PHY_TCK=6ns, t_latency_access=6 (default),
//  en_latency_additional=1 (set by hyperbus_tb*.sv).  Adjust for other configs.
//  Fine-tune using t_rx_clk_delay_phy1 / t_tx_clk_delay_phy1 registers at run-time.
//
// HyperBus CA layout (48 bits, MSB first on DQ[7:0]):
//   [47]    R/W_n  1=read  (DUT sets ca.write = ~tf_q.write, so 1 here = read)
//   [46]    address_space
//   [45]    burst_type
//   [44:16] addr_upper[28:0] = tf_q.address[31:3]
//   [15: 3] reserved (0)
//   [ 2: 0] addr_lower[2:0] = tf_q.address[2:0]
//   HyperBus word address = {addr_upper, addr_lower} = tf_q.address[31:0]

module hyperbus_ram_stub #(
    // CK half-cycles from the first CK edge after CS# until stub drives RWDS
    // and DQ on reads.  Default 29 → first rising RWDS edge arrives at the
    // controller just after the RWDS gate opens (~2 CK cycles of margin).
    parameter int unsigned HalfLatencyRead  = 29,
    // CK half-cycles from the first CK edge after CS# until the stub starts
    // capturing write data.  Default 27 → aligns with the first CK rising edge
    // at which the controller drives actual write data (PHY cycle 15 = 85.5 ns).
    parameter int unsigned HalfLatencyWrite = 27,
    // Label shown in $display messages
    parameter string       DebugName        = "phy1_stub"
) (
    inout  wire  DQ7,
    inout  wire  DQ6,
    inout  wire  DQ5,
    inout  wire  DQ4,
    inout  wire  DQ3,
    inout  wire  DQ2,
    inout  wire  DQ1,
    inout  wire  DQ0,
    inout  wire  RWDS,
    input  logic CSNeg,
    input  logic CK,
    input  logic CKNeg,
    input  logic RESETNeg
);

    // =========================================================================
    // Protocol state
    // =========================================================================

    typedef enum logic [2:0] {
        StIdle      = 3'd0,
        StCA        = 3'd1,
        StLatency   = 3'd2,
        StReadData  = 3'd3,
        StWriteData = 3'd4
    } state_t;

    // =========================================================================
    // Associative memory (16-bit words, 32-bit word address).
    // Unwritten locations return a distinctive debug pattern:
    //   high byte = 0xDE, low byte = addr[7:0].
    // =========================================================================

    logic [15:0] mem [logic [31:0]];

    function automatic logic [15:0] mem_read (input logic [31:0] addr);
        return (mem.exists(addr)) ? mem[addr] : {8'hDE, addr[7:0]};
    endfunction

    // =========================================================================
    // Protocol state registers (driven by proc_ck_sm and proc_cs_deassert)
    // =========================================================================

    state_t       state;
    int unsigned  half_cnt;     // CK half-cycles since first CK edge after CS#
    logic [47:0]  ca_reg;       // accumulated CA bytes
    logic         is_read_reg;  // decoded from CA[47] after CA is complete
    logic [31:0]  word_addr;    // current HyperBus word address
    logic [15:0]  rd_data;      // word pre-fetched for current read burst beat

    logic         rwds_drv;     // 1 = stub drives RWDS
    logic         rwds_val;     // value on RWDS when driving
    logic         dq_drv;       // 1 = stub drives DQ
    logic [7:0]   dq_val;       // value on DQ[7:0] when driving

    logic [7:0]   wr_hi_byte;   // write: high byte captured on CK↑
    logic         wr_hi_dis;    // write: RWDS for high byte (1 = mask)

    // =========================================================================
    // Tristate outputs
    // =========================================================================

    assign RWDS = rwds_drv ? rwds_val : 1'bz;
    assign DQ7  = dq_drv   ? dq_val[7] : 1'bz;
    assign DQ6  = dq_drv   ? dq_val[6] : 1'bz;
    assign DQ5  = dq_drv   ? dq_val[5] : 1'bz;
    assign DQ4  = dq_drv   ? dq_val[4] : 1'bz;
    assign DQ3  = dq_drv   ? dq_val[3] : 1'bz;
    assign DQ2  = dq_drv   ? dq_val[2] : 1'bz;
    assign DQ1  = dq_drv   ? dq_val[1] : 1'bz;
    assign DQ0  = dq_drv   ? dq_val[0] : 1'bz;

    // =========================================================================
    // Power-on defaults
    // =========================================================================

    initial begin : proc_init
        state        = StIdle;
        half_cnt     = 0;
        ca_reg       = '0;
        is_read_reg  = 1'b0;
        word_addr    = '0;
        rd_data      = '0;
        rwds_drv     = 1'b0;
        rwds_val     = 1'b0;
        dq_drv       = 1'b0;
        dq_val       = '0;
        wr_hi_byte   = '0;
        wr_hi_dis    = 1'b0;
    end

    // =========================================================================
    // CS# de-assertion: end transaction, release bus
    // =========================================================================

    always @(posedge CSNeg) begin : proc_cs_deassert
        state    <= StIdle;
        half_cnt <= 0;
        rwds_drv <= 1'b0;
        rwds_val <= 1'b0;
        dq_drv   <= 1'b0;
        dq_val   <= '0;
    end

    // =========================================================================
    // Main CK-edge state machine
    //
    //  Inside the always block:
    //    CK == 1  →  this was a rising  edge
    //    CK == 0  →  this was a falling edge
    //
    //  half_cnt starts at 1 on the first CK edge after CS# goes low.
    //  Odd  half_cnt values → rising  CK edges (CK = 1).
    //  Even half_cnt values → falling CK edges (CK = 0).
    //
    //  CA capture (6 half-cycles):
    //   half_cnt 1 (rising):   CA[47:40] captured in StIdle
    //   half_cnt 1 (StCA):     CA[39:32]
    //   half_cnt 2:            CA[31:24]
    //   half_cnt 3:            CA[23:16]
    //   half_cnt 4:            CA[15:8]
    //   half_cnt 5:            CA[7:0],  R/W decoded
    //   half_cnt 6+: StLatency
    //
    //  Latency decode (different for reads and writes):
    //   half_cnt = HalfLatencyWrite-1 (=26, falling) → if write: enter StWriteData
    //   half_cnt = HalfLatencyRead-1  (=28, falling) → if read:  enter StReadData
    //   In both cases the decode edge is falling so the NEXT (rising) edge is the
    //   first data-phase edge, giving correct DDR byte alignment.
    // =========================================================================

    always @(CK) begin : proc_ck_sm
        if (!CSNeg && RESETNeg) begin

            half_cnt <= half_cnt + 1;

            case (state)

                // ----------------------------------------------------------------
                // StIdle: first CK edge with CS# asserted → begin CA capture
                // ----------------------------------------------------------------
                StIdle: begin
                    half_cnt      <= 1;          // this edge is count 1
                    state         <= StCA;
                    // CA byte 5 (bits[47:40]) is valid on DQ at this edge.
                    ca_reg[47:40] <= {DQ7,DQ6,DQ5,DQ4,DQ3,DQ2,DQ1,DQ0};
                    // Drive RWDS=0 during CA to prevent the RWDS wire from
                    // floating (no pad pull resistor in the fixture).
                    // Driving 0 signals "no additional latency from device side";
                    // the controller uses en_latency_additional=1 (SW-set) for
                    // double latency regardless.
                    rwds_drv <= 1'b1;
                    rwds_val <= 1'b0;
                end

                // ----------------------------------------------------------------
                // StCA: collect remaining 5 CA bytes (half_cnt 1..5)
                //
                // At half_cnt 5 (last CA byte):
                //  • latch the R/W bit (ca_reg[47] stable since half_cnt 0)
                //  • for writes: release RWDS/DQ so the controller can drive them
                // ----------------------------------------------------------------
                StCA: begin
                    case (half_cnt)
                        1: ca_reg[39:32] <= {DQ7,DQ6,DQ5,DQ4,DQ3,DQ2,DQ1,DQ0};
                        2: ca_reg[31:24] <= {DQ7,DQ6,DQ5,DQ4,DQ3,DQ2,DQ1,DQ0};
                        3: ca_reg[23:16] <= {DQ7,DQ6,DQ5,DQ4,DQ3,DQ2,DQ1,DQ0};
                        4: ca_reg[15: 8] <= {DQ7,DQ6,DQ5,DQ4,DQ3,DQ2,DQ1,DQ0};
                        5: begin
                            ca_reg[ 7: 0] <= {DQ7,DQ6,DQ5,DQ4,DQ3,DQ2,DQ1,DQ0};
                            // ca_reg[47] was set at half_cnt=0 (StIdle) via
                            // non-blocking, already stable → safe to read here.
                            is_read_reg <= ca_reg[47];
                            state       <= StLatency;
                            // Writes: release bus now; controller drives RWDS as
                            // byte-mask and DQ as write data from Write state.
                            // (Controller asserts RWDS_OE at WaitLatAccess→Write
                            //  transition, ~half_cnt 24-25, well after this point.)
                            if (!ca_reg[47]) begin
                                rwds_drv <= 1'b0;
                                dq_drv   <= 1'b0;
                            end
                        end
                        default: ;
                    endcase
                end

                // ----------------------------------------------------------------
                // StLatency: hold RWDS=0 (for reads) and wait.
                //
                // The decode edge is chosen to be falling (even half_cnt) so that
                // the first data-phase edge is always rising (odd half_cnt), giving
                // correct DDR byte alignment.
                //
                // Write decode at half_cnt = HalfLatencyWrite-1 (default 26, even).
                // Read  decode at half_cnt = HalfLatencyRead -1 (default 28, even).
                // ----------------------------------------------------------------
                StLatency: begin
                    // Write decode: enter StWriteData, address decoded.
                    if (!is_read_reg && (half_cnt == HalfLatencyWrite - 1)) begin
                        word_addr <= {ca_reg[44:16], ca_reg[2:0]};
                        state     <= StWriteData;
                        $display("[%0t] %s: WRITE word_addr=0x%08h",
                                 $time, DebugName,
                                 {ca_reg[44:16], ca_reg[2:0]});
                    end
                    // Read decode: pre-fetch first word, enter StReadData.
                    if (is_read_reg && (half_cnt == HalfLatencyRead - 1)) begin
                        word_addr <= {ca_reg[44:16], ca_reg[2:0]};
                        rd_data   <= mem_read({ca_reg[44:16], ca_reg[2:0]});
                        state     <= StReadData;
                        dq_drv    <= 1'b1;
                        // RWDS stays 0; first rising edge below will take it to 1.
                        $display("[%0t] %s: READ  word_addr=0x%08h",
                                 $time, DebugName,
                                 {ca_reg[44:16], ca_reg[2:0]});
                    end
                end

                // ----------------------------------------------------------------
                // StReadData: drive RWDS (DDR data strobe) and DQ
                //
                //  CK↑ (CK==1): RWDS 0→1, DQ = high byte of current word.
                //    Controller latches DQ into fifo_in[15:8] on delayed RWDS ↑.
                //  CK↓ (CK==0): RWDS 1→0, DQ = low byte, advance word address.
                //    Controller writes FIFO on delayed RWDS ↓.
                //
                // DQ and RWDS change at the same CK edge.  The 2 ns RWDS delay
                // (t_rx_clk_delay_phy1=8 → 2 ns) means the controller captures
                // the new DQ value 2 ns after the stub sets it — well within the
                // 3 ns CK half-period (PHY_TCK=6 ns).
                // ----------------------------------------------------------------
                StReadData: begin
                    if (CK) begin
                        // Rising edge: RWDS goes high, present high byte
                        rwds_val <= 1'b1;
                        dq_val   <= rd_data[15:8];
                    end else begin
                        // Falling edge: RWDS goes low, present low byte,
                        //               pre-fetch next word
                        rwds_val  <= 1'b0;
                        dq_val    <= rd_data[7:0];
                        rd_data   <= mem_read(word_addr + 1);
                        word_addr <= word_addr + 1;
                    end
                end

                // ----------------------------------------------------------------
                // StWriteData: sample DDR DQ and RWDS byte-disable mask
                //
                //  CK↑ (CK==1): controller drives high byte on DQ and its mask
                //               on RWDS (1 = mask this byte, do NOT write).
                //  CK↓ (CK==0): controller drives low byte; assemble and store
                //               the 16-bit word with byte masking applied.
                // ----------------------------------------------------------------
                StWriteData: begin
                    if (CK) begin
                        wr_hi_byte <= {DQ7,DQ6,DQ5,DQ4,DQ3,DQ2,DQ1,DQ0};
                        wr_hi_dis  <= RWDS;
                    end else begin : blk_write
                        automatic logic [7:0]  lo_byte = {DQ7,DQ6,DQ5,DQ4,DQ3,DQ2,DQ1,DQ0};
                        automatic logic        lo_dis  = RWDS;
                        automatic logic [15:0] wr_word = mem_read(word_addr);
                        if (!wr_hi_dis) wr_word[15:8] = wr_hi_byte;
                        if (!lo_dis)    wr_word[ 7:0] = lo_byte;
                        mem[word_addr] = wr_word;
                        $display("[%0t] %s:       data=0x%04h mask=%b%b word_addr=0x%08h",
                                 $time, DebugName, wr_word,
                                 wr_hi_dis, lo_dis, word_addr);
                        word_addr <= word_addr + 1;
                    end
                end

                default: ;

            endcase
        end
    end // proc_ck_sm

endmodule : hyperbus_ram_stub
