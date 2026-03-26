// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

`timescale 1 ns/1 ps

// Dual-PHY testbench with one real HyperRAM and one behavioral stub.
//
// Purpose
// -------
// This testbench replaces the PHY 1 s27ks0641 device with hyperbus_ram_stub.
// It exercises the same AXI traffic as the standard hyperbus_tb.sv so that
// any dual-PHY problem caused specifically by the simultaneous use of two
// real HyperRAM devices can be isolated from problems in the controller
// architecture itself.
//
// Comparison:
//   hyperbus_tb.sv          — both PHYs use real s27ks0641 (production scenario)
//   hyperbus_tb_phy1_stub.sv — PHY 0 real, PHY 1 stub (debug / isolation scenario)
//
// If this testbench passes and hyperbus_tb.sv fails, the root cause lies in
// the interaction between two real HyperRAM devices (e.g. timing, bus loading,
// etc.), not in the dual-PHY controller logic.

module hyperbus_tb_phy1_stub;

    localparam NumPhys = 2;

    fixture_hyperbus_phy1_stub #(
        .NumChips(2),
        .NumPhys (NumPhys)
    ) fix ();

    logic error;

    initial begin
        fix.reset_end();
        #500us;

        // =====================================================================
        // Register configuration (identical to hyperbus_tb.sv)
        // =====================================================================

        // Enable additional latency (en_latency_additional = 1, reg 0x4)
        fix.i_rmaster.send_write('h4, 'h1, '1, error);
        // Reduce max burst length to 250 (t_burst_max, reg 0x8)
        fix.i_rmaster.send_write('h8, 'd250, '1, error);

        #200ns;

        $display("=================");
        $display("128 BIT MEGABURST");
        $display("=================");

        fix.write_axi('ha00, 4090, 4, 0, 'hffff);
        fix.read_axi ('ha00, 4090, 4);

        $display("=================");
        $display("128 BIT BURSTS");
        $display("=================");

        fix.write_axi('h110, 3, 4, 0, 'hffff);
        fix.read_axi ('h110, 3, 4);

        $display("=================");
        $display("64 BIT MEGABURST ");
        $display("=================");

        fix.write_axi('ha00, 4090, 3, 0, 'hffff);
        fix.read_axi ('ha00, 4090, 3);

        $display("=================");
        $display("64 BIT BURSTS");
        $display("=================");

        fix.write_axi('h210, 0, 3, 0, 'hffff);
        fix.read_axi ('h210, 0, 3);

        fix.write_axi('h228, 3, 3, 0, 'hffff);
        fix.read_axi ('h228, 3, 3);

        #1471ns;

        $display("=================");
        $display("32 BIT MEGABURST ");
        $display("=================");

        fix.write_axi('ha00, 4090, 2, 0, 'hffff);
        fix.read_axi ('ha00, 4090, 2);

        $display("=================");
        $display("32 BIT BURSTS");
        $display("=================");

        fix.write_axi('h304, 1, 2, 0, 'hffff);
        fix.read_axi ('h304, 1, 2);

        fix.write_axi('h314, 5, 2, 0, 'hffff);
        fix.read_axi ('h314, 5, 2);

        $display("=================");
        $display("16 BIT BURSTS");
        $display("=================");

        fix.write_axi('h410, 18, 1, 0, 'hffff);
        fix.read_axi ('h410, 18, 1);

        fix.write_axi('h402, 5, 1, 0, 'hffff);
        fix.read_axi ('h402, 5, 1);

        fix.write_axi('h470, 5, 1, 0, 'hffff);
        fix.read_axi ('h470, 5, 1);

        fix.write_axi('h452, 6, 1, 0, 'hffff);
        fix.read_axi ('h452, 6, 1);

        $display("=================");
        $display("8 BIT BURSTS");
        $display("=================");

        fix.write_axi('h500, 5, 0, 0, 'hffff);
        fix.read_axi ('h500, 5, 0);

        fix.write_axi('h513, 25, 0, 0, 'hffff);
        fix.read_axi ('h513, 25, 0);

        $display("=================");
        $display("128 BIT ALIGNED ACCESSES");
        $display("=================");

        fix.write_axi('h100, 0, 4, 'hbad0_beef_cafe_dead_b00b_8888_7777_aa55, 'hffff);
        fix.read_axi ('h100, 0, 4);

        $display("=================");
        $display("64 BIT ALIGNED ACCESSES");
        $display("=================");

        fix.write_axi('h00, 0, 3, 0, 'h00ff);
        fix.write_axi('h08, 0, 3, 0, 'hff00);
        fix.read_axi ('h00, 0, 3);
        fix.read_axi ('h08, 0, 3);

        #2557ns;

        for (int p = 0; p < 100; p++) begin
            fix.write_axi(p*8, 0, 3, 0, 'hffff);
            fix.read_axi (p*8, 0, 3);
        end

        fix.write_axi('h00, 100, 3, 1, 'hffff);
        fix.read_axi ('h0,  100, 3);

        $display("=================");
        $display("32 BIT ALIGNED ACCESSES");
        $display("=================");

        fix.write_axi('h10, 0, 2, 0, 'h000f);
        fix.write_axi('h14, 0, 2, 0, 'h00f0);
        fix.write_axi('h18, 0, 2, 0, 'h0f00);
        fix.write_axi('h1c, 0, 2, 0, 'hf000);
        fix.read_axi ('h10, 0, 2);
        fix.read_axi ('h14, 0, 2);
        fix.read_axi ('h18, 0, 2);
        fix.read_axi ('h1c, 0, 2);

        $display("=================");
        $display("16 BIT ALIGNED ACCESSES");
        $display("=================");

        fix.write_axi('h20, 0, 1, 0, 'h0003);
        fix.write_axi('h22, 0, 1, 0, 'h000c);
        fix.write_axi('h24, 0, 1, 0, 'h0030);
        fix.write_axi('h26, 0, 1, 0, 'h00c0);
        fix.write_axi('h28, 0, 1, 0, 'h0300);
        fix.write_axi('h2a, 0, 1, 0, 'h0c00);
        fix.write_axi('h2c, 0, 1, 0, 'h3000);
        fix.write_axi('h2e, 0, 1, 0, 'hc000);
        fix.read_axi ('h20, 0, 1);
        fix.read_axi ('h22, 0, 1);
        fix.read_axi ('h24, 0, 1);
        fix.read_axi ('h26, 0, 1);
        fix.read_axi ('h28, 0, 1);
        fix.read_axi ('h2a, 0, 1);
        fix.read_axi ('h2c, 0, 1);
        fix.read_axi ('h2e, 0, 1);

        $display("=================");
        $display("8 BIT ALIGNED ACCESSES");
        $display("=================");

        fix.write_axi('h30, 0, 0, 0, 'h0001);
        fix.write_axi('h31, 0, 0, 0, 'h0002);
        fix.write_axi('h32, 0, 0, 0, 'h0004);
        fix.write_axi('h33, 0, 0, 0, 'h0008);
        fix.write_axi('h34, 0, 0, 0, 'h0010);
        fix.write_axi('h35, 0, 0, 0, 'h0020);
        fix.write_axi('h36, 0, 0, 0, 'h0040);
        fix.write_axi('h37, 0, 0, 0, 'h0080);
        fix.write_axi('h38, 0, 0, 0, 'h0100);
        fix.write_axi('h39, 0, 0, 0, 'h0200);
        fix.write_axi('h3a, 0, 0, 0, 'h0400);
        fix.write_axi('h3b, 0, 0, 0, 'h0800);
        fix.write_axi('h3c, 0, 0, 0, 'h1000);
        fix.write_axi('h3d, 0, 0, 0, 'h2000);
        fix.write_axi('h3e, 0, 0, 0, 'h4000);
        fix.write_axi('h3f, 0, 0, 0, 'h8000);
        fix.read_axi ('h30, 0, 0);
        fix.read_axi ('h31, 0, 0);
        fix.read_axi ('h32, 0, 0);
        fix.read_axi ('h33, 0, 0);
        fix.read_axi ('h34, 0, 0);
        fix.read_axi ('h35, 0, 0);
        fix.read_axi ('h36, 0, 0);
        fix.read_axi ('h37, 0, 0);
        fix.read_axi ('h38, 0, 0);
        fix.read_axi ('h39, 0, 0);
        fix.read_axi ('h3a, 0, 0);
        fix.read_axi ('h3b, 0, 0);
        fix.read_axi ('h3c, 0, 0);
        fix.read_axi ('h3d, 0, 0);
        fix.read_axi ('h3e, 0, 0);
        fix.read_axi ('h3f, 0, 0);

        $display("=================");
        $display("COMBINED");
        $display("=================");

        fix.write_axi('h800, 0, 4, 1, 'hffff);
        fix.read_axi ('h800, 0, 4);
        fix.write_axi('h800, 0, 4, 0, 'hc03f);
        fix.read_axi ('h800, 0, 4);
        fix.write_axi('h806, 0, 1, 0, 'h00c0);
        fix.write_axi('h80a, 0, 1, 0, 'h0c00);
        fix.write_axi('h80e, 0, 1, 0, 'hc000);
        fix.read_axi ('h806, 0, 1);
        fix.read_axi ('h80a, 0, 1);
        fix.read_axi ('h80e, 0, 1);

        $display("=================");
        $display("UNALIGNED");
        $display("=================");

        fix.write_axi('h900, 10, 4, 1, 'hffff);
        fix.read_axi ('h900, 10, 4);

        fix.write_axi('h902, 2, 2, 0, 'hF0FF);
        fix.read_axi ('h902, 2, 2);

        fix.write_axi('h90a, 9, 2, 0, 'hF0FF);
        fix.read_axi ('h90a, 9, 2);

        fix.write_axi('h910, 10, 3, 1, 'hffff);
        fix.read_axi ('h910, 10, 3);
        fix.write_axi('h91C, 0, 3, 0, 'hFF0F);
        fix.read_axi ('h91C, 0, 3);

        fix.write_axi('h990, 5, 3, 1, 'hFFFF);
        fix.read_axi ('h990, 5, 3);
        fix.write_axi('h992, 4, 3, 0, 'hFF0F);
        fix.read_axi ('h992, 4, 3);

        fix.write_axi('h924, 0, 3, 0, 'hF0FF);
        fix.read_axi ('h924, 0, 3);

        fix.write_axi('h930, 0, 4, 0, 'hFFFF);
        fix.read_axi ('h930, 0, 4);

        fix.write_axi('h954, 0, 4, 1, 'hFFFF);
        fix.read_axi ('h954, 0, 4);

        fix.write_axi('h954, 0, 4, 0, 'hFFFF);
        fix.read_axi ('h954, 0, 4);

        fix.write_axi('h978, 0, 4, 1, 'hFFFF);
        fix.read_axi ('h978, 0, 4);

        fix.write_axi('h978, 0, 4, 0, 'hFFFF);
        fix.read_axi ('h978, 0, 4);

        fix.write_axi('h1c02, 4, 4, 0, 'hFFFF);
        fix.read_axi ('h1c02, 4, 4);

        fix.write_axi('ha00, 4090, 4, 0, 'hffff);
        fix.read_axi ('ha00, 4090, 4);

        fix.write_axi('ha00, 4090, 3, 0, 'hffff);
        fix.read_axi ('ha00, 4090, 3);

        fix.write_axi('ha00, 4090, 2, 0, 'hffff);
        fix.read_axi ('ha00, 4090, 2);

        $display("==============================");
        $display("AXI DONE WITH SUCCESS! (stub) ");
        $display("==============================");

        #5us;
        $stop();
    end

endmodule : hyperbus_tb_phy1_stub
