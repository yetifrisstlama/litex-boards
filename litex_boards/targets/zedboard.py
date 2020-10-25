#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2019-2020 Florent Kermarrec <florent@enjoy-digital.fr>,
# SPDX-License-Identifier: BSD-2-Clause

import os
import argparse

from migen import *

from litex_boards.platforms import zedboard
from litex.build.xilinx.vivado import vivado_build_args, vivado_build_argdict

from litex.soc.interconnect import axi
from litex.soc.interconnect import wishbone

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq, use_ps7_clk=False):
        self.clock_domains.cd_sys = ClockDomain()

        # # #

        if use_ps7_clk:
            assert sys_clk_freq == 100e6
            self.comb += ClockSignal("sys").eq(ClockSignal("ps7"))
            self.comb += ResetSignal("sys").eq(ResetSignal("ps7"))
        else:
            self.submodules.pll = pll = S7PLL(speedgrade=-1)
            pll.register_clkin(platform.request("clk100"), 100e6)
            pll.create_clkout(self.cd_sys, sys_clk_freq)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=int(100e6), **kwargs):
        platform = zedboard.Platform()

        if kwargs["uart_name"] == "serial":
            kwargs["uart_name"] = "usb_acm"  # Use USB-UART Pmod on JB.

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident          = "LiteX SoC on Zedboard",
            ident_version  = True,
            **kwargs)

        # Zynq7000 Integration ---------------------------------------------------------------------
        if kwargs.get("cpu_type", None) == "zynq7000":
            # Connect AXI GP0 to the SoC. See UG585 p. 112 for base_address
            wb_gp0  = wishbone.Interface()
            self.submodules += axi.AXI2Wishbone(
                axi          = self.cpu.add_axi_gp_master(),
                wishbone     = wb_gp0,
                base_address = 0x40000000)
            self.add_wb_master(wb_gp0)

        # CRG --------------------------------------------------------------------------------------
        # self.submodules.crg = _CRG(platform, sys_clk_freq)
        self.submodules.crg = _CRG(platform, sys_clk_freq, True)

        # Leds -------------------------------------------------------------------------------------
        self.submodules.leds = LedChaser(
            pads         = platform.request_all("user_led"),
            sys_clk_freq = sys_clk_freq)
        self.add_csr("leds")

        # OLED display -----------------------------------------------------------------------------
        self.cpu.add_emio_gpio()
        self.add_oled()

    def add_oled(self, SPI_N=0, SS_N=0, DC_GPIO=8, RST_GPIO=9):
        '''
        Use fabric to wire-up the on-board OLED display to the Zynq PS

        SPI_N: which SPI peripheral (0 or 1)
        SS_N: which slave select pin (0, 1, 2)
        DC_GPIO: PS GPIO pin to connect to  Data / Command input of display
        RST_GPIO: PS GPIO pin to connect to  Reset input of display

        The configuration in build/ip/gen_ip.tcl will be set accordingly to
        match these parameters
        '''
        self.platform.ps7_cfg['PCW_SPI0_PERIPHERAL_ENABLE'] = '1'
        oled = self.platform.request("zed_oled")
        oled_cs = Signal()
        oled_clk = Signal()
        oled_mosi = Signal()

        self.cpu.cpu_params[f"o_SPI{SPI_N}_SS{SS_N}_O"] = oled_cs
        self.cpu.cpu_params[f"o_SPI{SPI_N}_SCLK_O"] = oled_clk
        self.cpu.cpu_params[f"o_SPI{SPI_N}_MOSI_O"] = oled_mosi
        self.cpu.cpu_params[f"i_SPI{SPI_N}_MISO_I"] = 0

        self.comb += [
            # OLED power always on
            oled.vbat_n.eq(0),
            oled.vdd_n.eq(0),
            # Fake the missing OLED chip select by gating its MOSI and SCLK
            If(oled_cs,
                oled.clk.eq(0),
                oled.mosi.eq(0)
            ).Else(
                oled.clk.eq(oled_clk),
                oled.mosi.eq(oled_mosi)
            ),
            # Share SPI0 SCLK and MOSI
            # D/C = EMIO62
            oled.dc.eq(self.cpu.cpu_params["o_GPIO_O"][DC_GPIO]),
            # RESET_N = EMIO63
            oled.reset_n.eq(self.cpu.cpu_params["o_GPIO_O"][RST_GPIO])
        ]

        # connect unused PS input pins, to avoid vivado critical warnings
        for s, v in zip(["SCLK", "MOSI", "SS"], [0, 0, 1]):
            self.cpu.cpu_params[f"i_SPI{SPI_N}_{s}_I"] = v


# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on Zedboard")
    parser.add_argument("--build", action="store_true", help="Build bitstream")
    parser.add_argument("--load",  action="store_true", help="Load bitstream")
    builder_args(parser)
    soc_core_args(parser)
    vivado_build_args(parser)
    args = parser.parse_args()

    soc = BaseSoC(**soc_core_argdict(args))
    builder = Builder(soc, **builder_argdict(args))
    builder.build(**vivado_build_argdict(args), run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"), device=1)

if __name__ == "__main__":
    main()
