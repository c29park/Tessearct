# test_tt_um_example.py
#
# Cocotb-based Tiny Tapeout testbench for tt_um_example.
# Verifies that for each ui_in switch pattern, the design:
#   - comes out of reset cleanly (no X/Z on outputs),
#   - produces toggling VGA hsync/vsync over time.

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

# ~25 MHz pixel clock -> 40 ns period
CLK_PERIOD_NS = 40


async def reset_dut(dut):
    """Active-low reset pulse."""
    dut.rst_n.value = 0
    await Timer(2 * CLK_PERIOD_NS, units="ns")
    await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    # give one more clock edge after de-assert
    await RisingEdge(dut.clk)


async def run_for_switch(dut, switch_value: int, cycles: int = 2000):
    """
    Drive a specific ui_in value, reset, then run and sanity-check outputs.
    Ensures:
      - no X/Z on uo_out, uio_out, uio_oe
      - hsync and vsync both toggle at least once
    """
    dut.ui_in.value = switch_value

    # pulse reset so internal state is deterministic for this switch setting
    await reset_dut(dut)

    # let things settle a bit after reset
    for _ in range(10):
        await RisingEdge(dut.clk)

    # track whether syncs ever toggle
    hsync_seen_change = False
    vsync_seen_change = False

    # read initial outputs
    val = dut.uo_out.value
    assert val.is_resolvable, "uo_out has X/Z right after reset"
    prev_int = int(val)
    prev_hsync = (prev_int >> 7) & 1  # uo_out[7] = hsync
    prev_vsync = (prev_int >> 4) & 1  # uo_out[4] = vsync

    for _ in range(cycles):
        await RisingEdge(dut.clk)

        # no X/Z on any outputs
        assert dut.uo_out.value.is_resolvable, "uo_out has X/Z"
        assert dut.uio_out.value.is_resolvable, "uio_out has X/Z"
        assert dut.uio_oe.value.is_resolvable, "uio_oe has X/Z"

        cur_int = int(dut.uo_out.value)
        cur_hsync = (cur_int >> 7) & 1   # uo_out[7] = hsync
        cur_vsync = (cur_int >> 4) & 1   # uo_out[4] = vsync

        if cur_hsync != prev_hsync:
            hsync_seen_change = True
            prev_hsync = cur_hsync

        if cur_vsync != prev_vsync:
            vsync_seen_change = True
            prev_vsync = cur_vsync

    assert hsync_seen_change, "hsync never toggled during run"
    assert vsync_seen_change, "vsync never toggled during run"


@cocotb.test()
async def test_each_switch(dut):
    """
    Tiny Tapeout cocotb test for tt_um_example.

    For each of the 8 switches (ui_in[0]..ui_in[7]):
      * drive only that bit high
      * reset the design
      * run for a while
      * ensure outputs are well-defined (no X/Z)
      * ensure VGA syncs are toggling
    Also tests the 'all switches off' case.
    """

    # start clock
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, units="ns").start())

    # static inputs for Tiny Tapeout wrapper
    dut.ena.value = 1          # design enabled
    dut.uio_in.value = 0       # unused in your RTL

    # also test the "all switches off" case
    dut._log.info("Testing ui_in = 0x00 (all switches off)")
    await run_for_switch(dut, 0x00)

    # now test each individual switch high
    for i in range(8):
        value = 1 << i
        dut._log.info(f"Testing ui_in bit {i} high (0x{value:02X})")
        await run_for_switch(dut, value)
