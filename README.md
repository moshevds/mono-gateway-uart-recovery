# Mono Gateway UART Recovery

This tool is a proof-of-concept, and almost entirely LLM-written.

The Mono Gateway DK can be booted via JTAG, which makes it possible to unbrick
the device when no valid firmware remains on neither the SQPI flash or eMMC.
However, JTAG programmers tend to be rather expensive, which can be a roadblock
for do-it-yourself unbricking. OpenOCD also has a generic JTAG driver that can
work with any GPIO chip. The downside with this is that it is very very slow.

If you want to load U-Boot into memory via such a GPIO jtag driver, this can
easily take more than an hour simply to transfer the ~1MiB that it requires.
This PoC takes a different approach, it provides a small stub binary (~2KiB)
that will accept commands over UART. Currently, this can be used to flash the
QSPI NOR flash.

This is a minimal approach that can unbrick the Mono Gateway DK.

# Preparation

If you have a bricked Mono Gateway DK that you want to unbrick yourself, then
you need some tools, which you may or may not have already:

 - A linux computer. (Or other OS that can run OpenOCD.)
 - A USB-GPIO board, such as the CJMCU-2112, with USB cable.
   (An expensive programmer will also work, but then this project is optional.)
 - The Tag-Connect TC2050-IDC to connect to the Mono Gateway DK.
 - Soldering tools.
 - Jumper wires. (Also called DuPont wires or breadboard wires.)

Warning:
 - Sending too high of a voltage into the LS1046A can damage your Gateway DK.
 - Connecting the VCC pins of the Mono Gateway DK and the CJMCU-2112 together
   can also damage your Gateway DK.
 - In general, you yourself are responsible for the way you handle the
   equipment, even if you follow these instructions. These instructions may
   contain mistakes.

The CJMCU-2112 board is cheap and many clones are also available. Usually,
these boards are soldered for 3.3V use, but the Mono Gateway DK requires 1.8V.
Luckily, this is easy to change, but it requires you to remove an SMD 0Ω
resistor and place it back somewhere else. See
https://wiki.wut.ee/arm/cjmcu-2112#switching-from-33v-to-18v for this.

You then need to connect the CJMCU-2112 to a Linux computer via USB, and to the
TC2050-IDC via jumper wires. If you look head-on towards the connector of the
TC2050-IDC cable, then the pin-holes are numbered like this:
```
       +--+       
+------+  +------+
| 1  3  5  7  9  |
| 2  4  6  8  10 |
+----------------+
```
Note: If you look at the ribbon, the red wire is connected to pin 1.

On the CJMCU-2112, the pins are labeled, so that you can connect the right ones
together. It is a good idea to always check that pin 1 is not connected, and
that VCC (on the CJMCU-2112) is not conected either. These pins are powered and
you want to avoid connecting them together.

I suggest wiring it like this:
- pin 1 -> not connected
- pin 2 -> IO6
- pin 3 -> GND#1
- pin 4 -> IO5
- pin 5 -> GND#2
- pin 6 -> INT
- pin 7 -> not connected
- pin 8 -> IO7
- pin 9 -> WAK
- pin 10 -> RST

Because the CJMCU-2112 doesn't really understand JTAG and OpenOCD emulates it
in software, there is some flexibility with how you connect this. The wiring
here matches with the configuration in cp2112-jtag-gpiod.cfg which is in this
repo as well.

If you want to check whether the CJMCU-2112 has been modified correctly for
1.8V, you can check the voltage between VCC and GND.
You can check this on the Mono Gateway as well (between pin and pin 3), but
it is always 1.8V when the device has power.

Once everything is connected together, you can start the unbricking software.

# How to use

If you don't care about recompiling the binaries for the Mono Gateway DK, then
you can just run OpenOCD and the UART recovery application:

In terminal 1:

```
$ openocd \
     -f cp2112-jtag-gpiod.cfg \
     -c "transport select jtag" \
     -f target/ls1046a.cfg \
     -f mono-gateway-unbrick.tcl \
     -c "mono_gateway_unbrick /empty/folder/ /.../mono-uart-recovery-stub.bin"
```

In terminal 2:
```
$ cargo run --bin mono-uart-recovery -- \
     --device /dev/ttyUSB0 \
     --fast-baud 2000000 \
     restore your-mono-gateway-dk-qspi-firmware.bin
```

You likely need to set the correct CP2112_CHIP in cp2112-jtag-gpiod.cfg, and
use correct TTY as the device for mono-uart-recovery.


If something goes wrong during the OpenOCD procedure, then it may sometimes be
necessary to disconnect and reconnect power from the Gateway DK. The reason
for this is that the unbrick script instructs the SoC to go through the reset
sequence step by step. If this is interrupted on the host side, then the SoC
may continue to wait forever for new commands. In those cases, the reset
button is not always enough to get the SoC out of this mode, and the unbrick
script does not check for this condition either.
This can result in a situation where neither OpenOCD nor the SoC know what to
do. Resetting by removing power from the board fixes this issue.
