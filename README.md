# SMOOTHIEWARE-CHMT
This is a special Smoothieware firmware for the control board of the Charmhigh CHM-T36VA, CHM-T48VB and equivalent.
It is based on an old STM32 port of Smmothieware and specially tailored for the Charmhigh mainboard.

Precompiled firmware is available in the STM32F407xG folder. To flash the mainboard, a full chip erase is required. While flashing, the vacuum pump and the blower will run at 100% making some noise. This does not harm for the time the flashing takes. This firmware supports M115, which is used by OpenPnP for detection. Based this this, Issues & Solutions supports mostly automatic configuration. A sample machine.xml is available in this repository. It may serve as a quick start, however using Issues & Solutions on a fresh installation (no machine.xml in ~/.openpnp2) is recommended.

### !! Attention !! 
This version has not yet been tested on CHM-T48VB (with RS422). If you have any test results, please report. The last known good commit is [f306fb](https://github.com/janm012012/Smoothieware-CHMT/tree/f306fb6256647447e799c124dffefa7ebee5d7d8).

## New features:
* Command M119.1 added to just read the status of the drag pin. Returns "ok[01]" and requires the ACTUATOR_READ_REGEX `^ok(?<Value>\d)`.
* M817 (drag pin release) changed to return error and send the controller into HALT state if drag pin does not signal up after timeout. The error state is detected by OpenPnP if COMMAND_ERROR_REGEX is set to `^.*(error|!!).*` and it is acknowledged and released by adding M999 as first to CONNECT_COMMAND and reconnecting to the machine using the power button (this is required as OpenPnP can not continue otherwise). 
* Rhe response to M114 (Position report) has been compressed. This requires an updates POSITION_REPORT_REGEX in OpenPnep like `^okC:X:(?<X>-?\d+\.\d+)Y:(?<Y>-?\d+\.\d+)Z:(?<Z>-?\d+\.\d+)A:(?<A>-?\d+\.\d+)B:(?<B>-?\d+\.\d+)C:(?<C>-?\d+\.\d)D:(?<D>-?\d+\.\d)`. The response to M119 has been compressed as well, which requires a modified ACTUATOR_READ_REGEX of the drag pin actuator like `^okX_min:\dY_min:\dZ_min:\dpins-\(X\)P\d.\d+:\d\(Y\)P\d.\d+:\d\(Z\)P\d.\d+:\d\(Z\)P\d.\d+:(?<Value>\d)`
* rts_cts_handshake can now have different values: 0: default serial configuration without RTS/CTS, 1: UART2 only with vespamans RTS/CTS configuration detailed below, 2: USART2 only for CHMT-36VA with RTS on TX of second serial connector
* Serial bitrate up to 4Mbit with or without RTS/CTS (with DMA)
* Removed machine coordination of actuators in FW, letting OpenPnP deal with them as it should be. Original code always waited for machine to be still before enabling e.g. down led.
* Improved vacuum sensing. With all the speed-up, the vacuum sensing gave away that it dit not actually update the value very often (only every 50ms), rendering it useless. Now it is updated every millisecond.
* Selective (configurable) machine coordination for switches (currently enabled for vacuum valves and drag pin) added
* Drag pin inactivation are now handled smartly by actually sensing if the pin is up before sending acknowledge; this means that static delays are no longer needed in OpenPnP setup, and a much more robust drag pin operation.
* Anti Stiction Wiggle (ASW). If drag pin gets stuck, FW automatically tries to free it by quickly moving the drag pin in a X/Y back/forth/left/righ pattern until it is free (or give up if it is not freed). At any time ASW has been engaged, the 'ok' back to OpenPnP has a comment attached to it, detailing what the ASW result was - e.g. "2023-12-13 11:22:54.328 GcodeDriver$ReaderThread TRACE: [GcodeDriver:ttyUSB0] << ok ; ASW: l1,t4 (G1 X-0.1 Y-0.05)".
* Firmware management of drag pin PWM. The drag pin needs management of the power supplied to the solenoid to prevent it from burning. The FW deal with it by itself, instead of having OpenPnP sending the different levels. In order to have the firmware dealing with it, just send a M816 without any argument. If an 'S' argument is supplied, this will set aside/override the firmware management. So be careful - if 'S' argument (e.g. 'S 100') is sent, the firmware will not manage the current, and OpenPnP _needs_ to send e.g. 'S 10' in order to not burn the coil. Please note that a small delay (~10ms) is still required for the pin to go down.

ASW is dependant on the smart drag pin activation code. Both can be enabled by adding a new property "switch.dragpin.dragpin true" to the group of switch.dragpin in the config.default. This property tells the generic code, that this pin is connected to a drag pin, and to activate the advanced mechanisms. Setting its value to false disables any advanced logic.

Hardware flow control can be disabled by setting rts_cts_handshake to false in config.defaults. So in theory, this branch should work with stock machine, up to 115200 Baud (CHM-T36), as long as confirmation flow control is still enabled in OpenPnP. (115200 comes from the limitation of the rs232 level shifter, U32, populated on the controller board). A CHM-T48 should be able to achieve 480kBaud, limited by the rs422 interface driver.

In order to benefit from higher throughput and hardware flow control, you will need to modify your control board.
The changes needed, can be defined in two groups; one for the actual hardware flow control, and one for increased bitrate.
The latter probably needs the former to be useful.

This guide presumes that you want to go all in, and go for logic signalling levels directly to the isolators. Normally 3.3V signalling level.

Both 36 and 48 models share the same control board, with a little difference; the 48 has a native rs422 interface populated, whereas the 36 has rs232.
#### For the 48 models, the following needs to be done
* Add a 1+1 channel isolator chip to unpopulated position U28 e.g. ADUM121N0BRZ-RL7. Note; put a piece of kapton tape on pads 2 & 3, since they shall not be soldered to the board pads, but instead be connected to the rts/cts wires described below.
* Pull two wires from an unpopulated U18 pin 4 & 1 (SO8) (rs485 transceiver) to pin 2 & 3 of the new isolator chip (not to the pads!). These are the RTS/CTS signals.
* Position U32 is made for a rs232 level shifter. The rx/tx/rts/cts path needs to be connected by wires, since we don't want 232 levels, in order to achieve higher speeds. See picture. 
* Add a 4 pole through hole mount JST connector (board uses China brand Yeonho Electronics SMW250/SMH250 throughout, so if you have those at hand that would be nicer, but the JST are very similar in all aspects but the locking).
* Move the 0R resistor from position R132 to position R131. This will connect the rx signal from the rs232 input instead of the original rs422.
* Remove (or keep) the ESD protection network close to the connectors. (See benefits below).

#### For the 36 models, which already have the rs232 you need to
* disconnect pin 2 & 3 on U28 from board pads.
* Pull two wires from an unpopulated U18 pin 4 & 1 (SO8) (rs485 transceiver) to pin 2 & 3 of U28 (not to the pads!). These are the RTS/CTS signals.
* Remove U32 (see above) and connect 4 wires in its place.
* Remove (or keep) the ESD protection network close to the connectors. (See benefits below).

#### For both machines
* The USB-serial adapter is best kept as close to the controller board as possible, this is especially true if you decide to go for 3.3V signalling since it is very fast and more susceptible to external noise. 
* I have tested several USB-serial adapters (bridges), and found that the ones based on XR21B1420 (and siblings) works best (tested in linux only) because of very low latency. So far I have been using a XR21B1420IL28-0A-EVB, that I have stuffed just beside the control board with short wires. The serial adapter needs to power the isolators (needs be the same Voltage as the signalling level of the serial bridge) e.g. 3.3V. Others usb-serial bridges may work, but introduce serious delays for short messages.
* The ESD protection components are meant to protect the interface. This is needed especially if RS232 signalling levels are selected, and the RS232 are pulled outside the chmt casing. If 3,3V signalling levels, you probably should remove them, since the USB interface will be the interface to the outer world, and normally it already has protection. I have not tested to run my board with the ESD components fitted, so I don't know if it will work with them in place. But remember that original machine was for 115kbits, now we are running several mbits. If you remove them, you still need to make sure to put 0R resistors or solder blobs to complete the signal path.
* Option: If you like to stay with rs232 levels for whatever reason, you can populate U32 with e.g. SN65C3232EDR instead, which will allow speeds up to 921600bps. You then also will need to add a few SMD 100nF caps around U32 on the unpopulated positions.

#### How to configure the serial port 
If you did add RTS/CTS signals above;
In config.default there are two relevant lines; one for specifying the baud rate you wish to run, and one setting for enabling RTS/CTS hardware flow control.
In OpenPnP you will need to select RTS/CTS flow control, and uncheck the "Confirmation Flow Control" since we will not need it any more.

If you have a standard board, you need to set the baud rate to 115200/460k and set flow control to false in the config.defaults.

A picture of the patch prior removing the rs232 (U32) chip;
![rts_cts_patch](https://user-images.githubusercontent.com/18227864/158996475-5d222994-015a-4fb8-b81a-a45bb956cf9d.jpg)

## Notes
This version contains a few modifications with respect to the original code by Matt
* c-riegel's mods to support advanced motion in OpenPnp
* c-riegel's tweaked feed rate and acceleration limits
* increased z limits to allow the z axis to move to its physical limits
* vacuum and blower configuration changed to 16kHz pwm (M808/M810 S<percent>)
* drap bin configuration changed to 16kHz pwm (M816 S<percent>)
* buzzer configured (on: M820, off: M821)
* default build changed to PAXIS=7
* planner queue increased to 128
* lighting for down-looking camera added (via OT2, 16kHz pwm, M822 S<percent>)

Current build status: {{https://travis-ci.org/Smoothieware/Smoothieware.svg?branch=edge}}

## Old STM32/CHMT Notes from upstream
To build, follow normal smoothie build process to get setup.  Then checkout chmt branch and rebuild.

### Port Status:
* mbed hooks - Added, compiles, tested
* stm32f4xx libs - Added, compiles, tested
* timers - Ported, compiles, tested
* wdt - Ported, compiles, tested
* gpio - Ported, compiles, tested
* adc - Ported, compiles, tested
* pwm - Ported, compiles, tested
* build scripts - Added, project builds successfully

### CHMT Status:
* config file - 48VB Complete
* pin map - 48VB/36VA Complete
* operation/verfication - All System Functions Operational (excl. axis encoders)
* machine testing - 48VB All Systems Operational

### TODO:
* DONE: Target initialization and board bringup (clocks, mpu, etc)
* DONE: Verification of ported peripherals (step generation, watchdog, gpios)
* DONE: Debug/Comm uart setup
* DONE: CHMT controller specific configuration

### Notes/Caveats/Gotchas:
* smoothie mbed was ancient, so the oldest stm32 mbed available was integrated to reduce friction -- incompatibilities, and bugs from dated mbed may have been introduced
* MRI (gdb over serial) is not supported on stm32, use SWD/JTAG
* config file must be hardcoded into firmware build

### Next Steps/Priority
* CHMT Pinout Reversing -- Complete
* CHMT Config File Development -- 48VB Complete
* CHMT Machine Testing -- All Base Functions Operational, Long term and stability testing required.
* Synchronize System GCODEs to OpenPNP standards
* Stability Testing Required.
* WDT rewrite for longer timeout

# Smoothie

## Overview
Smoothie is a free, opensource, high performance G-code interpreter and CNC controller written in Object-Oriented C++ for the LPC17xx micro-controller ( ARM Cortex M3 architecture ). It will run on a mBed, a LPCXpresso, a SmoothieBoard, R2C2 or any other LPC17xx-based board. The motion control part is a port of the awesome grbl.

Documentation can be found here : [[http://smoothieware.org/]]

NOTE it is not necessary to build Smoothie yourself unless you want to. prebuilt binaries are available here: [[http://triffid-hunter.no-ip.info/Smoothie.html|Nightly builds]] and here: [[https://github.com/Smoothieware/Smoothieware/blob/edge/FirmwareBin/firmware.bin?raw=true|recent stable build]]

## Quick Start
These are the quick steps to get Smoothie dependencies installed on your computer:
* Pull down a clone of the Smoothie github project to your local machine.
* In the root subdirectory of the cloned Smoothie project, there are install scripts for the supported platforms.  Run the install script appropriate for your platform:
** Windows: win_install.cmd
** OS X: mac_install
** Linux: linux_install
* You can then run the BuildShell script which will be created during the install to properly configure the PATH environment variable to point to the required version of GCC for ARM which was just installed on your machine.  You may want to edit this script to further customize your development environment.

## Building Smoothie
Follow this guide... [[http://smoothieware.org/compiling-smoothie]]

In short...
From a shell, switch into the root Smoothie project directory and run:
{{{
make clean
make all
}}}

To upload you can do

{{{
make upload
}}}

if you have dfu-util installed.

Alternatively copy the file LPC1768/main.bin to the sdcard calling it firmware.bin and reset.

## Filing issues (for bugs ONLY)
Please follow this guide [[https://github.com/Smoothieware/Smoothieware/blob/edge/ISSUE_TEMPLATE.md]]

## Contributing

Please take a look at : 

* http://smoothieware.org/coding-standards
* http://smoothieware.org/developers-guide
* http://smoothieware.org/contribution-guidlines

Contributions very welcome !

## Donate
The Smoothie firmware is free software developed by volunteers. If you find this software useful, want to say thanks and encourage development, please consider a 
[[https://www.paypal.com/cgi-bin/webscr?cmd=_donations&business=9QDYFXXBPM6Y6&lc=US&item_name=Smoothieware%20development&currency_code=USD&bn=PP%2dDonationsBF%3abtn_donate_SM%2egif%3aNonHosted|Donation]]

## License

Smoothieware is released under the GNU GPL v3, which you can find at http://www.gnu.org/licenses/gpl-3.0.en.html


