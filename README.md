This is code to control a hacked hooverboarf via CRSF (ExpressLRS) or Wiimote input, with the primary payload being a lawnmower.

This file is most of my notes regarding stuff maybe not obvious from the code.

# Hardware

## Hooverboarf

### Control board

GD32 or STM32 based mono-boards are most hackable.

Split mainboards are less well known.

#### Power button

Power button is apparently wired to the battery through a voltage divider (30k, 5k resistors).

Schematics indicate they want to see about 2.65v max at the switch pin 1 (pulled to ground).

If going from 3.3v, a 500 ohm resistor in series should produce the same thing.  Untested.

#### Don't use PWM control input

The board design is really noisy and the PWM implementation in the open source firmware is pretty badly mangled, so just don't go there.

Even the serial ports are susceptible, however they implement checksums.  My stats indicate only like %6 of serial packets even pass checksum.  Soo...  yeah I'm basically burning a lot of power just to fail.

### Motors

350W

16mm axles; various axial lengths.

Various tire sizes.  You really, really want 8.5" tires for this conversion.

Kind of a challenge to mount.

Fair warning:  if you have a level lawn this is fine, but if you have a sloped or bumpy lawn two motors might not cut it.

### Battery

Typically come in 7s (25.2 nominal) or 10s (36v nominal) Li-Ion configurations.  Super dangerous.

XT60 connector.  No antispark devices.  Carnies.  Circus folk.  Small hands.  Smell like cabbage.  Might want to precharge caps before plugging in.

### Charging Connector

Connected to battery through a diode, so no pulling power from this port.  Only charging.

## Ryobi 40v Brushless Push Mower

Ryobi has too many confusing model numbers I'm not even going to try.  I have the one that looks like it was designed by a retarded 6 year old daydreaming about being an F1 driver.  Keep dreaming those big dreams, little guy.

This might not have been the best choice for this project, but whatevs.

### Front Wheels

Nominal 8", more like 7.75".

11mm OD shaft with M8 lock nut.

Wheelbase is about 16.75".

### Rear Wheels

10" with a 17.5" wheelbase.

11mm OD shaft, M8 lock nut.

### Single-point adjustment

With 8" front wheels and 10" rear wheels, in order to get even height change through a single adjuster some dimension has to lose.

That dimension is the span between front and rear wheels.  It goes from something like 24.75" to 25.5".

With a true fixed spacing/mount, you can't actually hit all possible adjustment points, although if you aim for the middle, the slack in the plastic frame will let you hit most of them.

### Handle

Cable going from the main mower to the handle has three pins -- yellow (bale), black (ground), red (power button).

The entire assembly can be removed by pulling the M8 bolts at the base, however you have to cut the cable somewhere.  In addition to the bale and the power switch, there's one more handle extension switch (which often goes bad) somewhere near the extension lock with the cable going through it.

The official arming sequence is:  bale goes first, then power button.

What appears to happen is the yellow wire is hooked up to full battery voltage (42v max!!) through a resistor+capacitor.

At rest, the red wire has nothing on it.

When the bale is pulled (yellow) to ground, the LEDs light up and 5v appears on the red wire.

When the red wire is then pulled to ground the mower starts.

### Mulching Plug

Big dumb hunk of plastic; mower will not start unless it senses the metal seated in the main body.

## MH-ET ESP32 Minikit

It's an ESP32 (the bad rev) on a PCB that is vaguely Wemos-D1-Mini-esque.

Wants 5v (which it regulates to 3.3v).

Special thanks to Espressif for not documenting Vdss max in open drain mode.  Espressif:  forcing customers to implement hardware solutions to documentation problems since 200-always.

Anyways, I had to add two external FETs to pull the mower controls to ground to start the mower.

### Wemos D1 Mini Proto Shield

This fits on the Minikit.

### Wemos D1 OLED Shield

Displays convenient data (raw inputs, temp, voltage) and provides two buttons.

### UARTS

The original reason for all this to exist was to translate serial between CRSF and a hooverboarf.

#### UART0

Default pins:  GPIO1 (TX), GPIO3 (RX)

This is used for flashing and serial monitor so no touché.  Also cannot be remapped.

#### UART1

Default pins:  GPIO10 (TX), GPIO9 (RX)

These pins are often used for SPI flash, however the Minikit doesn't have SPI flash.

#### UART2

Default pins:  GPIO17 (TX) GPIO16 (RX)

Free to use.

But does not exist on ESP8266.  Maybe softserial?

### GPIO

GPIO15 and under are busy and/or have WPU/WPD.

GPIO6 through GPIO11 are connected to SPI flash.

GPIO5 outputs PWM at boot?

GPIO34 through GPIO39 are input only

I2C pins are sitting on GPIO21 and GPIO22

### The Big Map

Gonna try to stick to ESP8266 pins as much as possible for the remote possibility of using one, plus those are the pins available on the proto shield.

UART1 GPIO19 (RX), GPIO18 (TX) hooked up to CRSF serial source.

UART2 GPIO17 (RX), GPIO16 (TX) hooked up to hooverboarf.

GPIO23 Mower bale pin.
GPIO5 Mower power pin.  GPIO5 I guess waggling at powerup is OK because GPIO23 is also "pulled open".

## HappyModel EP1/EP2

Runs ExpressLRS.  Wants 5v.

## 5v Regulator

Might want another regulator to switch fans on/off.

If powering direct off main voltage, make sure it can take it (10s max voltage is 42v!).

Can possibly power off the side board ports, however those are 14.5v 200ma max (total!).

# Software

## esp32-elrs-crsf-to-pwm

Pinched this code, the core of which was apparently pinched from PX4.

## EFeru FOC Hoverboard Firmware

I pretty much don't touch this code; just compile it and peek at the source/docs for protocol and...  weirdness.

Started with the PWM variant but switched to USART for checksums and feedback.

## ExpressLRS

Yo dawg I herd u liek long range RC.

## Arduino

Blech.  I feel like we're teaching an entire generation of future programmers all the wrong lessons.

## ESP32Wiimote

Dropped my X-Lite in the sandy loam multiple times.  Do you know how hard it is to clean that off that already annoying rubberized texture?  Also thanks FrSky for doing stuff like not building in a lanyard attachment and hosing your own product compatibility..  Everybody hates you now.  You destroyed all the goodwill you had built up in the community.  Nobody likes you.  How does that feel?

## CircularBuffer

Parsing the hooverboarf serial feedback.

# Todo/Options

## Flight controller

The original plan was to actually have a Betaflight controller in here but:

1.  Would have required fixing/modifying open drain support with Pinio.
2.  Didn't have any appropriate FCs laying around.
3.  I did have 3 Minikits laying around.
4.  Was going to just have it pump out PWM to the hooverboarf but PWM turned out to be a total cluster anyways.

## Additional controls

Separate hooverboarf powerup pin.

Separate fan power pin.

Need a smol, cheap stepdown regulator that will handle at least 42v.

## T-Watch 2020 interface

Might want some debug to pop out on that interface, maybe even a controller.

## Telemetry might be neat

Not looking forward to that though.

## Talking back through the Wiimote might also be fun

Bluetooth sucks.

## Solar charger

Need one of those boost chargers, but not like 1000W huge, just more like 50-100W.

Can go in through the charger port.  Duh.

# Random Sad Notes

Minikit 3.3v regulator appears to be insufficient; will get constant brownouts using the radio without a big cap on 3.3v line.

Either the libs or the hardware for the Wiimote are wonky.  I think nobody was thinking "failsafe".

Bluetooth turned on and reading consumes more power than a literal EP1/2 connected, running and reading.

Bluetooth turned on and reading also blocks WiFi AP mode unless you do some sort of sharing of radio slots.

In fact doing ... anything at all while also running WiFi caused it to crash in target.  So switching it on is a one-way ticket for everybody involved.
