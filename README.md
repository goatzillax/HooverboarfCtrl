This is code to control a hacked hooverboarf via CRSF (ExpressLRS) or Wiimote input, with the primary payload being a lawnmower.

This file is most of my notes regarding stuff maybe not obvious from the code.

# Hardware

## Hooverboarf

### Control board

GD32 or STM32 based mono-boards are most hackable.

Split mainboards are less well known.

#### Power button

Power button is apparently wired to the battery through a voltage divider (30k, 2k resistors).

Schematics indicate they want to see about 2.625v max at the switch pin 1 (pulled to ground).

If going from 3.3v, a 500 ohm resistor in series should produce the same thing.  Except there is some odd behavior with the power button line...

#### Don't use PWM control input

The board design is really noisy and the PWM implementation in the open source firmware is pretty badly mangled, so just don't go there.

Even the serial ports are susceptible, however they implement checksums.  My stats indicate only like 6% of serial packets even pass checksum.  Soo...  yeah I'm basically burning a lot of power just to fail.

### Motors

200-350W (seems to vary)

16mm axles; various axial lengths.

Various tire sizes.  You really, really want 8.5" tires for this conversion.

Kind of a challenge to mount.

Fair warning:  if you have a level lawn this is fine, but if you have a sloped or bumpy lawn two motors might not cut it.

### No Standards

Apparently there are no standards across (and sometimes within) hooverboarfs.

The motors tend to use female 3.5mm bullet connectors, with the controllers using male.

Sometimes the Hall sensors use JST-SM, sometimes they use JST-XH.

The wire coloring/ordering doesn't appear consistent between between components, however it looks like it's consistent within a component.

Seemingly you can make it work by following the color from the Hall sensor out the motor into the controller, then back out to the same color from controller to the original motor wire color.

i.e. Motor Hall Sensor Green -> PCB Hall Sensor Blue -> PCB Motor Power Wire Blue -> Motor Power Wire Green

Note it conceptually went from Green to Green.


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

### Extended Controls

OK I'm squatting more pins for extended controls.  These are ESP32 only; no more ESP8266.  We're well past that, Jerry.

* GND
* GPIO27 - Hooverboarf main power
* GPIO25 - Fan power
* GPIO22 - Something something something dark side

Nota bene:  My hooverboarf powers up automatically when I turn my ESP32 regulator on.  It...  shouldn't be doing that.  Not sure what the deal is; probably has something to do with some low side switching, I give up.

Fan power I'm just linking to whether or not the hooverboarf is powered up, i.e. if it's receiving feedback packets.  Might be able to PWM the EN of the regulators and tie it to temperature.

## HappyModel EP1/EP2

Runs ExpressLRS.  Wants 5v.

## Regulators

It's possible to power off the side board ports, however those are 14.5v 200ma max (total!), so nix on that.  Seems like unnecessary risk if you're going to run more than just the ESP32.

Getting off-the-shelf regulators that can handle 42v input is kind of a bastard.

So this isn't cool, but I'm using multiple regulators to power/control the peripherals.

### 120v (!) step down to 12v 3A

First regulator to taste the raw 42v looks like this one:

![Yamums a hoebag](/docs/images/reg_12v_front.jpg )
![Yamums a hoebag](/docs/images/reg_12v_back.jpg )

After the voltage has been dropped to 12v it's way easier to find regulators and even switches to handle the rest of the work.

### 4.5-20v step down adjustable 10W

The recent batch of widely available adjustable regulators looks like this:

![Yamums a hoebag](/docs/images/reg_adjust.jpg )

These are vaguely less overtly retarded than their predecessors because:

1.  Only need to desolder a jumper instead of literally trying to cut a tiny trace in order to disable the pot or use the EN port.
2.  TWO ground pads for users.

One regulator is fixed at 5v to power the ESP32 and EP1/2 and permanently enabled.

Two regulators are keep the adjustable pots and the EN port enabled so the ESP32 can turn them on/off.  These control cooling fans:  one for main board, one for battery.

Oh, and the EN port is active-high.  Don't ask me why their picture makes it look like you're supposed to use an open-drain output.

# Software

## esp32-elrs-crsf-to-pwm

Pinched this code, the core of which was apparently pinched from PX4.

## EFeru FOC Hoverboard Firmware

I pretty much don't touch this code; just compile it and peek at the source/docs for protocol and...  weirdness.

Started with the PWM variant but switched to USART for checksums and feedback.

## ExpressLRS

Yo dawg I herd u liek long range RC.

All kidding aside, there is a firmware for the EP1/2 which give it two PWM output channels, which is actually like the bare minimum to control the hooverboarf.  So, u know, good for testing.

## Arduino

Blech.  I feel like we're teaching an entire generation of future programmers all the wrong lessons.

## ESP32Wiimote

Dropped my X-Lite in the sandy loam multiple times.  Do you know how hard it is to clean that off the already annoying rubberized texture?  Also thanks FrSky for doing stuff like not building in a lanyard attachment and hosing your own product compatibility..  Everybody hates you now.  You destroyed all the goodwill you had built up in the community.  Nobody likes you.  How does that feel?

Anyways, I had to modify the lib a little because available() only reported input changes (bad for detecting failsafe) and totally shat itself when it came to disconnecting the nunchuk (which is also bad for detecting failsafe).  So yeah, low confidence.

## CircularBuffer

Parsing the hooverboarf serial feedback.

Kind of would like a shift(int count) for dropping a chunk of the buffer at once, but whatevs.

# Todo/Options

## Flight controller

The original plan was to actually have a Betaflight controller in here but:

1.  Would have required fixing/modifying open drain support with Pinio.
2.  Didn't have any appropriate FCs laying around.
3.  I did have 3 Minikits laying around.
4.  Was going to just have it pump out PWM to the hooverboarf but PWM turned out to be a total cluster anyways.

## T-Watch 2020 interface

Might want some debug to pop out on that interface, maybe even a controller.

## Telemetry might be neat

Not looking forward to that though.

## Talking back through the Wiimote might also be fun

https://wiibrew.org/wiki/Wiimote#Speaker

Bluetooth sucks.

Might go for the rumble as well.

## Solar charger

Need one of those boost chargers, but not like 1000W huge, just more like 50-100W.

Can go in through the charger port.  Duh.

# Random Sad Notes

Minikit 3.3v regulator appears to be insufficient; will get constant brownouts using the radio without a big cap on 3.3v line.

Either the libs or the hardware for the Wiimote are wonky.  I think nobody was thinking "failsafe".

Bluetooth turned on and reading consumes more power than a literal EP1/2 connected, running and reading.

Bluetooth turned on and reading also blocks WiFi AP mode unless you do some sort of sharing of radio slots.

In fact doing ... anything at all while also running WiFi caused it to crash in target.  So switching it on is a one-way ticket for everybody involved.
