# Nogasm

This is a fork of the [original nogasm project](https://github.com/nogasm/nogasm).

## Hardware

### Board

![Board render](img/board.png)

Count | Name | Digikey link | Comment |
-----|--------|-------------|---------|
1 | Teensy LC | https://www.digikey.com/short/31cn1n ||
13 | WS2812b RGB LED | https://www.digikey.com/short/31c04r ||
1 | MP3V5050GP | https://www.digikey.com/short/31c04n ||
1 | PEL12T-4225S-S1024 (COM-10982 encoder, backlit) | https://www.digikey.com/short/31cn19 ||
1 | COM-10597 (encoder knob, transparent) | https://www.digikey.com/short/djdq9jpq ||
1 | MCP6001RT-I/OT | https://www.digikey.com/short/31cnvz ||
1 | CGRA4004-G | https://www.digikey.com/short/31c0mw ||
1 | BUK92150-55A | https://www.digikey.com/short/31cnvb ||
3 | MMBT2222A | https://www.digikey.com/short/bmf4f7fn ||
1 | LM7805CT/NOPB | https://www.digikey.com/short/wcn5wvbh | SMD versions more common now |
2 | 10µF 20V tantalum cap 1206 | https://www.digikey.com/short/4pf7p859 || 
15 | 1µF 16V ceramic cap 0805 | https://www.digikey.com/short/31cdcp ||
1 | 0.33µF 16V ceramic cap 0603 | https://www.digikey.com/short/31cdt4 ||
3 | 10nF 16V ceramic cap 0805| https://www.digikey.com/short/31cd7f ||
1 | 10k Ohm trimmer | https://www.digikey.com/short/31cnvv ||
1 | 15k Ohm resistor 0805 | https://www.digikey.com/short/31c04h ||
2 | 10k Ohm resistor 0805 | https://www.digikey.com/short/31cdt9 ||
2 | 750 Ohm resistor 0805 | https://www.digikey.com/short/31cdtw ||
1 | 300 Ohm resistor 0805 | https://www.digikey.com/short/31c04q ||
2 | 180 Ohm resistor 0805 | https://www.digikey.com/short/31cdtn ||
1 | 4pos DIP switch | https://www.digikey.com/short/31cn1m ||
1 | Barreljack socket 1.3mm/3.5mm | https://www.digikey.com/short/31c0mt ||
1 | Barreljack plug 1.3mm/3.5mm | https://www.digikey.com/short/31c0mj ||
1 | Barreljack socket 2.5mm/5.5mm | https://www.digikey.com/short/31c0mh ||

[You can order all of the above parts in one cart from Digikey](https://www.digikey.com/short/d32z5423)

#### Aisler Kit
[You can order PCBs and/or all parts as a single kit on Aisler](https://aisler.net/DunkelRatte/nogasm/board)


#### Power
You'll need 12V/1.5A DC power supply with a 2.5mm/5.5mm barreljack plug (or fitting to above bigger socket).

### Vibrator
The case is designed for a RS-555 motor. [Aliexpress lists several versions](https://www.aliexpress.com/wholesale?SearchText=rs555).
You'll need to press a weight onto it.

Other 12V DC toys controlable with PWM and using up to 2A should work as well.

### Plug
An inflatable buttplug is used for pressure detection. For example "*Fanny Hill*" by *Seven Creations*.


## Software

### Flashing the teensy

[Install the Teensy Loader Application](https://www.pjrc.com/teensy/loader.html).
After There are two ways to get the firmware onto the device:

#### Flashing the compiled .hex
This is the easiest way. Download the `.hex` file and use the loader to upload the file to the device.

#### Compiling the code yourself
[Download and install Teensyduino](https://www.pjrc.com/teensy/td_download.html).
Afterwards get the sourcecode, compile and upload it via the arduino IDE.
