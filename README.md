# LCD controller

This isn't exactly polished. This directory was originally named
`lcdctrl.X` and worked with the MPLAB X IDE that Microchip provides
for working with AVR chips. It targets an ATTiny3216. No effort
has been made to make it portable.

This includes a copy of Peter Fleury's LCD library for
AVR chips.

	http://www.peterfleury.epizy.com/avr-software.html?i=1

My code is available for anyone who wants it with no warranty
of any kind, but also no restrictions or requirements; the LCD
library is GPLv3.

This implements an I2C-bus listener which can drive character
LCD displays. Unlike the GPIO-based backpacks some people make,
this one uses a very simple protocol and does not require the caller
to know about or care about the timing of the LCD display controller.

The protocol is, in essence, that any character outside the ASCII
control character range is the instruction to draw that character
at the current cursor position, the 0x0-0xF characters are customizable
graphics characters, and some of 0x10-0x1F are commands like
"activate display", "change backlight brightness", or "move cursor".

