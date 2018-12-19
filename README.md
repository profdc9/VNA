This a project to create a Vector Network Analyzer based on the EU1KY antenna tuner that can be reconfigured for different needs.  This material is copyrighted by Daniel Marks (KW4TI) and licensed under the Creative Commons License CC-BY-SA 4.0.  The software is licensed under the zlib license.  These are open source licenses that permit commercial use, modification and redistribution with attribution, and require licensees to assume all risk for using the copyrighted materials.

Currently what is on this github are the Kicad files for the VNA and the software.  The software uses the "stm32duino" extension to the Arduino IDE.  The Arduino IDE can be downloaded from

https://www.arduino.cc/en/Main/Software

The stm32duino software can be installed using the instructions on the stm32duino web site

http://wiki.stm32duino.com/index.php?title=Installation

which involves (at this time) downloading the git archive for the stm32duino and placing it under the "hardware" folder in your Arduino folder.

There are several Arduino libraries that are required for the SI5351A frequency synthesizer chip, ILI9341 TFT display module, and XPT2046 touchscreen controller.  These are provided for your use under the "libraries" directory of this archive, but may not be the most recent versions.  You could copy these directories into the "libraries" folder in your Arduino directory.

---------------------------------

The VNA may be used with or without a touchscreen.  If a touchscreen it used, it should be compiled with the "VNA_TOUCHSCREEN" defined in VNA.h.

The touchscreen used is a 2.8" touchscreen with a SPI interface to the ILI9341 and XPT2046.  These are commonly available at a low price.  To connect it to the SPI interface connector header "J1" on the board, use the following pin connections.  I modified Dupont wire jumpers to provide these connections, creating "Y" connectors with these jumpers to join the power and SPI bus lines together.  If a 2.8" size display is used, there are four holes on the PCB, J8, J10, J11, and J12 that may be used to mount the display to the PCB with standoffs, as many displays have this particular mounting hole pattern.

1 GND J1  - GND TFT
2 SCK J1  - ILI9341 SCK,  XPT2046 T_CLK
3 MISO J1 - ILI9341 SDO(MISO),  XPT2046 T_OUT
4 MOSI J1 - ILI9341 SDI(MOSI),  XPT2046 T_IN
5 CS1 J1  - ILI9341 CS
6 CS2 J1  - XPT2046 T_CS
7 CS3 J1  - ILI9341 D/C
8 +5V J1  - VCC TFT, LED TFT, RESET TFT through 100k pull-up resistor

The only pin on the TFT display that is not used is T_IRQ.
