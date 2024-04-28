This is a 2023 student project controlled by a nucleo board.
The led array consists of 128 WS2812B addressable leds connected in series in an arrangement of 16 columns and 8 rows. To clarify, there are 8 strips of 16 leds connected so that each row starts from the same side.

1. a Python script converts the bitmap images into an RGB bitstream sent via the UART to the board. 
2. the microcontroller then takes the stream and translates it into an SPI signal sent to the leds so that the timing of the '1', '0' and 'reset' leds match.

In python folder there are examples of images made in GIMP.
Make sure to chose the right COM port in python script.

https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf          //LEDs datasheet
