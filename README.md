
# Project Title

A brief description of what this project does and what it's for

Display build from an addressable led strip and controlled by a nucleo board which takes data from computer or onboard memory. The display consists of 128 RGB addressable LEDs connected in series, arranged in 8 rows and 16 columns

Program principle goes like this:  
1. Python script converts bitmap images into an RGB bitstream, which is sent via UART to the microcontroller.
2. Microcontroller processes the stream and translates it into values that control the LEDs accordingly. These values are stored in a buffer, which is then transmitted to the LED strip using the SPI protocol.

This approach allows the display to render any bitmap image consisting of 128 pixels 2:1 ratio. For example images were created using GIMP, an open-source editor. By designing a series of similar images and managing their transmission, it was possible to create smooth animations on the display.

In python folder there are examples of images made in GIMP. Make sure to chose the right COM port in python script.

https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf - LEDs datasheet
