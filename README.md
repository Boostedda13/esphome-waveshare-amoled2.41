# esphome-waveshare-amoled2.41
Configuration for the WaveShare AMOLED 2.41 ESP32-S3 development board 
This is a fairly simple implementation of the WaveShare AMOLED 2.41 inch development board that has the ESP32-S3 microprocessor on it.
I have managed to add in the config for the various components including:

-PSRAM, which is needed for the screen
-SPI in Quad mode, also needed for the screen
-Screen itself using the RM690B0 chip
-I2C, needed for the Touchscreen, Extended IO, RTC and IMU
-The Real Time Clock chip PCF85063
-Touch screen using the FT6336
-Extended IO using the TCA9554 chip, using the PCA9554 driver
-A few GPIOs and Extentended IOs
-The Liion battery charger, which is just some GPIO pins
-The obvious WiFi
-The QMI8658 Inertial Measurement Unit, this was pretty tricky as there isn't a driver for it, so I wrote a basic one

I didn't manage to sort out these bits though

-The SD Card, the concept of external files stores just isn't supported in ESPHome.

For my use case I wanted to turn this into an RGB light controller with a few options, so there is a bunch of code in there associated with that, it has some LVGL examples that may be useful to some people.

This is the first thing I have ever put on github so please don't be too harsh, any suggestions let me know.
