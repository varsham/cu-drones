# cu-drones

ESP32-S3 flight controller using an ICM-42688-P over SPI (SCK 12, MISO 13, MOSI 11, CS 10).

`bringup/` gets data off the sensor, `plotting/` streams it and plots it. `~old_attitude/` is earlier attitude work kept for reference.

next: flash `plotting/cube/cube.ino`, close the serial monitor, run `python3 plotting/cube/cube_viewer.py <port>` (or `sim` with no board). rock the board around one axis at a time and write down which axis moves and which direction is positive.
