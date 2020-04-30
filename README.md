# ball_catcher

NOTE: This setup uses an EV3, Raspberry Pi 3b+, and a PC with a camera

Due to not having a serial adapter available for my PC. Serial communications
are sent over bluetooth to the bluetooth virtual serial port on a raspberry pi.
A small script on the raspberry pi relays everything in the bluetooth port out
through the GPIO serial pins on the pi, and into the EV3.

drop_catcher.py or throw_catcher.py runs on the PC, ev3_main.py runs on the ev3
