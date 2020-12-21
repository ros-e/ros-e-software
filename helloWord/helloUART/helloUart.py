#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Hello World for using UART

short intro taken from: https://pythonhosted.org/pyserial/shortintro.html

see: https://electrosome.com/uart-raspberry-pi-python/ for using UART over the pins
see: https://pythonhosted.org/pyserial/ for documentation of pyserial
"""

import serial

# ser = serial.Serial ("/dev/ttyAMA0")    # Open named port (UART over GPIO pins if these pins are enabled for UART and named)
ser = serial.Serial ("/dev/ttyUSB0") # Open USB-UART

ser.baudrate = 115200  # Set baud rate to 115200
data = ser.read(10)    # Read ten characters from serial port to data
ser.write(data)        # Send back the received data


ser.close()            # Close UART

