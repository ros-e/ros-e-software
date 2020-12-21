#!/usr/bin/env python

"""
Python2 HelloWorld for I2C Access

Taken from 
http://www.netzmafia.de/skripten/hardware/RasPi/RasPi_I2C.html 
and
https://raspberry-projects.com/pi/programming-in-python/i2c-programming-in-python/using-the-i2c-interface-2 


"""

import smbus # System management bus ==> I2C compatible
import time

""" 
Some addresses of the system. 
I2C Addresses --> 7 Bit Length 
"""

JETSON_I2CADDRESS = 0x00

ARDUINO_I2CADDR = 0x08  

AUDIO_MAX9744_I2CADDR = 0x4B # https://learn.adafruit.com/adafruit-20w-stereo-audio-amplifier-class-d-max9744/digital-control

OLED_DISPLAY_1_I2CADDR = 0x3C # https://www.adafruit.com/product/938 
OLED_DISPLAY_2_I2CADDR = 0x3D


""" 
Create smbus instance and open the instance.
The jetson nano has two I2C busses.

I2C Bus 0:
SDA --> Pin 27
SCL --> Pin 28

I2C Bus 1:
SDA --> Pin 3
SCL --> Pin 5

If there are any I2C devices attached, you can scan that bus from the command line
$ i2cdetect -y -r 0 
$ i2cdetect -y -r 1
"""

bus = smbus.SMBus(1)

### Address of the slave device
addr = 0x08

### Example values
cmd = 0x55        # Command or register
byteVal = 0x1A    # byte value
wordVal = 0xABCD  # 2 byte value
listVal = [5, 10, 15, 50]  # value list

### Write operations
bus.write_byte(addr, byteVal)
bus.write_byte_data(addr, cmd, byteVal)
bus.write_word_data(addr, cmd, wordVal)
bus.write_block_data(addr, cmd, listVal)

### Read operations
readByte = bus.read_byte(addr)
readByte = bus.read_byte_data(addr, cmd)
readWord = bus.read_word_data(addr, cmd)
readList = bus.read_block_data(addr, cmd)


""" List of smbus commands

Send only the read / write bit 
long write_quick(int addr)

Read a single byte from a device, without specifying a device register. 
long read_byte(int addr)

Send a single byte to a device (without command or register)
long write_byte(int addr, char val)

Read a single byte from a device (cmd is the command or register declaration)
long read_byte_data(int addr, char cmd)

Send a single byte to a device (cmd is the command or register declaration)
long write_byte_data(int addr, char cmd, char val)
 
Read a 16 Bit word from a device (cmd is the command or register declaration)
long read_word_data(int addr, char cmd)

Send a 16 Bit word from to a device (cmd is the command or register declaration)
long write_word_data(int addr, char cmd, int val)

Read a block of data from a device (cmd is the command or register declaration)
long[] read_block_data(int addr, char cmd)

Send a block of data to a device (cmd is the command or register declaration).
The data block should be maximum 31 Byte.
The function adds a length byte before the data bytes.
write_block_data(int addr, char cmd, long vals[])

Process Call transaction
long process_call(int addr, char cmd, int val)

Block Process Call transaction
long[] block_process_call(int addr, char cmd, long vals[])

Read a block of raw data from a device (cmd is the command or register declaration)
long[] read_i2c_block_data(int addr, char cmd)

Send a block of raw data to a device (cmd is the command or register declaration).
write_i2c_block_data(int addr,char cmd, long vals[])

"""


