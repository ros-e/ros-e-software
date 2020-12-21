#!/usr/bin/env python

"""
This helloWorld can communicate directly with a connected display over i2c.

The frequency of i2c and thus the update frequency of the display is quite slow, so that you can see an obvious update process on the display.
One complete refresh needs about 200ms --> 5 Hz 

It's better to use a separate Arduino Nano and just tell him what to do. 
An Arduino reaches about 30 Hz update rate.

So this HelloWorld just shows the principle, but it's not recommended.

"""

from SSD1306 import SSD1306, CMDs
# from GFX import GFX

import random
import time

HEIGHT = 64
WIDTH = 128

OLED_DISPLAY_1_I2CADDR = 0x3C # https://www.adafruit.com/product/938 
OLED_DISPLAY_2_I2CADDR = 0x3D


addr = OLED_DISPLAY_2_I2CADDR

# Init display
display = SSD1306(WIDTH, HEIGHT, 1, OLED_DISPLAY_2_I2CADDR)
display.begin()
print "Started display test at " + str(addr)

# Clear display
display.clearDisplay()
display.display()


x = 1

# Loop to show random black and white pixels
while True:
  x += 1
  for i in range(20):
    display.drawPixel(random.randint(0, WIDTH), random.randint(0, HEIGHT), CMDs.WHITE)
  
  for i in range(20):
    display.drawPixel(random.randint(0, WIDTH), random.randint(0, HEIGHT), CMDs.BLACK)
  

  display.display()

  print(x)
  time.sleep(0.1)



# Loop to set each update cycle a new pixel
xP = 0
yP = 0
color = CMDs.WHITE
while True:
  display.drawPixel(xP, yP, CMDs.WHITE)
  display.display()
  xP += 1
  if (xP > WIDTH):
    xP = 0
    yP += 1
    if (yP > HEIGHT):
      yP = 0
      if (color == CMDs.WHITE):
        color = CMDs.BLACK
      else: color = CMDs.WHITE






