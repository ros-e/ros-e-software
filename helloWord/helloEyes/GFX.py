#!/usr/bin/env python

"""
Graphics class for drawing on the display
Adapted from https://github.com/adafruit/Adafruit-GFX-Library
See this link for missing drawing methods.

"""

class GFX(object):

  def __init__(self, width, height):
    # Constants, never change
    self.WIDTH = width
    self.HEIGHT = height

    # current values, could change if rotation changes
    self._width = self.WIDTH
    self._height = self.HEIGHT

    # self.buffer = [[0 for x in range( int((self.WIDTH + 7) / 8))] for y in range(self.HEIGHT)] 
    self.buffer = [ 0 for x in range( int(self.WIDTH * ((self.HEIGHT + 7) / 8)))  ]
    # print self.buffer 

    self.rotation  = 0
    self.cursor_y  = 0
    self.cursor_x  = 0
    self.textsize_x = 1
    self.textsize_y = 1
    self.textcolor = 0xFFFF
    self.textbgcolor = 0xFFFF
    self.wrap      = True
    self._cp437    = False
    self.gfxFont   = None
  
  def clearBuffer(self, pattern = 0):
    self.buffer = [ pattern for x in range( int(self.WIDTH * ((self.HEIGHT + 7) / 8)))  ]
    #self.buffer = [[pattern for x in range( int((self.WIDTH + 7) / 8))] for y in range(self.HEIGHT)] 

  def clearDisplay(self, pattern = 0):
    self.clearBuffer(pattern)

  def drawPixel(self, x, y, color):
    if ( (x >= 0) and (x < self._width) and (y >= 0) and (y < self._height) ):
      
      # Process rotation if necessary
      if (self.rotation == 1):
        x, y = y, x
        x = self.WIDTH - x - 1
      elif (self.rotation == 2):
        x = self.WIDTH  - x - 1
        y = self.HEIGHT - y - 1
      elif (self.rotation == 3):
        x, y = y, x
        y = self.HEIGHT - y - 1
      
      #print str(x) + " : " + str(y)

      # Process color
      if color == CMDs.WHITE:    
        self.buffer[ (x + int(y / 8) * self.WIDTH) ] |= (1 << (y & 7))
        # self.buffer[y][int(x / 8)] |= (1 << (x & 7))
      elif color == CMDs.BLACK:    
        # self.buffer[y][int(x / 8)] &= ~(1 << (x & 7))
        self.buffer[ (x + int(y / 8) * self.WIDTH) ] &= ~(1 << (y & 7))
      elif color == CMDs.INVERSE:    
        #self.buffer[y][int(x / 8)] ^= (1 << (x & 7))
        self.buffer[ (x + int(y / 8) * self.WIDTH) ] ^= (1 << (y & 7))

