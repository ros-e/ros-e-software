#!/usr/bin/env python

"""
Class to communicate with the OLED Display Control chip (SSD1306)
datasheet: http://www.adafruit.com/datasheets/SSD1306.pdf

Adapted from https://github.com/adafruit/Adafruit_SSD1306 

"""

from GFX import GFX
from i2cWrapper import I2C
import time

class CMDs():
  # Commands for communication with the SSD1306 Display control chip
  BLACK =                          0 # ///< Draw 'off' pixels
  WHITE =                          1 # ///< Draw 'on' pixels
  INVERSE =                        2 # ///< Invert pixels

  SSD1306_MEMORYMODE =          0x20 # ///< See datasheet
  SSD1306_COLUMNADDR =          0x21 # ///< See datasheet
  SSD1306_PAGEADDR =            0x22 # ///< See datasheet
  SSD1306_SETCONTRAST =         0x81 # ///< See datasheet
  SSD1306_CHARGEPUMP =          0x8D # ///< See datasheet
  SSD1306_SEGREMAP =            0xA0 # ///< See datasheet
  SSD1306_DISPLAYALLON_RESUME = 0xA4 # ///< See datasheet
  SSD1306_DISPLAYALLON =        0xA5 # ///< Not currently used
  SSD1306_NORMALDISPLAY =       0xA6 # ///< See datasheet
  SSD1306_INVERTDISPLAY =       0xA7 # ///< See datasheet
  SSD1306_SETMULTIPLEX =        0xA8 # ///< See datasheet
  SSD1306_DISPLAYOFF =          0xAE # ///< See datasheet
  SSD1306_DISPLAYON =           0xAF # ///< See datasheet
  SSD1306_COMSCANINC =          0xC0 # ///< Not currently used
  SSD1306_COMSCANDEC =          0xC8 # ///< See datasheet
  SSD1306_SETDISPLAYOFFSET =    0xD3 # ///< See datasheet
  SSD1306_SETDISPLAYCLOCKDIV =  0xD5 # ///< See datasheet
  SSD1306_SETPRECHARGE =        0xD9 # ///< See datasheet
  SSD1306_SETCOMPINS =          0xDA # ///< See datasheet
  SSD1306_SETVCOMDETECT =       0xDB # ///< See datasheet

  SSD1306_SETLOWCOLUMN =        0x00 # ///< Not currently used
  SSD1306_SETHIGHCOLUMN =       0x10 # ///< Not currently used
  SSD1306_SETSTARTLINE =        0x40 # ///< See datasheet

  SSD1306_EXTERNALVCC =         0x01 # ///< External display voltage source
  SSD1306_SWITCHCAPVCC =        0x02 # ///< Gen. display voltage from 3.3V

  SSD1306_RIGHT_HORIZONTAL_SCROLL =              0x26 # ///< Init rt scroll
  SSD1306_LEFT_HORIZONTAL_SCROLL =               0x27 # ///< Init left scroll
  SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL = 0x29 # ///< Init diag scroll
  SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL =  0x2A # ///< Init diag scroll
  SSD1306_DEACTIVATE_SCROLL =                    0x2E # ///< Stop scroll
  SSD1306_ACTIVATE_SCROLL =                      0x2F # ///< Start scroll
  SSD1306_SET_VERTICAL_SCROLL_AREA =             0xA3 # ///< Set scroll range



class SSD1306(GFX):
  def __init__(self, w, h, i2cBus, i2cAddress):
    GFX.__init__(self, w, h)
    self.i2c = I2C(i2cBus, i2cAddress)
    time.sleep(0.5)
    

  """
  Send a list of commands to the display
  """
  def ssd1306_command(self, command):
    self.i2c.writeCmdByte(0x00, command)

  """
  Send a single command to the display
  """
  def ssd1306_commandList(self, commands):
    self.i2c.writeCmdBlockData(0x00, commands)

  """
  Sequence to initialize the display
  """
  def begin(self, vccstate = CMDs.SSD1306_SWITCHCAPVCC):
    
    # Init sequence
    init1 = [
      CMDs.SSD1306_DISPLAYOFF,            # // 0xAE
      CMDs.SSD1306_SETDISPLAYCLOCKDIV,    # // 0xD5
      # 0x80,                               # // the suggested ratio 0x80
      0xF0,                               # // the suggested ratio 0x80
      CMDs.SSD1306_SETMULTIPLEX           # // 0xA8
      
      ]              
    self.ssd1306_commandList(init1)
    self.ssd1306_command(self.HEIGHT - 1 )

    init2 = [
      CMDs.SSD1306_SETDISPLAYOFFSET,    # // 0xD3
      0x0,                              # // no offset
      CMDs.SSD1306_SETSTARTLINE | 0x0,  # // line #0
      CMDs.SSD1306_CHARGEPUMP           # // 0x8D
      
    ]                
    self.ssd1306_commandList(init2)
    self.ssd1306_command(0x10 if vccstate == CMDs.SSD1306_EXTERNALVCC else 0x14)

    init3 = [
      CMDs.SSD1306_MEMORYMODE,                   # // 0x20
      0x00,                                 # // 0x0 act like ks0108
      CMDs.SSD1306_SEGREMAP | 0x1,
      CMDs.SSD1306_COMSCANDEC 
    ]
    self.ssd1306_commandList(init3)

    if((self.WIDTH == 128) and (self.HEIGHT == 32)):
      init4a = [
        CMDs.SSD1306_SETCOMPINS,                 # // 0xDA
        0x02,
        CMDs.SSD1306_SETCONTRAST,                # // 0x81
        0x8F 
      ]
      self.ssd1306_commandList(init4a)

    elif((self.WIDTH == 128) and (self.HEIGHT == 64)):
      init4b = [
        CMDs.SSD1306_SETCOMPINS,      # // 0xDA
        0x12,
        CMDs.SSD1306_SETCONTRAST     # // 0x81
        
      ]              
      self.ssd1306_commandList(init4b)
      self.ssd1306_command(0x9F if vccstate == CMDs.SSD1306_EXTERNALVCC else 0xCF)
      
    elif((self.WIDTH == 96) and (self.HEIGHT == 16)):
      init4c = [
        CMDs.SSD1306_SETCOMPINS,    # // 0xDA
        0x2,                        # // ada x12
        CMDs.SSD1306_SETCONTRAST    # // 0x81
      ]              
      self.ssd1306_commandList(init4c)
      self.ssd1306_command(0x10 if vccstate == CMDs.SSD1306_EXTERNALVCC else 0xAF)

    else:
      pass
      # // Other screen varieties -- TBD
    
    self.ssd1306_command(CMDs.SSD1306_SETPRECHARGE) # // 0xd9
    self.ssd1306_command( 0x22 if vccstate == CMDs.SSD1306_EXTERNALVCC else 0xF1)

    init5 = [
      CMDs.SSD1306_SETVCOMDETECT,           # // 0xDB
      0x40,
      CMDs.SSD1306_DISPLAYALLON_RESUME,     # // 0xA4
      CMDs.SSD1306_NORMALDISPLAY,           # // 0xA6
      CMDs.SSD1306_DEACTIVATE_SCROLL,
      CMDs.SSD1306_DISPLAYON                # // Main screen turn on
    ]                 
    self.ssd1306_commandList(init5)

    return True # // Success
    

  """
  Show the buffer on the display 
  """
  def display(self):
    dlist1 = [ CMDs.SSD1306_PAGEADDR,
              0, # Page start address
              0xFF, # Page end (not really, but works here)
              CMDs.SSD1306_COLUMNADDR,
              0
              #,self.WIDTH - 1
    ]
    
    self.ssd1306_commandList(dlist1)
    self.ssd1306_command(self.WIDTH - 1)

    # Send buffer
    self.i2c.writeCmdBlockData(0x40, self.buffer)

  

         
