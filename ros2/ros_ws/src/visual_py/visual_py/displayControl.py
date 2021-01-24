#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import os
import sys
import threading
import time

# ROS 2 Imports
import rclpy
from rclpy.node import Node, SrvTypeResponse

from std_msgs.msg import String, Float32, UInt8, Int16, ColorRGBA, Bool
from systemcore.msg import I2Cwrite8, I2Cwrite16, I2CwriteArray 
from visual.msg import Polygon, PolygonPoint # DisplayConnectedPixel, Point, DisplayPixel


def toUint8(x):
    return x % 256

def toUint16(x):
    return x % (256 * 256) 

def uint16ToBytes(x):
    return [int(x >> 8), int(x & 0x00FF)]

# See: https://icampusnet.th-wildau.de/gitlab/ros-e/tischroboter-doku/wikis/System/Arduino-I2C-Kommandos
class Commands():

    CMD_DISPLAY_CLEAR = 0x10
    CMD_DISPLAY_SET_PIXEL = 0x11

    CMD_DISPLAY_UINT8_100VI_EYE_SINGLE = 0x20
    CMD_DISPLAY_UINT8_100VI_EYE_START_SEQ = 0x21
    CMD_DISPLAY_UINT8_100VI_EYE_CONT_SEQ = 0x22
    CMD_DISPLAY_UINT8_100VI_EYE_END_SEQ = 0x23

    CMD_DISPLAY_UINT8_100VI_IRIS_SINGLE = 0x28
    CMD_DISPLAY_UINT8_100VI_IRIS_START_SEQ = 0x29
    CMD_DISPLAY_UINT8_100VI_IRIS_CONT_SEQ = 0x2A
    CMD_DISPLAY_UINT8_100VI_IRIS_END_SEQ = 0x2B


    CMD_DISPLAY_UINT8_CUSTOMVI_EYE_SINGLE = 0x30
    CMD_DISPLAY_UINT8_CUSTOMVI_EYE_START_SEQ = 0x31
    CMD_DISPLAY_UINT8_CUSTOMVI_EYE_CONT_SEQ = 0x32
    CMD_DISPLAY_UINT8_CUSTOMVI_EYE_END_SEQ = 0x33

    CMD_DISPLAY_UINT8_CUSTOMVI_IRIS_SINGLE = 0x38
    CMD_DISPLAY_UINT8_CUSTOMVI_IRIS_START_SEQ = 0x39
    CMD_DISPLAY_UINT8_CUSTOMVI_IRIS_CONT_SEQ = 0x3A
    CMD_DISPLAY_UINT8_CUSTOMVI_IRIS_END_SEQ = 0x3B


    CMD_DISPLAY_INT16_EYE_SINGLE = 0x40
    CMD_DISPLAY_INT16_EYE_START_SEQ = 0x41
    CMD_DISPLAY_INT16_EYE_CONT_SEQ = 0x42
    CMD_DISPLAY_INT16_EYE_END_SEQ = 0x43

    CMD_DISPLAY_INT16_IRIS_SINGLE = 0x48
    CMD_DISPLAY_INT16_IRIS_START_SEQ = 0x49
    CMD_DISPLAY_INT16_IRIS_CONT_SEQ = 0x4A
    CMD_DISPLAY_INT16_IRIS_END_SEQ = 0x4B
  
    CMD_VIEW_DIRECTION = 0xA0
    CMD_FILL_POLYGON = 0xA8

#define CMD_VIEW_DIRECTION 0xA0


i2cSem = threading.Semaphore()

class Display():
    def __init__(self, parentNode, i2cArrayPublisher, side, arduinoAddress):

        self.arduinoI2C = arduinoAddress
        self.side = side
        self.fill = True

        self.parentNode = parentNode
        self.logger = self.parentNode.get_logger()

        # Messages for set eye pixel
        self.parentNode.create_subscription(Polygon, "visual/display/setEye", self.onSetEye, 10)
        self.parentNode.create_subscription(Polygon, "visual/display/" + self.side + "/setEye", self.onSetEye, 10)

        # Messages for set iris pixel
        self.parentNode.create_subscription(Polygon, "visual/display/setIris", self.onSetIris, 10)
        self.parentNode.create_subscription(Polygon, "visual/display/" + self.side + "/setIris", self.onSetIris, 10)

        # Messages for set polygon fill
        self.parentNode.create_subscription(Bool, "visual/display/setFill", self.onSetFill, 10)
        self.parentNode.create_subscription(Polygon, "visual/display/" + self.side + "/setIris", self.onSetIris, 10)


        # Messages for Display a single pixel on a Display
        # self.create_subscription(DisplayPixel, "visual/display/setPixel", self.onSetPixel, 10)
        # self.create_subscription(DisplayPixel, "visual/display/" + self.side + "/setPixel", self.onSetPixel, 10)

        # Messages for clearing the displays
        # self.create_subscription(UInt8, "visual/display/clear", self.onClearDisplay, 10)
        # self.create_subscription(UInt8, "visual/display/" + self.side + "/clear", self.onClearDisplay, 10)

        # self.pubI2Cwrite8 = self.parentNode.create_publisher(I2Cwrite8, "system/i2c/write8", 10)
        # self.pubI2Cwrite16 = self.parentNode.create_publisher(I2Cwrite16, "system/i2c/write16", 10)
        self.pubI2CwriteArray = i2cArrayPublisher

        ################################
        ### LOG INFO ###################
        self.logger.info("################### Init {} display node ################### ".format(self.side))
        self.logger.info("Subscribed: visual/display/setEye        |  Msg: visual/DisplayConnectedPixel")
        self.logger.info("Subscribed: visual/display/{}/setEye  |  Msg: visual/DisplayConnectedPixel".format(self.side))

        self.logger.info("Subscribed: visual/display/setIris       |  Msg: visual/DisplayConnectedPixel")
        self.logger.info("Subscribed: visual/display/{}/setIris |  Msg: visual/DisplayConnectedPixel".format(self.side))

        self.logger.info("Subscribed: visual/display/setPixel        |  Msg: visual/DisplayPixel")
        self.logger.info("Subscribed: visual/display/{}/setPixel  |  Msg: visual/DisplayPixel".format(self.side))

        self.logger.info("Subscribed: visual/display/clear         |  Msg: std_msgs/UInt8")
        self.logger.info("Subscribed: visual/display/{}/clear   |  Msg: std_msgs/UInt8".format(self.side))

        self.logger.info("--------------------------------------------------------------------------")
        # self.logger.info("Publish on: system/i2c/write8      |  Msg: system/I2Cwrite8")
        # self.logger.info("Publish on: system/i2c/write16     |  Msg: system/I2Cwrite16")
        self.logger.info("Publish on: system/i2c/writeArray  |  Msg: system/I2CwriteArray")

        self.logger.info("Started {} Display Control Node".format(self.side))

    # def transmitPixelData(self, msg, cmdControlData, cmdData):
    
    #     i2cSem.acquire()
        
    #     time.sleep(0.01)    

    #     # Transmit the control data
    #     o = I2CwriteArray()
    #     o.address = self.arduinoI2C
    #     o.command = cmdControlData
    #     o.data = [int(msg.curve), int(msg.animation), int(msg.value), int(len(msg.points))]

    #     self.pubI2CwriteArray.publish(o)
    #     time.sleep(0.01)

    #     # Transmit the pixel data
    #     o = I2CwriteArray()
    #     o.address = self.arduinoI2C
    #     o.command = cmdData

    #     data = []

    #     for point in msg.points:
    #         data.append(int(point.row))
    #         data.append(int(point.col))

    #     o.data = data
    #     self.pubI2CwriteArray.publish(o)

    #     i2cSem.release()

    def publish_i2c_array(self, cmd, data):
        i2cSem.acquire()

        o = I2CwriteArray()
        o.address = self.arduinoI2C
        o.command = cmd
        o.data = data

        self.pubI2CwriteArray.publish(o)
        i2cSem.release()


    def publishPoints(self, points, viewInfluence, seqNumber, finalSeq=True, type="eye"):

        data = []

        command = 0

        is_uint8 = True
        customVI = viewInfluence != 100

        ### Check if uint8 data ###
        for p in points:
            if p.x < 0 or p.x > 255: is_uint8 = False
            if p.y < 0 or p.y > 255: is_uint8 = False
            if p.cx < 0 or p.cx > 255: is_uint8 = False
            if p.cy < 0 or p.cy > 255: is_uint8 = False

        if is_uint8:
            if customVI:
                command = Commands.CMD_DISPLAY_UINT8_100VI_EYE_SINGLE
            else:
                command = Commands.CMD_DISPLAY_UINT8_CUSTOMVI_EYE_SINGLE
                data.append(viewInfluence)
        else:
            command = Commands.CMD_DISPLAY_INT16_EYE_SINGLE
            data.append(viewInfluence)

        if type == "iris":   # If eye cmd stay as it is, otherwise increase by 8
            command += 0x08

        if finalSeq:
            if seqNumber == 0:      # single sequence
                pass # No cmd increasind
            else:                   # end sequences
                command += 0x03
        else:  # k of n sequences
            if seqNumber == 0:      # beginning sequence
                command += 0x01
            else:                   # continuing sequence
                command += 0x02

        if is_uint8:
            for p in points:
                data.append(toUint8(p.x))
                data.append(toUint8(p.y))
                data.append(toUint8(p.c))
                data.append(toUint8(p.cx))
                data.append(toUint8(p.cy))
        else:
            for p in points:
                data += uint16ToBytes(toUint16(p.x))
                data += uint16ToBytes(toUint16(p.y))
                data.append(toUint8(p.c))
                data += uint16ToBytes(toUint16(p.cx))
                data += uint16ToBytes(toUint16(p.cy))

        self.publish_i2c_array(command, data)


    def handlePolygonMessage(self, msg, type="eye"):
        points = msg.points

        if len(points) <= 0: return 

        currentVI = points[0].view_influence
        seqNumber = 0

        pointsToPublish = []
        
        for p in points:
            if (p.view_influence == currentVI):
                pointsToPublish.append(p)
            else:
                self.publishPoints(points=pointsToPublish, viewInfluence=currentVI, 
                                    seqNumber=seqNumber, finalSeq=False, type=type )
                pointsToPublish.clear()
                seqNumber += 1

                pointsToPublish.append(p)
                currentVI = p.view_influence
        
        self.publishPoints(points=pointsToPublish, viewInfluence=currentVI, 
                        seqNumber=seqNumber, finalSeq=True, type=type )


    def onSetEye(self, msg):
        self.logger.info("Set {} eye --> points: {}".format(self.side, msg.points))
        self.handlePolygonMessage(msg, type = "eye")


    def onSetIris(self, msg):
        self.logger.info("Set {} iris --> points: {}".format(self.side, msg.points))
        self.handlePolygonMessage(msg, type = "iris")

    def onSetFill(self, msg):
        self.fill = msg.data
        self.logger.info("Set {} polygon fill --> {}".format(self.side, self.fill))
        self.publish_i2c_array(Commands.CMD_FILL_POLYGON, [0x01 if self.fill else 0x00])

    def onShutdown(self):
        pass

  # def onSetPixel(self, msg):
  #   self.get_logger().info("Set new pixel on {} display: {}, {}, {}".format(self.side, msg.r, msg.c, msg.v))

  #   o = I2CwriteArray()
  #   o.address = self.arduinoI2C
  #   o.command = 0x71
  #   o.data = [int(msg.r), int(msg.c), int(msg.v)]

  #   self.pubI2CwriteArray.publish(o)


    # def onClearDisplay(self, msg):
    #     self.get_logger().info("Clear {} display: {}".format(self.side, msg.data))

    #     o = I2Cwrite8()
    #     o.address = self.arduinoI2C
    #     o.command = 0x70
    #     o.data = msg.data

    #     self.pubI2Cwrite8.publish(o)
    #     time.sleep(0.01)

class DisplayControl(Node):
    def __init__(self):
        super().__init__('displayControl_node')

        self.i2cArrayPublisher = self.create_publisher(I2CwriteArray, "system/i2c/writeArray", 10)

        self.nodeLeft = Display(self, i2cArrayPublisher=self.i2cArrayPublisher, side = "left", arduinoAddress = 0x0A)
        self.nodeRight = Display(self, i2cArrayPublisher=self.i2cArrayPublisher, side = "right", arduinoAddress = 0x0B)


    
    def onShutdown(self):
        self.nodeLeft.onShutdown()
        self.nodeRight.onShutdown()

def main(args=None):

    rclpy.init(args=args) # 'displayControl_node'

    node = DisplayControl()

    rclpy.spin(node)
    node.onShutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()