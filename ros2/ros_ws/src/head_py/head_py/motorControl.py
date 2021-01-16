#!/usr/bin/env python
# -*- coding: UTF-8 -*-

"""
This script controls the head motors

Altered by Johannes Sommerfeldt

"""

import os
import sys

import redis

# ROS 2 Imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String, Int16, Bool
from head.msg import MotorPosition
from systemcore.msg import I2Cwrite8, I2Cwrite16, I2CwriteArray

import time
from threading import Timer
import threading


class Commands():
  """
  This class contains the I2C Commands for the Arduino Motor Control. 
  """
  MOTOR_SET_STIFFNESS = 0x10
  MOTOR_TURN_SET_ABSOLUTE = 0x11
  MOTOR_TURN_SET_RELATIVE = 0x12  # Deprecated
  MOTOR_PITCH_SET_ABSOLUTE = 0x13
  MOTOR_PITCH_SET_RELATIVE = 0x14 # Deprecated
  MOTOR_SET_SPEED = 0x20

class I2cDataConstants():
  """ 
  This class contains constant values that are sent in the data of Arduino commands
  """
  MOTOR_CONTROL_SPEED = 0
  MOTOR_CONTROL_DURATION = 1
  MOTOR_MAX_SPEED = 50      # Motors will move with: (<value> / 10 * msg.speed) pwm per millisecond


class NodeSpinner(threading.Thread):
  def __init__(self, node):
    threading.Thread.__init__(self)
    self.node = node

  def run(self):
     rclpy.spin(self.node)


###########################################################################################################
## Object representing a single motor #####################################################################
###########################################################################################################
class Motor():  # class Motor(Node):
  """
  Object representing a single motor of the robot.
  Each new physical Motor should get its own Motor-Object in this script.
  """

  def __init__(self, parentNode,
    name, redisTopicLastPWM, redisTopicLastAngle, 
    redisKeyMaxPWM, redisKeyMinPWM, redisKey0PWM, redisKey90PWM,
    rosTopicSetPWM, rosTopicSetAngle, rosTopicChangeAngle, rosTopicIsMoving,
    cmdSetAbsolute, cmdSetRelative, i2cAddress, i2cArrayPublisher):
    """Constructor for a Motor Object

        Args:
            parentNode (Node): The ROS2-Node over which subscriptions and publisher are created. Be aware to have only ONE single node instance for every started ROS-node.
            name (String): Name of the motor
            redisTopicLastPWM (String): Redis Key under which the last set PWM value is stored and published.\n
            redisTopicLastAngle (String): Redis Key under which the last set Motor Angle is stored and published.\n
            redisKeyMaxPWM (String): Redis Key under which the Max PWM value for this motor is stored.\n
            redisKeyMinPWM (String): Redis Key under which the Min PWM value for this motor is stored.\n
            redisKey0PWM (String): Redis Key under which the PWM value for 0 degree is stored for this motor.\n
            redisKey90PWM (String): Redis Key under which the PWM relative absolute value for changing the motor position by 90 degree is stored.\n
            rosTopicSetPWM (String): The subscribed ROS-Topic to set an absolute PWM value for this motor.\n
            rosTopicSetAngle (String): The subscribed ROS-Topic to set an absolute angle for this motor.\n
            rosTopicChangeAngle (String): The subscribed ROS-Topic to change the current angle of this motor.\n
            cmdSetAbsolute (int): The I2C command linked to this motor to set an absolute value.\n 
            i2cAddress (int): The I2C address of the arduino controlling the motor.\n
            i2cArrayPublisher (rclpy.Publisher): The ROS-Publisher object for publishing I2C-Arrays.\n

        In order to use the motor correctly, the default angle values (for 0° and the delta for 90°) and 
        the min and max PWM values must be stored under the given Redis-Keys.
        The current default values for the motors can be found under: https://icampusnet.th-wildau.de/gitlab/ros-e/tischroboter-software-sbc/wikis/Redis-Systemwerte 

        The arduino commands can be found under: https://icampusnet.th-wildau.de/gitlab/ros-e/tischroboter-software-sbc/wikis/Arduino-I2C-Kommandos#ansteuerung-der-motoren 
        
    """

    # super().__init__('motor_node_{}'.format(name))
    super().__init__()
    self.parentNode = parentNode

    self.name = name # Just used for debugging

    ### Create redis objects
    self.r = redis.Redis(host="localhost", port=6379, db=0) # Redis object to store and get key-values
    self.p = self.r.pubsub(ignore_subscribe_messages=True)  # PubSub to publish redis messages

    ### Redis publish topics for current motor status
    self.redisTopicLastPWM = redisTopicLastPWM
    self.redisTopicLastAngle = redisTopicLastAngle

    ### Min and Max pwm values for the motor 
    self.maxPWM = int(self.r.get(redisKeyMaxPWM))
    self.minPWM = int(self.r.get(redisKeyMinPWM))
    self.value0PWM = int(self.r.get(redisKey0PWM))
    self.value90PWM = int(self.r.get(redisKey90PWM))

    ### I2C Command and Address
    self.cmdSetAbsolute = cmdSetAbsolute
    #self.cmdSetRelative = cmdSetRelative   # unused

    self.i2cAddress = i2cAddress
    self.i2cArrayPublisher = i2cArrayPublisher

    ### Ros subscriber topics for input commands and publisher for status info
    self.rosTopicSetPWM = rosTopicSetPWM
    self.rosTopicSetAngle = rosTopicSetAngle
    self.rosTopicChangeAngle = rosTopicChangeAngle
    self.rosTopicIsMoving = rosTopicIsMoving

    ### Motor specific topics subscriber and publisher
    if (self.rosTopicSetPWM is not None): self.parentNode.create_subscription(Int16, self.rosTopicSetPWM, self.onSetPWM, 10)
    if (self.rosTopicSetAngle is not None): self.parentNode.create_subscription(MotorPosition, self.rosTopicSetAngle, self.onSetAngle, 10)
    if (self.rosTopicChangeAngle is not None): self.parentNode.create_subscription(MotorPosition, self.rosTopicChangeAngle, self.onChangeAngle, 10)
    self.isMovingPublisher = self.parentNode.create_publisher(Bool, self.rosTopicIsMoving, 10)

    self.logger = self.parentNode.get_logger()

    # variables for the publisher
    self.isMoving = False
    self.isMovingTimer = None

    self.logger.info("Subscribed: {:20}  |  Msg: head/MotorPosition".format(self.rosTopicSetAngle))
    self.logger.info("Subscribed: {:20}  |  Msg: head/MotorPosition".format(self.rosTopicChangeAngle))
    self.logger.info("Subscribed: {:20}  |  Msg: Int16".format(self.rosTopicSetPWM))
    self.logger.info("Publishes:  {:20}  |  Msg: Bool".format(self.rosTopicIsMoving))

    ### Init head position

    # Assume the pwm value the motor had at the end of the last session as the current value.
    self.currentPWM = int(self.r.get(redisTopicLastPWM))
    # If no value was found in redis, reset to looking straight
    if self.currentPWM == None: 
      self.currentPWM = self.value0PWM
      self.logger.info(name + " found no redis entry for the last pwm value. Reset to: " + str(self.currentPWM))

    # waiting seems to be necessary in order for the published motor movement to be functioning
    time.sleep(1)
    # move to the expected pwm value to avoid inconsistencies in case the arduino moved the motors without this node and without updating redis
    self.onSetPWM(Int16(data=self.currentPWM))

        
    ################################
    ### LOG INFO ###################

    self.logger.info("Publish on: system/i2c/write8      |  Msg: system/I2Cwrite8")
    self.logger.info("Publish on: system/i2c/writeArray  |  Msg: system/I2CwriteArray")    
    self.logger.info("Started Motor Control Node {}".format(name))

    ################################


  def onShutdown(self):
    pass

  def onSetPWM(self, msg):
    """Method to set an absolute PWM value for this motor. This method will take care about the maximum pwm values.

    Args:
        msg (std_msgs.msg.Int16): ROS Int16 message object. 
    """

    self.logger.info("Got message on set {} pwm: \n{}".format(self.name, msg))

    pwm = int(msg.data)

    # Create a MotorPosition object so the setPWM command can be handled like the set/change angle methods
    motorPosition = MotorPosition()
    motorPosition.duration = 0
    motorPosition.speed = 10.0

    self.commitMovement(pwm, motorPosition)

    
  def onSetAngle(self, msg):
    """Method to set an absolute motor angle. This method will take care about the maximum motor angles.

    Args:
        msg (head.msg.MotorPosition): ROS MotorPosition message object. 
    """
    self.logger.info("Got message on set {} angle:\n{}".format(self.name, msg))

    angle = int(msg.angle)

    # use the PWM value for 0 degrees (straight view angle) as reference to turn to an absolute angle
    pwm = self.value0PWM + self.getPwmDeltaFromAngle(angle)

    self.commitMovement(pwm, msg)


  def onChangeAngle(self, msg):
    """Method to change the current motor angle relatively. This method will take care about the maximum motor angles.

    Args:
        msg (head.msg.MotorPosition): ROS MotorPosition message object. 
    """
    self.logger.info("Got message on change {} angle:\n{}".format(self.name, msg))

    angle = int(msg.angle)

    # use the current PWM value as reference to turn to an angle relative to the previous position
    pwm = self.currentPWM + self.getPwmDeltaFromAngle(angle)

    self.commitMovement(pwm, msg)


  def commitMovement(self, pwm, msg):
    """ The code all turn commands have in common. Handles everything about the turn. """

    # Make sure the PWM is in a range the motor can actually turn to
    pwm = self.limitPWM(pwm)
    angle = round(self.getAngleFromPWM(pwm))

    self.logger.info("Moving with PWM = {} (Calced angle: {})".format(pwm, angle))

    # Store the last values in Redis
    if (self.redisTopicLastPWM is not None): 
      self.r.set(self.redisTopicLastPWM, pwm)
      self.r.publish(self.redisTopicLastPWM, pwm)
    if (self.redisTopicLastAngle is not None): 
      self.r.set(self.redisTopicLastAngle, angle)
      self.r.publish(self.redisTopicLastAngle, angle)

    # Update the motor position with the calculated PWM value
    self.updateMotorPosition(self.cmdSetAbsolute, pwm, int(round(msg.speed)), int(msg.duration))

    # Publish info that the motor is moving
    deltaPwm = pwm - self.currentPWM
    self.pubMotorActivity(deltaPwm, msg.speed, msg.duration)

    self.currentPWM = pwm

    
  def getPwmDeltaFromAngle(self, angle):
    """ Returns the PWM delta that matches the specified angle delta. """
    return int(float(angle) / 90.0 * self.value90PWM)

  def getAngleFromPWM(self, pwm):
    """ Returns the absolute angle that matches the specified pwm value. """
    return float(pwm - self.value0PWM) * 90.0 / float(self.value90PWM) 

  def limitPWM(self, pwm):
    """ Returns the maximum or minimum pwm value that can be turned to, if the specified pwm value is too great/small.
    If the pwm value is already within the legal range, it is returned unchanged. """
    return min(max(pwm, self.minPWM), self.maxPWM)


  ### Generic method to take care of the I2C publishing of new motor positions ###
  def updateMotorPosition(self, cmd, pwm, speed, duration):
    """Generic method to update the motor position to a new PWM value. This method is handling the necessary I2C publishing.

    Args:
        cmd (int): Arduino Command for the motor position update. 
        pwm (int): New PWM value for the motor.
        speed (int): Speed value between 1 and 100 to reach the new position. The speed argument is only considered if the duration argument == 0.
        duration (int): Duration in ms to reach the new motor position. If duration != 0, the speed argument is not considered.
    """

    self.logger.info("{} --> pwm: {} | speed: {} | duration: {}".format(cmd, pwm, speed, duration))

    # Creating the I2C Array Object for publishing to the I2C Bridge node.
    o = I2CwriteArray()
    o.address = self.i2cAddress
    o.command = cmd
    
    # differ between speed or duration value
    if duration is not None and duration > 0:
      o.data = [int(pwm >> 8), int(pwm & 0x00FF), int(I2cDataConstants.MOTOR_CONTROL_DURATION), int(duration >> 8), int(duration & 0x00FF)]
    else:
      o.data = [int(pwm >> 8), int(pwm & 0x00FF), int(I2cDataConstants.MOTOR_CONTROL_SPEED), int(speed & 0x00FF), int(0)]

    self.i2cArrayPublisher.publish(o)


  def pubMotorActivity(self, deltaPwm, speed, duration):
    """ Handles publishing of the motor's activity flag. """

    # find out how long the movement will take, so a timer can handle resetting the isMoving flag
    # "speed factor" is the pwm per milliscond to move the motor and "speed" is the percentage of that factor to use
    timeActiveMillis = duration if duration is not None and duration > 0 else abs(deltaPwm) / (float(speed) / 100 * I2cDataConstants.MOTOR_MAX_SPEED)
    timeActiveSeconds = float(timeActiveMillis) / 1000

    # add a small constant time to make sure the movement-stopped-info is sent after the hardware actually stopped even if there is a tiny hardware delay
    timeActiveSeconds += 0.1

    # If the motor was not moving before, publish that it started moving
    if self.isMoving == False:
      self.isMovingPublisher.publish(Bool(data = True))
      self.isMoving = True
      self.logger.info("Published info that '" + str(self.name) + "' started moving.")
    else: 
      # If another command is received while the motor is still moving, the first movement will be overwritten and the new movement will begin immediately.
      # In that case, the timer is now outdated and has to be shut down and started again with the new duration 
      # to make sure it only publishes when the new movement will finish rather than when the cancelled movement would have finished.
      
      #self.isMovingTimer.shutdown()
      self.isMovingTimer.cancel()
    
    # The timer. Sets isMoving to False again after the calculated time for the motor movement has passed
    self.isMovingTimer = Timer(interval=timeActiveSeconds, function=self.pubInactive)
    self.isMovingTimer.start()
    # self.isMovingTimer = self.create_timer(period=timeActiveSeconds, callback=self.pubInactive, oneshot=True)


  def pubInactive(self):
    """ Handles setting the motor's activity flag to inactive. """
    self.isMovingPublisher.publish(Bool(data = False))
    self.isMovingTimer.cancel()
    self.isMoving = False
    self.logger.info("Published info that '" + str(self.name) + "' stopped moving.")



###########################################################################################################
## ROS Node, contains all motor objects ###################################################################
###########################################################################################################
class MotorControl(Node):
  """ ROS Node containing all motor objects """


  def __init__(self):
    super().__init__('motorControl_node')

    # Arduino address of the Arduino controlling the head motors
    self.arduinoI2C = 0x08

    ### Publisher for I2C Connection
    self.pubI2Cwrite8 = self.create_publisher(I2Cwrite8, "system/i2c/write8", 10)
    self.pubI2CwriteArray = self.create_publisher(I2CwriteArray, "system/i2c/writeArray", 10)

    

    ### Publish the maximum speed constant to the arduino so this node and hardware have the same value
    o = I2Cwrite8()
    o.address = self.arduinoI2C
    o.command = Commands.MOTOR_SET_SPEED
    o.data = int(I2cDataConstants.MOTOR_MAX_SPEED & 0x00FF)
    self.pubI2Cwrite8.publish(o)

    # time.sleep(1)

    # Creating the object for the head turn motor
    self.motorTurn = Motor(parentNode = self,
      name="turn", redisTopicLastPWM="head/motorturn/lastPWM", redisTopicLastAngle="head/turn/lastAngle", 
      redisKeyMaxPWM="head/motorturn/maxPWM", redisKeyMinPWM="head/motorturn/minPWM", 
      redisKey0PWM="head/motorturn/pwm0degree", redisKey90PWM="head/motorturn/pwm90degree", 
      rosTopicSetPWM="head/motorturn/setPWM", rosTopicSetAngle="head/turn/setAngle", 
      rosTopicChangeAngle="head/turn/changeAngle", rosTopicIsMoving="head/turn/isMoving",
      cmdSetAbsolute=Commands.MOTOR_TURN_SET_ABSOLUTE, cmdSetRelative=Commands.MOTOR_TURN_SET_RELATIVE,
      i2cAddress=self.arduinoI2C, i2cArrayPublisher=self.pubI2CwriteArray
      )

    # time.sleep(1)

    # Creating the object for the head pitch motor
    self.motorPitch = Motor(parentNode = self,
      name="pitch", redisTopicLastPWM="head/motorpitch/lastPWM", redisTopicLastAngle="head/pitch/lastAngle", 
      redisKeyMaxPWM="head/motorpitch/maxPWM", redisKeyMinPWM="head/motorpitch/minPWM", 
      redisKey0PWM="head/motorpitch/pwm0degree", redisKey90PWM="head/motorpitch/pwm90degree", 
      rosTopicSetPWM="head/motorpitch/setPWM", rosTopicSetAngle="head/pitch/setAngle", 
      rosTopicChangeAngle="head/pitch/changeAngle", rosTopicIsMoving="head/pitch/isMoving",
      cmdSetAbsolute=Commands.MOTOR_PITCH_SET_ABSOLUTE, cmdSetRelative=Commands.MOTOR_PITCH_SET_RELATIVE,
      i2cAddress=self.arduinoI2C, i2cArrayPublisher=self.pubI2CwriteArray
      )

    # rclpy.spin(self.motorPitch)
    # rclpy.spin(self.motorTurn)

    # self.spinNode(self.motorTurn)
    # self.spinNode(self.motorPitch)

  def onShutdown(self):
    self.motorTurn.onShutdown()
    self.motorPitch.onShutdown()

  # def spinNode(self, node):
  #   thread = NodeSpinner(node)
  #   thread.start()


def main(args=None):

  rclpy.init(args=args) # 'motorControl_node'

  # Init all motors
  node = MotorControl()

  # Spin forever
  rclpy.spin(node)
  node.onShutdown()
  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()