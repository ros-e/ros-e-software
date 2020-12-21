#!/usr/bin/env python
# -*- coding: UTF-8 -*-

"""
Makes the head turn to the direction of sounds. 
Publishes turn commands to head/turn/changeAngle

@author Johannes Sommerfeldt
"""

# ROS 2 Imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Float64, String, Int16, Int32, Bool
from head.msg import MotorPosition
from sound.msg import Spike



import time
import datetime
from collections import deque


### constants for turning preferences
FRONT_FILTER_WIDTH = 10       # direction angle (relative to viewing direction) to ignore sounds from. Used to prevent constant mini adjustments.
DIRECTION_FILTER_WIDTH = 10   # how much greater or smaller direction values may be than the new direction
DIRECTION_HISTORY_LENGTH = 2  # how many previous directions must be similar to the new direction to pass the filter
TURN_DURATION = 1.0           # number of seconds the motors have to turn
COOLDOWN_AFTER_TURN = 0.5     # how many seconds to ignore any sound after finishing to turn. Prevents chaining turns together.

### enable or disable filter
USE_FRONT_FILTER = True       # only turn to sounds that are not right in front of the head (=near the viewing direction)
USE_SPIKE_FILTER = False       # only turn to loud sounds
USE_DIRECTION_FILTER = True   # only turn to long/repeated sounds from the same direction


class SoundToTurnControl(Node):

    def __init__(self):
        
        super().__init__('soundToTurnControl_node')

        self.create_subscription(Int32, 'sound/direction', self.on_direction, 10)
        # self.create_subscription(Bool, 'sound/spike', self.on_spike, 10)              # old
        self.create_subscription(Spike, 'sound/spike', self.on_spike, 10)               # new
        self.create_subscription(Bool, 'head/turn/isMoving', self.on_turn_moving, 10)
        self.create_subscription(Bool, 'head/pitch/isMoving', self.on_pitch_moving, 10)

        self.turn_publisher = self.create_publisher(MotorPosition, "head/turn/changeAngle", 10)

        self.get_logger().info("Subscribed: sound/direction")
        # self.get_logger().info("Subscribed: sound/spike")
        self.get_logger().info("Subscribed: head/turn/isMoving")
        self.get_logger().info("Subscribed: head/pitch/isMoving")
        self.get_logger().info("Publishes: head/turn/changeAngle")

        ### variables for filters
        self.is_spike = False
        self.is_turn_moving = False
        self.is_pitch_moving = False

        # Queue that stores the latest received directions, dropping the oldest one when full
        self.direction_history = deque(maxlen=DIRECTION_HISTORY_LENGTH)
        self.latest_angle = 0

        # Dictionary to store directions that arrived while is_spike was False (using their time or arrival as keys)
        self.directions_before_spike = {}

        self.get_logger().info("Started Sound To Turn Control Node")

    # new
    def on_spike(self, msg):

        # update the value for the spike filter
        self.is_spike = msg.spike_status
        self.get_logger().info("Got Spike " + str(self.is_spike))

        ### The spike detection often takes longer than the on-board DOA calculation of the ReSpeaker. Therefore:

        # If the spike flag changed to true...
        if self.is_spike == False: return

        spike_starting_time = msg.timestamp
        if spike_starting_time == None: return
        # self.get_logger().info("Spike starting time: " + str(spike_starting_time) + " \t now: " + str(self.get_clock().now()))

        # ... give the directions that came from that sound, but got ignored due to the delay of the spike detection, a second chance
        for time_of_arrival in self.directions_before_spike:
            if time_of_arrival > spike_starting_time:
                self.get_logger().info("ATTEMPTING AGAIN: " + str (self.directions_before_spike[time_of_arrival]))
                self.filter_and_turn(self.directions_before_spike[time_of_arrival])

        # discard all the directions since they either got a second change or arrived when it was actually quiet
        self.directions_before_spike.clear()


    # old
    # def on_spike(self, msg):
    #     self.is_spike = msg.data
    #     self.get_logger().info("Got Spike " + str(self.is_spike))
    

    def on_turn_moving(self, msg):
        self.get_logger().info("Got Turn movement: " + str(msg.data))

        # prevent turning to motor sound echoes
        if msg.data == False: time.sleep(COOLDOWN_AFTER_TURN)

        self.is_turn_moving = msg.data


    def on_pitch_moving(self, msg):
        self.get_logger().info("Got Pitch movement: " + str(msg.data))

        # prevent turning to motor sound echoes
        if msg.data == False: time.sleep(COOLDOWN_AFTER_TURN)

        self.is_pitch_moving = msg.data


    def on_direction(self, msg):
        new_angle = int(msg.data)
        self.get_logger().info("Got direction: {}".format(new_angle))

        # check if all filters can be passed and, if yes, turn
        self.filter_and_turn(new_angle)

        # add values to the history, which is used for direction filtering, unless the new direction is equal to the one before
        if (new_angle != self.latest_angle):
            self.direction_history.append(new_angle)
            self.latest_angle = new_angle


    def filter_and_turn(self, angle):
        """ Check if all filter criteria allow turning and initiates (publishes) the turn. """

        # ignore sounds while the motors are moving
        if self.is_turn_moving or self.is_pitch_moving:
            # self.get_logger().info("Turn denied: motor is moving")
            return

        ### optional filters:

        if USE_FRONT_FILTER:
            # ignore minor direction changes to not constant make tiny adjustments when already facing the sound source
            if abs(angle) <= FRONT_FILTER_WIDTH: 
                self.get_logger().info("Turn denied: small angle ignored")
                return

        if USE_SPIKE_FILTER:
            # only turn while the volume is spiking
            if self.is_spike == False: 
                self.get_logger().info("Turn denied: volume is not spiking")
                # remember this value since it might only be blocked due to spike detection delay
                # self.directions_before_spike[self.get_clock().now()] = angle
                self.directions_before_spike[datetime.datetime.now()] = angle
                return

        if USE_DIRECTION_FILTER:
            # ignore directions that only appear shortly to avoid turning to echoes or clicks
            if self.can_pass_direction_filter(angle) == False: 
                self.get_logger().info("Turn denied: sound came too shortly from this direction")
                return

        ##############

        # create a MotorPosition object to publish
        mp = MotorPosition()
        mp.angle = angle
        mp.speed = 0.0
        mp.duration = int(TURN_DURATION * 1000)     # 1000 because milliseconds

        # initiate turn
        self.turn_publisher.publish(mp)
        self.get_logger().info("Turn successfully published: " + str(angle))

        # reset direction filter
        self.direction_history.clear


    def can_pass_direction_filter(self, angle):
        """ Checks whether the specified angle can currently pass the direction filter. """

        # prevent letting short sounds through just because the history is not filled yet
        if len(self.direction_history) != self.direction_history.maxlen: return False

        # check whether all directions in the history are close to the new angle
        can_pass_filter = True
        for direction in self.direction_history:
            if direction > angle + DIRECTION_FILTER_WIDTH or direction < angle - DIRECTION_FILTER_WIDTH:
                can_pass_filter = False
        
        return can_pass_filter


def main(args=None):

  rclpy.init(args=args) # 'soundToTurnControl_node'

  node = SoundToTurnControl()

  rclpy.spin(node)
  node.onShutdown()
  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()