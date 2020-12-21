#!/usr/bin/python
# -*- coding: utf-8 -*-

import json
import os
import redis
import sys
import time

# ROS 2 Imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32, Bool, ColorRGBA


# needed for correct speech output !!!
reload(sys)  # Reload is a hack
sys.setdefaultencoding('UTF8')

class svoxNode(Node):
    super().__init__('svox_node')
    ttsSub = self.create_subscription(String, 'sound/tts', say, 10)

    self.get_logger().info("SVOX Node started")

    def say(self, data):
        sayText = data.data.decode('utf-8')
        # print(rospy.get_caller_id() + "I heard: " + sayText)
        print("I heard: " + sayText)
        sayText = sayText.split('\n')
        r = redis.Redis()
        language = r.get('language').decode("utf-8")
        r.close()
        for text in sayText:
            self.get_logger().debug('text: {}'.format(text))
            if text is '\n':
                time.sleep(1)
                continue
            os.system('pico2wave -l ' + language + ' -w /tmp/SVOX.wav "' + text + '"')
            os.system('aplay /tmp/SVOX.wav')
            os.system('rm /tmp/SVOX.wav')

def main(args=None):
    ttsNode = rclpy.init(args=args) # 'svox_node'

    node = svoxNode()
    
    rclpy.spin(node)
    node.onShutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()