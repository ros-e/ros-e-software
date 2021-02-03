
"""
Tutorial from https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber/

Python Nodes has to be added in setup.py file of the package

"""

# Imports for ROS2
import rclpy
from rclpy.node import Node

# Message Imports
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback, # Callback function
            10)

        self.subscription  # prevent unused variable warning

    # Callback function for subscription
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    try:
        # Continue until node is stopped
        rclpy.spin(minimal_subscriber)

    except KeyboardInterrupt:
        print("Interrupted! Shutting down. ")
        
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
  main()