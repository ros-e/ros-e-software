
"""
Tutorial from https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber/

Python Nodes has to be added in setup.py file of the package

"""

# Imports for ROS2
import rclpy
from rclpy.node import Node

# Message Imports
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        # Create publisher
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        
        # Publish message
        self.publisher_.publish(msg)
        
        # Logging
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def terminate(self):
        self.get_logger().info("node shutting down")
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    # to catch other Signals than SIGINT
    # import signal
    # signal.signal(signal.SIGTERM, (lambda signum, frame: minimal_publisher.terminate()))

    try:
        # Continue until node is stopped
        rclpy.spin(minimal_publisher)

    except KeyboardInterrupt:
        print("Stopped spinning due to receiving SIGINT!")

    finally:
        minimal_publisher.terminate()
        
    print("after exception!")



if __name__ == '__main__':
  main()