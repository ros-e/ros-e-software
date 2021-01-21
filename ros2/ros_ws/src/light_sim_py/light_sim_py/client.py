import sys

import rclpy
from rclpy.node import Node

from light_sim.msg import LightState
from light_sim.srv import GetLightState, SetLightState


class LightStateClientNode(Node):
    """This Node is sample client for the "light_sim_service" node.

    See the C++ project (light_sim) for msg and srv types.

    Usage: ros2 run light_sim_py client <state> 
           Where <state> is 0 for "off" or any other number for "on".
    """

    def __init__(self):
        super().__init__("light_sim_client")

        # Subscribe to the "light/state_changed" topic.
        self._subscription = self.create_subscription(
            LightState, "light/state_changed",
            self._on_light_state_changed, 10
        )

        # Create clients for the "light/get_state" & "light/set_state" services.
        self._get_client = self.create_client(GetLightState, "light/get_state")
        self._set_client = self.create_client(SetLightState, "light/set_state")

        self.get_logger().info("ready")

    def _on_light_state_changed(self, msg):
        """Callback for the "light/state_changed" topic."""
        self.get_logger().info(
            "(light/state_changed) -> on={0}".format(msg.on)
        )

    def test_get_state(self):
        """Tests the "light/get_state" service and logs the result."""
        # Call the service with an empty request.
        return self._get_client.call_async(GetLightState.Request())

    def test_set_state(self):
        """Tests the "light/set_state" service and logs the result."""
        # Create the request with the command line argument.
        request = SetLightState.Request()
        request.state = LightState()
        request.state.on = (int(sys.argv[1]) != 0)

        # Call the service with the request.
        return self._set_client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = LightStateClientNode()

    # Test the "light/get_state" service.
    future = node.test_get_state()

    # Spin/Run until the future finished.
    while rclpy.ok():
        rclpy.spin_once(node)

        if future.done():
            break

    node.get_logger().info(
        "(light/get_state) >> on={0}".format(future.result().state.on)
    )

    # Test the "light/set_state" service.
    future = node.test_set_state()

    # Spin/Run until the future finished.
    while rclpy.ok():
        rclpy.spin_once(node)

        if future.done():
            break

    node.get_logger().info(
        "(light/set_state) >> ok={0}".format(future.result().ok)
    )

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
