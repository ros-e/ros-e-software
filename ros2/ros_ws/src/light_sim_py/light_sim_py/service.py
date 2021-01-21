import time

import rclpy
from rclpy.node import Node

from light_sim.msg import LightState
from light_sim.srv import GetLightState, SetLightState


class LigthServiceNode(Node):
    """This Node provides a sample service for setting and retrieving the state
    of a simulated light bulb.

    See the C++ project (light_sim) for msg and srv types.

    Usage: ros2 run light_sim_py service
    """
    _on = False

    def __init__(self):
        super().__init__("light_sim_service")

        # Create a publisher for state changes.
        self._publisher = self.create_publisher(
            LightState, "light/state_changed", 10
        )

        # Create a service to get the current state.
        self._get_service = self.create_service(
            GetLightState, "light/get_state", self._on_get_state
        )

        # Create a service to set the current state.
        self._set_service = self.create_service(
            SetLightState, "light/set_state", self._on_set_state
        )

        self.get_logger().info("ready")

    def _on_get_state(self, request, response):
        """Callback for the "light/get_state" service."""
        self.get_logger().info(
            "(light/get_state) <<"
        )

        # Create a new response object with the current state.
        response.state = LightState()
        response.state.on = self._on

        self.get_logger().info(
            "(light/get_state) >> on={0}".format(response.state.on)
        )

        return response

    def _on_set_state(self, request, response):
        """Callback for the "light/set_state" service."""
        self.get_logger().info(
            "(light/set_state) << on={0}".format(request.state.on)
        )

        # Simulate some work, if the light is not yet in the desired state.
        if self._on != request.state.on:
            self.get_logger().info("(light/set_state) changing state...")
            time.sleep(2)

        # Update the current state.
        self._on = request.state.on
        response.ok = True

        # Publish the new state to the "light/state_changed" topic.
        msg = LightState()
        msg.on = self._on
        self._publisher.publish(msg)

        self.get_logger().info(
            "(light/set_state) >> ok={0}".format(response.ok)
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = LigthServiceNode()

    # Run and block until shutdown.
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
