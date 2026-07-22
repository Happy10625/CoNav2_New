"""Apply RealSense hardware-clock synchronization settings when the camera appears."""

import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter


class CameraSyncConfigurator(Node):
    def __init__(self):
        super().__init__("camera_sync_configurator")
        self.declare_parameter("camera_node", "/camera/camera")
        camera_node = self.get_parameter("camera_node").value.rstrip("/")
        self.client = self.create_client(SetParameters, f"{camera_node}/set_parameters")
        self.pending = False
        self.done = False
        self.timer = self.create_timer(1.0, self.apply)

    def apply(self):
        if self.done or self.pending or not self.client.service_is_ready():
            return
        request = SetParameters.Request()
        request.parameters = [
            Parameter("rgb_camera.global_time_enabled", value=True).to_parameter_msg(),
            Parameter("depth_module.global_time_enabled", value=True).to_parameter_msg(),
            Parameter("enable_sync", value=True).to_parameter_msg(),
        ]
        self.pending = True
        self.client.call_async(request).add_done_callback(self.on_result)

    def on_result(self, future):
        self.pending = False
        try:
            response = future.result()
            failures = [result.reason for result in response.results if not result.successful]
        except Exception as error:
            self.get_logger().warn(f"Cannot configure camera time synchronization: {error}")
            return
        if failures:
            self.get_logger().warn(
                "Camera rejected time synchronization parameters: " + "; ".join(failures))
            return
        self.done = True
        self.timer.cancel()
        self.get_logger().info(
            "RealSense global timestamps and RGB-depth synchronization enabled")


def main(args=None):
    rclpy.init(args=args)
    node = CameraSyncConfigurator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

