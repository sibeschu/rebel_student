import rclpy
import time
from rclpy.node import Node
from igus_rebel_msgs.srv import SetDigitalOutput


class DigitalOutputClient(Node):
    def __init__(self):
        super().__init__('digital_output_client')
        self.cli = self.create_client(SetDigitalOutput, '/set_digital_output')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_digital_output service...')

    def set_output(self, output_number: int, state: bool):
        req = SetDigitalOutput.Request()
        req.output.output = output_number
        req.output.is_on = state

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        # TODO: more elegant way to wait for completion
        time.sleep(1)
        
        return future.result()
