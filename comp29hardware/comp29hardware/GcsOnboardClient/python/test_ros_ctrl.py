import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class DroneCommandClient(Node):

    def __init__(self):
        super().__init__('drone_command_client')
        self.cli_arm = self.create_client(Trigger, 'arm')
        self.cli_takeoff = self.create_client(Trigger, 'takeoff')
        self.cli_land = self.create_client(Trigger, 'land')

        for client in [self.cli_arm, self.cli_takeoff, self.cli_land]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Service not available, waiting again...')

        self.command_sequence = ['arm', 'takeoff', 'land']
        self.command_index = 0
        self.send_next_command()

    def send_next_command(self):
        if self.command_index < len(self.command_sequence):
            command = self.command_sequence[self.command_index]
            if command == 'arm':
                self.send_command(self.cli_arm)
            elif command == 'takeoff':
                self.send_command(self.cli_takeoff)
            elif command == 'land':
                self.send_command(self.cli_land)
        else:
            self.get_logger().info('All commands sent. Shutting down.')
            rclpy.shutdown()

    def send_command(self, client):
        req = Trigger.Request()
        self.future = client.call_async(req)
        self.future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Received response: {response.message}')
            else:
                self.get_logger().info(f'Command failed: {response.message}')
            self.command_index += 1
            self.send_next_command()
        except Exception as e:
            self.get_logger().info(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    drone_command_client = DroneCommandClient()
    rclpy.spin(drone_command_client)
    drone_command_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
