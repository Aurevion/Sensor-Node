import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import socket
import struct
import threading
from sensor_node.srv import StartSensor, StopSensor  # Import your services
from sensor_node.msg import SensorData       #Import custom message


class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

        # Declare parameters
        self.declare_parameter('interval', 1000)

        # Create publishers for sensor parameters
        self.sensor_data_pub = self.create_publisher(SensorData, 'sensor_data', 10) #Publisher for custom message

        # Setup socket for TCP communication
        self.sensor_ip = '127.0.0.1' # Replace with actual sensor IP
        self.sensor_port = 2000
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect to sensor
        try:
            self.socket.connect((self.sensor_ip, self.sensor_port))
            self.get_logger().info('Connected to sensor.')
        except socket.error as e:
            self.get_logger().error(f'Failed to connect to sensor: {e}')

        # Send start command with interval
        interval = self.get_parameter('interval').value
        self.send_start_command(interval)

        # Start a thread to listen for incoming messages
        self.listener_thread = threading.Thread(target=self.listen_to_sensor, daemon=True)
        self.listener_thread.start()

        # Create a service to handle start/stop commands
        self.start_service = self.create_service(StartSensor, 'start_sensor', self.handle_start_sensor)
        self.stop_service = self.create_service(StopSensor, 'stop_sensor', self.handle_stop_sensor)

    def send_start_command(self, interval):

        try:
            command_id = '03'
            payload = struct.pack('<H', interval).hex().upper() # Encode as little-endian
            message = f'#{command_id}{payload}\r\n'
            self.socket.sendall(message.encode())
            self.get_logger().info(f'Sent start command with interval {interval} ms.')
        except Exception as e:
            self.get_logger().error(f'Failed to send start command: {e}')

    def send_stop_command(self):

        try:
            command_id = '09'
            message = f'#{command_id}\r\n'
            self.socket.sendall(message.encode())
            self.get_logger().info('Sent stop command.')
        except Exception as e:
            self.get_logger().error(f'Failed to send stop command: {e}')

    def listen_to_sensor(self):

        try:
            while True:
                data = self.socket.recv(1024).decode()
                if data:
                    try:
                        self.decode_message(data)
                    except Exception as decode_error:
                        self.get_logger().error(f'Error decoding message: {decode_error}')
        except Exception as e:
            self.get_logger().error(f'Error receiving data: {e}')

    def decode_status_message(self, payload):
        try:
            if len(payload) != 20:
                self.get_logger().warning(f'Invalid status message length: {len(payload)}')
                return

            supply_voltage, env_temp, yaw, pitch, roll = struct.unpack('<Hhhhh', bytes.fromhex(payload))

            # Publish using the custom message
            sensor_data_msg = SensorData()
            sensor_data_msg.supply_voltage = supply_voltage / 1000.0
            sensor_data_msg.env_temp = env_temp / 10.0
            sensor_data_msg.yaw = yaw / 10.0
            sensor_data_msg.pitch = pitch / 10.0
            sensor_data_msg.roll = roll / 10.0
            self.sensor_data_pub.publish(sensor_data_msg)

            self.get_logger().info(f'Status Message - Voltage: {supply_voltage} mV, Temp: {env_temp} dC, Yaw: {yaw}, Pitch: {pitch}, Roll: {roll}')
        except Exception as e:
            self.get_logger().error(f'Failed to decode status message: {e}')

    def handle_start_sensor(self, request, response):
        self.send_start_command(request.interval)
        response.success = True
        response.message = 'Start command sent.'
        return response

    def handle_stop_sensor(self, request, response):
        self.send_stop_command()
        response.success = True
        response.message = 'Stop command sent.'
        return response

    def destroy_node(self):
        self.socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted. Shutting down...')
    finally:
        node.destroy_node() #Important to close the socket
        rclpy.shutdown()


if __name__ == '__main__':
    main()