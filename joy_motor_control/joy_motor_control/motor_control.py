import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
from crccheck.crc import Crc8

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        # 1. Initialize variables (Fixes the AttributeError)
        self.recording_processes = {}
        self.packet_header = [0xDE, 0xAD]
        self.packet_footer = [0xEF]
        self.lx = 0
        self.ly = 1
        self.THRESHOLD = 0.2

        # 2. Open Serial Port ONCE at startup
        try:
            # Change /dev/ttyUSB0 to /dev/ttyACM0 if necessary
            self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
            self.get_logger().info('Serial port opened successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            # Keep running even if serial fails so you can debug logs
            self.ser = None

        self.subscription = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        
        self.get_logger().info('Motor Control Node has been started.')

    def joy_callback(self, msg):
        print(f"[motor_control][callback]")
        # Prevent crash if index is out of range
        if len(msg.axes) <= max(self.lx, self.ly):
            return

        joy_lx = msg.axes[self.lx]
        joy_ly = msg.axes[self.ly]

        command = 0
        if joy_ly < -self.THRESHOLD: command += 6
        elif joy_ly > self.THRESHOLD: command += 0
        else: command += 3

        if joy_lx < -self.THRESHOLD: command += 2
        elif joy_lx > self.THRESHOLD: command += 0
        else: command += 1

        self.write_to_serial(command)

    def calc_crc8(self, data):
        return Crc8.calc(data)

    def write_to_serial(self, command):
        if self.ser and self.ser.is_open:
            data = []
            data += self.packet_header
            data += [command]
            data += self.packet_footer
            crc_value = self.calc_crc8(bytes(data))
            data.append(crc_value)
            
            try:
                self.ser.write(bytearray(data))
                # Log only every few messages to prevent terminal spam
                # self.get_logger().info(f"Sent: {command}")
            except Exception as e:
                self.get_logger().error(f"Write failed: {e}")
        else:
            self.get_logger().warning("Serial port is not available.")

def main():
    rclpy.init()
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Check if dict exists and has items before iterating
        if hasattr(node, 'recording_processes'):
            for p in node.recording_processes.values():
                if p: p.kill()
        
        if hasattr(node, 'ser') and node.ser:
            node.ser.close()
            
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
