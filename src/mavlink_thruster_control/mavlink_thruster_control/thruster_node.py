import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from pymavlink import mavutil
import time

class ThrusterNode(Node):
    def __init__(self):
        super().__init__('thruster_node')
        self.get_logger().info("initialized")

        self.master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

        self.master.wait_heartbeat()
        self.get_logger().info("mavlink connected")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)
        self.get_logger().info("armed")

        self.subscription = self.create_subscription(
            Int32,
            'thruster_pwm',
            self.pwm_callback,
            10
        )

    def pwm_callback(self, msg):
        pwm = msg.data
        self.get_logger().info(f"PWM: {pwm}")

        scaled_x = int((pwm - 1500) * 2)
        scaled_x = max(-1000, min(1000, scaled_x))

        self.master.mav.manual_control_send(
            self.master.target_system,
            x=scaled_x,
            y=0,
            z=500,
            r=0,
            buttons=0
        )

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
