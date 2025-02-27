# ASU RISE LAB FURI
# MATTHEW HERDEGEN

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBool_Request, SetMode, SetMode_Request

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offb_node_py')

        # Subscribers
        self.state_sub = self.create_subscription(State, "mavros/state", self.state_cb, 10)
        self.pos_sub = self.create_subscription(PoseStamped. "mavros/local_position/pose", self.pos_cb, 10)

        # Publishers
        self.local_pos_pub = self.create_publisher(PoseStamped, "mavros/setpoint_raw/local", 10)
        self.pos_pub = self.create_publisher(PoseStamped, "mavros/local_position/pose",10)

        # Services
        self.arming_client = self.create_client(CommandBool, "mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "mavros/set_mode")

        # Positioning
        self.current_state = State()
        self.pose = PoseStamped()
        self.wapoints = [
            [0, 0, 2],
            [2, 2, 2],
            [0, 0, 2],
            [0, 0, 0.1]
        ]

        # Timer for setpoint publishing
        self.timer = self.create_timer(0.1, self.send_setpoint)

        # Track last request time
        self.last_request = self.get_clock().now()

    def state_cb(self, msg):
        self.current_state = msg

    def send_setpoint(self):
        """Publishes setpoints and handles mode switching & arming."""
        if not self.current_state.connected:
            return

        now = self.get_clock().now()

        # Ensure the mode is set to GUIDED
        if self.current_state.mode != "GUIDED" and (now - self.last_request).nanoseconds > 5e9:
            set_mode_req = SetMode_Request()
            set_mode_req.custom_mode = "GUIDED"
            future = self.set_mode_client.call_async(set_mode_req)
            future.add_done_callback(self.mode_callback)
            self.last_request = now

        # Ensure drone is armed
        elif not self.current_state.armed and (now - self.last_request).nanoseconds > 5e9:
            arm_req = CommandBool_Request()
            arm_req.value = True
            future = self.arming_client.call_async(arm_req)
            future.add_done_callback(self.arming_callback)
            self.last_request = now

        # Publish position setpoint
        self.local_pos_pub.publish(self.pose)


    def mode_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("GUIDED mode enabled")
        except Exception as e:
            self.get_logger().error(f"Failed to set GUIDED mode: {e}")

    def arming_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Vehicle armed")
        except Exception as e:
            self.get_logger().error(f"Failed to arm vehicle: {e}")

def main(args=None):
    rclpy.init(args=args)
    takeoff_landing_node = OffboardControl()
    rclpy.spin(node)
    takeoff_landing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()import rclpy
