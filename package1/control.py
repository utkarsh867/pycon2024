import rclpy
from px4_msgs.msg import (OffboardControlMode, VehicleCommand,
                          VehicleCommandAck, VehicleThrustSetpoint)
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)


class ControlNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.vehicle_cmd_publisher = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_profile
        )
        self.offboard_control_publisher = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile
        )
        self.thrust_pub = self.create_publisher(
            VehicleThrustSetpoint, "/fmu/in/vehicle_thrust_setpoint", qos_profile
        )
        self.vehicle_cmd_sub = self.create_subscription(
            VehicleCommandAck,
            "/fmu/out/vehicle_command_ack",
            self.get_vehicle_command_ack,
            qos_profile,
        )
        self.arm_state = False

        self.create_timer(1.0, self.offboard_control_callback)
        self.create_timer(0.001, self.move_forward)

        self.arm_vehicle()

    def get_vehicle_command_ack(self, msg: VehicleCommandAck):
        cmd = msg.command
        res = msg.result
        self.get_logger().info("ACK: {}, {}".format(cmd, res))
        if cmd == 400 and res == 0:
            self.arm_state = True

    def offboard_control_callback(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.thrust_and_torque = True
        msg.direct_actuator = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_publisher.publish(msg)

    def send_vehicle_command(self, command, param1, param2):
        vehicle_cmd_msg = VehicleCommand()
        vehicle_cmd_msg.param1 = param1
        vehicle_cmd_msg.param2 = param2
        vehicle_cmd_msg.command = command
        vehicle_cmd_msg.target_system = 1
        vehicle_cmd_msg.target_component = 1
        vehicle_cmd_msg.source_system = 1
        vehicle_cmd_msg.source_component = 1
        vehicle_cmd_msg.from_external = True
        vehicle_cmd_msg.timestamp = int(
            self.get_clock().now().nanoseconds / 1000)
        self.vehicle_cmd_publisher.publish(vehicle_cmd_msg)

    def arm_vehicle(self):
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0
        )

    def move_forward(self):
        if self.arm_state:
            msg = VehicleThrustSetpoint()
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            msg.xyz = [1.0, 0.0, 0.0]
            self.thrust_pub.publish(msg)


def main():
    rclpy.init()

    node = ControlNode("control_node")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
