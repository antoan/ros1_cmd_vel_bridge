import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import roslibpy
import atexit

class CmdVelForwarder(Node):
    def __init__(self):
        super().__init__('cmd_vel_forwarder')
        atexit.register(self.shutdown)
        self.declare_parameter('ros1_host', 'localhost')
        self.declare_parameter('ros1_port', 9090)

        self.ros1_host = self.get_parameter('ros1_host').get_parameter_value().string_value
        self.ros1_port = self.get_parameter('ros1_port').get_parameter_value().integer_value

        self.get_logger().info(f'Connecting to rosbridge server at {self.ros1_host}:{self.ros1_port}')

        self.ros1_client = roslibpy.Ros(host=self.ros1_host, port=self.ros1_port)
        self.ros1_client.run()

        self.ros1_cmd_vel_pub = roslibpy.Topic(self.ros1_client, '/cmd_vel', 'geometry_msgs/Twist')

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'Forwarding: {msg}')
        ros1_msg = roslibpy.Message({
            'linear': {
                'x': msg.linear.x,
                'y': msg.linear.y,
                'z': msg.linear.z
            },
            'angular': {
                'x': msg.angular.x,
                'y': msg.angular.y,
                'z': msg.angular.z
            }
        })
        self.ros1_cmd_vel_pub.publish(ros1_msg)

    def shutdown(self):
        self.get_logger().info('Shutting down, sending zero velocity...')
        zero_vel = roslibpy.Message({
            'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        })
        self.ros1_cmd_vel_pub.publish(zero_vel)
        self.ros1_client.terminate()


def main(args=None):
    rclpy.init(args=args)
    cmd_vel_forwarder = CmdVelForwarder()
    rclpy.spin(cmd_vel_forwarder)
    cmd_vel_forwarder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
