#!/usr/bin/env python3
"""
Republie /youbot/joint_states en retirant le prefixe Gazebo 'youbot::'
des noms d'articulations, pour que robot_state_publisher puisse les
faire correspondre aux noms definis dans youbot.urdf.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateDebridge(Node):
    def __init__(self):
        super().__init__('joint_state_debridge')
        self.sub = self.create_subscription(
            JointState, '/youbot/joint_states', self.callback, 10)
        self.pub = self.create_publisher(
            JointState, '/youbot/joint_states_clean', 10)
        self.get_logger().info('Joint state debridge demarre.')

    def callback(self, msg: JointState):
        clean = JointState()
        clean.header = msg.header
        clean.name = [n.split('::')[-1] for n in msg.name]
        clean.position = msg.position
        clean.velocity = msg.velocity
        clean.effort = msg.effort
        self.pub.publish(clean)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateDebridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
