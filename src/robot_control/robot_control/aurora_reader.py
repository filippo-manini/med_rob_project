import rclpy
from rclpy.node import Node

from aurora_ndi_ros2_driver.msg import AuroraData


class AuroraReader(Node):

    def __init__(self):
        super().__init__('aurora_reader')

        self.subscription = self.create_subscription(
            AuroraData,
            '/aurora/sensor0',
            self.aurora_callback,
            10
        )

        self.get_logger().info(
            'Aurora reader avviato. In ascolto su /aurora/sensor0'
        )

    def aurora_callback(self, msg):

        position = msg.position
        orientation = msg.orientation

        self.get_logger().info(
            f'Position: '
            f'x={position.x:.3f}, '
            f'y={position.y:.3f}, '
            f'z={position.z:.3f} | '
            f'Visible={msg.visible} | '
            f'Error={msg.error:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = AuroraReader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
