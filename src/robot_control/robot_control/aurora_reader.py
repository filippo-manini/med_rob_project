import rclpy
from rclpy.node import Node

from aurora_ndi_ros2_driver.msg import AuroraData


class AuroraReader(Node):

    def __init__(self):
        super().__init__('aurora_reader')

        # Parameters coming from the Aurora configuration YAML
        self.declare_parameter('num_sensors', 1)
        self.declare_parameter(
            'topic_names',
            ['/aurora/sensor0']
        )

        num_sensors = self.get_parameter('num_sensors').value
        topic_names = self.get_parameter('topic_names').value

        # The YAML currently defines topic_names as a nested list:
        #
        # topic_names:
        #   - ["aurora/sensor0", "aurora/sensor1"]
        #
        # Flatten it if necessary.
        if (
            len(topic_names) == 1
            and isinstance(topic_names[0], list)
        ):
            topic_names = topic_names[0]

        # Check configuration consistency
        if len(topic_names) != num_sensors:
            raise ValueError(
                f'Configuration error: num_sensors={num_sensors}, '
                f'but {len(topic_names)} topics were provided.'
            )

        # Keep references to all subscriptions
        self.sensor_subscriptions = []

        # Create one subscription for each sensor
        for sensor_id, topic in enumerate(topic_names):

            subscription = self.create_subscription(
                AuroraData,
                topic,
                lambda msg, sensor_id=sensor_id:
                    self.aurora_callback(msg, sensor_id),
                10
            )

            self.sensor_subscriptions.append(subscription)

            self.get_logger().info(
                f'Sensor {sensor_id}: listening on {topic}'
            )

        self.get_logger().info(
            f'Aurora reader started. '
            f'Listening to {num_sensors} sensor(s).'
        )

    def aurora_callback(self, msg, sensor_id):

        position = msg.position
        orientation = msg.orientation

        self.get_logger().info(
            f'[sensor{sensor_id}] '
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
