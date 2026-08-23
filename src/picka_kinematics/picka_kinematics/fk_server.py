import math

import numpy as np
import rclpy
from rclpy.node import Node

from picka_interfaces.srv import CalculateFK
from picka_kinematics.forward_kinematics_node import forward_kinematics


def rotation_matrix_to_rpy(rotation):
    r20 = float(np.clip(rotation[2, 0], -1.0, 1.0))

    if abs(r20) < 1.0 - 1e-9:
        pitch = math.asin(-r20)
        roll = math.atan2(
            rotation[2, 1],
            rotation[2, 2]
        )
        yaw = math.atan2(
            rotation[1, 0],
            rotation[0, 0]
        )

    else:
        pitch = (
            math.pi / 2
            if r20 <= -1.0
            else -math.pi / 2
        )

        roll = 0.0

        yaw = math.atan2(
            -rotation[0, 1],
            rotation[1, 1]
        )

    return roll, pitch, yaw


class FKServer(Node):

    def __init__(self):
        super().__init__('fk_server')

        self.service = self.create_service(
            CalculateFK,
            'calculate_fk',
            self.calculate_fk_callback
        )

        self.get_logger().info(
            'FK service is ready.'
        )

    def calculate_fk_callback(
        self,
        request,
        response
    ):
        try:
            joint_angles = list(
                request.joint_angles
            )

            if len(joint_angles) != 4:
                raise ValueError(
                    f'Expected 4 arm-joint angles, '
                    f'received {len(joint_angles)}.'
                )

            # New URDF-based FK
            transform, _ = forward_kinematics(
                joint_angles=joint_angles,
                angles_in_degrees=True,
            )

            # XYZ
            position = transform[:3, 3]

            # Rotation matrix
            rotation = transform[:3, :3]

            roll, pitch, yaw = (
                rotation_matrix_to_rpy(rotation)
            )

            response.x = float(position[0])
            response.y = float(position[1])
            response.z = float(position[2])

            # Return orientation in degrees
            response.roll = math.degrees(roll)
            response.pitch = math.degrees(pitch)
            response.yaw = math.degrees(yaw)

            response.success = True

            response.message = (
                'Forward kinematics calculated successfully.'
            )

            self.get_logger().info(
                f'FK request: {joint_angles} degrees'
            )

        except Exception as error:

            response.success = False
            response.message = str(error)

            self.get_logger().error(
                f'FK calculation failed: {error}'
            )

        return response


def main(args=None):
    rclpy.init(args=args)

    node = FKServer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()