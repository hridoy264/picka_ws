import rclpy
from rclpy.node import Node

from picka_interfaces.srv import CalculateIK

from picka_kinematics.inverse_kinematics_node import (
    InverseKinematicsSolver
)


class IKServer(Node):

    def __init__(self):

        super().__init__('ik_server')

        self.solver = InverseKinematicsSolver()

        self.service = self.create_service(
            CalculateIK,
            'calculate_ik',
            self.calculate_ik_callback
        )

        self.get_logger().info(
            'Inverse Kinematics server is ready.'
        )

    def calculate_ik_callback(
        self,
        request,
        response
    ):

        target = [
            request.x,
            request.y,
            request.z
        ]

        self.get_logger().info(
            f'IK request: '
            f'x={request.x:.4f}, '
            f'y={request.y:.4f}, '
            f'z={request.z:.4f}'
        )

        success, joint_angles, error = (
            self.solver.solve(target)
        )

        response.success = bool(success)

        response.joint_angles = [
            float(q)
            for q in joint_angles
        ]

        response.position_error = float(error)

        if success:

            response.message = (
                'IK solution found successfully.'
            )

            joint_degrees = [
                round(float(q), 2)
                for q in
                __import__('numpy').degrees(joint_angles)
            ]

            self.get_logger().info(
                f'IK solution [deg]: {joint_degrees}'
            )

        else:

            response.message = (
                'IK solver could not reach the requested '
                'position within tolerance.'
            )

            self.get_logger().warning(
                f'IK failed. Final error: '
                f'{error:.6f} m'
            )

        return response


def main(args=None):

    rclpy.init(args=args)

    node = IKServer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()