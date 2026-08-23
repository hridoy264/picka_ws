import math

import rclpy
from rclpy.node import Node

from picka_interfaces.srv import CalculateFK
from picka_interfaces.srv import CalculateIK

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class PickaCLI(Node):

    def __init__(self):
        super().__init__('picka_cli')

        # -------------------------
        # FK service client
        # -------------------------
        self.fk_client = self.create_client(
            CalculateFK,
            'calculate_fk'
        )

        # -------------------------
        # IK service client
        # -------------------------
        self.ik_client = self.create_client(
            CalculateIK,
            'calculate_ik'
        )

        # -------------------------
        # Gazebo trajectory publisher
        # -------------------------
        self.arm_command_publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10,
        )

    # ==========================================================
    # FK
    # ==========================================================

    def wait_for_fk_server(self):

        while not self.fk_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                'Waiting for FK server...'
            )

    def request_fk(self, joint_angles):

        request = CalculateFK.Request()

        # FK expects joint angles in degrees
        request.joint_angles = joint_angles

        future = self.fk_client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        return future.result()

    # ==========================================================
    # IK
    # ==========================================================

    def wait_for_ik_server(self):

        while not self.ik_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                'Waiting for IK server...'
            )

    def request_ik(self, x, y, z):

        request = CalculateIK.Request()

        request.x = x
        request.y = y
        request.z = z

        future = self.ik_client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        return future.result()

    # ==========================================================
    # Gazebo
    # ==========================================================

    def move_gazebo_arm(self, joint_angles_degrees):

        if len(joint_angles_degrees) != 4:
            raise ValueError(
                'Exactly four arm angles are required.'
            )

        trajectory = JointTrajectory()

        trajectory.joint_names = [
            'base_joint',
            'shoulder_joint',
            'elbow_joint',
            'wrist_joint',
        ]

        point = JointTrajectoryPoint()

        # ros2_control expects radians
        point.positions = [
            math.radians(angle)
            for angle in joint_angles_degrees
        ]

        point.time_from_start = Duration(
            sec=2,
            nanosec=0,
        )

        trajectory.points = [point]

        self.arm_command_publisher.publish(
            trajectory
        )

        self.get_logger().info(
            f'Gazebo command sent: '
            f'{joint_angles_degrees} degrees'
        )


# ==============================================================
# Input functions
# ==============================================================

def read_joint_angles():

    user_input = input(
        'Enter θ1 θ2 θ3 θ4 in degrees, separated by spaces: '
    )

    values = [
        float(value)
        for value in user_input.split()
    ]

    if len(values) != 4:
        raise ValueError(
            'You must enter exactly four arm-joint angles.'
        )

    return values


def read_target_position():

    user_input = input(
        'Enter target x y z in meters, separated by spaces: '
    )

    values = [
        float(value)
        for value in user_input.split()
    ]

    if len(values) != 3:
        raise ValueError(
            'You must enter exactly x y z.'
        )

    return values


# ==============================================================
# Main
# ==============================================================

def main(args=None):

    rclpy.init(args=args)

    node = PickaCLI()

    try:

        print('\nRobotic Arm Kinematics')
        print('1. Forward Kinematics')
        print('2. Inverse Kinematics')
        print('0. Exit')

        choice = input(
            'Select an operation: '
        ).strip()

        # ======================================================
        # FORWARD KINEMATICS
        # ======================================================

        if choice == '1':

            joint_angles = read_joint_angles()

            node.wait_for_fk_server()

            response = node.request_fk(
                joint_angles
            )

            if response is None:

                print(
                    'The FK service returned no response.'
                )

            elif response.success:

                print('\nEnd-effector pose')

                print(
                    f'x     = {response.x:.4f} m'
                )

                print(
                    f'y     = {response.y:.4f} m'
                )

                print(
                    f'z     = {response.z:.4f} m'
                )

                print(
                    f'roll  = {response.roll:.2f} degrees'
                )

                print(
                    f'pitch = {response.pitch:.2f} degrees'
                )

                print(
                    f'yaw   = {response.yaw:.2f} degrees'
                )

                # Move Gazebo using entered angles
                node.move_gazebo_arm(
                    joint_angles
                )

                rclpy.spin_once(
                    node,
                    timeout_sec=0.5
                )

            else:

                print(
                    f'FK failed: {response.message}'
                )

        # ======================================================
        # INVERSE KINEMATICS
        # ======================================================

        elif choice == '2':

            x, y, z = read_target_position()

            node.wait_for_ik_server()

            response = node.request_ik(
                x,
                y,
                z
            )

            if response is None:

                print(
                    'The IK service returned no response.'
                )

            elif response.success:

                # IK server returns radians
                joint_angles_radians = (
                    response.joint_angles
                )

                # Convert for display and existing Gazebo function
                joint_angles_degrees = [
                    math.degrees(angle)
                    for angle in joint_angles_radians
                ]

                print('\nIK solution')

                print(
                    f'θ1 = {joint_angles_degrees[0]:.2f}°'
                )

                print(
                    f'θ2 = {joint_angles_degrees[1]:.2f}°'
                )

                print(
                    f'θ3 = {joint_angles_degrees[2]:.2f}°'
                )

                print(
                    f'θ4 = {joint_angles_degrees[3]:.2f}°'
                )

                print(
                    f'Position error = '
                    f'{response.position_error:.6f} m'
                )

                # Move arm to IK solution
                node.move_gazebo_arm(
                    joint_angles_degrees
                )

                rclpy.spin_once(
                    node,
                    timeout_sec=0.5
                )

            else:

                print(
                    f'IK failed: {response.message}'
                )

                print(
                    f'Final position error: '
                    f'{response.position_error:.6f} m'
                )

        elif choice == '0':

            print('Exiting.')

        else:

            print('Invalid selection.')

    except ValueError as error:

        print(
            f'Input error: {error}'
        )

    except KeyboardInterrupt:

        print(
            '\nInterrupted by user.'
        )

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()