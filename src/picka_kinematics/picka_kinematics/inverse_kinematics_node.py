import math
import numpy as np

from picka_kinematics.forward_kinematics_node import forward_kinematics


class InverseKinematicsSolver:
    """
    Numerical inverse kinematics using Damped Least Squares.

    Joint order:
        0 -> base_joint
        1 -> shoulder_joint
        2 -> elbow_joint
        3 -> wrist_joint

    Internally all angles are radians.
    """

    def __init__(self):

        # Joint limits in radians
        self.lower_limits = np.radians([
            -160.0,   # base
            -45.0,    # shoulder
            0.0,      # elbow
            -80.0     # wrist
        ])

        self.upper_limits = np.radians([
            160.0,
            90.0,
            120.0,
            80.0
        ])

        # Numerical IK parameters
        self.max_iterations = 500
        self.tolerance = 0.001       # 1 mm
        self.damping = 0.05
        self.delta = 1e-5

        # Limit how much a joint can change in one iteration
        self.max_joint_step = math.radians(5.0)

    def get_position(self, joint_angles):

        angles_deg = np.degrees(joint_angles).tolist()

        T, _ = forward_kinematics(angles_deg)

        T = np.asarray(T, dtype=float)

        return T[:3, 3]

    def calculate_jacobian(self, joint_angles):
        """
        Numerically calculate the translational Jacobian.

        J has dimensions:

             3 x 4

        because:
            output = x, y, z
            inputs = q1, q2, q3, q4
        """

        n = len(joint_angles)

        J = np.zeros((3, n))

        current_position = self.get_position(joint_angles)

        for i in range(n):

            perturbed = joint_angles.copy()

            perturbed[i] += self.delta

            new_position = self.get_position(perturbed)

            J[:, i] = (
                new_position - current_position
            ) / self.delta

        return J

    def solve(self, target_position, initial_guess=None):
        """
        Solve inverse kinematics.

        Parameters
        ----------
        target_position:
            [x, y, z] in meters

        initial_guess:
            optional joint configuration in radians

        Returns
        -------
        success
        joint_angles
        final_error
        """

        target_position = np.asarray(
            target_position,
            dtype=float
        )

        if initial_guess is None:

            # Reasonable default starting position
            q = np.radians([
                0.0,
                20.0,
                60.0,
                0.0
            ])

        else:

            q = np.asarray(
                initial_guess,
                dtype=float
            )

        # Make sure starting values satisfy joint limits
        q = np.clip(
            q,
            self.lower_limits,
            self.upper_limits
        )

        for iteration in range(self.max_iterations):

            current_position = self.get_position(q)

            error_vector = (
                target_position - current_position
            )

            error = np.linalg.norm(error_vector)

            # -------------------------------
            # Solution found
            # -------------------------------

            if error < self.tolerance:

                return (
                    True,
                    q,
                    error
                )

            # -------------------------------
            # Calculate Jacobian
            # -------------------------------

            J = self.calculate_jacobian(q)

            # Damped Least-Squares inverse:
            #
            # dq =
            # J^T (J J^T + λ²I)^-1 e
            #

            identity = np.eye(3)

            A = (
                J @ J.T
                + (self.damping ** 2) * identity
            )

            try:

                dq = (
                    J.T
                    @ np.linalg.solve(
                        A,
                        error_vector
                    )
                )

            except np.linalg.LinAlgError:

                return (
                    False,
                    q,
                    error
                )

            # Prevent huge jumps
            dq = np.clip(
                dq,
                -self.max_joint_step,
                self.max_joint_step
            )

            # Update joints
            q += dq

            # Enforce physical joint limits
            q = np.clip(
                q,
                self.lower_limits,
                self.upper_limits
            )

        # Failed to converge
        final_position = self.get_position(q)

        final_error = np.linalg.norm(
            target_position - final_position
        )

        return (
            False,
            q,
            final_error
        )