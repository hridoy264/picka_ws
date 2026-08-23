import numpy as np


# ============================================================
# Joint limits in degrees
# ============================================================

JOINT_LIMITS = [
    (-160.0, 160.0),   # base
    (-45.0, 90.0),     # shoulder
    (0.0, 120.0),      # elbow
    (-80.0, 80.0),     # wrist
]


# ============================================================
# Basic homogeneous transformations
# ============================================================

def translation_matrix(x, y, z):
    """Create a 4x4 translation matrix."""

    T = np.eye(4)

    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z

    return T


def axis_angle_matrix(axis, angle):
    """
    Create homogeneous rotation matrix for rotation around
    an arbitrary axis using Rodrigues' rotation formula.

    Parameters
    ----------
    axis:
        [x, y, z]

    angle:
        radians
    """

    axis = np.asarray(axis, dtype=float)

    axis = axis / np.linalg.norm(axis)

    x, y, z = axis

    c = np.cos(angle)
    s = np.sin(angle)
    C = 1.0 - c

    R = np.array([
        [
            c + x*x*C,
            x*y*C - z*s,
            x*z*C + y*s
        ],
        [
            y*x*C + z*s,
            c + y*y*C,
            y*z*C - x*s
        ],
        [
            z*x*C - y*s,
            z*y*C + x*s,
            c + z*z*C
        ]
    ])

    T = np.eye(4)

    T[:3, :3] = R

    return T


# ============================================================
# Forward kinematics
# ============================================================

def forward_kinematics(
    joint_angles,
    angles_in_degrees=True
):
    """
    Exact FK matching the current URDF/Xacro model.

    Joint order:
        q1 = base_joint
        q2 = shoulder_joint
        q3 = elbow_joint
        q4 = wrist_joint

    Returns transform:

        base_link -> wrist_1

    Parameters
    ----------
    joint_angles:
        [q1, q2, q3, q4]

    angles_in_degrees:
        True  -> input angles are degrees
        False -> input angles are radians
    """

    if len(joint_angles) != 4:
        raise ValueError(
            "Exactly four joint angles are required."
        )

    q = np.asarray(
        joint_angles,
        dtype=float
    )

    if angles_in_degrees:
        q = np.radians(q)

    q1, q2, q3, q4 = q

    # Start at base_link
    T = np.eye(4)

    transformations = []

    # ========================================================
    # base_link -> stepper_1
    # fixed joint
    # ========================================================

    T = T @ translation_matrix(
        -0.073,
        0.0,
        0.0211
    )

    # ========================================================
    # base_joint
    #
    # stepper_1 -> bracket_1
    #
    # origin:
    #   xyz = [0, 0, 0.0675]
    #
    # axis:
    #   [0, 0, 1]
    # ========================================================

    T = (
        T
        @ translation_matrix(
            0.0,
            0.0,
            0.0675
        )
        @ axis_angle_matrix(
            [0.0, 0.0, 1.0],
            q1
        )
    )

    transformations.append(T.copy())

    # ========================================================
    # bracket_1 -> shoulder_1
    # fixed
    # ========================================================

    T = T @ translation_matrix(
        0.0,
        0.0,
        0.005
    )

    # ========================================================
    # shoulder_joint
    #
    # shoulder_1 -> backarm_1
    #
    # origin:
    #   [0.01, -0.012153, 0.095]
    #
    # axis:
    #   [-1, 0, 0]
    # ========================================================

    T = (
        T
        @ translation_matrix(
            0.01,
            -0.012153,
            0.095
        )
        @ axis_angle_matrix(
            [-1.0, 0.0, 0.0],
            q2
        )
    )

    transformations.append(T.copy())

    # ========================================================
    # backarm_1 -> forearm_servo_1
    # fixed
    # ========================================================

    T = T @ translation_matrix(
        -0.085,
        -0.055439,
        0.259573
    )

    # ========================================================
    # elbow_joint
    #
    # forearm_servo_1 -> forearm_horn_1
    #
    # origin:
    #   [0.050401, 0.000193, -0.181193]
    #
    # axis:
    #   [1, 0, 0]
    # ========================================================

    T = (
        T
        @ translation_matrix(
            0.050401,
            0.000193,
            -0.181193
        )
        @ axis_angle_matrix(
            [1.0, 0.0, 0.0],
            q3
        )
    )

    transformations.append(T.copy())

    # ========================================================
    # forearm_horn_1 -> forearm_1
    # fixed
    # ========================================================

    T = T @ translation_matrix(
        -0.001,
        0.000005,
        0.000001
    )

    # ========================================================
    # forearm_1 -> wrist_servo_1
    # fixed
    # ========================================================

    T = T @ translation_matrix(
        -0.007717,
        0.035004,
        0.011029
    )

    # ========================================================
    # wrist_joint
    #
    # wrist_servo_1 -> wrist_horn_1
    #
    # origin:
    #   [0.000222, 0.040739, 0.020546]
    #
    # axis:
    #   [-0.005, -0.970284, -0.241919]
    #
    # Notice: this is a skew axis, so arbitrary-axis
    # Rodrigues rotation is required.
    # ========================================================

    T = (
        T
        @ translation_matrix(
            0.000222,
            0.040739,
            0.020546
        )
        @ axis_angle_matrix(
            [
                -0.005,
                -0.970284,
                -0.241919
            ],
            q4
        )
    )

    transformations.append(T.copy())

    # ========================================================
    # wrist_horn_1 -> wrist_1
    # fixed
    # ========================================================

    T = T @ translation_matrix(
        0.000005,
        0.000970,
        0.000242
    )

    return T, transformations


# ============================================================
# Test
# ============================================================

if __name__ == "__main__":

    test_cases = [
        [0.0, 0.0, 0.0, 0.0],
        [30.0, 20.0, 40.0, 10.0],
        [-30.0, 45.0, 60.0, -20.0],
    ]

    for q in test_cases:

        T, _ = forward_kinematics(q)

        print("\n--------------------------------")
        print("Joint angles:", q)

        print("\nTransformation:")
        print(np.round(T, 6))

        print("\nXYZ:")
        print(np.round(T[:3, 3], 6))