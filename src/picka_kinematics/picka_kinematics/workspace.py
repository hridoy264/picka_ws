import numpy as np
import matplotlib.pyplot as plt

from picka_kinematics.forward_kinematics_node import forward_kinematics


# Joint samples
q1_values = np.linspace(-160, 160, 15)
q2_values = np.linspace(-45, 90, 12)
q3_values = np.linspace(0, 120, 12)
q4_values = np.linspace(-80, 80, 10)

x_points = []
y_points = []
z_points = []


for q1 in q1_values:
    for q2 in q2_values:
        for q3 in q3_values:
            for q4 in q4_values:

                T, _ = forward_kinematics(
                    [q1, q2, q3, q4],
                    angles_in_degrees=True
                )

                x_points.append(T[0, 3])
                y_points.append(T[1, 3])
                z_points.append(T[2, 3])


fig = plt.figure()

ax = fig.add_subplot(
    111,
    projection='3d'
)

ax.scatter(
    x_points,
    y_points,
    z_points,
    s=2
)

ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')

ax.set_title('Robotic Arm Reachable Workspace')

plt.show()