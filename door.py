import numpy as np
import matplotlib.pyplot as plt
import math

from rotations import R_x, R_y

# Conversion Constants
MM_TO_M = 1 / 1000

#############################################################
# Parameters of hinge frame
roll = 0.0
pitch = 0.0

# Parameters from OEM
hinge_upper_point = np.array([753.367, 896.342, 512.62]) * MM_TO_M
hinge_lower_point = np.array([749.783, 910.696, 101.85]) * MM_TO_M
center_of_mass = np.array([1252.737, 845.036, 400.716]) * MM_TO_M
mass_in_kg = 36.1736
max_door_angle_in_degrees = 67
#############################################################
### Convert parameters into values required for comp

# Compute rotational axis around which the door rotates
rotation_axis = hinge_upper_point - hinge_lower_point
rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)

# Compute the closest point around which the center of mass rotates
# This will become the origin of all reference frames
# origin = center_of_mass.dot(rotation_axis) * rotation_axis
center_of_mass = center_of_mass - hinge_lower_point
origin = center_of_mass.dot(rotation_axis) * rotation_axis

# Compute the axis from center of mass to origin
# needs to going the other way
cm_to_origin = center_of_mass - origin
cm_to_origin_distance = np.linalg.norm(cm_to_origin)
cm_to_origin = cm_to_origin / cm_to_origin_distance

# Compute the hinge reference frame
# using the rotation axis and cm_to_origin
# and computing cross product between them
hinge_x = np.cross(rotation_axis, cm_to_origin)
R_hinge_to_inertial = np.vstack([cm_to_origin, hinge_x, rotation_axis])
R_inertial_to_hinge = R_hinge_to_inertial.T

# Add roll and pitch
R_roll_pitch = R_x(roll) @ R_y(pitch)
R_inertial_to_hinge = R_roll_pitch @ R_inertial_to_hinge

#############################################################

fig = plt.figure()
ax = plt.axes(projection="3d")

# Plot hinge frame
num_points = 10
x = np.linspace(0, 1, num=num_points)
for i in range(3):
    points = x * R_inertial_to_hinge[:, i].reshape(3, 1)
    if i == 0:
        ax.plot3D(points[0], points[1], points[2], "blue", label="Door Frame")
    else:
        ax.plot3D(points[0], points[1], points[2], "blue")

# Plot inertial frame
num_points = 10
zeros = np.zeros(num_points)
ones = np.linspace(0, 1, num=num_points)
ax.plot3D(ones, zeros, zeros, "red", label="Inertial Frame")
ax.plot3D(zeros, ones, zeros, "red")
ax.plot3D(zeros, zeros, ones, "red")

# Plot car frame
fx = R_roll_pitch @ np.vstack([ones, zeros, zeros])
fy = R_roll_pitch @ np.vstack([zeros, ones, zeros])
fz = R_roll_pitch @ np.vstack([zeros, zeros, ones])
ax.plot3D(fx[0], fx[1], fx[2], "teal", label="Car Frame - Car Rear")
ax.plot3D(fy[0], fy[1], fy[2], "orange", label="Car Frame")
ax.plot3D(fz[0], fz[1], fz[2], "orange")

# Compute the path of the door in hinge frame
num_points = 100
angles = np.linspace(0, math.radians(max_door_angle_in_degrees), num=num_points)
points = cm_to_origin_distance * np.array(
    [
        np.cos(angles),
        np.sin(angles),
        np.zeros(num_points),
    ]
)

# Convert door path to the inertial frame
points = R_inertial_to_hinge @ points

ax.plot3D(points[0], points[1], points[2], "purple", label="Door Motion")
ax.axis([-1, 1, -1, 1])

plt.legend()
plt.show()


### OLD CODE
############

# # Euler angle equations
# def R_x(angle):
#     return np.array(
#         [
#             [1, 0, 0],
#             [0, np.cos(angle), -np.sin(angle)],
#             [0, np.sin(angle), np.cos(angle)],
#         ]
#     )


# def R_y(angle):
#     return np.array(
#         [
#             [np.cos(angle), 0, np.sin(angle)],
#             [0, 1, 0],
#             [-np.sin(angle), 0, np.cos(angle)],
#         ]
#     )


# def R_z(angle):
#     return np.array(
#         [
#             [np.cos(angle), -np.sin(angle), 0],
#             [np.sin(angle), np.cos(angle), 0],
#             [0, 0, 1],
#         ]
#     )

# # Convert points to inertial frame
# for i in range(num_points):
#     point = Vector3(x=points[0, i], y=points[1, i], z=points[2, i])
#     point = R_inertial_to_hinge * point
#     points[0, i] = point.x
#     points[1, i] = point.y
#     points[2, i] = point.z

# ax.plot3D(points[0], points[1], points[2], label="Door Motion")

# # Add start point to door motion
# angle = 0
# start = np.array(
#     np.linspace(0, 1, num=50).reshape(-1, 1)
#     * [
#         np.cos(pitch) * np.sin(angle),
#         np.sin(roll) * np.sin(pitch) * np.sin(angle) + np.cos(roll) * np.cos(angle),
#         -np.cos(roll) * np.sin(pitch) * np.sin(angle) + np.sin(roll) * np.cos(angle),
#     ]
# )

# ax.plot3D(start[0], start[1], start[2], "cyan", label="start")

# # Try building my own rotation matrix
# R1 = R_y(pitch)
# R2 = R_x(roll)
# R_mine = R2 @ R1

# l = 0.8
# points_verify = np.array(
#     [
#         l * np.sin(angles),
#         l * np.cos(angles),
#         np.zeros(num_points),
#     ]
# )
# points_verify = R_mine @ points_verify

# ax.plot3D(
#     points_verify[0],
#     points_verify[1],
#     points_verify[2],
#     "orange",
#     label="Door Motion - My Rotation",
# )


# Compute rotation matrix
# R_inertial_to_hinge = Rotation(roll=roll, pitch=pitch, yaw=0)

# # Plot door frame
# num_points = 10
# x = np.linspace(0, 1, num=num_points)
# for axis_ind, axis in enumerate(
#     [copy.deepcopy(hinge_x), copy.deepcopy(fi_y), copy.deepcopy(fi_z)]
# ):

#     for i in range(num_points):
#         point = Vector3(x=axis[0, i], y=axis[1, i], z=axis[2, i])
#         point = R_inertial_to_hinge * point
#         axis[0, i] = point.x
#         axis[1, i] = point.y
#         axis[2, i] = point.z

#     if axis_ind == 0:
#         ax.plot3D(axis[0], axis[1], axis[2], "cyan", label="Car Front")
#     elif axis_ind == 1:
#         ax.plot3D(axis[0], axis[1], axis[2], "blue", label="Hinge Frame")
#     else:
#         ax.plot3D(axis[0], axis[1], axis[2], "blue")

# rotation_axis_points = x * rotation_axis.reshape(3, 1)
# ax.plot3D(
#     rotation_axis_points[0],
#     rotation_axis_points[1],
#     rotation_axis_points[2],
#     "brown",
#     label="Rotation axis",
# )

# # Plot the initial door location
# cm_to_origin_points = x * cm_to_origin.reshape(3, 1)
# ax.plot3D(
#     cm_to_origin_points[0],
#     cm_to_origin_points[1],
#     cm_to_origin_points[2],
#     "green",
#     label="Initial door position",
# )

# hinge_x_points = x * hinge_x.reshape(3, 1)
# ax.plot3D(
#     hinge_x_points[0],
#     hinge_x_points[1],
#     hinge_x_points[2],
#     "brown",
#     label="Door frame",
# )
