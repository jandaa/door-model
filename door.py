import numpy as np
import matplotlib.pyplot as plt
from rigid_transform import Rotation, Vector3
import math
import copy

# Conversion Constants
MM_TO_M = 1 / 1000

#############################################################
# Parameters of hinge frame
roll = 0
pitch = 0

exaggerate_angle = 0

# Parameters from OEM
hinge_upper_point = np.array([753.367, 896.342 - exaggerate_angle, 512.62]) * MM_TO_M
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
center_of_mass = center_of_mass
origin = center_of_mass.dot(rotation_axis) * rotation_axis

# Compute the axis from center of mass to origin
# needs to going the other way
cm_to_origin = center_of_mass - origin
l = np.linalg.norm(cm_to_origin)
cm_to_origin = cm_to_origin / np.linalg.norm(cm_to_origin)

# Compute the hinge reference frame
# using the rotation axis and cm_to_origin
# and computing cross product between them
hinge_x = np.cross(cm_to_origin, rotation_axis)
R_hinge_to_inertial = np.vstack([hinge_x, cm_to_origin, rotation_axis])
R_inertial_to_hinge = R_hinge_to_inertial.T

# Compute the starting angle between
to_axis = np.array([1, 0, 0])
starting_angle = np.arccos(cm_to_origin.dot(to_axis) / l)

#############################################################

fig = plt.figure()
ax = plt.axes(projection="3d")

# Plot hinge frame
num_points = 10
x = np.linspace(0, 1, num=num_points)
for i in range(3):
    points = x * R_inertial_to_hinge[:, i].reshape(3, 1)
    ax.plot3D(points[0], points[1], points[2], "blue", label="Car Front")

# Plot inertial frame
num_points = 10
zeros = np.zeros(num_points)
ones = np.linspace(0, 1, num=num_points)
ax.plot3D(ones, zeros, zeros, "cyan", label="Inertial Frame - Car Rear")
ax.plot3D(zeros, ones, zeros, "red", label="Inertial Frame")
ax.plot3D(zeros, zeros, ones, "red")

# Create motion of door in hinge frame
num_points = 100
angles = np.linspace(-math.radians(max_door_angle_in_degrees), 0, num=num_points)
points = np.array(
    [
        l * np.cos(angles),
        l * np.sin(angles),
        np.zeros(num_points),
    ]
)

# my door motion equation
my_points = np.array(
    [
        np.cos(pitch) * np.sin(angles),
        np.sin(roll) * np.sin(pitch) * np.sin(angles) + np.cos(roll) * np.cos(angles),
        -np.cos(roll) * np.sin(pitch) * np.sin(angles) + np.sin(roll) * np.cos(angles),
    ]
)

# convert to door hinge
my_points = R_inertial_to_hinge @ my_points

ax.plot3D(
    my_points[0], my_points[1], my_points[2], "purple", label="Door Motion - My Eq"
)
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
