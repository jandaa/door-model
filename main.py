from ctypes import c_char_p
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Cursor
import math

from rotations import R_x, R_y

# Constants
MM_TO_M = 1 / 1000
G = 9.81  # Force of Gravity (m/s^2)

#############################################################
# Parameters of hinge frame
# roll = math.radians(6.27)
roll = 0
pitch = math.radians(11.8)
# pitch = 0

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
hinge_y = np.cross(rotation_axis, cm_to_origin)
R_hinge_to_car = np.vstack([cm_to_origin, hinge_y, rotation_axis])
R_car_to_hinge = R_hinge_to_car.T

# Add roll and pitch
R_roll_pitch = R_x(roll) @ R_y(pitch)
R_inertial_to_hinge = R_roll_pitch @ R_car_to_hinge

# # Verify my rotation matrix
# # CORRECT
# c_p = np.cos(pitch)
# s_p = np.sin(pitch)
# c_r = np.cos(roll)
# s_r = np.sin(roll)
# my_R_roll_pitch = np.array(
#     [[c_p, 0, s_p], [s_r * s_p, c_r, -s_r * c_p], [-c_r * s_p, s_r, c_r * c_p]]
# )

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

# # Compute the path using my equation
# my_points = cm_to_origin_distance * np.array(
#     [
#         np.cos(pitch) * np.cos(angles),
#         np.sin(roll) * np.sin(pitch) * np.cos(angles) + np.cos(roll) * np.sin(angles),
#         -np.cos(roll) * np.sin(pitch) * np.cos(angles) + np.sin(roll) * np.sin(angles),
#     ]
# )
# my_points = R_car_to_hinge @ my_points
# ax.plot3D(my_points[0], my_points[1], my_points[2], "green", label="Door Motion - mine")

plt.legend()
plt.show()

#############################################################
### Compute the torque at each point numerically

# Compute torques using lagrangian
# torque_eq = lambda x: (
#     R_inertial_to_hinge[2, 1] * np.sin(x) - R_inertial_to_hinge[2, 1] * np.cos(x)
# )
# my_torques = mass_in_kg * G * np.apply_along_axis(torque_eq, 0, angles)

torques = []
gravity = np.array([0, 0, mass_in_kg * G])
for point_ind in range(num_points):

    # Compute tangent vectors
    tangent = (
        R_inertial_to_hinge
        @ np.array([np.sin(angles[point_ind]), -np.cos(angles[point_ind]), 0])
        # + points[:, point_ind]
    )

    # project gravity onto tangent vector
    gravity_in_hinge_direction = (
        gravity.dot(tangent) * tangent / np.linalg.norm(tangent)
    )

    # Compute torque as force * lever arm (Nm)
    torque = np.linalg.norm(gravity_in_hinge_direction) * cm_to_origin_distance

    torques.append(torque)

    # ax.plot3D(
    #     [tangent[0], points[0, point_ind]],
    #     [tangent[1], points[1, point_ind]],
    #     [tangent[2], points[2, point_ind]],
    #     "cyan",
    #     label="Tangent",
    # )

# plot torque curves
fig = plt.figure()
ax = plt.axes()

ax.plot(angles * 180 / math.pi, torques, label="Torque")
# ax.plot(angles * 180 / math.pi, my_torques, label="Torque - Eq")
plt.title(f"Torque @ Pitch: {math.degrees(pitch)}, Roll: {math.degrees(roll)}")
ax.tick_params(width=2)
ax.set_xlabel("Door Position From Closed (degrees)")
ax.set_ylabel("Torque at Hinge (Nm)")
# plt.legend()
plt.show()

#############################################################
### Compute the torque at each point using equations


### OLD CODE
############

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
