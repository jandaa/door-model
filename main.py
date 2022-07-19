from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt
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


@dataclass
class CarPose:
    pitch: float = 0
    roll: float = 0


class DoorModel:
    """
    Computes the torques for a given set of door hinge parameters with pitch and roll
    """

    def __init__(self):

        # Parameters from OEM
        self.hinge_upper_point = np.array([753.367, 896.342, 512.62]) * MM_TO_M
        self.hinge_lower_point = np.array([749.783, 910.696, 101.85]) * MM_TO_M
        self.center_of_mass = np.array([1252.737, 845.036, 400.716]) * MM_TO_M
        self.mass_in_kg = 36.1736
        self.max_door_angle_in_degrees = 67

        # Parameters of model
        self.num_door_angles = 30

        # Derived values
        self.force_gravity = np.array([0, 0, self.mass_in_kg * G])
        self.max_door_angle = math.radians(self.max_door_angle_in_degrees)

        # Preprocessing
        self._compute_hinge_reference_frame()

    def _compute_hinge_reference_frame(self):
        """Compute reference frame of hinge w.r.t. inertial frame."""

        # Compute rotational axis around which the door rotates
        rotation_axis = self.hinge_upper_point - self.hinge_lower_point
        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)

        # Compute the closest point around which the center of mass rotates
        # This will become the origin of all reference frames
        self.center_of_mass = self.center_of_mass - self.hinge_lower_point
        origin = self.center_of_mass.dot(rotation_axis) * rotation_axis

        # Compute the axis from center of mass to origin
        # needs to going the other way
        cm_to_origin = self.center_of_mass - origin
        cm_to_origin_distance = np.linalg.norm(cm_to_origin)
        cm_to_origin = cm_to_origin / cm_to_origin_distance

        # Compute the hinge reference frame
        # using the rotation axis and cm_to_origin
        # and computing cross product between them
        hinge_y = np.cross(rotation_axis, cm_to_origin)
        R_hinge_to_car = np.vstack([cm_to_origin, hinge_y, rotation_axis])
        R_car_to_hinge = R_hinge_to_car.T

        self.R_car_to_hinge = R_car_to_hinge
        self.cm_to_origin_distance = cm_to_origin_distance

    def get_rotation_inertial_to_car(self, roll, pitch):
        return R_x(roll) @ R_y(pitch)

    def get_rotation_inertial_to_hinge(self, R_inertial_to_car):
        return R_inertial_to_car @ self.R_car_to_hinge

    @property
    def angles(self):
        return np.linspace(0, self.max_door_angle, num=self.num_door_angles)

    def visualize_in_3D(self, roll, pitch):

        ax = plt.axes(projection="3d")

        # Compute rotation matrices
        R_inertial_to_car = self.get_rotation_inertial_to_car(roll, pitch)
        R_inertial_to_hinge = self.get_rotation_inertial_to_hinge(R_inertial_to_car)

        # Plot hinge frame
        num_points = 2
        x = np.linspace(0, 1, num=num_points)
        for i in range(3):
            points = x * R_inertial_to_hinge[:, i].reshape(3, 1)
            if i == 0:
                ax.plot3D(points[0], points[1], points[2], "blue", label="Door Frame")
            else:
                ax.plot3D(points[0], points[1], points[2], "blue")

        # Plot inertial frame
        num_points = 2
        zeros = np.zeros(num_points)
        ones = np.linspace(0, 1, num=num_points)
        ax.plot3D(ones, zeros, zeros, "red", label="Inertial Frame")
        ax.plot3D(zeros, ones, zeros, "red")
        ax.plot3D(zeros, zeros, ones, "red")

        # Plot car frame
        fx = R_inertial_to_car @ np.vstack([ones, zeros, zeros])
        fy = R_inertial_to_car @ np.vstack([zeros, ones, zeros])
        fz = R_inertial_to_car @ np.vstack([zeros, zeros, ones])
        ax.plot3D(fx[0], fx[1], fx[2], "teal", label="Car Frame - Car Rear")
        ax.plot3D(fy[0], fy[1], fy[2], "orange", label="Car Frame")
        ax.plot3D(fz[0], fz[1], fz[2], "orange")

        # Compute the path of the door in hinge frame
        angles = self.angles
        points = self.cm_to_origin_distance * np.array(
            [
                np.cos(angles),
                np.sin(angles),
                np.zeros(self.angles.size),
            ]
        )

        # Convert door path to the inertial frame
        points = R_inertial_to_hinge @ points

        ax.plot3D(points[0], points[1], points[2], "purple", label="Door Motion")
        ax.axis([-1, 1, -1, 1])

        plt.legend()
        plt.show()

    def compute_torque_along_door_motion(self, roll, pitch):

        # Compute rotation matrices
        R_inertial_to_car = self.get_rotation_inertial_to_car(roll, pitch)
        R_inertial_to_hinge = self.get_rotation_inertial_to_hinge(R_inertial_to_car)

        angles = self.angles

        torques = []
        for point_ind in range(self.num_door_angles):

            # Compute tangent vectors
            # by applying the inertial frame on the derivative of the door motion
            tangent = R_inertial_to_hinge @ np.array(
                [np.sin(angles[point_ind]), -np.cos(angles[point_ind]), 0]
            )

            # project gravity onto tangent vector
            gravity_in_hinge_direction = (
                self.force_gravity.dot(tangent) * tangent / np.linalg.norm(tangent)
            )

            # Compute torque as force * lever arm (Nm)
            torque = (
                np.linalg.norm(gravity_in_hinge_direction) * self.cm_to_origin_distance
            )
            torques.append(torque)

        return torques

    def plot_torques(self, poses: list[CarPose]):

        ax = plt.axes()
        angles = self.angles
        for pose in poses:
            torques = self.compute_torque_along_door_motion(
                roll=pose.roll, pitch=pose.pitch
            )
            ax.plot(
                angles * 180 / math.pi,
                torques,
                label=f"Roll: {math.degrees(pose.roll)}, Pitch: {math.degrees(pose.pitch)}",
            )

        plt.title(f"Hinge Torque")
        ax.tick_params(width=2)
        ax.set_xlabel("Door Position From Closed (Degrees)")
        ax.set_ylabel("Torque at Hinge (Nm)")
        plt.legend()
        plt.show()


if __name__ == "__main__":

    door_model = DoorModel()
    door_model.visualize_in_3D(pitch=math.radians(11.8), roll=0)
    door_model.plot_torques(
        [
            CarPose(pitch=math.radians(11.8), roll=0),
            CarPose(pitch=0, roll=math.radians(6.27)),
        ]
    )

    door_model.plot_torques(
        [
            CarPose(pitch=0, roll=0),
        ]
    )
