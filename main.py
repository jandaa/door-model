from bdb import effective
from dataclasses import dataclass
import math
from this import d
import numpy as np
import matplotlib.pyplot as plt
import plotly.graph_objects as go


from rotations import R_x, R_y

# Constants
M_TO_MM = 1000
MM_TO_M = 1 / M_TO_MM
G = 9.81  # Force of Gravity (m/s^2)


@dataclass
class CarPose:
    pitch: float = 0
    roll: float = 0


# Define the different cases
PITCH_UP = CarPose(pitch=math.radians(11.8), roll=0)
PITCH_DOWN = CarPose(pitch=math.radians(-11.8), roll=0)
ROLL_PASSENGER_SIDE = CarPose(pitch=0, roll=math.radians(-6.27))
ROLL_DRIVER_SIDE = CarPose(pitch=0, roll=math.radians(6.27))


class DoorModel:
    """
    Computes the torques for a given set of door hinge parameters with pitch and roll
    """

    def __init__(self):

        # Parameters from OEM
        self.hinge_upper_point = np.array([753.367, 896.342, 512.62]) * MM_TO_M
        self.hinge_lower_point = np.array([749.783, 910.696, 101.85]) * MM_TO_M
        self.body_pillar_point = np.array([840.641, 831.029, 365.224]) * MM_TO_M
        self.actuator_pivot_point = np.array([897.322, 840.001, 365.043]) * MM_TO_M
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
        self._compute_actuation_reference_frame()

    def _compute_hinge_reference_frame(self):
        """Compute reference frame of hinge w.r.t. inertial frame."""

        # Compute rotational axis around which the door rotates
        self.rotation_axis = self.hinge_upper_point - self.hinge_lower_point
        self.rotation_axis = self.rotation_axis / np.linalg.norm(self.rotation_axis)

        # Compute the closest point around which the center of mass rotates
        # This will become the origin of all reference frames
        self.center_of_mass = self.center_of_mass - self.hinge_lower_point
        self.origin = self.center_of_mass.dot(self.rotation_axis) * self.rotation_axis

        # Compute the axis from center of mass to origin
        # needs to going the other way
        cm_to_origin = self.center_of_mass - self.origin
        cm_to_origin_distance = np.linalg.norm(cm_to_origin)
        cm_to_origin = cm_to_origin / cm_to_origin_distance

        # Compute the hinge reference frame
        # using the rotation axis and cm_to_origin
        # and computing cross product between them
        hinge_y = np.cross(self.rotation_axis, cm_to_origin)
        R_hinge_to_car = np.vstack([cm_to_origin, hinge_y, self.rotation_axis])
        R_car_to_hinge = R_hinge_to_car.T

        self.R_car_to_hinge = R_car_to_hinge
        self.cm_to_origin_distance = cm_to_origin_distance

    def _compute_actuation_reference_frame(self):
        """Compute actuation frame of hinge w.r.t. inertial frame."""

        # Set all vectors around lower hinge point
        self.body_pillar_point -= self.hinge_lower_point
        self.actuator_pivot_point -= self.hinge_lower_point

        # Compute rotational axis around which the door rotates
        rotation_axis = self.hinge_upper_point - self.hinge_lower_point
        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)

        # Compute the actuation origin
        self.body_pillar_origin = (
            self.body_pillar_point.dot(rotation_axis) * rotation_axis
        )

        self.actuation_origin = (
            self.actuator_pivot_point.dot(rotation_axis) * rotation_axis
        )

        # Compute Actuation axis
        actuation_axis = self.actuator_pivot_point - self.actuation_origin
        actuation_axis /= np.linalg.norm(actuation_axis)

        # Compute Actuation Frame
        actuation_y = np.cross(self.rotation_axis, actuation_axis)
        self.R_actuation_to_car = np.vstack(
            [actuation_axis, actuation_y, self.rotation_axis]
        )

        # Compute Body Pillar axis
        body_pillar_axis = self.body_pillar_point - self.body_pillar_origin
        body_pillar_axis /= np.linalg.norm(body_pillar_axis)

        # Compute Body Pillar Frame
        body_pillar_y = np.cross(self.rotation_axis, body_pillar_axis)
        self.R_body_pillar_to_car = np.vstack(
            [body_pillar_axis, body_pillar_y, self.rotation_axis]
        )

        # How to go between the two reference frames?
        R_car_to_actuation = self.R_actuation_to_car.T
        self.R_body_pillar_to_actuation = self.R_body_pillar_to_car @ R_car_to_actuation

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
            direction = gravity_in_hinge_direction / np.linalg.norm(
                gravity_in_hinge_direction
            )
            if direction.dot(tangent) < 0:
                direction = 1
            else:
                direction = -1
            torque = direction * (
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

    def plot_torques_new(self, poses: list[CarPose]):

        fig = go.Figure()
        angles = self.angles
        for pose in poses:
            torques = self.compute_torque_along_door_motion(
                roll=pose.roll, pitch=pose.pitch
            )
            fig.add_trace(
                go.Scatter(
                    x=angles * 180 / math.pi,
                    y=torques,
                    mode="lines",
                    name=f"Roll: {math.degrees(pose.roll)}, Pitch: {math.degrees(pose.pitch)}",
                )
            )

        fig.update_layout(
            title="Hinge Torque",
            xaxis_title="Door Position From Closed (Degrees)",
            yaxis_title="Torque at Hinge (Nm)",
        )
        fig.show()

    def compute_effective_moment_arm_along_door_motion(self):
        """
        Compute effective moment arm all in the body pillar frame.
        Where   t = F * l_e
                l_e = cos(theta) * l --> effective moment arm
        """

        # Compute actuation points in body pillar frame
        actuator_pivot_point = self.actuator_pivot_point - self.body_pillar_origin
        actuator_pivot_point = self.R_body_pillar_to_car @ actuator_pivot_point
        body_pillar_point = self.body_pillar_point - self.body_pillar_origin
        body_pillar_point = self.R_body_pillar_to_car @ body_pillar_point

        # Compute door trajectory
        angles = self.angles
        points = np.linalg.norm(actuator_pivot_point) * np.array(
            [
                np.cos(angles),
                np.sin(angles),
                np.zeros(self.angles.size),
            ]
        )
        points = self.R_body_pillar_to_actuation @ points
        points[2] += actuator_pivot_point[2]

        # Setup constants
        moment_arm = np.linalg.norm(body_pillar_point)
        torque_axis = np.array([0, 1, 0])

        effective_moment_arms = []
        for point_ind in range(self.num_door_angles):

            # Project Actuator force onto torque direction
            force_axis = points[:, point_ind] - body_pillar_point

            cos_theta = np.dot(force_axis, torque_axis) / (
                np.linalg.norm(force_axis) * np.linalg.norm(torque_axis)
            )
            moment_arm_effective_length = cos_theta * moment_arm * M_TO_MM
            effective_moment_arms.append(moment_arm_effective_length)

        return effective_moment_arms

    def plot_moment_arm(self):

        ax = plt.axes()

        moment_arm = self.compute_effective_moment_arm_along_door_motion()
        ax.plot(self.angles * 180 / math.pi, moment_arm)

        plt.title(f"Moment Arm vs Door Position")
        ax.tick_params(width=2)
        ax.set_xlabel("Door Position From Closed (Degrees)")
        ax.set_ylabel("Moment Arm (mm)")
        ax.set_ylim(bottom=0)
        ax.set_xlim(left=0)
        ax.set_yticks(range(0, 125, 5))
        plt.grid()
        plt.legend()
        plt.show()

    def visualize_hinge_and_center_of_mass(self):
        ax = plt.axes(projection="3d")

        ax.scatter3D(
            self.center_of_mass[0],
            self.center_of_mass[1],
            self.center_of_mass[2],
            "green",
            label="Center of Mass",
        )
        ax.scatter3D(
            self.origin[0],
            self.origin[1],
            self.origin[2],
            "orange",
            label="origin",
        )
        ax.scatter3D(
            self.body_pillar_point[0],
            self.body_pillar_point[1],
            self.body_pillar_point[2],
            "purple",
            label="Body Pillar Point",
        )
        ax.scatter3D(
            self.actuator_pivot_point[0],
            self.actuator_pivot_point[1],
            self.actuator_pivot_point[2],
            "blue",
            label="Actuator Pivot Point",
        )
        ax.scatter3D(
            self.actuation_origin[0],
            self.actuation_origin[1],
            self.actuation_origin[2],
            "blue",
            label="Actuation Origin",
        )
        ax.plot3D(
            [self.hinge_upper_point[0] - self.hinge_lower_point[0], 0],
            [self.hinge_upper_point[1] - self.hinge_lower_point[1], 0],
            [self.hinge_upper_point[2] - self.hinge_lower_point[2], 0],
            "brown",
            label="Rotation Axis",
        )
        ax.plot3D(
            [self.origin[0], self.center_of_mass[0]],
            [self.origin[1], self.center_of_mass[1]],
            [self.origin[2], self.center_of_mass[2]],
            "orange",
            label="cm to origin",
        )
        ax.plot3D(
            [self.actuator_pivot_point[0], self.center_of_mass[0]],
            [self.actuator_pivot_point[1], self.center_of_mass[1]],
            [self.actuator_pivot_point[2], self.center_of_mass[2]],
            "gray",
            label="CM to Actuation Point - Rigid",
        )
        ax.plot3D(
            [self.actuator_pivot_point[0], self.actuation_origin[0]],
            [self.actuator_pivot_point[1], self.actuation_origin[1]],
            [self.actuator_pivot_point[2], self.actuation_origin[2]],
            "gray",
            label="Actuation Point to Rotational Axis - Rigid",
        )
        ax.plot3D(
            [self.body_pillar_point[0], self.actuation_origin[0]],
            [self.body_pillar_point[1], self.actuation_origin[1]],
            [self.body_pillar_point[2], self.actuation_origin[2]],
            "magenta",
            label="Body Pillar Moment Arm - Rigid",
        )

        plt.legend()
        plt.show()

    def visualize_actuation_frames(self):

        # Compute actuation points in body pillar frame
        actuator_pivot_point = self.actuator_pivot_point - self.body_pillar_origin
        actuator_pivot_point = self.R_body_pillar_to_car @ actuator_pivot_point
        body_pillar_point = self.body_pillar_point - self.body_pillar_origin
        body_pillar_point = self.R_body_pillar_to_car @ body_pillar_point

        angles = self.angles
        points = np.linalg.norm(actuator_pivot_point) * np.array(
            [
                np.cos(angles),
                np.sin(angles),
                np.zeros(self.angles.size),
            ]
        )
        points = self.R_body_pillar_to_actuation @ points
        points[2] += actuator_pivot_point[2]

        # Project Actuator force onto torque direction
        v1 = actuator_pivot_point - body_pillar_point
        v2 = np.array([0, 1, 0])
        v_proj = np.dot(v1, v2) * v2 + body_pillar_point

        # Plot the axes
        ax = plt.axes(projection="3d")
        ax.scatter3D(
            body_pillar_point[0],
            body_pillar_point[1],
            body_pillar_point[2],
            "orange",
            label="Body Pillar Point",
        )
        ax.plot3D(
            [body_pillar_point[0], 0],
            [body_pillar_point[1], 0],
            [body_pillar_point[2], 0],
            "green",
            label="Body Pillar Point Axis",
        )
        ax.scatter3D(
            actuator_pivot_point[0],
            actuator_pivot_point[1],
            actuator_pivot_point[2],
            "blue",
            label="Actuator Pivot Point",
        )
        ax.plot3D(
            [actuator_pivot_point[0], 0],
            [actuator_pivot_point[1], 0],
            [actuator_pivot_point[2], 0],
            "yellow",
            label="Actuator Pivot Axis",
        )
        ax.plot3D(
            [actuator_pivot_point[0], body_pillar_point[0]],
            [actuator_pivot_point[1], body_pillar_point[1]],
            [actuator_pivot_point[2], body_pillar_point[2]],
            "purple",
            label="Force",
        )
        ax.plot3D(
            [body_pillar_point[0], body_pillar_point[0]],
            [0.1, body_pillar_point[1]],
            [0, 0],
            "brown",
            label="Torque Direction",
        )
        ax.plot3D(
            [v_proj[0], body_pillar_point[0]],
            [v_proj[1], body_pillar_point[1]],
            [v_proj[2], body_pillar_point[2]],
            "pink",
            label="Torque Magnitude",
        )
        ax.plot3D(
            points[0],
            points[1],
            points[2],
            "blue",
            label="Actuator Path",
        )

        ax.set_zlim3d(-0.05, 0.2)
        plt.legend()
        plt.show()


if __name__ == "__main__":

    door_model = DoorModel()

    # Visualizations
    door_model.visualize_hinge_and_center_of_mass()
    door_model.visualize_in_3D(pitch=0, roll=0)
    door_model.visualize_actuation_frames()

    # Plot moment arm
    door_model.plot_moment_arm()

    # Plot torques
    door_model.plot_torques_new(
        [
            CarPose(pitch=math.radians(11.8), roll=0),
            CarPose(pitch=0, roll=math.radians(6.27)),
            CarPose(pitch=0, roll=math.radians(-6.27)),
            CarPose(pitch=math.radians(-11.8), roll=0),
        ]
    )
    door_model.plot_torques_new(
        [
            CarPose(pitch=0, roll=0),
        ]
    )
