import sys
import math
from pathlib import Path
import numpy as np

from PyQt6 import QtWidgets, QtGui, QtCore
from PyQt6 import uic

from door_model import DoorModel, CarPose, DoorModelParameters

# Get taskbar working
try:
    from ctypes import windll  # Only exists on Windows.

    myappid = "mycompany.myproduct.subproduct.version"
    windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)
except ImportError:
    pass

try:
    # PyInstaller creates a temp folder and stores path in _MEIPASS
    base_path = Path(sys._MEIPASS)
except Exception:
    base_path = Path.cwd()

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import (
    FigureCanvasQTAgg as Canvas,
    NavigationToolbar2QT as NavigationToolbar,
)
import matplotlib

# Ensure using PyQt6 backend
matplotlib.use("QT5Agg")

# Matplotlib canvas class to create figure
class MplCanvas(Canvas):
    def __init__(self, is_3d=False):
        self.fig = Figure()
        if is_3d:
            self.ax = self.fig.add_subplot(111, projection="3d")
            self.ax.set_position([-0.2, 0, 1, 1])
        else:
            self.ax = self.fig.add_subplot(111)

        Canvas.__init__(self, self.fig)
        Canvas.setSizePolicy(
            self,
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Expanding,
        )
        Canvas.updateGeometry(self)


# Matplotlib widget
class MplWidget(QtWidgets.QWidget):
    def __init__(self, parent=None, is_3d=False):
        QtWidgets.QWidget.__init__(self, parent)  # Inherit from QWidget
        self.canvas = MplCanvas(is_3d=is_3d)  # Create canvas object
        self.vbl = QtWidgets.QVBoxLayout()  # Set box for plotting
        self.vbl.addWidget(self.canvas)
        self.setLayout(self.vbl)


car_poses = [
    CarPose(pitch=math.radians(11.8), roll=0),
    CarPose(pitch=0, roll=math.radians(6.27)),
    CarPose(pitch=0, roll=math.radians(-6.27)),
    CarPose(pitch=math.radians(-11.8), roll=0),
]


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        ui_file = base_path / "gui/door_model.ui"
        icon_file = base_path / "gui/power_side_door.ico"

        # Load ui template
        uic.load_ui.loadUi(str(ui_file), self)

        # Set plot sizes
        graph_2d_size = QtCore.QRect(5, 50, 1200, 550)
        graph_3d_size = QtCore.QRect(5, 50, 750, 500)

        # Configure plots
        self.torque_plot = MplWidget(self.torque)
        NavigationToolbar(self.torque_plot.canvas, self.torque)
        self.torque_plot.setGeometry(graph_2d_size)
        self.torque_plot.setObjectName("torquePlot")

        self.moment_plot = MplWidget(self.moment_arm)
        NavigationToolbar(self.moment_plot.canvas, self.moment_arm)
        self.moment_plot.setGeometry(graph_2d_size)
        self.moment_plot.setObjectName("momentPlot")

        self.configuration_plot = MplWidget(self.vis_configuration, is_3d=True)
        NavigationToolbar(self.configuration_plot.canvas, self.vis_configuration)
        self.configuration_plot.setGeometry(graph_3d_size)
        self.configuration_plot.setObjectName("configurationPlot")

        self.incline_plot = MplWidget(self.vis_incline, is_3d=True)
        NavigationToolbar(self.incline_plot.canvas, self.vis_incline)
        self.incline_plot.setGeometry(graph_3d_size)
        self.incline_plot.setObjectName("inclinePlot")

        self.moment_arm_plot = MplWidget(self.vis_moment_arm, is_3d=True)
        NavigationToolbar(self.moment_arm_plot.canvas, self.vis_moment_arm)
        self.moment_arm_plot.setGeometry(graph_3d_size)
        self.moment_arm_plot.setObjectName("momentArmPlot")

        # Set default parameters
        self.hinge_upper_x.setValue(753.367)
        self.hinge_upper_y.setValue(896.342)
        self.hinge_upper_z.setValue(512.62)
        self.hinge_lower_x.setValue(749.783)
        self.hinge_lower_y.setValue(910.696)
        self.hinge_lower_z.setValue(101.85)
        self.body_pillar_x.setValue(840.641)
        self.body_pillar_y.setValue(831.029)
        self.body_pillar_z.setValue(365.224)
        self.actuator_pivot_x.setValue(897.322)
        self.actuator_pivot_y.setValue(840.001)
        self.actuator_pivot_z.setValue(365.043)
        self.center_of_mass_x.setValue(1252.737)
        self.center_of_mass_y.setValue(845.036)
        self.center_of_mass_z.setValue(400.716)
        self.mass_in_kg.setValue(36.1736)
        self.max_door_angle_in_degrees.setValue(67)

        self.calculate()

        self.calculate_btn.clicked.connect(self.calculate)

        self.setWindowTitle("Power Side Door")
        self.setWindowIcon(QtGui.QIcon(str(icon_file)))
        self.show()

    def calculate(self):

        # Get parameters from user inputs
        parameters = DoorModelParameters()
        parameters.hinge_upper_point = np.array(
            [
                self.hinge_upper_x.value(),
                self.hinge_upper_y.value(),
                self.hinge_upper_z.value(),
            ]
        )
        parameters.hinge_lower_point = np.array(
            [
                self.hinge_lower_x.value(),
                self.hinge_lower_y.value(),
                self.hinge_lower_z.value(),
            ]
        )
        parameters.body_pillar_point = np.array(
            [
                self.body_pillar_x.value(),
                self.body_pillar_y.value(),
                self.body_pillar_z.value(),
            ]
        )
        parameters.actuator_pivot_point = np.array(
            [
                self.actuator_pivot_x.value(),
                self.actuator_pivot_y.value(),
                self.actuator_pivot_z.value(),
            ]
        )
        parameters.center_of_mass = np.array(
            [
                self.center_of_mass_x.value(),
                self.center_of_mass_y.value(),
                self.center_of_mass_z.value(),
            ]
        )
        parameters.center_of_mass = np.array(
            [
                self.center_of_mass_x.value(),
                self.center_of_mass_y.value(),
                self.center_of_mass_z.value(),
            ]
        )
        parameters.mass_in_kg = self.mass_in_kg.value()
        parameters.max_door_angle_in_degrees = self.max_door_angle_in_degrees.value()

        # Create door model
        self.door_model = DoorModel(parameters=parameters)

        # Plot torque
        self.torque_plot.canvas.ax.clear()
        self.door_model.plot_torques_gui(
            [CarPose(pitch=self.pitch.value(), roll=self.roll.value())],
            self.torque_plot.canvas.ax,
        )
        self.torque_plot.canvas.draw()

        # Plot moment arm
        self.moment_plot.canvas.ax.clear()
        self.door_model.plot_moment_arm_gui(self.moment_plot.canvas.ax)
        self.moment_plot.canvas.draw()

        # Plot configuration
        self.configuration_plot.canvas.ax.clear()
        self.door_model.visualize_hinge_and_center_of_mass(
            self.configuration_plot.canvas.ax
        )
        self.configuration_plot.canvas.draw()

        # Plot incline
        self.incline_plot.canvas.ax.clear()
        self.door_model.visualize_in_3D(0, 0, self.incline_plot.canvas.ax)
        self.incline_plot.canvas.draw()

        # Plot moment arm visualization
        self.moment_arm_plot.canvas.ax.clear()
        self.door_model.visualize_actuation_frames(ax=self.moment_arm_plot.canvas.ax)
        self.moment_arm_plot.canvas.draw()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    app.exec()
