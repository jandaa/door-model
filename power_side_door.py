import sys
import math
from pathlib import Path

from PyQt6 import QtWidgets, QtGui, QtCore
from PyQt6 import uic

from door_model import DoorModel, CarPose

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

        graph_2d_size = QtCore.QRect(5, 50, 1200, 550)
        graph_3d_size = QtCore.QRect(5, 50, 750, 500)

        # Create door model
        self.door_model = DoorModel()

        # Load ui template
        uic.load_ui.loadUi(str(ui_file), self)

        # Plot torque
        self.torque_plot = MplWidget(self.torque)
        NavigationToolbar(self.torque_plot.canvas, self.torque)
        self.torque_plot.setGeometry(graph_2d_size)
        self.torque_plot.setObjectName("torquePlot")
        self.door_model.plot_torques_gui(car_poses, self.torque_plot.canvas.ax)

        # Plot moment arm
        self.moment_plot = MplWidget(self.moment_arm)
        NavigationToolbar(self.moment_plot.canvas, self.moment_arm)
        self.moment_plot.setGeometry(graph_2d_size)
        self.moment_plot.setObjectName("momentPlot")
        self.door_model.plot_moment_arm_gui(self.moment_plot.canvas.ax)

        # Plot configuration
        self.configuration_plot = MplWidget(self.vis_configuration, is_3d=True)
        NavigationToolbar(self.configuration_plot.canvas, self.vis_configuration)
        self.configuration_plot.setGeometry(graph_3d_size)
        self.configuration_plot.setObjectName("configurationPlot")
        self.door_model.visualize_hinge_and_center_of_mass(
            self.configuration_plot.canvas.ax
        )

        # Plot incline
        self.incline_plot = MplWidget(self.vis_incline, is_3d=True)
        NavigationToolbar(self.incline_plot.canvas, self.vis_incline)
        self.incline_plot.setGeometry(graph_3d_size)
        self.incline_plot.setObjectName("inclinePlot")
        self.door_model.visualize_in_3D(0, 0, self.incline_plot.canvas.ax)

        # Plot moment arm visualization
        self.moment_arm_plot = MplWidget(self.vis_moment_arm, is_3d=True)
        NavigationToolbar(self.moment_arm_plot.canvas, self.vis_moment_arm)
        self.moment_arm_plot.setGeometry(graph_3d_size)
        self.moment_arm_plot.setObjectName("momentArmPlot")
        self.door_model.visualize_actuation_frames(ax=self.moment_arm_plot.canvas.ax)

        self.setWindowTitle("Power Side Door")
        self.setWindowIcon(QtGui.QIcon(str(icon_file)))
        self.show()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    app.exec()
