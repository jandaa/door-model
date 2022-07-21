import sys
from pathlib import Path

from PyQt6 import QtWidgets, QtGui
from PyQt6 import uic

from PyInstaller import building

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


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        ui_file = base_path / "gui/door_model.ui"
        icon_file = base_path / "gui/power_side_door.ico"

        # Load ui template
        uic.load_ui.loadUi(str(ui_file), self)

        self.setWindowTitle("Power Side Door")
        self.setWindowIcon(QtGui.QIcon(str(icon_file)))
        self.show()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    app.exec()
