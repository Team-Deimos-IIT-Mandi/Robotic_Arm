import sys
import cv2
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QWidget, QFrame
)
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QTimer


class TypingGUI(QMainWindow):
    def __init__(self, data_provider):
        """
        Initialize the GUI with a data provider for points and images.

        Args:
            data_provider (function): Function to fetch the camera feeds and coordinates.
        """
        super().__init__()

        self.data_provider = data_provider
        self.initUI()

        # Timer for refreshing the UI with new data
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(100)  # Update every 100 ms

    def initUI(self):
        """Set up the user interface."""
        self.setWindowTitle("Autonomous Typing GUI")
        self.setGeometry(100, 100, 1200, 600)

        # Main widget
        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)

        # Camera feed labels
        self.left_camera_label = QLabel("Left Camera Feed")
        self.right_camera_label = QLabel("Right Camera Feed")
        self.left_camera_label.setFrameShape(QFrame.Box)
        self.right_camera_label.setFrameShape(QFrame.Box)

        # 2D coordinates display
        self.left_2d_label = QLabel("Left 2D Coordinates: ")
        self.right_2d_label = QLabel("Right 2D Coordinates: ")

        # 3D coordinates display
        self.coords_3d_label = QLabel("3D Coordinates: ")

        # Layout setup
        camera_layout = QHBoxLayout()
        camera_layout.addWidget(self.left_camera_label)
        camera_layout.addWidget(self.right_camera_label)

        coords_layout = QVBoxLayout()
        coords_layout.addWidget(self.left_2d_label)
        coords_layout.addWidget(self.right_2d_label)
        coords_layout.addWidget(self.coords_3d_label)

        main_layout = QVBoxLayout()
        main_layout.addLayout(camera_layout)
        main_layout.addLayout(coords_layout)

        self.main_widget.setLayout(main_layout)

    def update_display(self):
        """Update the GUI with new data from the data provider."""
        try:
            # Get data from the provider
            data = self.data_provider()
            if data is None or len(data) != 5:
                raise ValueError("Invalid data received from data provider.")

            left_img, right_img, key_2d_left, key_2d_right, key_3d_position = data

            # Update left camera feed
            if left_img is not None and left_img.size > 0:
                self.display_image(left_img, self.left_camera_label)
            else:
                self.left_camera_label.setText("No Left Camera Feed Available")

            # Update right camera feed
            if right_img is not None and right_img.size > 0:
                self.display_image(right_img, self.right_camera_label)
            else:
                self.right_camera_label.setText("No Right Camera Feed Available")

            # Update coordinates display
            self.left_2d_label.setText(f"Left 2D Coordinates: {key_2d_left}")
            self.right_2d_label.setText(f"Right 2D Coordinates: {key_2d_right}")
            self.coords_3d_label.setText(f"3D Coordinates: {key_3d_position}")
        except Exception as e:
            print(f"Error updating display: {e}")

    def display_image(self, img, label):
        """
        Convert OpenCV image to QPixmap and display on QLabel.

        Args:
            img (np.ndarray): The image to be displayed.
            label (QLabel): The label to display the image on.
        """
        try:
            rgb_image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            height, width, channel = rgb_image.shape
            bytes_per_line = 3 * width
            qt_image = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            label.setPixmap(pixmap)
            label.setScaledContents(True)
        except Exception as e:
            label.setText(f"Error displaying image: {e}")


def real_data_provider():
    """
    Provide real data to the GUI.

    Returns:
        tuple: Contains left_img, right_img, key_2d_left, key_2d_right, key_3d_position.
    """
    try:
        # Example: Capturing real-time video feed from cameras
        left_camera = cv2.VideoCapture(0)  # Replace 0 with the left camera index
        right_camera = cv2.VideoCapture(1)  # Replace 1 with the right camera index

        if not left_camera.isOpened() or not right_camera.isOpened():
            raise ValueError("Unable to open one or both cameras")

        # Capture frames
        ret_left, left_img = left_camera.read()
        ret_right, right_img = right_camera.read()

        # Ensure frames are captured
        if not ret_left or not ret_right:
            raise ValueError("Failed to capture frames from cameras")

        # Placeholder for actual coordinate calculations
        key_2d_left = [100, 200]  # Replace with real 2D coordinates
        key_2d_right = [95, 200]  # Replace with real 2D coordinates
        key_3d_position = [0.5, 0.3, 1.2]  # Replace with real 3D coordinates

        # Release cameras after capturing
        left_camera.release()
        right_camera.release()

        return left_img, right_img, key_2d_left, key_2d_right, key_3d_position
    except Exception as e:
        print(f"Error in real data provider: {e}")
        return None


if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = TypingGUI(real_data_provider)
    gui.show()
    sys.exit(app.exec_())
