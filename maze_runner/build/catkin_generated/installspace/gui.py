#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
import gc  # Adding garbage collection
from PyQt5.QtWidgets import (QApplication, QWidget, QPushButton, QVBoxLayout, 
                            QLabel, QTextEdit, QGridLayout, QFrame, QDesktopWidget,
                            QSizePolicy)
from PyQt5.QtCore import QTimer, Qt, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap, QPalette, QColor
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class PlayGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()
        self.setDarkMode()
        self.initUI()
        rospy.set_param('/robot_ready', False)

        # Initialize variables for memory management
        self.gc_counter = 0
        self.last_camera_timestamp = rospy.Time.now()
        self.image_throttle_rate = rospy.Duration(0.1)  # Throttle to 10Hz max

        # ROS Subscribers
        self.log_sub = rospy.Subscriber('/maze_gui/log', String, self.update_log)
        self.camera_sub = rospy.Subscriber('/aruco_tracker/output_image', Image, self.update_camera)
        self.maze_sub = rospy.Subscriber('/maze/visualisation', Image, self.update_maze)

        # Timer to process ROS events in the Qt loop
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.ros_spin_once)
        self.ros_timer.start(50)  # milliseconds (20 Hz)

    def setDarkMode(self):
        """Apply dark mode styling to the application"""
        # Create a dark palette for the application
        dark_palette = QPalette()
        
        # Set colors for various UI elements
        dark_color = QColor(35, 35, 35)         # Dark background
        medium_color = QColor(53, 53, 53)       # Medium dark (for elements)
        light_color = QColor(80, 80, 80)        # Light dark (for highlights)
        text_color = QColor(220, 220, 220)      # Light text color
        accent_color = QColor(97, 175, 239)     # Blue accent like Claude
        
        # Set color roles in the palette
        dark_palette.setColor(QPalette.Window, dark_color)
        dark_palette.setColor(QPalette.WindowText, text_color)
        dark_palette.setColor(QPalette.Base, medium_color)
        dark_palette.setColor(QPalette.AlternateBase, dark_color)
        dark_palette.setColor(QPalette.ToolTipBase, dark_color)
        dark_palette.setColor(QPalette.ToolTipText, text_color)
        dark_palette.setColor(QPalette.Text, text_color)
        dark_palette.setColor(QPalette.Button, medium_color)
        dark_palette.setColor(QPalette.ButtonText, text_color)
        dark_palette.setColor(QPalette.BrightText, Qt.red)
        dark_palette.setColor(QPalette.Link, accent_color)
        dark_palette.setColor(QPalette.Highlight, accent_color)
        dark_palette.setColor(QPalette.HighlightedText, Qt.black)
        
        # Apply the palette to the application
        self.setPalette(dark_palette)
        
        # Set the stylesheet for custom elements
        self.setStyleSheet("""
            QFrame {
                background-color: #232323;
                border: 1px solid #3D3D3D;
                border-radius: 4px;
            }
            QPushButton {
                background-color: #404040;
                color: #DCDCDC;
                border: 1px solid #505050;
                border-radius: 4px;
                padding: 6px;
            }
            QPushButton:hover {
                background-color: #505050;
            }
            QPushButton:pressed {
                background-color: #5F5F5F;
            }
            QPushButton:disabled {
                background-color: #333333;
                color: #707070;
            }
            QTextEdit {
                background-color: #2A2A2A;
                color: #DCDCDC;
                border: 1px solid #3D3D3D;
                border-radius: 2px;
            }
            QLabel {
                color: #DCDCDC;
            }
        """)
        
    def initUI(self):
        self.setWindowTitle('Maze Runner')
        # Set to full screen
        self.showFullScreen()
        
        # Create main grid layout with 2x2 grid
        main_layout = QGridLayout()
        
        # Create frames for each quadrant with borders
        self.control_frame = self.create_frame()
        self.log_frame = self.create_frame()
        self.camera_frame = self.create_frame()
        self.maze_frame = self.create_frame()
        
        # Top-left quadrant: Controls
        control_layout = QVBoxLayout(self.control_frame)
        self.status_label = QLabel('Status: Waiting for Start', self)
        self.status_label.setStyleSheet("color: #FFD700;")  # Gold color for waiting status
        self.start_button = QPushButton('Start', self)
        self.start_button.clicked.connect(self.play)
        self.start_button.setMinimumHeight(50)
        self.start_button.setStyleSheet("background-color: #3A6EA5; color: white;")  # Blue start button
        
        # Add additional placeholder buttons for future use
        self.reset_button = QPushButton('Reset', self)
        self.reset_button.setEnabled(False)
        self.reset_button.setMinimumHeight(50)
        
        self.exit_button = QPushButton('Exit', self)
        self.exit_button.clicked.connect(self.close)
        self.exit_button.setMinimumHeight(50)
        self.exit_button.setStyleSheet("background-color: #802020; color: white;")  # Red exit button
        
        control_layout.addWidget(self.status_label)
        control_layout.addWidget(self.start_button)
        control_layout.addWidget(self.reset_button)
        control_layout.addWidget(self.exit_button)
        control_layout.addStretch()
        
        # Bottom-left quadrant: Log output (smaller)
        log_layout = QVBoxLayout(self.log_frame)
        self.log_output = QTextEdit(self)
        self.log_output.setReadOnly(True)
        self.log_output.setStyleSheet("background-color: #1A1A1A; color: #B0B0B0; font-family: 'Courier New', monospace;")
        self.log_label = QLabel('System Log', self)
        log_layout.addWidget(self.log_label)
        log_layout.addWidget(self.log_output)
        
        # Top-right quadrant: Camera feed
        camera_layout = QVBoxLayout(self.camera_frame)
        self.camera_label = QLabel('Camera Feed', self)
        self.camera_view = QLabel(self)
        self.camera_view.setAlignment(Qt.AlignCenter)
        self.camera_view.setText("Waiting for camera feed...")
        self.camera_view.setStyleSheet("background-color: #1A1A1A; color: #808080;")
        # Set a fixed size policy to prevent automatic resizing
        self.camera_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.camera_view.setMinimumSize(320, 240)
        self.camera_view.setMaximumSize(800, 600)
        camera_layout.addWidget(self.camera_label)
        camera_layout.addWidget(self.camera_view)
        
        # Bottom-right quadrant: Maze visualization
        maze_layout = QVBoxLayout(self.maze_frame)
        self.maze_label = QLabel('Maze Visualization', self)
        self.maze_view = QLabel(self)
        self.maze_view.setAlignment(Qt.AlignCenter)
        self.maze_view.setText("Waiting for maze visualization...")
        self.maze_view.setStyleSheet("background-color: #1A1A1A; color: #808080;")
        # Set a fixed size policy to prevent automatic resizing
        self.maze_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.maze_view.setMinimumSize(320, 240)
        self.maze_view.setMaximumSize(800, 600)
        maze_layout.addWidget(self.maze_label)
        maze_layout.addWidget(self.maze_view)
        
        # Add the frames to the main grid layout
        main_layout.addWidget(self.control_frame, 0, 0)
        main_layout.addWidget(self.log_frame, 1, 0)
        main_layout.addWidget(self.camera_frame, 0, 1)
        main_layout.addWidget(self.maze_frame, 1, 1)
        
        # Set column and row stretches
        main_layout.setColumnStretch(0, 1)   # Left column (controls and logs)
        main_layout.setColumnStretch(1, 2)   # Right column (camera and maze)
        main_layout.setRowStretch(0, 1)      # Top row (controls and camera) - equal size
        main_layout.setRowStretch(1, 1)      # Bottom row (logs and maze) - equal size
        
        self.setLayout(main_layout)

    def create_frame(self):
        """Helper function to create a styled frame"""
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        frame.setFrameShadow(QFrame.Raised)
        frame.setLineWidth(1)  # Reduced from 2 for a more subtle look in dark mode
        return frame

    def play(self):
        rospy.set_param('/robot_ready', True)
        self.status_label.setText('Status: ✅ Ready')
        self.status_label.setStyleSheet("color: #7FFF7F;")  # Light green for success status
        self.start_button.setEnabled(False)
        self.reset_button.setEnabled(True)
        self.log_output.append("Robot is ready to start maze navigation")

    def update_log(self, msg):
        self.log_output.append(msg.data)

    def update_camera(self, msg):
        # Throttle image updates to prevent overwhelming the GUI
        current_time = rospy.Time.now()
        if current_time - self.last_camera_timestamp < self.image_throttle_rate:
            return  # Skip this update to avoid overwhelming the GUI
        
        self.last_camera_timestamp = current_time
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.display_image(cv_image, self.camera_view)
            # Explicitly delete the cv_image to help with memory management
            del cv_image
            # Call garbage collection occasionally
            if hasattr(self, 'gc_counter'):
                self.gc_counter += 1
                if self.gc_counter > 30:  # Run GC every 30 frames
                    gc.collect()
                    self.gc_counter = 0
            else:
                self.gc_counter = 0
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def update_maze(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.display_image(cv_image, self.maze_view)
            # Explicitly delete the cv_image
            del cv_image
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def display_image(self, cv_image, display_widget):
        """Convert OpenCV image to QPixmap and display it"""
        # Limit the maximum size to prevent excessive growth
        max_display_size = 800  # Maximum width or height in pixels
        height, width, channel = cv_image.shape
        
        # Calculate scale factor to limit maximum size while maintaining aspect ratio
        scale_factor = 1.0
        if width > max_display_size or height > max_display_size:
            scale_factor = min(max_display_size / width, max_display_size / height)
            new_width = int(width * scale_factor)
            new_height = int(height * scale_factor)
            cv_image = cv2.resize(cv_image, (new_width, new_height), interpolation=cv2.INTER_AREA)
            height, width = new_height, new_width
        
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        
        # Create a new pixmap rather than scaling the widget (more efficient)
        pixmap = QPixmap.fromImage(q_image)
        display_widget.setPixmap(pixmap)
        
        # Force a garbage collection to help prevent memory leaks
        cv_image = None
        q_image = None

    def ros_spin_once(self):
        rospy.sleep(0.001)  # needed to yield control to ROS callbacks

    def keyPressEvent(self, event):
        # Allow ESC key to exit fullscreen mode
        if event.key() == Qt.Key_Escape:
            self.showNormal()
        super().keyPressEvent(event)

if __name__ == '__main__':
    rospy.init_node('maze_gui', anonymous=True, disable_signals=True)  # disable signals for Qt compatibility
    app = QApplication(sys.argv)
    
    # Set application-wide attributes for better dark mode
    app.setStyle('Fusion')  # Use Fusion style for better dark mode support
    
    gui = PlayGUI()
    gui.show()
    sys.exit(app.exec_())