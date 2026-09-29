import sys
import numpy as np
from PyQt5 import QtCore, QtWidgets
import pyqtgraph.opengl as gl
from ballbot.runtime.robotKinematics import RobotKinematics  
import math

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, robot):
        super().__init__()
        self.robot = robot  # your imported robot instance
        self.setWindowTitle("3RRS Robot Simulator")
        self.resize(1000, 600)

        # Create the main widget and layout (horizontal: left for 3D plot, right for input panel)
        main_widget = QtWidgets.QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QtWidgets.QHBoxLayout(main_widget)

        self.alpha = 0.0
        self.beta = 0.0
        self.gamma = 1.0
        self.h = 0.0 

        # --------------------------
        # Left Panel: 3D Plot Setup
        # --------------------------
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.opts['distance'] = 20  # adjust camera distance as needed
        main_layout.addWidget(self.gl_widget, 3)  # occupy ~75% of the space

        # Add a grid for spatial reference
        grid = gl.GLGridItem()
        grid.scale(2, 2, 1)
        self.gl_widget.addItem(grid)

        # Scatter plot items for A (red), B (blue), and C (green)
        self.scatter_A = gl.GLScatterPlotItem(color=np.array([(1.0, 0.2, 0.2, 1.0)] * 3), size=10)
        self.scatter_B = gl.GLScatterPlotItem(color=np.array([(0.0, 0.57, 0.8, 1.0)] * 3), size=10)
        self.scatter_C = gl.GLScatterPlotItem(color=np.array([(0.99, 0.84, 0.57, 1.0)] * 3), size=10)
        self.gl_widget.addItem(self.scatter_A)
        self.gl_widget.addItem(self.scatter_B)
        self.gl_widget.addItem(self.scatter_C)

        # Line items for A loop (red) and B loop (blue)
        self.line_A = gl.GLLinePlotItem(color=(1.0, 0.2, 0.2, 1.0), width=5, antialias=True)
        self.line_B = gl.GLLinePlotItem(color=(0.0, 0.57, 0.8, 1.0), width=5, antialias=True)
        self.gl_widget.addItem(self.line_A)
        self.gl_widget.addItem(self.line_B)

        # Three separate line items for C connections (green)
        self.line_C1 = gl.GLLinePlotItem(color=(0.99, 0.84, 0.57, 1.0), width=5, antialias=True)
        self.line_C2 = gl.GLLinePlotItem(color=(0.99, 0.84, 0.57, 1.0), width=5, antialias=True)
        self.line_C3 = gl.GLLinePlotItem(color=(0.99, 0.84, 0.57, 1.0), width=5, antialias=True)
        self.gl_widget.addItem(self.line_C1)
        self.gl_widget.addItem(self.line_C2)
        self.gl_widget.addItem(self.line_C3)

        # --------------------------
        # Right Panel: Fake Input Portal
        # --------------------------
        self.input_panel = QtWidgets.QWidget()

        # Create a group box with a title for better presentation
        group_box = QtWidgets.QGroupBox("Robot Parameters")
        group_box.setStyleSheet("QGroupBox { font-weight: bold; border: 1.5px solid gray; margin-top: 10px; }"
                        "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px; }")

        # Create a grid layout inside the group box
        grid = QtWidgets.QGridLayout()

        # Row 0: lp
        grid.addWidget(QtWidgets.QLabel("lp:"), 0, 0)
        self.input_lp = QtWidgets.QDoubleSpinBox()
        self.input_lp.setRange(0, 100)
        self.input_lp.setValue(self.robot.lp)
        self.input_lp.setSingleStep(0.1)
        grid.addWidget(self.input_lp, 0, 1)

        # Row 1: l1
        grid.addWidget(QtWidgets.QLabel("l1:"), 1, 0)
        self.input_l1 = QtWidgets.QDoubleSpinBox()
        self.input_l1.setRange(0, 100)
        self.input_l1.setValue(self.robot.l1)
        self.input_l1.setSingleStep(0.1)
        grid.addWidget(self.input_l1, 1, 1)

        # Row 2: l2
        grid.addWidget(QtWidgets.QLabel("l2:"), 2, 0)
        self.input_l2 = QtWidgets.QDoubleSpinBox()
        self.input_l2.setRange(0, 100)
        self.input_l2.setValue(self.robot.l2)
        self.input_l2.setSingleStep(0.1)
        grid.addWidget(self.input_l2, 2, 1)

        # Row 3: lb
        grid.addWidget(QtWidgets.QLabel("lb:"), 3, 0)
        self.input_lb = QtWidgets.QDoubleSpinBox()
        self.input_lb.setRange(0, 100)
        self.input_lb.setValue(self.robot.lb)
        self.input_lb.setSingleStep(0.1)
        grid.addWidget(self.input_lb, 3, 1)

        # Row 4: maxh
        grid.addWidget(QtWidgets.QLabel("maxh:"), 4, 0)
        self.label_maxh = QtWidgets.QLabel("")
        grid.addWidget(self.label_maxh, 4, 1)

        grid.addWidget(QtWidgets.QLabel("minh:"), 5, 0)
        self.label_minh = QtWidgets.QLabel("")
        grid.addWidget(self.label_minh, 5, 1)

        # Row 5: invert (checkbox)
        grid.addWidget(QtWidgets.QLabel("Invert:"), 6, 0)
        self.input_invert = QtWidgets.QCheckBox()
        self.input_invert.setChecked(self.robot.invert)
        grid.addWidget(self.input_invert, 6, 1)

        # Row 6: Create a horizontal layout for the "Apply Changes" button and the "Auto Update" checkbox.
        hbox = QtWidgets.QHBoxLayout()
        submit_button = QtWidgets.QPushButton("Apply")
        hbox.addWidget(submit_button)
        self.auto_update_checkbox = QtWidgets.QCheckBox("Auto Update")
        hbox.addWidget(self.auto_update_checkbox)
        # Center the hbox across the two columns:
        grid.addLayout(hbox, 7, 0, 1, 2, QtCore.Qt.AlignCenter)

        # Set the grid layout on a group box (for aesthetics)
        group_box.setLayout(grid)

        # Add the group box to the input panel's layout
        input_layout = QtWidgets.QVBoxLayout(self.input_panel)
        input_layout.addWidget(group_box)
        input_layout.addStretch(1)

        # --- Connect Signals ---
        # Instead of directly connecting the valueChanged signals to update_robot_parameters,
        # we route them through a helper method that checks the auto update checkbox.
        self.input_lp.valueChanged.connect(self.input_changed)
        self.input_l1.valueChanged.connect(self.input_changed)
        self.input_l2.valueChanged.connect(self.input_changed)
        self.input_lb.valueChanged.connect(self.input_changed)
        self.input_invert.toggled.connect(self.input_changed)
        submit_button.clicked.connect(self.validate_parameters)








        input_group = QtWidgets.QGroupBox("Input")
        input_group.setStyleSheet(
            "QGroupBox { font-weight: bold; border: 1.5px solid gray; margin-top: 10px; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px; }"
        )

        # Create a grid layout for the sliders and spin boxes
        grid2 = QtWidgets.QGridLayout()

        # Common slider style: white groove and handle
        slider_style = (
            "QSlider::groove:horizontal { height: 3px; background: grey; } "
            "QSlider::sub-page:horizontal { background: white; } "
            "QSlider::add-page:horizontal { background: grey; } "
            "QSlider::handle:horizontal { background: grey; border: 1px solid #5c5c5c; width: 18px; margin: -4px 0; border-radius: 3px; }"
        )

        # Row 0: θ (theta) [Range: 0° to 180°]
        grid2.addWidget(QtWidgets.QLabel("θ:"), 0, 0)
        self.slider_theta = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_theta.setMinimum(0)
        self.slider_theta.setMaximum(1500)  # 0° to 30.00° (scale factor 100)
        self.slider_theta.setValue(0)       # Default ~0.00°
        self.slider_theta.setStyleSheet(slider_style)
        grid2.addWidget(self.slider_theta, 0, 1)
        self.spin_theta = QtWidgets.QDoubleSpinBox()
        self.spin_theta.setRange(0.0, 15.0)
        self.spin_theta.setDecimals(2)
        self.spin_theta.setSingleStep(0.01)
        self.spin_theta.setValue(0.00)
        grid2.addWidget(self.spin_theta, 0, 2)

        # Row 1: φ (phi) [Range: 0° to 360°]
        grid2.addWidget(QtWidgets.QLabel("φ:"), 1, 0)
        self.slider_phi = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_phi.setMinimum(0)
        self.slider_phi.setMaximum(36000)  # 0° to 360.00° (scale factor 100)
        self.slider_phi.setValue(0)      # Default ~180.00°
        self.slider_phi.setStyleSheet(slider_style)
        grid2.addWidget(self.slider_phi, 1, 1)
        self.spin_phi = QtWidgets.QDoubleSpinBox()
        self.spin_phi.setRange(0.0, 360.0)
        self.spin_phi.setDecimals(2)
        self.spin_phi.setSingleStep(0.01)
        self.spin_phi.setValue(0.00)
        grid2.addWidget(self.spin_phi, 1, 2)

        # Row 2: ℎ (Planck's constant symbol) remains unchanged
        grid2.addWidget(QtWidgets.QLabel("ℎ:"), 2, 0)
        self.slider_h = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_h.setMinimum(0)
        self.slider_h.setMaximum(10000)  # representing 0.0 to 1000.0 (scaled by 100)
        self.slider_h.setValue(0)       # Default 0.0
        self.slider_h.setStyleSheet(slider_style)
        grid2.addWidget(self.slider_h, 2, 1)
        self.spin_h = QtWidgets.QDoubleSpinBox()
        self.spin_h.setRange(0.0, 100.0)
        self.spin_h.setDecimals(2)
        self.spin_h.setSingleStep(0.1)
        self.spin_h.setValue(0.0)
        grid2.addWidget(self.spin_h, 2, 2)


        reset_button = QtWidgets.QPushButton("Reset")
        grid2.addWidget(reset_button, 3, 0, 1, 3, QtCore.Qt.AlignCenter)
        
        reset_button.clicked.connect(self.reset_input_values)

        # Set the grid layout on the input group (assumed already created)
        input_group.setLayout(grid2)

        # Add the group box to the existing input panel's layout (assume it's stored in 'input_layout')
        input_layout.addWidget(input_group)

        # --- Synchronize Sliders and Spin Boxes with conversion ---
        self.slider_theta.valueChanged.connect(lambda val: self.spin_theta.setValue(val/100))
        self.spin_theta.valueChanged.connect(lambda val: self.slider_theta.setValue(int(val*100)))

        self.slider_phi.valueChanged.connect(lambda val: self.spin_phi.setValue(val/100))
        self.spin_phi.valueChanged.connect(lambda val: self.slider_phi.setValue(int(val*100)))

        self.slider_h.valueChanged.connect(lambda val: self.spin_h.setValue(val/100))
        self.spin_h.valueChanged.connect(lambda val: self.slider_h.setValue(int(val*100)))





        return_group = QtWidgets.QGroupBox("Input")
        return_group.setStyleSheet(
            "QGroupBox { font-weight: bold; border: 1.5px solid gray; margin-top: 10px; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px; }"
        )

        # Create a grid layout for the sliders and spin boxes
        grid3 = QtWidgets.QGridLayout()

        label_width = 30  # Adjust width as needed
        # Row 0: α, β, γ, ℎ in one row
        grid3.addWidget(QtWidgets.QLabel("α:"), 0, 0)
        self.label_alpha = QtWidgets.QLabel("0.00")
        self.label_alpha.setFixedWidth(label_width)
        self.label_alpha.setAlignment(QtCore.Qt.AlignCenter)
        grid3.addWidget(self.label_alpha, 0, 1)

        grid3.addWidget(QtWidgets.QLabel("β:"), 0, 2)
        self.label_beta = QtWidgets.QLabel("0.00")
        self.label_beta.setFixedWidth(label_width)
        self.label_beta.setAlignment(QtCore.Qt.AlignCenter)
        grid3.addWidget(self.label_beta, 0, 3)

        grid3.addWidget(QtWidgets.QLabel("γ:"), 0, 4)
        self.label_gamma = QtWidgets.QLabel("0.00")
        self.label_gamma.setFixedWidth(label_width)
        self.label_gamma.setAlignment(QtCore.Qt.AlignCenter)
        grid3.addWidget(self.label_gamma, 0, 5)

        grid3.addWidget(QtWidgets.QLabel("ℎ:"), 0, 6)
        self.label_h = QtWidgets.QLabel("0.00")
        self.label_h.setFixedWidth(label_width)
        self.label_h.setAlignment(QtCore.Qt.AlignCenter)
        grid3.addWidget(self.label_h, 0, 7)

        # Row 1: θ₁, θ₂, θ₃ in the next row
        grid3.addWidget(QtWidgets.QLabel("θ₁:"), 1, 0)
        self.label_theta1 = QtWidgets.QLabel("0.00")
        self.label_theta1.setFixedWidth(label_width)
        self.label_theta1.setAlignment(QtCore.Qt.AlignCenter)
        grid3.addWidget(self.label_theta1, 1, 1)

        grid3.addWidget(QtWidgets.QLabel("θ₂:"), 1, 2)
        self.label_theta2 = QtWidgets.QLabel("0.00")
        self.label_theta2.setFixedWidth(label_width)
        self.label_theta2.setAlignment(QtCore.Qt.AlignCenter)
        grid3.addWidget(self.label_theta2, 1, 3)

        grid3.addWidget(QtWidgets.QLabel("θ₃:"), 1, 4)
        self.label_theta3 = QtWidgets.QLabel("0.00")
        self.label_theta3.setFixedWidth(label_width)
        self.label_theta3.setAlignment(QtCore.Qt.AlignCenter)
        grid3.addWidget(self.label_theta3, 1, 5)

        # Add grid3 to the existing input panel's layout
        return_group.setLayout(grid3)
        input_layout.addWidget(return_group)







        # Finally, add the input panel to the main layout (adjust the stretch factor as needed)
        main_layout.addWidget(self.input_panel, 1)



        self.slider_theta.valueChanged.connect(self.update_inverse_kinematics)
        self.slider_phi.valueChanged.connect(self.update_inverse_kinematics)
        self.slider_h.valueChanged.connect(self.update_inverse_kinematics)

        self.validate_parameters()

        # --------------------------
        # Timer for Real-Time Updates
        # --------------------------
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)  # update every 50 milliseconds

    def compute_vector(self):

        theta_deg = self.slider_theta.value() / 100.0  
        phi_deg   = self.slider_phi.value() / 100.0     
        theta = math.radians(theta_deg)
        phi = math.radians(phi_deg)
        self.alpha = math.sin(theta) * math.cos(phi)
        self.beta  = math.sin(theta) * math.sin(phi)
        self.gamma = math.cos(theta)
        self.h = self.slider_h.value() / 100.0

        self.label_alpha.setText(f"{self.alpha:.2f}")
        self.label_beta.setText(f"{self.beta:.2f}")
        self.label_gamma.setText(f"{self.gamma:.2f}")
        self.label_h.setText(f"{self.h:.2f}")
        
    def reset_input_values(self):
        self.slider_theta.setValue(0)
        self.slider_phi.setValue(0)
        h_ = int((self.robot.maxh - self.robot.minh)*100//2)
        self.slider_h.setValue(h_)

    def input_changed(self):
        if self.auto_update_checkbox.isChecked():
            self.validate_parameters()

    def update_robot_parameters(self):
        self.robot.lp = self.input_lp.value()
        self.robot.l1 = self.input_l1.value()
        self.robot.l2 = self.input_l2.value()
        self.robot.lb = self.input_lb.value()
        self.robot.invert = self.input_invert.isChecked()

    def validate_parameters(self):

        l1 = self.input_l1.value()
        l2 = self.input_l2.value()
        lp = self.input_lp.value()
        lb = self.input_lb.value()
        
        if (l1 + l2) <= abs(lp - lb):
            QtWidgets.QMessageBox.warning(
                self, 
                "Invalid Parameters", 
                "The values must satisfy (l1+l2)^2 > |lp - lb|.\nChange rejected."
            )

            self.input_l1.setValue(self.robot.l1)
            self.input_l2.setValue(self.robot.l2)
            self.input_lp.setValue(self.robot.lp)
            self.input_lb.setValue(self.robot.lb)
            return

        # Recalculate maxh based on the new values.
        new_maxh = self.compute_maxh()
        self.robot.maxh = new_maxh
        self.label_maxh.setText(f"{new_maxh:.4f}") 

        new_minh = self.compute_minh()
        self.robot.minh = new_minh
        self.label_minh.setText(f"{new_minh:.4f}") 

        self.slider_h.setMaximum(int(new_maxh*100))
        self.spin_h.setMaximum(new_maxh)
        self.slider_h.setMinimum(int(new_minh*100))
        self.spin_h.setMinimum(new_minh)

        self.reset_input_values()
        self.update_robot_parameters()
        self.update_inverse_kinematics()

    def compute_maxh(self):
        l1, l2, lp, lb = self.input_l1.value(), self.input_l2.value(), self.input_lp.value(), self.input_lb.value()
        maxh = math.sqrt(((l1 + l2) ** 2) - ((lp - lb) ** 2))
        return maxh
    
    def compute_minh(self):
        l1, l2, lp, lb = self.input_l1.value(), self.input_l2.value(), self.input_lp.value(), self.input_lb.value()

        if l1 > l2:
            minh = math.sqrt((l1 ** 2) - ((lb + l2 - lp) ** 2))
        elif l2 > l1:
            minh = math.sqrt(((l2 - l1) ** 2) - ((lp - lb) ** 2))
        else:
            minh = 0

        return minh

    def update_inverse_kinematics(self):
        self.compute_vector()
        self.robot.solve_inverse_kinematics_vector(self.alpha, self.beta, self.gamma, self.h)

        self.label_theta1.setText(f"{self.robot.theta1:.2f}")
        self.label_theta2.setText(f"{self.robot.theta2:.2f}")
        self.label_theta3.setText(f"{self.robot.theta3:.2f}")


    def update_plot(self):
        """Update the 3D plot elements based on the current robot coordinates."""
        # Read positions from the robot (each attribute is assumed to be a 3-element array)
        A_points = np.array([self.robot.A1, self.robot.A2, self.robot.A3])
        B_points = np.array([self.robot.B1, self.robot.B2, self.robot.B3])
        C_points = np.array([self.robot.C1, self.robot.C2, self.robot.C3])

        self.scatter_A.setData(pos=A_points)
        self.scatter_B.setData(pos=B_points)
        self.scatter_C.setData(pos=C_points)

        # A loop: A1 → A2, A2 → A3, A3 → A1 (red)
        self.line_A.setData(pos=[
            self.robot.A1, self.robot.A2,
            self.robot.A2, self.robot.A3,
            self.robot.A3, self.robot.A1
        ])

        # B loop: B1 → B2, B2 → B3, B3 → B1 (blue)
        self.line_B.setData(pos=[
            self.robot.B1, self.robot.B2,
            self.robot.B2, self.robot.B3,
            self.robot.B3, self.robot.B1
        ])

        # C connections (green): three separate lines:
        # Line for connection between A1, C1, and B1
        self.line_C1.setData(pos=[self.robot.A1, self.robot.C1, self.robot.C1, self.robot.B1])
        # Line for connection between A2, C2, and B2
        self.line_C2.setData(pos=[self.robot.A2, self.robot.C2, self.robot.C2, self.robot.B2])
        # Line for connection between A3, C3, and B3
        self.line_C3.setData(pos=[self.robot.A3, self.robot.C3, self.robot.C3, self.robot.B3])


# ------------------------------------------------------------------------------
# **Main Execution**
# ------------------------------------------------------------------------------
if __name__ == '__main__':
    
    app = QtWidgets.QApplication(sys.argv)
    
    # Instantiate your robot (modify parameters as necessary)
    robot = RobotKinematics(lp=7.125, l1=6.2, l2=4.5, lb=4.0, invert=False)

    # Launch the visualization window
    window = MainWindow(robot)
    window.show()

    sys.exit(app.exec_())
