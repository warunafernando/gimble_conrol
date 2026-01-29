#!/usr/bin/env python3
"""
Pan-Tilt Board Test Utility - Windows GUI
Comprehensive testing tool for all board functions:
- Pan/Tilt servo control
- IMU data (roll, pitch, yaw, accel, gyro, magnetometer)
- Power monitoring (INA219)
- Servo feedback
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports
import json
import threading
import time
from datetime import datetime
import queue

class PanTiltTestGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Pan-Tilt Board Test Utility")
        self.root.geometry("1200x800")
        # Force window to front and center
        self.root.lift()
        self.root.attributes('-topmost', True)
        self.root.after_idle(lambda: self.root.attributes('-topmost', False))
        self.root.update()
        
        # Serial connection
        self.serial_port = None
        self.serial_thread = None
        self.running = False
        self.message_queue = queue.Queue()
        
        # Data storage
        self.last_imu_data = {}
        self.last_ina_data = {}
        self.last_servo_data = {}
        self.ping_responses = {}  # Store ping test results
        self.register_responses = {}  # For Servo Registers tab (T=2101, 2121, etc.)
        self._last_pan_only_sent = None   # Last pan value sent (avoid duplicate)
        self._last_tilt_only_sent = None  # Last tilt value sent (avoid duplicate)
        self._pan_only_timer_id = None    # Throttle: send pan while dragging
        self._tilt_only_timer_id = None   # Throttle: send tilt while dragging
        
        # Create GUI
        self.create_widgets()
        
        # Start message processing
        self.process_messages()
        
    def create_widgets(self):
        # Top frame: Connection
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=10)
        conn_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(conn_frame, text="COM Port:").grid(row=0, column=0, padx=5)
        self.port_var = tk.StringVar(value="COM9")
        port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=10, state="readonly")
        port_combo.grid(row=0, column=1, padx=5)
        self.refresh_ports(port_combo)
        ttk.Button(conn_frame, text="Refresh", command=lambda: self.refresh_ports(port_combo)).grid(row=0, column=2, padx=5)
        
        ttk.Label(conn_frame, text="Baud:").grid(row=0, column=3, padx=5)
        self.baud_var = tk.StringVar(value="921600")
        ttk.Combobox(conn_frame, textvariable=self.baud_var, values=["115200", "230400", "460800", "921600"], 
                     width=10, state="readonly").grid(row=0, column=4, padx=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=10)
        
        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=0, column=6, padx=10)
        
        # Main content: Notebook with tabs
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Tab 1: Pan/Tilt Control
        self.create_pan_tilt_tab(notebook)
        
        # Tab 2: IMU Data
        self.create_imu_tab(notebook)
        
        # Tab 3: Power Monitoring
        self.create_power_tab(notebook)
        
        # Tab 4: Servo Feedback
        self.create_servo_tab(notebook)
        
        # Tab 5: System Control
        self.create_system_tab(notebook)
        
        # Tab 6: Servo Test
        self.create_servo_test_tab(notebook)
        
        # Tab 7: Servo Registers (full ST3215: PID, torque, mode, limits, generic R/W)
        self.create_servo_registers_tab(notebook)
        
        # Tab 8: Log/Console
        self.create_log_tab(notebook)
        
    def refresh_ports(self, combo):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        combo['values'] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0] if ports else "")
    
    def create_pan_tilt_tab(self, notebook):
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="Pan/Tilt Control")
        
        # Single-axis control (main): apply on mouse release, only when value changed
        single_frame = ttk.LabelFrame(frame, text="Single-Axis Control (release to apply)", padding=10)
        single_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(single_frame, text="Pan Only:").grid(row=0, column=0, padx=5)
        self.pan_only_var = tk.DoubleVar(value=0.0)
        pan_only_scale = ttk.Scale(single_frame, from_=-180, to=180, variable=self.pan_only_var, orient=tk.HORIZONTAL, length=400)
        pan_only_scale.grid(row=0, column=1, padx=5)
        pan_only_scale.configure(command=lambda v: self._on_pan_scale_change(v))
        ttk.Label(single_frame, textvariable=self.pan_only_var, width=8).grid(row=0, column=2, padx=5)
        ttk.Button(single_frame, text="Set 0°", command=self._set_pan_zero).grid(row=0, column=3, padx=5)
        pan_only_scale.bind('<ButtonRelease-1>', self._on_pan_only_release)
        
        ttk.Label(single_frame, text="Tilt Only:").grid(row=1, column=0, padx=5)
        self.tilt_only_var = tk.DoubleVar(value=15.0)
        tilt_only_scale = ttk.Scale(single_frame, from_=-90, to=120, variable=self.tilt_only_var, orient=tk.HORIZONTAL, length=400)
        tilt_only_scale.grid(row=1, column=1, padx=5)
        tilt_only_scale.configure(command=lambda v: self._on_tilt_scale_change(v))
        ttk.Label(single_frame, textvariable=self.tilt_only_var, width=8).grid(row=1, column=2, padx=5)
        ttk.Button(single_frame, text="Set 40°", command=self._set_tilt_40).grid(row=1, column=3, padx=5)
        tilt_only_scale.bind('<ButtonRelease-1>', self._on_tilt_only_release)
        
        # Absolute control
        abs_frame = ttk.LabelFrame(frame, text="Absolute Position Control", padding=10)
        abs_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(abs_frame, text="Pan (deg):").grid(row=0, column=0, padx=5)
        self.pan_abs_var = tk.DoubleVar(value=0.0)
        pan_scale = ttk.Scale(abs_frame, from_=-180, to=180, variable=self.pan_abs_var, orient=tk.HORIZONTAL, length=300)
        pan_scale.grid(row=0, column=1, padx=5)
        pan_label = ttk.Label(abs_frame, textvariable=self.pan_abs_var, width=8)
        pan_label.grid(row=0, column=2, padx=5)
        pan_scale.configure(command=lambda v: self.pan_abs_var.set(round(float(v), 1)))
        
        ttk.Label(abs_frame, text="Tilt (deg):").grid(row=1, column=0, padx=5)
        self.tilt_abs_var = tk.DoubleVar(value=15.0)
        tilt_scale = ttk.Scale(abs_frame, from_=-90, to=120, variable=self.tilt_abs_var, orient=tk.HORIZONTAL, length=300)
        tilt_scale.grid(row=1, column=1, padx=5)
        tilt_label = ttk.Label(abs_frame, textvariable=self.tilt_abs_var, width=8)
        tilt_label.grid(row=1, column=2, padx=5)
        tilt_scale.configure(command=lambda v: self.tilt_abs_var.set(round(float(v), 1)))
        
        ttk.Button(abs_frame, text="Set Absolute", command=self.send_absolute).grid(row=0, column=3, rowspan=2, padx=10)
        ttk.Button(abs_frame, text="Center", command=self.center_pan_tilt).grid(row=0, column=4, rowspan=2, padx=5)
        
        # Move control
        move_frame = ttk.LabelFrame(frame, text="Relative Move Control", padding=10)
        move_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(move_frame, text="Pan (deg):").grid(row=0, column=0, padx=5)
        self.pan_move_var = tk.DoubleVar(value=0.0)
        ttk.Spinbox(move_frame, from_=-180, to=180, textvariable=self.pan_move_var, width=10).grid(row=0, column=1, padx=5)
        
        ttk.Label(move_frame, text="Tilt (deg):").grid(row=0, column=2, padx=5)
        self.tilt_move_var = tk.DoubleVar(value=15.0)
        ttk.Spinbox(move_frame, from_=-90, to=120, textvariable=self.tilt_move_var, width=10).grid(row=0, column=3, padx=5)
        
        ttk.Label(move_frame, text="Pan Speed:").grid(row=0, column=4, padx=5)
        self.pan_speed_var = tk.IntVar(value=300)
        ttk.Spinbox(move_frame, from_=1, to=1000, textvariable=self.pan_speed_var, width=10).grid(row=0, column=5, padx=5)
        
        ttk.Label(move_frame, text="Tilt Speed:").grid(row=1, column=4, padx=5)
        self.tilt_speed_var = tk.IntVar(value=300)
        ttk.Spinbox(move_frame, from_=1, to=1000, textvariable=self.tilt_speed_var, width=10).grid(row=1, column=5, padx=5)
        
        ttk.Button(move_frame, text="Move", command=self.send_move).grid(row=0, column=6, rowspan=2, padx=10)
        ttk.Button(move_frame, text="Stop", command=self.send_stop).grid(row=0, column=7, rowspan=2, padx=5)
        
        # Lock controls
        lock_frame = ttk.LabelFrame(frame, text="Axis Lock Control", padding=10)
        lock_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(lock_frame, text="Lock Pan", command=lambda: self.send_lock(170, 1)).grid(row=0, column=0, padx=5)
        ttk.Button(lock_frame, text="Unlock Pan", command=lambda: self.send_lock(170, 0)).grid(row=0, column=1, padx=5)
        ttk.Button(lock_frame, text="Lock Tilt", command=lambda: self.send_lock(171, 1)).grid(row=0, column=2, padx=5)
        ttk.Button(lock_frame, text="Unlock Tilt", command=lambda: self.send_lock(171, 0)).grid(row=0, column=3, padx=5)
        
        # User control (relative)
        user_frame = ttk.LabelFrame(frame, text="User Control (Relative)", padding=10)
        user_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(user_frame, text="X:").grid(row=0, column=0, padx=5)
        self.user_x_var = tk.IntVar(value=0)
        ttk.Spinbox(user_frame, from_=-2, to=2, textvariable=self.user_x_var, width=5).grid(row=0, column=1, padx=5)
        
        ttk.Label(user_frame, text="Y:").grid(row=0, column=2, padx=5)
        self.user_y_var = tk.IntVar(value=0)
        ttk.Spinbox(user_frame, from_=-2, to=2, textvariable=self.user_y_var, width=5).grid(row=0, column=3, padx=5)
        
        ttk.Label(user_frame, text="Speed:").grid(row=0, column=4, padx=5)
        self.user_speed_var = tk.IntVar(value=300)
        ttk.Spinbox(user_frame, from_=1, to=1000, textvariable=self.user_speed_var, width=10).grid(row=0, column=5, padx=5)
        
        ttk.Button(user_frame, text="Send User Control", command=self.send_user_control).grid(row=0, column=6, padx=10)
    
    def create_imu_tab(self, notebook):
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="IMU Data")
        
        # Control buttons
        ctrl_frame = ttk.Frame(frame)
        ctrl_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(ctrl_frame, text="Get IMU Data", command=self.request_imu).pack(side=tk.LEFT, padx=5)
        self.imu_auto_var = tk.BooleanVar()
        ttk.Checkbutton(ctrl_frame, text="Auto-update (1 Hz)", variable=self.imu_auto_var, 
                       command=self.toggle_imu_auto).pack(side=tk.LEFT, padx=5)
        
        # Data display
        data_frame = ttk.LabelFrame(frame, text="IMU Data", padding=10)
        data_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Euler angles
        euler_frame = ttk.LabelFrame(data_frame, text="Euler Angles (degrees)", padding=5)
        euler_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(euler_frame, text="Roll:").grid(row=0, column=0, padx=5, sticky=tk.W)
        self.roll_label = ttk.Label(euler_frame, text="0.0", font=("Arial", 12, "bold"), width=12)
        self.roll_label.grid(row=0, column=1, padx=5)
        
        ttk.Label(euler_frame, text="Pitch:").grid(row=0, column=2, padx=5, sticky=tk.W)
        self.pitch_label = ttk.Label(euler_frame, text="0.0", font=("Arial", 12, "bold"), width=12)
        self.pitch_label.grid(row=0, column=3, padx=5)
        
        ttk.Label(euler_frame, text="Yaw:").grid(row=0, column=4, padx=5, sticky=tk.W)
        self.yaw_label = ttk.Label(euler_frame, text="0.0", font=("Arial", 12, "bold"), width=12)
        self.yaw_label.grid(row=0, column=5, padx=5)
        
        # Accelerometer
        accel_frame = ttk.LabelFrame(data_frame, text="Accelerometer (g)", padding=5)
        accel_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(accel_frame, text="X:").grid(row=0, column=0, padx=5, sticky=tk.W)
        self.ax_label = ttk.Label(accel_frame, text="0.0", width=12)
        self.ax_label.grid(row=0, column=1, padx=5)
        ttk.Label(accel_frame, text="Y:").grid(row=0, column=2, padx=5, sticky=tk.W)
        self.ay_label = ttk.Label(accel_frame, text="0.0", width=12)
        self.ay_label.grid(row=0, column=3, padx=5)
        ttk.Label(accel_frame, text="Z:").grid(row=0, column=4, padx=5, sticky=tk.W)
        self.az_label = ttk.Label(accel_frame, text="0.0", width=12)
        self.az_label.grid(row=0, column=5, padx=5)
        
        # Gyroscope
        gyro_frame = ttk.LabelFrame(data_frame, text="Gyroscope (deg/s)", padding=5)
        gyro_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(gyro_frame, text="X:").grid(row=0, column=0, padx=5, sticky=tk.W)
        self.gx_label = ttk.Label(gyro_frame, text="0.0", width=12)
        self.gx_label.grid(row=0, column=1, padx=5)
        ttk.Label(gyro_frame, text="Y:").grid(row=0, column=2, padx=5, sticky=tk.W)
        self.gy_label = ttk.Label(gyro_frame, text="0.0", width=12)
        self.gy_label.grid(row=0, column=3, padx=5)
        ttk.Label(gyro_frame, text="Z:").grid(row=0, column=4, padx=5, sticky=tk.W)
        self.gz_label = ttk.Label(gyro_frame, text="0.0", width=12)
        self.gz_label.grid(row=0, column=5, padx=5)
        
        # Magnetometer
        mag_frame = ttk.LabelFrame(data_frame, text="Magnetometer", padding=5)
        mag_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(mag_frame, text="X:").grid(row=0, column=0, padx=5, sticky=tk.W)
        self.mx_label = ttk.Label(mag_frame, text="0", width=12)
        self.mx_label.grid(row=0, column=1, padx=5)
        ttk.Label(mag_frame, text="Y:").grid(row=0, column=2, padx=5, sticky=tk.W)
        self.my_label = ttk.Label(mag_frame, text="0", width=12)
        self.my_label.grid(row=0, column=3, padx=5)
        ttk.Label(mag_frame, text="Z:").grid(row=0, column=4, padx=5, sticky=tk.W)
        self.mz_label = ttk.Label(mag_frame, text="0", width=12)
        self.mz_label.grid(row=0, column=5, padx=5)
        
        # Temperature
        temp_frame = ttk.LabelFrame(data_frame, text="Temperature", padding=5)
        temp_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(temp_frame, text="IMU Temp:").grid(row=0, column=0, padx=5, sticky=tk.W)
        self.imu_temp_label = ttk.Label(temp_frame, text="0.0 °C", font=("Arial", 10, "bold"), width=12)
        self.imu_temp_label.grid(row=0, column=1, padx=5)
    
    def create_power_tab(self, notebook):
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="Power Monitoring")
        
        # Control
        ctrl_frame = ttk.Frame(frame)
        ctrl_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(ctrl_frame, text="Get Power Data", command=self.request_ina).pack(side=tk.LEFT, padx=5)
        self.ina_auto_var = tk.BooleanVar()
        ttk.Checkbutton(ctrl_frame, text="Auto-update (2 Hz)", variable=self.ina_auto_var,
                       command=self.toggle_ina_auto).pack(side=tk.LEFT, padx=5)
        
        # Data display
        data_frame = ttk.LabelFrame(frame, text="INA219 Power Monitor", padding=10)
        data_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Voltage
        ttk.Label(data_frame, text="Bus Voltage:", font=("Arial", 11, "bold")).grid(row=0, column=0, padx=10, pady=5, sticky=tk.W)
        self.bus_v_label = ttk.Label(data_frame, text="0.00 V", font=("Arial", 14, "bold"), foreground="blue")
        self.bus_v_label.grid(row=0, column=1, padx=10, pady=5)
        
        ttk.Label(data_frame, text="Load Voltage:", font=("Arial", 11)).grid(row=1, column=0, padx=10, pady=5, sticky=tk.W)
        self.load_v_label = ttk.Label(data_frame, text="0.00 V", font=("Arial", 12))
        self.load_v_label.grid(row=1, column=1, padx=10, pady=5)
        
        ttk.Label(data_frame, text="Shunt Voltage:", font=("Arial", 11)).grid(row=2, column=0, padx=10, pady=5, sticky=tk.W)
        self.shunt_v_label = ttk.Label(data_frame, text="0.00 mV", font=("Arial", 12))
        self.shunt_v_label.grid(row=2, column=1, padx=10, pady=5)
        
        # Current and Power
        ttk.Label(data_frame, text="Current:", font=("Arial", 11, "bold")).grid(row=3, column=0, padx=10, pady=5, sticky=tk.W)
        self.current_label = ttk.Label(data_frame, text="0.00 mA", font=("Arial", 14, "bold"), foreground="green")
        self.current_label.grid(row=3, column=1, padx=10, pady=5)
        
        ttk.Label(data_frame, text="Power:", font=("Arial", 11, "bold")).grid(row=4, column=0, padx=10, pady=5, sticky=tk.W)
        self.power_label = ttk.Label(data_frame, text="0.00 mW", font=("Arial", 14, "bold"), foreground="orange")
        self.power_label.grid(row=4, column=1, padx=10, pady=5)
        
        # Overflow status
        ttk.Label(data_frame, text="Overflow:", font=("Arial", 11)).grid(row=5, column=0, padx=10, pady=5, sticky=tk.W)
        self.overflow_label = ttk.Label(data_frame, text="No", font=("Arial", 12), foreground="green")
        self.overflow_label.grid(row=5, column=1, padx=10, pady=5)
    
    def create_servo_tab(self, notebook):
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="Servo Feedback")
        
        # Control
        ctrl_frame = ttk.Frame(frame)
        ctrl_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(ctrl_frame, text="Get Servo Data", command=self.request_servo).pack(side=tk.LEFT, padx=5)
        self.servo_auto_var = tk.BooleanVar()
        ttk.Checkbutton(ctrl_frame, text="Auto-update (2 Hz)", variable=self.servo_auto_var,
                       command=self.toggle_servo_auto).pack(side=tk.LEFT, padx=5)
        
        # Pan servo
        pan_frame = ttk.LabelFrame(frame, text="Pan Servo (ID 2)", padding=10)
        pan_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(pan_frame, text="Position:").grid(row=0, column=0, padx=5, sticky=tk.W)
        self.pan_pos_label = ttk.Label(pan_frame, text="0", font=("Arial", 11, "bold"), width=10)
        self.pan_pos_label.grid(row=0, column=1, padx=5)
        ttk.Label(pan_frame, text="(deg):").grid(row=0, column=2, padx=5)
        self.pan_deg_label = ttk.Label(pan_frame, text="0.0", width=8)
        self.pan_deg_label.grid(row=0, column=3, padx=5)
        
        ttk.Label(pan_frame, text="Speed:").grid(row=1, column=0, padx=5, sticky=tk.W)
        self.pan_speed_label = ttk.Label(pan_frame, text="0", width=10)
        self.pan_speed_label.grid(row=1, column=1, padx=5)
        
        ttk.Label(pan_frame, text="Load:").grid(row=1, column=2, padx=5, sticky=tk.W)
        self.pan_load_label = ttk.Label(pan_frame, text="0", width=10)
        self.pan_load_label.grid(row=1, column=3, padx=5)
        
        ttk.Label(pan_frame, text="Voltage:").grid(row=2, column=0, padx=5, sticky=tk.W)
        self.pan_voltage_label = ttk.Label(pan_frame, text="0.0 V", width=10)
        self.pan_voltage_label.grid(row=2, column=1, padx=5)
        
        ttk.Label(pan_frame, text="Temperature:").grid(row=2, column=2, padx=5, sticky=tk.W)
        self.pan_temp_label = ttk.Label(pan_frame, text="0 °C", width=10)
        self.pan_temp_label.grid(row=2, column=3, padx=5)
        
        ttk.Label(pan_frame, text="Mode:").grid(row=3, column=0, padx=5, sticky=tk.W)
        self.pan_mode_label = ttk.Label(pan_frame, text="Unknown", width=10)
        self.pan_mode_label.grid(row=3, column=1, padx=5)
        
        # Tilt servo
        tilt_frame = ttk.LabelFrame(frame, text="Tilt Servo (ID 1)", padding=10)
        tilt_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(tilt_frame, text="Position:").grid(row=0, column=0, padx=5, sticky=tk.W)
        self.tilt_pos_label = ttk.Label(tilt_frame, text="0", font=("Arial", 11, "bold"), width=10)
        self.tilt_pos_label.grid(row=0, column=1, padx=5)
        ttk.Label(tilt_frame, text="(deg):").grid(row=0, column=2, padx=5)
        self.tilt_deg_label = ttk.Label(tilt_frame, text="0.0", width=8)
        self.tilt_deg_label.grid(row=0, column=3, padx=5)
        
        ttk.Label(tilt_frame, text="Speed:").grid(row=1, column=0, padx=5, sticky=tk.W)
        self.tilt_speed_label = ttk.Label(tilt_frame, text="0", width=10)
        self.tilt_speed_label.grid(row=1, column=1, padx=5)
        
        ttk.Label(tilt_frame, text="Load:").grid(row=1, column=2, padx=5, sticky=tk.W)
        self.tilt_load_label = ttk.Label(tilt_frame, text="0", width=10)
        self.tilt_load_label.grid(row=1, column=3, padx=5)
        
        ttk.Label(tilt_frame, text="Voltage:").grid(row=2, column=0, padx=5, sticky=tk.W)
        self.tilt_voltage_label = ttk.Label(tilt_frame, text="0.0 V", width=10)
        self.tilt_voltage_label.grid(row=2, column=1, padx=5)
        
        ttk.Label(tilt_frame, text="Temperature:").grid(row=2, column=2, padx=5, sticky=tk.W)
        self.tilt_temp_label = ttk.Label(tilt_frame, text="0 °C", width=10)
        self.tilt_temp_label.grid(row=2, column=3, padx=5)
        
        ttk.Label(tilt_frame, text="Mode:").grid(row=3, column=0, padx=5, sticky=tk.W)
        self.tilt_mode_label = ttk.Label(tilt_frame, text="Unknown", width=10)
        self.tilt_mode_label.grid(row=3, column=1, padx=5)
    
    def create_system_tab(self, notebook):
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="System Control")
        
        # Feedback control
        fb_frame = ttk.LabelFrame(frame, text="Feedback Control", padding=10)
        fb_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(fb_frame, text="Interval (ms):").grid(row=0, column=0, padx=5)
        self.fb_interval_var = tk.IntVar(value=100)
        ttk.Spinbox(fb_frame, from_=10, to=1000, textvariable=self.fb_interval_var, width=10).grid(row=0, column=1, padx=5)
        ttk.Button(fb_frame, text="Set Interval", command=self.set_feedback_interval).grid(row=0, column=2, padx=10)
        
        ttk.Button(fb_frame, text="Enable Feedback", command=lambda: self.set_feedback(1)).grid(row=1, column=0, padx=5)
        ttk.Button(fb_frame, text="Disable Feedback", command=lambda: self.set_feedback(0)).grid(row=1, column=1, padx=5)
        
        # Test buttons
        test_frame = ttk.LabelFrame(frame, text="Quick Tests", padding=10)
        test_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(test_frame, text="Test All Sensors", command=self.test_all_sensors).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(test_frame, text="Sweep Pan", command=self.test_sweep_pan).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(test_frame, text="Sweep Tilt", command=self.test_sweep_tilt).grid(row=0, column=2, padx=5, pady=5)
        ttk.Button(test_frame, text="Figure-8 Pattern", command=self.test_figure8).grid(row=0, column=3, padx=5, pady=5)
    
    def create_servo_test_tab(self, notebook):
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="Servo Test")
        
        # Servo ID Ping Test
        ping_frame = ttk.LabelFrame(frame, text="Servo ID Ping Test", padding=10)
        ping_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(ping_frame, text="Test Servo ID:").grid(row=0, column=0, padx=5)
        self.test_id_var = tk.IntVar(value=1)
        ttk.Spinbox(ping_frame, from_=1, to=254, textvariable=self.test_id_var, width=10).grid(row=0, column=1, padx=5)
        ttk.Button(ping_frame, text="Ping Servo", command=self.test_ping_servo).grid(row=0, column=2, padx=10)
        ttk.Button(ping_frame, text="Test All IDs (1-10)", command=self.test_all_servo_ids).grid(row=0, column=3, padx=10)
        
        # Ping Results
        self.ping_result_text = scrolledtext.ScrolledText(ping_frame, height=8, wrap=tk.WORD)
        self.ping_result_text.grid(row=1, column=0, columnspan=4, sticky="ew", padx=5, pady=5)
        ping_frame.columnconfigure(0, weight=1)
        
        # Servo Movement Test
        move_test_frame = ttk.LabelFrame(frame, text="Servo Movement Test", padding=10)
        move_test_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(move_test_frame, text="Servo ID:").grid(row=0, column=0, padx=5)
        self.move_test_id_var = tk.IntVar(value=1)
        ttk.Spinbox(move_test_frame, from_=1, to=254, textvariable=self.move_test_id_var, width=10).grid(row=0, column=1, padx=5)
        
        ttk.Label(move_test_frame, text="Position (0-4095):").grid(row=0, column=2, padx=5)
        self.move_test_pos_var = tk.IntVar(value=2048)
        ttk.Spinbox(move_test_frame, from_=0, to=4095, textvariable=self.move_test_pos_var, width=10).grid(row=0, column=3, padx=5)
        
        ttk.Label(move_test_frame, text="Speed:").grid(row=0, column=4, padx=5)
        self.move_test_speed_var = tk.IntVar(value=500)
        ttk.Spinbox(move_test_frame, from_=0, to=3400, textvariable=self.move_test_speed_var, width=10).grid(row=0, column=5, padx=5)
        
        ttk.Button(move_test_frame, text="Move Servo", command=self.test_move_servo).grid(row=0, column=6, padx=10)
        ttk.Button(move_test_frame, text="Center (2048)", command=self.test_center_servo).grid(row=0, column=7, padx=5)
        
        # Movement Test Results
        self.move_test_result_text = scrolledtext.ScrolledText(move_test_frame, height=6, wrap=tk.WORD)
        self.move_test_result_text.grid(row=1, column=0, columnspan=8, sticky="ew", padx=5, pady=5)
        move_test_frame.columnconfigure(0, weight=1)
        
        # Set Servo ID
        set_id_frame = ttk.LabelFrame(frame, text="Set Servo ID (Program Servo)", padding=10)
        set_id_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(set_id_frame, text="Change ID from:").grid(row=0, column=0, padx=5)
        self.set_id_from_var = tk.IntVar(value=1)
        ttk.Spinbox(set_id_frame, from_=1, to=254, textvariable=self.set_id_from_var, width=10).grid(row=0, column=1, padx=5)
        
        ttk.Label(set_id_frame, text="to:").grid(row=0, column=2, padx=5)
        self.set_id_to_var = tk.IntVar(value=2)
        ttk.Spinbox(set_id_frame, from_=1, to=254, textvariable=self.set_id_to_var, width=10).grid(row=0, column=3, padx=5)
        
        # Create a style for the warning button
        style = ttk.Style()
        style.configure("Warning.TButton", foreground="red")
        set_id_btn = ttk.Button(set_id_frame, text="Set Servo ID", command=self.set_servo_id, 
                                style="Warning.TButton")
        set_id_btn.grid(row=0, column=4, padx=10)
        
        warning_label = tk.Label(set_id_frame, text="⚠️ WARNING: This permanently changes the servo ID!", 
                                foreground="red", font=("Arial", 9, "bold"))
        warning_label.grid(row=1, column=0, columnspan=5, padx=5, pady=5)
        
        # Set ID Results
        self.set_id_result_text = scrolledtext.ScrolledText(set_id_frame, height=4, wrap=tk.WORD)
        self.set_id_result_text.grid(row=2, column=0, columnspan=5, sticky="ew", padx=5, pady=5)
        set_id_frame.columnconfigure(0, weight=1)
        
        # Quick Tests
        quick_test_frame = ttk.LabelFrame(frame, text="Quick Diagnostic Tests", padding=10)
        quick_test_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(quick_test_frame, text="Test ID 1 Only", command=self.test_id1_only).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(quick_test_frame, text="Test ID 2 Only", command=self.test_id2_only).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(quick_test_frame, text="Test Both IDs", command=self.test_both_ids).grid(row=0, column=2, padx=5, pady=5)
        ttk.Button(quick_test_frame, text="Clear Results", command=self.clear_servo_test_results).grid(row=0, column=3, padx=5, pady=5)
        
        # Store ping responses
        self.ping_responses = {}
    
    def create_servo_registers_tab(self, notebook):
        """Full ST3215 register access: PID, torque, mode, limits, generic R/W."""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="Servo Registers")
        
        # Servo ID
        id_frame = ttk.LabelFrame(frame, text="Servo ID", padding=5)
        id_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(id_frame, text="ID:").grid(row=0, column=0, padx=5)
        self.reg_id_var = tk.IntVar(value=1)
        ttk.Spinbox(id_frame, from_=1, to=254, textvariable=self.reg_id_var, width=6).grid(row=0, column=1, padx=5)
        
        # PID (addrs 21=P, 22=D, 23=I)
        pid_frame = ttk.LabelFrame(frame, text="PID (addrs 21=P, 22=D, 23=I)", padding=5)
        pid_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(pid_frame, text="P:").grid(row=0, column=0, padx=2)
        self.reg_p_var = tk.IntVar(value=16)
        ttk.Spinbox(pid_frame, from_=0, to=255, textvariable=self.reg_p_var, width=6).grid(row=0, column=1, padx=2)
        ttk.Label(pid_frame, text="I:").grid(row=0, column=2, padx=2)
        self.reg_i_var = tk.IntVar(value=0)
        ttk.Spinbox(pid_frame, from_=0, to=255, textvariable=self.reg_i_var, width=6).grid(row=0, column=3, padx=2)
        ttk.Label(pid_frame, text="D:").grid(row=0, column=4, padx=2)
        self.reg_d_var = tk.IntVar(value=0)
        ttk.Spinbox(pid_frame, from_=0, to=255, textvariable=self.reg_d_var, width=6).grid(row=0, column=5, padx=2)
        ttk.Button(pid_frame, text="Read PID", command=self.reg_read_pid).grid(row=0, column=6, padx=5)
        ttk.Button(pid_frame, text="Write PID", command=self.reg_write_pid).grid(row=0, column=7, padx=5)
        
        # Torque: Enable (40), Limit (48)
        torque_frame = ttk.LabelFrame(frame, text="Torque (40=enable, 48=limit)", padding=5)
        torque_frame.pack(fill=tk.X, padx=5, pady=2)
        self.reg_torque_enable_var = tk.IntVar(value=1)
        ttk.Checkbutton(torque_frame, text="Enable", variable=self.reg_torque_enable_var).grid(row=0, column=0, padx=5)
        ttk.Label(torque_frame, text="Limit (0-1000):").grid(row=0, column=1, padx=5)
        self.reg_torque_limit_var = tk.IntVar(value=1000)
        ttk.Spinbox(torque_frame, from_=0, to=1000, textvariable=self.reg_torque_limit_var, width=8).grid(row=0, column=2, padx=5)
        ttk.Button(torque_frame, text="Read Torque", command=self.reg_read_torque).grid(row=0, column=3, padx=5)
        ttk.Button(torque_frame, text="Write Torque", command=self.reg_write_torque).grid(row=0, column=4, padx=5)
        
        # Mode (33): 0=position, 1=wheel
        mode_frame = ttk.LabelFrame(frame, text="Mode (33): 0=position, 1=wheel", padding=5)
        mode_frame.pack(fill=tk.X, padx=5, pady=2)
        self.reg_mode_var = tk.IntVar(value=0)
        ttk.Radiobutton(mode_frame, text="Position", variable=self.reg_mode_var, value=0).grid(row=0, column=0, padx=5)
        ttk.Radiobutton(mode_frame, text="Wheel", variable=self.reg_mode_var, value=1).grid(row=0, column=1, padx=5)
        ttk.Button(mode_frame, text="Read Mode", command=self.reg_read_mode).grid(row=0, column=2, padx=5)
        ttk.Button(mode_frame, text="Write Mode", command=self.reg_write_mode).grid(row=0, column=3, padx=5)
        
        # Position (42=goal, 56=present), Speed, Load, Voltage, Temp, Current
        read_frame = ttk.LabelFrame(frame, text="Read: Position(56), Speed(58), Load(60), Voltage(62), Temp(63), Current(69)", padding=5)
        read_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Button(read_frame, text="Read All", command=self.reg_read_all).grid(row=0, column=0, padx=5)
        
        # Angle limits (9-12)
        limits_frame = ttk.LabelFrame(frame, text="Angle Limits (9-12 min, 11-12 max)", padding=5)
        limits_frame.pack(fill=tk.X, padx=5, pady=2)
        self.reg_min_limit_var = tk.IntVar(value=0)
        self.reg_max_limit_var = tk.IntVar(value=4095)
        ttk.Label(limits_frame, text="Min:").grid(row=0, column=0, padx=2)
        ttk.Spinbox(limits_frame, from_=0, to=4095, textvariable=self.reg_min_limit_var, width=8).grid(row=0, column=1, padx=2)
        ttk.Label(limits_frame, text="Max:").grid(row=0, column=2, padx=2)
        ttk.Spinbox(limits_frame, from_=0, to=4095, textvariable=self.reg_max_limit_var, width=8).grid(row=0, column=3, padx=2)
        ttk.Button(limits_frame, text="Read Limits", command=self.reg_read_limits).grid(row=0, column=4, padx=5)
        ttk.Button(limits_frame, text="Write Limits", command=self.reg_write_limits).grid(row=0, column=5, padx=5)
        
        # EPROM & Calibrate
        eprom_frame = ttk.LabelFrame(frame, text="EPROM (55) & Calibrate", padding=5)
        eprom_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Button(eprom_frame, text="Unlock EPROM (55=0)", command=lambda: self.reg_write_byte(55, 0)).grid(row=0, column=0, padx=5)
        ttk.Button(eprom_frame, text="Lock EPROM (55=1)", command=lambda: self.reg_write_byte(55, 1)).grid(row=0, column=1, padx=5)
        ttk.Button(eprom_frame, text="Calibrate (set current as middle)", command=self.reg_calibrate).grid(row=0, column=2, padx=5)
        
        # Generic read/write
        gen_frame = ttk.LabelFrame(frame, text="Generic: Read/Write Byte or Word", padding=5)
        gen_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(gen_frame, text="Addr:").grid(row=0, column=0, padx=2)
        self.reg_addr_var = tk.IntVar(value=40)
        ttk.Spinbox(gen_frame, from_=0, to=70, textvariable=self.reg_addr_var, width=6).grid(row=0, column=1, padx=2)
        ttk.Label(gen_frame, text="Value:").grid(row=0, column=2, padx=2)
        self.reg_value_var = tk.IntVar(value=0)
        ttk.Spinbox(gen_frame, from_=0, to=65535, textvariable=self.reg_value_var, width=8).grid(row=0, column=3, padx=2)
        ttk.Button(gen_frame, text="Read Byte", command=self.reg_read_byte).grid(row=0, column=4, padx=5)
        ttk.Button(gen_frame, text="Write Byte", command=self.reg_write_byte_generic).grid(row=0, column=5, padx=5)
        ttk.Button(gen_frame, text="Read Word", command=self.reg_read_word).grid(row=0, column=6, padx=5)
        ttk.Button(gen_frame, text="Write Word", command=self.reg_write_word_generic).grid(row=0, column=7, padx=5)
        
        # Results
        res_frame = ttk.LabelFrame(frame, text="Register Results", padding=5)
        res_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.reg_result_text = scrolledtext.ScrolledText(res_frame, height=12, wrap=tk.WORD)
        self.reg_result_text.pack(fill=tk.BOTH, expand=True)
        ttk.Button(res_frame, text="Clear", command=lambda: self.reg_result_text.delete(1.0, tk.END)).pack(anchor=tk.W)
    
    def _reg_id(self):
        return max(1, min(254, self.reg_id_var.get()))
    
    def _reg_append(self, msg):
        self.reg_result_text.insert(tk.END, msg + "\n")
        self.reg_result_text.see(tk.END)
    
    def handle_register_response(self, reg_type, data):
        if reg_type == 2101:  # Read byte
            self._reg_append(f"Read Byte: id={data.get('id')} addr={data.get('addr')} value={data.get('value')}")
        elif reg_type == 2111:
            self._reg_append(f"Write Byte: id={data.get('id')} addr={data.get('addr')} ok={data.get('ok')}")
        elif reg_type == 2121:  # Read word
            self._reg_append(f"Read Word: id={data.get('id')} addr={data.get('addr')} value={data.get('value')}")
        elif reg_type == 2131:
            self._reg_append(f"Write Word: id={data.get('id')} addr={data.get('addr')} ok={data.get('ok')}")
        elif reg_type == 5021:
            self._reg_append(f"Calibrate: id={data.get('id')} ok={data.get('ok')}")
    
    def reg_read_pid(self):
        sid = self._reg_id()
        for addr, name in [(21, "P"), (22, "D"), (23, "I")]:
            self.send_command({"T": 210, "id": sid, "addr": addr})
    
    def reg_write_pid(self):
        sid = self._reg_id()
        try:
            p, i, d = self.reg_p_var.get(), self.reg_i_var.get(), self.reg_d_var.get()
        except tk.TclError:
            p, i, d = 16, 0, 0
        for addr, val in [(21, p), (22, d), (23, i)]:
            self.send_command({"T": 211, "id": sid, "addr": addr, "value": max(0, min(255, val))})
    
    def reg_read_torque(self):
        sid = self._reg_id()
        self.send_command({"T": 210, "id": sid, "addr": 40})
        self.send_command({"T": 212, "id": sid, "addr": 48})
    
    def reg_write_torque(self):
        sid = self._reg_id()
        self.send_command({"T": 211, "id": sid, "addr": 40, "value": 1 if self.reg_torque_enable_var.get() else 0})
        try:
            lim = max(0, min(1000, self.reg_torque_limit_var.get()))
        except tk.TclError:
            lim = 1000
        self.send_command({"T": 213, "id": sid, "addr": 48, "value": lim})
    
    def reg_read_mode(self):
        self.send_command({"T": 210, "id": self._reg_id(), "addr": 33})
    
    def reg_write_mode(self):
        self.send_command({"T": 211, "id": self._reg_id(), "addr": 33, "value": self.reg_mode_var.get()})
    
    def reg_read_all(self):
        sid = self._reg_id()
        for addr in [56, 58, 60, 62, 63, 69]:
            if addr in (56, 58, 60, 69):
                self.send_command({"T": 212, "id": sid, "addr": addr})
            else:
                self.send_command({"T": 210, "id": sid, "addr": addr})
    
    def reg_read_limits(self):
        sid = self._reg_id()
        self.send_command({"T": 212, "id": sid, "addr": 9})
        self.send_command({"T": 212, "id": sid, "addr": 11})
    
    def reg_write_limits(self):
        sid = self._reg_id()
        try:
            mn = max(0, min(4095, self.reg_min_limit_var.get()))
            mx = max(0, min(4095, self.reg_max_limit_var.get()))
        except tk.TclError:
            mn, mx = 0, 4095
        self.send_command({"T": 213, "id": sid, "addr": 9, "value": mn})
        self.send_command({"T": 213, "id": sid, "addr": 11, "value": mx})
    
    def reg_write_byte(self, addr, value):
        self.send_command({"T": 211, "id": self._reg_id(), "addr": addr, "value": value & 0xFF})
    
    def reg_calibrate(self):
        self.send_command({"T": 502, "id": self._reg_id()})
    
    def reg_read_byte(self):
        self.send_command({"T": 210, "id": self._reg_id(), "addr": self.reg_addr_var.get()})
    
    def reg_write_byte_generic(self):
        try:
            v = self.reg_value_var.get() & 0xFF
        except tk.TclError:
            v = 0
        self.send_command({"T": 211, "id": self._reg_id(), "addr": self.reg_addr_var.get(), "value": v})
    
    def reg_read_word(self):
        self.send_command({"T": 212, "id": self._reg_id(), "addr": self.reg_addr_var.get()})
    
    def reg_write_word_generic(self):
        try:
            v = self.reg_value_var.get() & 0xFFFF
        except tk.TclError:
            v = 0
        self.send_command({"T": 213, "id": self._reg_id(), "addr": self.reg_addr_var.get(), "value": v})
    
    def create_log_tab(self, notebook):
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="Log/Console")
        
        # Log display
        self.log_text = scrolledtext.ScrolledText(frame, height=30, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Control buttons
        btn_frame = ttk.Frame(frame)
        btn_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(btn_frame, text="Clear Log", command=self.clear_log).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Save Log", command=self.save_log).pack(side=tk.LEFT, padx=5)
        self.log_auto_scroll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(btn_frame, text="Auto-scroll", variable=self.log_auto_scroll_var).pack(side=tk.LEFT, padx=5)
    
    def log(self, message, level="INFO"):
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] [{level}] {message}\n"
        self.log_text.insert(tk.END, log_entry)
        if self.log_auto_scroll_var.get():
            self.log_text.see(tk.END)
    
    def clear_log(self):
        self.log_text.delete(1.0, tk.END)
    
    def save_log(self):
        from tkinter import filedialog
        filename = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
        )
        if filename:
            with open(filename, 'w') as f:
                f.write(self.log_text.get(1.0, tk.END))
            self.log(f"Log saved to {filename}")
    
    def toggle_connection(self):
        if self.serial_port and self.serial_port.is_open:
            self.disconnect()
        else:
            self.connect()
    
    def connect(self):
        try:
            port = self.port_var.get()
            baud = int(self.baud_var.get())
            
            self.serial_port = serial.Serial(port, baud, timeout=1)
            time.sleep(0.5)  # Give port time to initialize
            
            self.running = True
            self.serial_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
            self.serial_thread.start()
            
            self.connect_btn.config(text="Disconnect")
            self.status_label.config(text="Connected", foreground="green")
            self.log(f"Connected to {port} at {baud} baud")
            
            time.sleep(0.2)
            self.set_feedback(1)  # Enable periodic feedback
            time.sleep(0.1)
            
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect:\n{e}")
            self.log(f"Connection failed: {e}", "ERROR")
    
    def disconnect(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.serial_port = None
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="Disconnected", foreground="red")
        self.log("Disconnected")
    
    def serial_read_loop(self):
        buffer = ""
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            self.message_queue.put(('RX', line))
                else:
                    time.sleep(0.01)
            except Exception as e:
                if self.running:
                    self.message_queue.put(('ERROR', str(e)))
                break
    
    def process_messages(self):
        try:
            while True:
                msg_type, data = self.message_queue.get_nowait()
                
                if msg_type == 'RX':
                    self.handle_received_message(data)
                elif msg_type == 'REG':
                    reg_type, reg_data = data
                    self.handle_register_response(reg_type, reg_data)
                elif msg_type == 'ERROR':
                    self.log(f"Serial error: {data}", "ERROR")
                    self.disconnect()
        except queue.Empty:
            pass
        
        self.root.after(50, self.process_messages)
    
    def handle_received_message(self, message):
        try:
            data = json.loads(message)
            msg_type = data.get('T', 0)
            
            if msg_type == 1002:  # IMU feedback
                self.update_imu_display(data)
            elif msg_type == 1010:  # INA219 feedback
                self.update_ina_display(data)
            elif msg_type == 1011:  # Servo feedback
                self.update_servo_display(data)
            elif msg_type in (2101, 2111, 2121, 2131, 5021):  # Register read/write / calibrate
                self.message_queue.put(('REG', (msg_type, data)))
            elif msg_type == 2001:  # Ping response
                self.handle_ping_response(data)
            elif msg_type == 5002:  # Set ID success
                self.handle_set_id_success(data)
            elif msg_type == 5003:  # Set ID verification
                self.handle_set_id_verification(data)
            elif msg_type == 5001:  # Set ID error
                self.handle_set_id_error(data)
            
            self.log(f"RX: {message}")
        except json.JSONDecodeError:
            self.log(f"RX (non-JSON): {message}")
        except Exception as e:
            self.log(f"Error processing message: {e}", "ERROR")
    
    def send_command(self, cmd_dict):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not Connected", "Please connect to a serial port first")
            return False
        
        try:
            cmd_json = json.dumps(cmd_dict, separators=(',', ':'))
            bytes_written = self.serial_port.write((cmd_json + '\n').encode('utf-8'))
            self.serial_port.flush()  # Ensure data is sent immediately
            self.log(f"TX: {cmd_json} ({bytes_written} bytes)")
            return True
        except Exception as e:
            self.log(f"Send error: {e}", "ERROR")
            messagebox.showerror("Send Error", f"Failed to send command:\n{e}")
            return False
    
    # Command senders
    def send_absolute(self):
        pan_val = round(self.pan_abs_var.get(), 1)
        tilt_val = round(self.tilt_abs_var.get(), 1)
        self.log(f"Sending Absolute: Pan={pan_val}°, Tilt={tilt_val}° (T=133)", "DEBUG")
        self.send_command({
            "T": 133,
            "X": pan_val,
            "Y": tilt_val,
            "SPD": 3400,
            "ACC": 100
        })
    
    def center_pan_tilt(self):
        self.pan_abs_var.set(0.0)
        self.tilt_abs_var.set(15.0)  # tilt range -90–120, center 15°
        self.send_absolute()
    
    def _on_pan_scale_change(self, v):
        self.pan_only_var.set(round(float(v), 1))
        self._schedule_pan_send()
    
    def _on_tilt_scale_change(self, v):
        self.tilt_only_var.set(round(float(v), 1))
        self._schedule_tilt_send()
    
    def _schedule_pan_send(self):
        if self._pan_only_timer_id:
            self.root.after_cancel(self._pan_only_timer_id)
        self._pan_only_timer_id = self.root.after(80, self._throttled_send_pan)
    
    def _schedule_tilt_send(self):
        if self._tilt_only_timer_id:
            self.root.after_cancel(self._tilt_only_timer_id)
        self._tilt_only_timer_id = self.root.after(80, self._throttled_send_tilt)
    
    def _throttled_send_pan(self):
        self._pan_only_timer_id = None
        pan_val = round(self.pan_only_var.get(), 1)
        if pan_val != self._last_pan_only_sent:
            self._last_pan_only_sent = pan_val
            self.send_pan_only()
    
    def _throttled_send_tilt(self):
        self._tilt_only_timer_id = None
        tilt_val = round(self.tilt_only_var.get(), 1)
        if tilt_val != self._last_tilt_only_sent:
            self._last_tilt_only_sent = tilt_val
            self.send_tilt_only()
    
    def _on_pan_only_release(self, event):
        if self._pan_only_timer_id:
            self.root.after_cancel(self._pan_only_timer_id)
            self._pan_only_timer_id = None
        pan_val = round(self.pan_only_var.get(), 1)
        if pan_val != self._last_pan_only_sent:
            self._last_pan_only_sent = pan_val
            self.send_pan_only()
    
    def _on_tilt_only_release(self, event):
        if self._tilt_only_timer_id:
            self.root.after_cancel(self._tilt_only_timer_id)
            self._tilt_only_timer_id = None
        tilt_val = round(self.tilt_only_var.get(), 1)
        if tilt_val != self._last_tilt_only_sent:
            self._last_tilt_only_sent = tilt_val
            self.send_tilt_only()
    
    def _set_pan_zero(self):
        """Set Pan Only to 0° and send command."""
        self.pan_only_var.set(0.0)
        self._last_pan_only_sent = 0.0
        self.send_pan_only()
    
    def _set_tilt_40(self):
        """Set Tilt Only to 40° and send command."""
        self.tilt_only_var.set(40.0)
        self._last_tilt_only_sent = 40.0
        self.send_tilt_only()
    
    def send_pan_only(self):
        pan_val = round(self.pan_only_var.get(), 1)
        self.log(f"Sending Pan Only: {pan_val}° (T=172)", "DEBUG")
        result = self.send_command({
            "T": 172,
            "X": pan_val,
            "SPD": 3400,
            "ACC": 100
        })
        if not result:
            self.log("Failed to send Pan Only command", "ERROR")
    
    def send_tilt_only(self):
        tilt_val = round(self.tilt_only_var.get(), 1)
        self.log(f"Sending Tilt Only: {tilt_val}° (T=173)", "DEBUG")
        result = self.send_command({
            "T": 173,
            "Y": tilt_val,
            "SPD": 3400,
            "ACC": 100
        })
        if not result:
            self.log("Failed to send Tilt Only command", "ERROR")
    
    def send_move(self):
        try:
            pan = round(float(self.pan_move_var.get()), 1)
            tilt = round(float(self.tilt_move_var.get()), 1)
            sx = int(self.pan_speed_var.get())
            sy = int(self.tilt_speed_var.get())
        except (tk.TclError, ValueError):
            pan = tilt = 0.0
            sx = sy = 3400
        result = self.send_command({
            "T": 134,
            "X": pan,
            "Y": tilt,
            "SX": min(3400, max(1, sx)),
            "SY": min(3400, max(1, sy))
        })
    
    def send_stop(self):
        self.send_command({"T": 135})
    
    def send_lock(self, cmd_type, lock_state):
        self.send_command({"T": cmd_type, "cmd": lock_state})
    
    def send_user_control(self):
        try:
            x = int(self.user_x_var.get())
            y = int(self.user_y_var.get())
            spd = int(self.user_speed_var.get())
        except (tk.TclError, ValueError):
            x = y = 0
            spd = 300
        result = self.send_command({
            "T": 141,
            "X": max(-1, min(2, x)),
            "Y": max(-1, min(2, y)),
            "SPD": min(1000, max(1, spd))
        })
    
    def request_imu(self):
        self.send_command({"T": 126})
    
    def request_ina(self):
        self.send_command({"T": 160})
    
    def request_servo(self):
        # Servo feedback is typically sent automatically, but we can request it
        # For now, just enable periodic feedback if not already enabled
        self.set_feedback(1)
    
    def set_feedback_interval(self):
        self.send_command({
            "T": 142,
            "cmd": self.fb_interval_var.get()
        })
    
    def set_feedback(self, enable):
        self.send_command({
            "T": 131,
            "cmd": 1 if enable else 0
        })
    
    # Display updaters
    def update_imu_display(self, data):
        self.last_imu_data = data
        self.roll_label.config(text=f"{data.get('r', 0.0):.2f}°")
        self.pitch_label.config(text=f"{data.get('p', 0.0):.2f}°")
        self.yaw_label.config(text=f"{data.get('y', 0.0):.2f}°")
        self.ax_label.config(text=f"{data.get('ax', 0.0):.3f}")
        self.ay_label.config(text=f"{data.get('ay', 0.0):.3f}")
        self.az_label.config(text=f"{data.get('az', 0.0):.3f}")
        self.gx_label.config(text=f"{data.get('gx', 0.0):.2f}")
        self.gy_label.config(text=f"{data.get('gy', 0.0):.2f}")
        self.gz_label.config(text=f"{data.get('gz', 0.0):.2f}")
        self.mx_label.config(text=f"{data.get('mx', 0)}")
        self.my_label.config(text=f"{data.get('my', 0)}")
        self.mz_label.config(text=f"{data.get('mz', 0)}")
        self.imu_temp_label.config(text=f"{data.get('temp', 0.0):.1f} °C")
    
    def update_ina_display(self, data):
        self.last_ina_data = data
        self.bus_v_label.config(text=f"{data.get('bus_v', 0.0):.2f} V")
        self.load_v_label.config(text=f"{data.get('load_v', 0.0):.2f} V")
        self.shunt_v_label.config(text=f"{data.get('shunt_mv', 0.0):.2f} mV")
        self.current_label.config(text=f"{data.get('current_ma', 0.0):.2f} mA")
        self.power_label.config(text=f"{data.get('power_mw', 0.0):.2f} mW")
        overflow = data.get('overflow', 0)
        if overflow:
            self.overflow_label.config(text="Yes", foreground="red")
        else:
            self.overflow_label.config(text="No", foreground="green")
    
    def update_servo_display(self, data):
        self.last_servo_data = data
        pan = data.get('pan', {})
        tilt = data.get('tilt', {})
        
        # Pan
        pan_pos = pan.get('pos', 0)
        pan_deg = (pan_pos - 2047) * 360.0 / 4095.0
        self.pan_pos_label.config(text=str(pan_pos))
        self.pan_deg_label.config(text=f"{pan_deg:.1f}°")
        self.pan_speed_label.config(text=str(pan.get('speed', 0)))
        self.pan_load_label.config(text=str(pan.get('load', 0)))
        self.pan_voltage_label.config(text=f"{pan.get('v', 0.0):.1f} V")
        self.pan_temp_label.config(text=f"{pan.get('temp', 0)} °C")
        mode = pan.get('mode', 0)
        self.pan_mode_label.config(text="Locked" if mode == 1 else "Free")
        
        # Tilt
        tilt_pos = tilt.get('pos', 0)
        tilt_deg = (tilt_pos - 2047) * 360.0 / 4095.0
        self.tilt_pos_label.config(text=str(tilt_pos))
        self.tilt_deg_label.config(text=f"{tilt_deg:.1f}°")
        self.tilt_speed_label.config(text=str(tilt.get('speed', 0)))
        self.tilt_load_label.config(text=str(tilt.get('load', 0)))
        self.tilt_voltage_label.config(text=f"{tilt.get('v', 0.0):.1f} V")
        self.tilt_temp_label.config(text=f"{tilt.get('temp', 0)} °C")
        mode = tilt.get('mode', 0)
        self.tilt_mode_label.config(text="Locked" if mode == 1 else "Free")
    
    # Auto-update handlers
    def toggle_imu_auto(self):
        if self.imu_auto_var.get():
            self.auto_update_imu()
    
    def toggle_ina_auto(self):
        if self.ina_auto_var.get():
            self.auto_update_ina()
    
    def toggle_servo_auto(self):
        if self.servo_auto_var.get():
            self.auto_update_servo()
    
    def auto_update_imu(self):
        if self.imu_auto_var.get() and self.serial_port and self.serial_port.is_open:
            self.request_imu()
            self.root.after(1000, self.auto_update_imu)  # 1 Hz
    
    def auto_update_ina(self):
        if self.ina_auto_var.get() and self.serial_port and self.serial_port.is_open:
            self.request_ina()
            self.root.after(500, self.auto_update_ina)  # 2 Hz
    
    def auto_update_servo(self):
        if self.servo_auto_var.get() and self.serial_port and self.serial_port.is_open:
            # Servo feedback comes automatically if feedback is enabled
            self.set_feedback(1)
            self.root.after(500, self.auto_update_servo)  # 2 Hz
    
    # Test functions
    def test_all_sensors(self):
        self.log("Testing all sensors...")
        self.request_imu()
        time.sleep(0.2)
        self.request_ina()
        time.sleep(0.2)
        self.request_servo()
    
    def test_sweep_pan(self):
        self.log("Testing pan sweep...")
        def sweep():
            for angle in range(-180, 181, 10):
                if not (self.serial_port and self.serial_port.is_open):
                    break
                self.send_command({"T": 172, "X": float(angle), "SPD": 3400, "ACC": 100})
                time.sleep(0.2)
            # Return to center
            self.send_command({"T": 172, "X": 0.0, "SPD": 3400, "ACC": 100})
        threading.Thread(target=sweep, daemon=True).start()
    
    def test_sweep_tilt(self):
        self.log("Testing tilt sweep...")
        def sweep():
            for angle in range(-90, 121, 5):
                if not (self.serial_port and self.serial_port.is_open):
                    break
                self.send_command({"T": 173, "Y": float(angle), "SPD": 3400, "ACC": 100})
                time.sleep(0.2)
            # Return to center
            self.send_command({"T": 173, "Y": 15.0, "SPD": 3400, "ACC": 100})
        threading.Thread(target=sweep, daemon=True).start()
    
    def test_figure8(self):
        self.log("Testing figure-8 pattern...")
        def figure8():
            import math
            steps = 50
            for i in range(steps):
                if not (self.serial_port and self.serial_port.is_open):
                    break
                t = (i / steps) * 2 * math.pi
                pan = 90 * math.sin(t)
                tilt = 15.0 + 30 * math.sin(2 * t)  # center 15°, range -90–120
                self.send_command({"T": 133, "X": round(pan, 1), "Y": round(tilt, 1), "SPD": 3400, "ACC": 100})
                time.sleep(0.1)
            # Return to center
            self.send_command({"T": 133, "X": 0.0, "Y": 15.0, "SPD": 3400, "ACC": 100})
        threading.Thread(target=figure8, daemon=True).start()
    
    # Servo test functions
    def handle_ping_response(self, data):
        """Handle ping response from firmware (T=2001)"""
        servo_id = data.get('id', 0)
        responded = data.get('responded', 0)
        result = data.get('result', -1)
        
        timestamp = datetime.now().strftime("%H:%M:%S")
        status = "✓ RESPONDED" if responded else "✗ NO RESPONSE"
        color = "green" if responded else "red"
        
        result_text = f"[{timestamp}] Servo ID {servo_id}: {status} (result={result})\n"
        
        if hasattr(self, 'ping_result_text'):
            self.ping_result_text.insert(tk.END, result_text)
            self.ping_result_text.see(tk.END)
        
        # Store response
        if not hasattr(self, 'ping_responses'):
            self.ping_responses = {}
        self.ping_responses[servo_id] = {
            'responded': responded,
            'result': result,
            'timestamp': timestamp
        }
        
        self.log(f"Ping response: ID {servo_id} {'responded' if responded else 'no response'}")
    
    def test_ping_servo(self):
        """Ping a single servo ID"""
        servo_id = self.test_id_var.get()
        self.log(f"Pinging servo ID {servo_id}...")
        
        if hasattr(self, 'ping_result_text'):
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.ping_result_text.insert(tk.END, f"[{timestamp}] Pinging servo ID {servo_id}...\n")
            self.ping_result_text.see(tk.END)
        
        self.send_command({
            "T": 200,
            "id": servo_id
        })
    
    def test_all_servo_ids(self):
        """Test all servo IDs from 1 to 10"""
        self.log("Testing servo IDs 1-10...")
        
        if hasattr(self, 'ping_result_text'):
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.ping_result_text.insert(tk.END, f"[{timestamp}] Testing servo IDs 1-10...\n")
            self.ping_result_text.see(tk.END)
        
        # Send pings with small delays
        def ping_next_id(idx):
            if idx <= 10:
                self.send_command({
                    "T": 200,
                    "id": idx
                })
                self.root.after(300, lambda: ping_next_id(idx + 1))
        
        ping_next_id(1)
    
    def test_move_servo(self):
        """Test moving a servo to a specific position"""
        servo_id = self.move_test_id_var.get()
        position = self.move_test_pos_var.get()
        speed = self.move_test_speed_var.get()
        
        self.log(f"Moving servo ID {servo_id} to position {position} at speed {speed}...")
        
        if hasattr(self, 'move_test_result_text'):
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.move_test_result_text.insert(tk.END, 
                f"[{timestamp}] Moving servo ID {servo_id} to {position} (speed={speed})...\n")
            self.move_test_result_text.see(tk.END)
        
        # Convert position to degrees for pan/tilt commands
        # Position 2048 = 0 degrees (center)
        # Full range: 0-4095 = -180 to +180 degrees (approximately)
        degrees = ((position - 2048) / 2048.0) * 180.0
        
        # Use pan-only or tilt-only command based on ID
        # This is a test - we'll use pan for ID 1, tilt for ID 2
        if servo_id == 1:
            self.send_command({
                "T": 172,  # Pan only
                "X": round(degrees, 1),
                "SPD": 3400,  # High torque
                "ACC": 100
            })
        elif servo_id == 2:
            self.send_command({
                "T": 173,  # Tilt only
                "Y": round(degrees, 1),
                "SPD": 3400,  # High torque
                "ACC": 100
            })
        else:
            self.log(f"Warning: Direct servo control not implemented for ID {servo_id}", "WARNING")
    
    def test_center_servo(self):
        """Center a servo (position 2048)"""
        self.move_test_pos_var.set(2048)
        self.test_move_servo()
    
    def test_id1_only(self):
        """Test ID 1 only - move pan"""
        self.log("Testing ID 1 (Pan) only...")
        
        if hasattr(self, 'ping_result_text'):
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.ping_result_text.insert(tk.END, f"[{timestamp}] Testing ID 1 (Pan)...\n")
            self.ping_result_text.see(tk.END)
        
        # Ping first
        self.send_command({"T": 200, "id": 1})
        
        # Then try to move with high torque
        self.root.after(500, lambda: self.send_command({
            "T": 172,  # Pan only
            "X": 45.0,
            "SPD": 3400,  # High torque
            "ACC": 100
        }))
        
        self.root.after(2000, lambda: self.send_command({
            "T": 172,  # Pan only
            "X": -45.0,
            "SPD": 3400,  # High torque
            "ACC": 100
        }))
        
        self.root.after(3500, lambda: self.send_command({
            "T": 172,  # Pan only
            "X": 0.0,
            "SPD": 3400,  # High torque
            "ACC": 100
        }))
    
    def test_id2_only(self):
        """Test ID 2 only - move tilt"""
        self.log("Testing ID 2 (Tilt) only...")
        
        if hasattr(self, 'ping_result_text'):
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.ping_result_text.insert(tk.END, f"[{timestamp}] Testing ID 2 (Tilt)...\n")
            self.ping_result_text.see(tk.END)
        
        # Ping first
        self.send_command({"T": 200, "id": 2})
        
        # Then try to move with high torque
        self.root.after(500, lambda: self.send_command({
            "T": 173,  # Tilt only
            "Y": 30.0,
            "SPD": 3400,  # High torque
            "ACC": 100
        }))
        
        self.root.after(2000, lambda: self.send_command({
            "T": 173,  # Tilt only
            "Y": -90.0,
            "SPD": 3400,  # High torque
            "ACC": 100
        }))
        
        self.root.after(3500, lambda: self.send_command({
            "T": 173,  # Tilt only
            "Y": 15.0,
            "SPD": 3400,  # High torque
            "ACC": 100
        }))
    
    def test_both_ids(self):
        """Test both IDs - ping both"""
        self.log("Testing both servo IDs...")
        
        if hasattr(self, 'ping_result_text'):
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.ping_result_text.insert(tk.END, f"[{timestamp}] Testing both IDs...\n")
            self.ping_result_text.see(tk.END)
        
        # Ping both
        self.send_command({"T": 200, "id": 1})
        self.root.after(300, lambda: self.send_command({"T": 200, "id": 2}))
    
    def clear_servo_test_results(self):
        """Clear test result displays"""
        if hasattr(self, 'ping_result_text'):
            self.ping_result_text.delete(1.0, tk.END)
        if hasattr(self, 'move_test_result_text'):
            self.move_test_result_text.delete(1.0, tk.END)
        if not hasattr(self, 'ping_responses'):
            self.ping_responses = {}
        self.ping_responses = {}
        self.log("Servo test results cleared")
    
    def set_servo_id(self):
        """Set servo ID (permanently changes servo ID)"""
        from_id = self.set_id_from_var.get()
        to_id = self.set_id_to_var.get()
        
        if from_id == to_id:
            messagebox.showwarning("Invalid", "From ID and To ID must be different!")
            return
        
        # Confirm action
        result = messagebox.askyesno(
            "Confirm Servo ID Change",
            f"This will PERMANENTLY change servo ID from {from_id} to {to_id}.\n\n"
            f"Make sure only ONE servo is connected with ID {from_id}.\n\n"
            f"Continue?",
            icon="warning"
        )
        
        if not result:
            return
        
        self.log(f"Setting servo ID from {from_id} to {to_id}...")
        
        if hasattr(self, 'set_id_result_text'):
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.set_id_result_text.insert(tk.END, 
                f"[{timestamp}] Changing servo ID from {from_id} to {to_id}...\n")
            self.set_id_result_text.see(tk.END)
        
        self.send_command({
            "T": 501,
            "from": from_id,
            "to": to_id
        })
    
    def handle_set_id_success(self, data):
        """Handle successful servo ID change"""
        from_id = data.get('from', 0)
        to_id = data.get('to', 0)
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        result_text = f"[{timestamp}] ✓ Successfully changed servo ID from {from_id} to {to_id}\n"
        result_text += f"    Verifying new ID...\n"
        
        if hasattr(self, 'set_id_result_text'):
            self.set_id_result_text.insert(tk.END, result_text)
            self.set_id_result_text.see(tk.END)
        
        self.log(f"Servo ID changed successfully: {from_id} -> {to_id}")
    
    def handle_set_id_verification(self, data):
        """Handle servo ID verification after change"""
        servo_id = data.get('id', 0)
        verified = data.get('verified', 0)
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        if verified:
            result_text = f"[{timestamp}] ✓✓ Verification SUCCESS: Servo ID {servo_id} is now active!\n"
            result_text += f"    You can now ping ID {servo_id} to confirm.\n"
            color = "green"
        else:
            result_text = f"[{timestamp}] ⚠ Verification FAILED: Servo ID {servo_id} did not respond.\n"
            result_text += f"    Try power cycling the servo and ping again.\n"
            color = "orange"
        
        if hasattr(self, 'set_id_result_text'):
            self.set_id_result_text.insert(tk.END, result_text)
            self.set_id_result_text.see(tk.END)
        
        if verified:
            messagebox.showinfo("Success", f"Servo ID successfully changed to {servo_id}!\n\nVerification confirmed.")
        else:
            messagebox.showwarning("Verification Failed", 
                f"Servo ID change may have succeeded, but verification failed.\n\n"
                f"Try power cycling the servo and testing again.")
    
    def handle_set_id_error(self, data):
        """Handle servo ID change error"""
        error = data.get('error', 'Unknown error')
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        result_text = f"[{timestamp}] ✗ ERROR: {error}\n"
        
        if hasattr(self, 'set_id_result_text'):
            self.set_id_result_text.insert(tk.END, result_text)
            self.set_id_result_text.see(tk.END)
        
        messagebox.showerror("Set ID Failed", f"Failed to change servo ID:\n{error}")
        self.log(f"Set ID error: {error}", "ERROR")


def main():
    print("Starting GUI...")
    print("Creating root window...")
    root = tk.Tk()
    print("Root window created")
    print("Creating application...")
    app = PanTiltTestGUI(root)
    print("Application created, starting mainloop...")
    print("GUI should now be visible!")
    root.mainloop()
    print("Mainloop ended")


if __name__ == '__main__':
    main()
