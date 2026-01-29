#!/usr/bin/env python3
"""
IMU Data Monitor GUI for ESP32 Pan-Tilt System

A graphical interface to read and display IMU data from the ESP32 via USB serial.
The ESP32 must be running the pan_tilt_base_v0.9 firmware.

Usage:
    python imu_monitor_gui.py
"""

import json
import sys
import threading
import time
import math
from datetime import datetime
from tkinter import ttk, Tk, StringVar, Label, Frame, Button, Entry, messagebox, LabelFrame, Canvas
import serial


class IMUMonitorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("IMU Data Monitor - ESP32 Pan-Tilt")
        self.root.geometry("900x700")
        
        # Serial connection
        self.ser = None
        self.is_connected = False
        self.is_running = False
        self.update_thread = None
        
        # Data variables
        self.data_vars = {
            'roll': StringVar(value="0.00"),
            'pitch': StringVar(value="0.00"),
            'yaw': StringVar(value="0.00"),
            'ax': StringVar(value="0.000"),
            'ay': StringVar(value="0.000"),
            'az': StringVar(value="0.000"),
            'gx': StringVar(value="0.000"),
            'gy': StringVar(value="0.000"),
            'gz': StringVar(value="0.000"),
            'mx': StringVar(value="0"),
            'my': StringVar(value="0"),
            'mz': StringVar(value="0"),
            'temp': StringVar(value="N/A"),
            'status': StringVar(value="Disconnected"),
            'timestamp': StringVar(value="--:--:--"),
            'heading': StringVar(value="0.0°"),
            'direction': StringVar(value="N")
        }
        
        # Debug mode (set to True to print received data)
        self.debug = False
        
        # Magnetic declination (adjust for your location)
        # Find your declination at: http://www.magnetic-declination.com/
        self.magnetic_declination = 0.0  # degrees (positive = east, negative = west)
        
        self.create_widgets()
        
    def create_widgets(self):
        # Connection frame
        conn_frame = Frame(self.root, padx=10, pady=10)
        conn_frame.pack(fill='x')
        
        Label(conn_frame, text="Port:", font=('Arial', 10)).grid(row=0, column=0, padx=5)
        self.port_entry = Entry(conn_frame, width=10)
        self.port_entry.insert(0, "COM6")
        self.port_entry.grid(row=0, column=1, padx=5)
        
        Label(conn_frame, text="Baud:", font=('Arial', 10)).grid(row=0, column=2, padx=5)
        self.baud_entry = Entry(conn_frame, width=10)
        self.baud_entry.insert(0, "115200")
        self.baud_entry.grid(row=0, column=3, padx=5)
        
        self.connect_btn = Button(conn_frame, text="Connect", command=self.toggle_connection, 
                                  bg='#4CAF50', fg='white', font=('Arial', 10, 'bold'), width=10)
        self.connect_btn.grid(row=0, column=4, padx=10)
        
        # Status label
        status_frame = Frame(self.root, padx=10, pady=5)
        status_frame.pack(fill='x')
        Label(status_frame, text="Status:", font=('Arial', 10, 'bold')).pack(side='left', padx=5)
        self.status_label = Label(status_frame, textvariable=self.data_vars['status'], 
                                  font=('Arial', 10), fg='red')
        self.status_label.pack(side='left', padx=5)
        
        # Main data display frame
        data_frame = Frame(self.root, padx=20, pady=10)
        data_frame.pack(fill='both', expand=True)
        
        # Title
        title_label = Label(data_frame, text="IMU Sensor Data", 
                           font=('Arial', 16, 'bold'))
        title_label.pack(pady=(0, 20))
        
        # Euler Angles section
        angles_frame = LabelFrame(data_frame, text="Euler Angles (degrees)", 
                                  font=('Arial', 12, 'bold'), padx=10, pady=10)
        angles_frame.pack(fill='x', pady=5)
        
        self.create_data_row(angles_frame, "Roll:", self.data_vars['roll'], "°", 0)
        self.create_data_row(angles_frame, "Pitch:", self.data_vars['pitch'], "°", 1)
        self.create_data_row(angles_frame, "Yaw:", self.data_vars['yaw'], "°", 2)
        
        # Accelerometer section
        accel_frame = LabelFrame(data_frame, text="Accelerometer (m/s²)", 
                                font=('Arial', 12, 'bold'), padx=10, pady=10)
        accel_frame.pack(fill='x', pady=5)
        
        self.create_data_row(accel_frame, "X:", self.data_vars['ax'], "m/s²", 0)
        self.create_data_row(accel_frame, "Y:", self.data_vars['ay'], "m/s²", 1)
        self.create_data_row(accel_frame, "Z:", self.data_vars['az'], "m/s²", 2)
        
        # Gyroscope section
        gyro_frame = LabelFrame(data_frame, text="Gyroscope (rad/s)", 
                                font=('Arial', 12, 'bold'), padx=10, pady=10)
        gyro_frame.pack(fill='x', pady=5)
        
        self.create_data_row(gyro_frame, "X:", self.data_vars['gx'], "rad/s", 0)
        self.create_data_row(gyro_frame, "Y:", self.data_vars['gy'], "rad/s", 1)
        self.create_data_row(gyro_frame, "Z:", self.data_vars['gz'], "rad/s", 2)
        
        # Magnetometer section
        mag_frame = LabelFrame(data_frame, text="Magnetometer (uT)", 
                              font=('Arial', 12, 'bold'), padx=10, pady=10)
        mag_frame.pack(fill='x', pady=5)
        
        self.create_data_row(mag_frame, "X:", self.data_vars['mx'], "uT", 0)
        self.create_data_row(mag_frame, "Y:", self.data_vars['my'], "uT", 1)
        self.create_data_row(mag_frame, "Z:", self.data_vars['mz'], "uT", 2)
        
        # Temperature
        temp_frame = Frame(data_frame)
        temp_frame.pack(fill='x', pady=5)
        Label(temp_frame, text="Temperature:", font=('Arial', 10)).pack(side='left', padx=5)
        Label(temp_frame, textvariable=self.data_vars['temp'], 
              font=('Arial', 10, 'bold'), fg='blue').pack(side='left', padx=5)
        Label(temp_frame, text="°C", font=('Arial', 10)).pack(side='left')
        
        # Timestamp
        timestamp_frame = Frame(data_frame)
        timestamp_frame.pack(fill='x', pady=5)
        Label(timestamp_frame, text="Last Update:", font=('Arial', 9)).pack(side='left', padx=5)
        Label(timestamp_frame, textvariable=self.data_vars['timestamp'], 
              font=('Arial', 9), fg='gray').pack(side='left', padx=5)
        
        # Compass section
        compass_frame = LabelFrame(data_frame, text="Compass / North Finding", 
                                   font=('Arial', 12, 'bold'), padx=10, pady=10)
        compass_frame.pack(fill='x', pady=10)
        
        # Compass display
        compass_display_frame = Frame(compass_frame)
        compass_display_frame.pack(pady=10)
        
        # Compass canvas
        self.compass_canvas = Canvas(compass_display_frame, width=200, height=200, bg='white', relief='sunken', bd=2)
        self.compass_canvas.pack(side='left', padx=10)
        
        # Heading info
        heading_info_frame = Frame(compass_display_frame)
        heading_info_frame.pack(side='left', padx=20)
        
        Label(heading_info_frame, text="Heading:", font=('Arial', 12, 'bold')).pack(anchor='w', pady=5)
        heading_value_label = Label(heading_info_frame, textvariable=self.data_vars['heading'], 
                                   font=('Arial', 16, 'bold'), fg='blue')
        heading_value_label.pack(anchor='w', pady=5)
        
        Label(heading_info_frame, text="Direction:", font=('Arial', 12, 'bold')).pack(anchor='w', pady=5)
        direction_label = Label(heading_info_frame, textvariable=self.data_vars['direction'], 
                               font=('Arial', 20, 'bold'), fg='red', width=5)
        direction_label.pack(anchor='w', pady=5)
        
        # Draw initial compass
        self.draw_compass(0)
        
    def create_data_row(self, parent, label_text, var, unit, row):
        row_frame = Frame(parent)
        row_frame.grid(row=row, column=0, sticky='w', pady=2)
        Label(row_frame, text=label_text, font=('Arial', 10), width=8, anchor='w').pack(side='left', padx=5)
        value_label = Label(row_frame, textvariable=var, font=('Arial', 10, 'bold'), 
                           fg='blue', width=12, anchor='e')
        value_label.pack(side='left', padx=5)
        Label(row_frame, text=unit, font=('Arial', 10)).pack(side='left', padx=2)
    
    def calculate_heading(self, mx, my, mz, roll, pitch):
        """Calculate compass heading from magnetometer data."""
        # Convert roll and pitch to radians
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        
        # Tilt compensation: rotate magnetometer vector to horizontal plane
        # Using simplified tilt compensation
        cos_roll = math.cos(roll_rad)
        sin_roll = math.sin(roll_rad)
        cos_pitch = math.cos(pitch_rad)
        sin_pitch = math.sin(pitch_rad)
        
        # Compensate for tilt
        Xh = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch
        Yh = my * cos_roll - mz * sin_roll
        
        # Calculate heading in degrees
        heading = math.degrees(math.atan2(Yh, Xh))
        
        # Normalize to 0-360
        if heading < 0:
            heading += 360
        
        # Apply magnetic declination
        heading += self.magnetic_declination
        if heading >= 360:
            heading -= 360
        elif heading < 0:
            heading += 360
        
        return heading
    
    def get_direction_name(self, heading):
        """Get direction name from heading angle."""
        directions = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE',
                     'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW']
        index = int((heading + 11.25) / 22.5) % 16
        return directions[index]
    
    def draw_compass(self, heading):
        """Draw compass with heading indicator."""
        self.compass_canvas.delete("all")
        
        center_x = 100
        center_y = 100
        radius = 80
        
        # Draw compass circle
        self.compass_canvas.create_oval(center_x - radius, center_y - radius,
                                        center_x + radius, center_y + radius,
                                        outline='black', width=2)
        
        # Draw cardinal directions
        directions = ['N', 'E', 'S', 'W']
        angles = [0, 90, 180, 270]
        
        for i, (direction, angle) in enumerate(zip(directions, angles)):
            angle_rad = math.radians(angle - heading)
            x = center_x + (radius - 15) * math.sin(angle_rad)
            y = center_y - (radius - 15) * math.cos(angle_rad)
            
            color = 'red' if direction == 'N' else 'black'
            font_size = 14 if direction == 'N' else 12
            self.compass_canvas.create_text(x, y, text=direction, 
                                           font=('Arial', font_size, 'bold'),
                                           fill=color)
        
        # Draw intermediate directions
        inter_directions = ['NE', 'SE', 'SW', 'NW']
        inter_angles = [45, 135, 225, 315]
        
        for direction, angle in zip(inter_directions, inter_angles):
            angle_rad = math.radians(angle - heading)
            x = center_x + (radius - 10) * math.sin(angle_rad)
            y = center_y - (radius - 10) * math.cos(angle_rad)
            self.compass_canvas.create_text(x, y, text=direction, 
                                           font=('Arial', 8),
                                           fill='gray')
        
        # Draw tick marks
        for angle in range(0, 360, 30):
            angle_rad = math.radians(angle - heading)
            x1 = center_x + (radius - 5) * math.sin(angle_rad)
            y1 = center_y - (radius - 5) * math.cos(angle_rad)
            x2 = center_x + radius * math.sin(angle_rad)
            y2 = center_y - radius * math.cos(angle_rad)
            self.compass_canvas.create_line(x1, y1, x2, y2, width=1, fill='black')
        
        # Draw north pointer (red arrow pointing up)
        arrow_size = 20
        self.compass_canvas.create_polygon(
            center_x, center_y - radius + 5,  # Top point
            center_x - arrow_size/2, center_y - radius + arrow_size,  # Bottom left
            center_x, center_y - radius + arrow_size/2,  # Bottom center
            center_x + arrow_size/2, center_y - radius + arrow_size,  # Bottom right
            fill='red', outline='darkred', width=2
        )
        
        # Draw center dot
        self.compass_canvas.create_oval(center_x - 3, center_y - 3,
                                       center_x + 3, center_y + 3,
                                       fill='black')
        
    def toggle_connection(self):
        if not self.is_connected:
            self.connect()
        else:
            self.disconnect()
    
    def connect(self):
        port = self.port_entry.get().strip()
        try:
            baud = int(self.baud_entry.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid baud rate")
            return
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.5)
            time.sleep(0.5)
            self.ser.reset_input_buffer()
            
            self.is_connected = True
            self.is_running = True
            self.data_vars['status'].set(f"Connected to {port}")
            self.status_label.config(fg='green')
            self.connect_btn.config(text="Disconnect", bg='#f44336')
            self.port_entry.config(state='disabled')
            self.baud_entry.config(state='disabled')
            
            # Start update thread
            self.update_thread = threading.Thread(target=self.update_loop, daemon=True)
            self.update_thread.start()
            
        except serial.SerialException as e:
            messagebox.showerror("Connection Error", 
                               f"Could not connect to {port}:\n{str(e)}\n\n"
                               "Make sure:\n"
                               "1. The ESP32 is connected\n"
                               "2. The port is correct\n"
                               "3. No other program is using the port")
        except Exception as e:
            messagebox.showerror("Error", f"Unexpected error: {str(e)}")
    
    def disconnect(self):
        self.is_running = False
        self.is_connected = False
        
        if self.ser and self.ser.is_open:
            self.ser.close()
        
        self.data_vars['status'].set("Disconnected")
        self.status_label.config(fg='red')
        self.connect_btn.config(text="Connect", bg='#4CAF50')
        self.port_entry.config(state='normal')
        self.baud_entry.config(state='normal')
        
        # Reset all values
        for var in self.data_vars.values():
            if var != self.data_vars['status']:
                if var == self.data_vars['temp']:
                    var.set("N/A")
                elif var == self.data_vars['timestamp']:
                    var.set("--:--:--")
                else:
                    var.set("0.00" if '.' in var.get() else "0")
    
    def send_command(self, cmd):
        """Send JSON command to ESP32."""
        if not self.ser or not self.ser.is_open:
            return
        try:
            line = json.dumps(cmd, separators=(",", ":")) + "\n"
            self.ser.write(line.encode("utf-8"))
            self.ser.flush()
            if self.debug:
                print(f"[DEBUG] Sent command: {cmd}")
        except Exception as e:
            if self.debug:
                print(f"[DEBUG] Error sending command: {e}")
    
    def read_response(self, timeout=0.5):
        """Read JSON response from ESP32."""
        if not self.ser or not self.ser.is_open:
            if self.debug:
                print("[DEBUG] Serial port not open")
            return {}
        
        end_time = time.time() + timeout
        buffer = ""
        
        if self.debug:
            print(f"[DEBUG] Reading response (timeout: {timeout}s)")
        
        while time.time() < end_time and self.is_running:
            if self.ser.in_waiting > 0:
                try:
                    data = self.ser.read(self.ser.in_waiting)
                    decoded = data.decode("utf-8", errors="replace")
                    buffer += decoded
                    if self.debug:
                        print(f"[DEBUG] Read {len(data)} bytes: {repr(decoded[:100])}")
                except Exception as e:
                    if self.debug:
                        print(f"[DEBUG] Error reading: {e}")
                
                # Try to parse complete JSON lines
                lines = buffer.split("\n")
                buffer = lines[-1]  # Keep incomplete line in buffer
                
                for line in lines[:-1]:
                    line = line.strip()
                    if line and line.startswith("{"):
                        try:
                            parsed = json.loads(line)
                            if self.debug:
                                print(f"[DEBUG] Parsed JSON: {parsed}")
                                print(f"[DEBUG] Type T: {parsed.get('T')}, Expected: 1002")
                            # Return the first valid JSON we find
                            return parsed
                        except json.JSONDecodeError as e:
                            if self.debug:
                                print(f"[DEBUG] Failed to parse JSON: {line[:100]}")
                                print(f"[DEBUG] Error: {e}")
                            continue
            
            time.sleep(0.02)
        
        if self.debug and not buffer:
            print("[DEBUG] Timeout - no data received")
        elif self.debug:
            print(f"[DEBUG] Timeout - buffer contains: {repr(buffer[:100])}")
        
        return {}
    
    def update_data(self, response):
        """Update GUI with IMU data."""
        if self.debug:
            print(f"[DEBUG] update_data called with: {response}")
            print(f"[DEBUG] Response T value: {response.get('T')}, Expected: 1002")
        
        if response.get("T") != 1002:
            if self.debug:
                print(f"[DEBUG] Skipping update - wrong message type")
            return
        
        # Update Euler angles
        self.data_vars['roll'].set(f"{response.get('r', 0.0):.2f}")
        self.data_vars['pitch'].set(f"{response.get('p', 0.0):.2f}")
        self.data_vars['yaw'].set(f"{response.get('y', 0.0):.2f}")
        
        # Update accelerometer
        self.data_vars['ax'].set(f"{response.get('ax', 0.0):.3f}")
        self.data_vars['ay'].set(f"{response.get('ay', 0.0):.3f}")
        self.data_vars['az'].set(f"{response.get('az', 0.0):.3f}")
        
        # Update gyroscope
        self.data_vars['gx'].set(f"{response.get('gx', 0.0):.3f}")
        self.data_vars['gy'].set(f"{response.get('gy', 0.0):.3f}")
        self.data_vars['gz'].set(f"{response.get('gz', 0.0):.3f}")
        
        # Update magnetometer
        self.data_vars['mx'].set(str(response.get('mx', 0)))
        self.data_vars['my'].set(str(response.get('my', 0)))
        self.data_vars['mz'].set(str(response.get('mz', 0)))
        
        # Update temperature
        temp = response.get('temp', 0.0)
        if temp != 0.0:
            self.data_vars['temp'].set(f"{temp:.1f}")
        else:
            self.data_vars['temp'].set("N/A")
        
        # Update timestamp
        self.data_vars['timestamp'].set(datetime.now().strftime("%H:%M:%S"))
        
        # Calculate and update compass heading
        mx = response.get('mx', 0)
        my = response.get('my', 0)
        mz = response.get('mz', 0)
        roll = response.get('r', 0.0)
        pitch = response.get('p', 0.0)
        yaw = response.get('y', 0.0)
        
        # Use yaw from IMU (already includes magnetometer) or calculate from raw magnetometer
        if abs(yaw) > 0.01:  # If yaw is available and non-zero
            heading = yaw
            if heading < 0:
                heading += 360
        else:
            # Calculate from raw magnetometer data
            heading = self.calculate_heading(mx, my, mz, roll, pitch)
        
        # Update heading display
        self.data_vars['heading'].set(f"{heading:.1f}°")
        self.data_vars['direction'].set(self.get_direction_name(heading))
        
        # Update compass visualization
        self.draw_compass(heading)
    
    def update_loop(self):
        """Background thread to continuously update IMU data."""
        update_interval = 0.2  # 5 Hz
        
        while self.is_running and self.is_connected:
            start_time = time.time()
            
            # Clear any pending data first
            if self.ser and self.ser.in_waiting > 0:
                pending = self.ser.in_waiting
                self.ser.reset_input_buffer()
                if self.debug:
                    print(f"[DEBUG] Cleared {pending} bytes from input buffer")
                time.sleep(0.05)
            
            self.send_command({"T": 126})
            time.sleep(0.2)  # Give more time for response
            
            # Try to read IMU data response
            response = self.read_response(timeout=0.8)
            
            # Keep reading until we get IMU data or timeout
            attempts = 0
            max_attempts = 5
            while attempts < max_attempts and self.is_running and self.is_connected:
                if response and response.get("T") == 1002:
                    if self.debug:
                        print(f"[DEBUG] Got IMU data on attempt {attempts + 1}")
                    # Update GUI in main thread
                    self.root.after(0, lambda r=response: self.update_data(r))
                    break
                elif response:
                    if self.debug:
                        print(f"[DEBUG] Got response but not IMU data (T={response.get('T')}), attempt {attempts + 1}/{max_attempts}")
                    # Got a response but not IMU data, try reading more
                    time.sleep(0.15)
                    response = self.read_response(timeout=0.4)
                    attempts += 1
                else:
                    if self.debug:
                        print(f"[DEBUG] No response received, attempt {attempts + 1}/{max_attempts}")
                    if attempts < max_attempts - 1:
                        time.sleep(0.1)
                        response = self.read_response(timeout=0.4)
                    attempts += 1
                    if attempts >= max_attempts:
                        if self.debug:
                            print("[DEBUG] Max attempts reached, no IMU data")
                        break
            
            # Maintain update rate
            elapsed = time.time() - start_time
            sleep_time = max(0, update_interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def on_closing(self):
        """Handle window closing."""
        self.disconnect()
        self.root.destroy()


def main():
    root = Tk()
    app = IMUMonitorGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
