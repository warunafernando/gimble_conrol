#!/usr/bin/env python3
"""
Magnetometer & Gyroscope Data Monitor GUI

A graphical interface to read and display magnetometer and gyroscope data
from the ESP32 via USB serial port.

Usage:
    python mag_gyro_monitor_gui.py
"""

import json
import sys
import threading
import time
import math
from datetime import datetime
from tkinter import Tk, StringVar, Label, Frame, Button, Entry, messagebox, LabelFrame, Canvas
import serial


class MagGyroMonitorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Magnetometer & Gyroscope Monitor - ESP32")
        self.root.geometry("1000x800")
        
        # Serial connection
        self.ser = None
        self.is_connected = False
        self.is_running = False
        self.update_thread = None
        
        # Data variables
        self.data_vars = {
            'mx': StringVar(value="0"),
            'my': StringVar(value="0"),
            'mz': StringVar(value="0"),
            'gx': StringVar(value="0.000"),
            'gy': StringVar(value="0.000"),
            'gz': StringVar(value="0.000"),
            'mag_magnitude': StringVar(value="0.0"),
            'gyro_magnitude': StringVar(value="0.000"),
            'heading': StringVar(value="0.0°"),
            'direction': StringVar(value="N"),
            'status': StringVar(value="Disconnected"),
            'timestamp': StringVar(value="--:--:--")
        }
        
        # Data history for plotting (last 100 points)
        self.mag_history = {'x': [], 'y': [], 'z': [], 'time': []}
        self.gyro_history = {'x': [], 'y': [], 'z': [], 'time': []}
        self.max_history = 100
        
        # Magnetic declination
        self.magnetic_declination = 0.0  # degrees
        
        self.create_widgets()
        
    def create_widgets(self):
        # Connection frame
        conn_frame = Frame(self.root, padx=10, pady=10, bg='#f0f0f0')
        conn_frame.pack(fill='x')
        
        Label(conn_frame, text="Port:", font=('Arial', 10), bg='#f0f0f0').grid(row=0, column=0, padx=5)
        self.port_entry = Entry(conn_frame, width=10)
        self.port_entry.insert(0, "COM6")
        self.port_entry.grid(row=0, column=1, padx=5)
        
        Label(conn_frame, text="Baud:", font=('Arial', 10), bg='#f0f0f0').grid(row=0, column=2, padx=5)
        self.baud_entry = Entry(conn_frame, width=10)
        self.baud_entry.insert(0, "115200")
        self.baud_entry.grid(row=0, column=3, padx=5)
        
        self.connect_btn = Button(conn_frame, text="Connect", command=self.toggle_connection, 
                                  bg='#4CAF50', fg='white', font=('Arial', 10, 'bold'), width=10)
        self.connect_btn.grid(row=0, column=4, padx=10)
        
        # Status label
        status_frame = Frame(self.root, padx=10, pady=5, bg='#f0f0f0')
        status_frame.pack(fill='x')
        Label(status_frame, text="Status:", font=('Arial', 10, 'bold'), bg='#f0f0f0').pack(side='left', padx=5)
        self.status_label = Label(status_frame, textvariable=self.data_vars['status'], 
                                  font=('Arial', 10), fg='red', bg='#f0f0f0')
        self.status_label.pack(side='left', padx=5)
        
        # Main content frame
        main_frame = Frame(self.root, padx=10, pady=10)
        main_frame.pack(fill='both', expand=True)
        
        # Left side - Data display
        left_frame = Frame(main_frame)
        left_frame.pack(side='left', fill='both', expand=True, padx=5)
        
        # Magnetometer section
        mag_frame = LabelFrame(left_frame, text="Magnetometer (uT)", 
                              font=('Arial', 14, 'bold'), padx=15, pady=15)
        mag_frame.pack(fill='x', pady=5)
        
        self.create_data_row(mag_frame, "X:", self.data_vars['mx'], "uT", 0)
        self.create_data_row(mag_frame, "Y:", self.data_vars['my'], "uT", 1)
        self.create_data_row(mag_frame, "Z:", self.data_vars['mz'], "uT", 2)
        self.create_data_row(mag_frame, "Magnitude:", self.data_vars['mag_magnitude'], "uT", 3)
        
        # Gyroscope section
        gyro_frame = LabelFrame(left_frame, text="Gyroscope (rad/s)", 
                               font=('Arial', 14, 'bold'), padx=15, pady=15)
        gyro_frame.pack(fill='x', pady=5)
        
        self.create_data_row(gyro_frame, "X:", self.data_vars['gx'], "rad/s", 0)
        self.create_data_row(gyro_frame, "Y:", self.data_vars['gy'], "rad/s", 1)
        self.create_data_row(gyro_frame, "Z:", self.data_vars['gz'], "rad/s", 2)
        self.create_data_row(gyro_frame, "Magnitude:", self.data_vars['gyro_magnitude'], "rad/s", 3)
        
        # Compass section
        compass_frame = LabelFrame(left_frame, text="Compass / Heading", 
                                  font=('Arial', 14, 'bold'), padx=15, pady=15)
        compass_frame.pack(fill='x', pady=5)
        
        compass_display_frame = Frame(compass_frame)
        compass_display_frame.pack(pady=10)
        
        # Compass canvas
        self.compass_canvas = Canvas(compass_display_frame, width=180, height=180, 
                                    bg='white', relief='sunken', bd=2)
        self.compass_canvas.pack(side='left', padx=10)
        
        # Heading info
        heading_info_frame = Frame(compass_display_frame)
        heading_info_frame.pack(side='left', padx=20)
        
        Label(heading_info_frame, text="Heading:", font=('Arial', 12, 'bold')).pack(anchor='w', pady=5)
        heading_value_label = Label(heading_info_frame, textvariable=self.data_vars['heading'], 
                                   font=('Arial', 18, 'bold'), fg='blue')
        heading_value_label.pack(anchor='w', pady=5)
        
        Label(heading_info_frame, text="Direction:", font=('Arial', 12, 'bold')).pack(anchor='w', pady=5)
        direction_label = Label(heading_info_frame, textvariable=self.data_vars['direction'], 
                               font=('Arial', 24, 'bold'), fg='red', width=5)
        direction_label.pack(anchor='w', pady=5)
        
        # Timestamp
        timestamp_frame = Frame(left_frame)
        timestamp_frame.pack(fill='x', pady=5)
        Label(timestamp_frame, text="Last Update:", font=('Arial', 9)).pack(side='left', padx=5)
        Label(timestamp_frame, textvariable=self.data_vars['timestamp'], 
              font=('Arial', 9), fg='gray').pack(side='left', padx=5)
        
        # Right side - Data plots
        right_frame = Frame(main_frame)
        right_frame.pack(side='right', fill='both', expand=True, padx=5)
        
        # Magnetometer plot
        mag_plot_frame = LabelFrame(right_frame, text="Magnetometer Data Plot", 
                                   font=('Arial', 12, 'bold'), padx=10, pady=10)
        mag_plot_frame.pack(fill='both', expand=True, pady=5)
        
        self.mag_plot_canvas = Canvas(mag_plot_frame, width=400, height=200, bg='white', relief='sunken', bd=1)
        self.mag_plot_canvas.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Gyroscope plot
        gyro_plot_frame = LabelFrame(right_frame, text="Gyroscope Data Plot", 
                                     font=('Arial', 12, 'bold'), padx=10, pady=10)
        gyro_plot_frame.pack(fill='both', expand=True, pady=5)
        
        self.gyro_plot_canvas = Canvas(gyro_plot_frame, width=400, height=200, bg='white', relief='sunken', bd=1)
        self.gyro_plot_canvas.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Draw initial compass
        self.draw_compass(0)
        
    def create_data_row(self, parent, label_text, var, unit, row):
        row_frame = Frame(parent)
        row_frame.grid(row=row, column=0, sticky='w', pady=3)
        Label(row_frame, text=label_text, font=('Arial', 11), width=12, anchor='w').pack(side='left', padx=5)
        value_label = Label(row_frame, textvariable=var, font=('Arial', 11, 'bold'), 
                           fg='blue', width=15, anchor='e')
        value_label.pack(side='left', padx=5)
        Label(row_frame, text=unit, font=('Arial', 11)).pack(side='left', padx=2)
    
    def calculate_heading(self, mx, my, mz, roll=0, pitch=0):
        """Calculate compass heading from magnetometer data."""
        # Simple 2D heading calculation (assuming device is mostly level)
        heading = math.degrees(math.atan2(my, mx))
        
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
        
        center_x = 90
        center_y = 90
        radius = 70
        
        # Draw compass circle
        self.compass_canvas.create_oval(center_x - radius, center_y - radius,
                                        center_x + radius, center_y + radius,
                                        outline='black', width=2)
        
        # Draw cardinal directions
        directions = ['N', 'E', 'S', 'W']
        angles = [0, 90, 180, 270]
        
        for direction, angle in zip(directions, angles):
            angle_rad = math.radians(angle - heading)
            x = center_x + (radius - 12) * math.sin(angle_rad)
            y = center_y - (radius - 12) * math.cos(angle_rad)
            
            color = 'red' if direction == 'N' else 'black'
            font_size = 14 if direction == 'N' else 12
            self.compass_canvas.create_text(x, y, text=direction, 
                                           font=('Arial', font_size, 'bold'),
                                           fill=color)
        
        # Draw north pointer (red arrow)
        arrow_size = 18
        self.compass_canvas.create_polygon(
            center_x, center_y - radius + 3,
            center_x - arrow_size/2, center_y - radius + arrow_size,
            center_x, center_y - radius + arrow_size/2,
            center_x + arrow_size/2, center_y - radius + arrow_size,
            fill='red', outline='darkred', width=2
        )
        
        # Draw center dot
        self.compass_canvas.create_oval(center_x - 3, center_y - 3,
                                       center_x + 3, center_y + 3,
                                       fill='black')
    
    def plot_data(self, canvas, history_dict, colors, y_label, y_range=None):
        """Plot data history on canvas."""
        canvas.delete("all")
        
        width = canvas.winfo_width()
        height = canvas.winfo_height()
        
        if width <= 1 or height <= 1:
            return
        
        # Draw axes
        margin = 30
        plot_width = width - 2 * margin
        plot_height = height - 2 * margin
        
        # X axis
        canvas.create_line(margin, height - margin, width - margin, height - margin, width=2)
        # Y axis
        canvas.create_line(margin, margin, margin, height - margin, width=2)
        
        # Labels
        canvas.create_text(width // 2, height - 10, text="Time (samples)", font=('Arial', 8))
        canvas.create_text(15, height // 2, text=y_label, font=('Arial', 8), angle=90)
        
        if not history_dict['time']:
            return
        
        # Auto-scale Y axis if range not provided
        if y_range is None:
            all_values = []
            for key in colors.keys():
                all_values.extend(history_dict[key])
            if all_values:
                min_val = min(all_values)
                max_val = max(all_values)
                y_range = (min_val - abs(min_val) * 0.1, max_val + abs(max_val) * 0.1)
            else:
                y_range = (-1, 1)
        
        y_min, y_max = y_range
        y_span = y_max - y_min if y_max != y_min else 1
        
        # Draw zero line if applicable
        if y_min <= 0 <= y_max:
            zero_y = height - margin - (0 - y_min) / y_span * plot_height
            canvas.create_line(margin, zero_y, width - margin, zero_y, 
                             fill='gray', dash=(2, 2), width=1)
        
        # Plot each data series
        for key, color in colors.items():
            if len(history_dict[key]) < 2:
                continue
            
            points = []
            for i, value in enumerate(history_dict[key]):
                x = margin + (i / (len(history_dict[key]) - 1)) * plot_width if len(history_dict[key]) > 1 else margin
                y = height - margin - ((value - y_min) / y_span) * plot_height
                points.append((x, y))
            
            # Draw line
            for i in range(len(points) - 1):
                canvas.create_line(points[i][0], points[i][1], 
                                 points[i+1][0], points[i+1][1],
                                 fill=color, width=2)
        
        # Draw legend
        legend_y = 15
        for i, (key, color) in enumerate(colors.items()):
            x_pos = width - 100 + (i % 2) * 50
            y_pos = legend_y + (i // 2) * 15
            canvas.create_line(x_pos, y_pos, x_pos + 20, y_pos, fill=color, width=2)
            canvas.create_text(x_pos + 25, y_pos, text=key.upper(), 
                             font=('Arial', 8), anchor='w')
    
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
            
            # Clear history
            for key in self.mag_history:
                self.mag_history[key] = []
            for key in self.gyro_history:
                self.gyro_history[key] = []
            
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
    
    def send_command(self, cmd):
        """Send JSON command to ESP32."""
        if not self.ser or not self.ser.is_open:
            return
        try:
            line = json.dumps(cmd, separators=(",", ":")) + "\n"
            self.ser.write(line.encode("utf-8"))
            self.ser.flush()
        except Exception:
            pass
    
    def read_response(self, timeout=0.5):
        """Read JSON response from ESP32."""
        if not self.ser or not self.ser.is_open:
            return {}
        
        end_time = time.time() + timeout
        buffer = ""
        
        while time.time() < end_time and self.is_running:
            if self.ser.in_waiting > 0:
                try:
                    data = self.ser.read(self.ser.in_waiting)
                    buffer += data.decode("utf-8", errors="replace")
                except Exception:
                    pass
                
                lines = buffer.split("\n")
                buffer = lines[-1]
                
                for line in lines[:-1]:
                    line = line.strip()
                    if line and line.startswith("{"):
                        try:
                            return json.loads(line)
                        except json.JSONDecodeError:
                            continue
            
            time.sleep(0.02)
        
        return {}
    
    def update_data(self, response):
        """Update GUI with sensor data."""
        if response.get("T") != 1002:
            return
        
        # Get magnetometer data
        mx = response.get('mx', 0)
        my = response.get('my', 0)
        mz = response.get('mz', 0)
        
        # Get gyroscope data
        gx = response.get('gx', 0.0)
        gy = response.get('gy', 0.0)
        gz = response.get('gz', 0.0)
        
        # Update display
        self.data_vars['mx'].set(str(mx))
        self.data_vars['my'].set(str(my))
        self.data_vars['mz'].set(str(mz))
        
        self.data_vars['gx'].set(f"{gx:.3f}")
        self.data_vars['gy'].set(f"{gy:.3f}")
        self.data_vars['gz'].set(f"{gz:.3f}")
        
        # Calculate magnitudes
        mag_magnitude = math.sqrt(mx*mx + my*my + mz*mz)
        gyro_magnitude = math.sqrt(gx*gx + gy*gy + gz*gz)
        
        self.data_vars['mag_magnitude'].set(f"{mag_magnitude:.1f}")
        self.data_vars['gyro_magnitude'].set(f"{gyro_magnitude:.3f}")
        
        # Calculate heading from magnetometer
        heading = self.calculate_heading(mx, my, mz)
        self.data_vars['heading'].set(f"{heading:.1f}°")
        self.data_vars['direction'].set(self.get_direction_name(heading))
        
        # Update compass
        self.draw_compass(heading)
        
        # Update history
        current_time = time.time()
        
        # Magnetometer history
        self.mag_history['x'].append(mx)
        self.mag_history['y'].append(my)
        self.mag_history['z'].append(mz)
        self.mag_history['time'].append(current_time)
        
        # Gyroscope history
        self.gyro_history['x'].append(gx)
        self.gyro_history['y'].append(gy)
        self.gyro_history['z'].append(gz)
        self.gyro_history['time'].append(current_time)
        
        # Limit history size
        if len(self.mag_history['time']) > self.max_history:
            for key in self.mag_history:
                self.mag_history[key] = self.mag_history[key][-self.max_history:]
        if len(self.gyro_history['time']) > self.max_history:
            for key in self.gyro_history:
                self.gyro_history[key] = self.gyro_history[key][-self.max_history:]
        
        # Update plots
        self.plot_data(self.mag_plot_canvas, self.mag_history, 
                      {'x': 'red', 'y': 'green', 'z': 'blue'}, 
                      'Magnetometer (uT)')
        self.plot_data(self.gyro_plot_canvas, self.gyro_history,
                      {'x': 'red', 'y': 'green', 'z': 'blue'},
                      'Gyroscope (rad/s)')
        
        # Update timestamp
        self.data_vars['timestamp'].set(datetime.now().strftime("%H:%M:%S"))
    
    def update_loop(self):
        """Background thread to continuously update sensor data."""
        update_interval = 0.1  # 10 Hz
        
        while self.is_running and self.is_connected:
            start_time = time.time()
            
            # Clear buffer
            if self.ser and self.ser.in_waiting > 0:
                self.ser.reset_input_buffer()
                time.sleep(0.05)
            
            self.send_command({"T": 126})
            time.sleep(0.15)
            
            response = self.read_response(timeout=0.6)
            
            attempts = 0
            max_attempts = 3
            while attempts < max_attempts and self.is_running and self.is_connected:
                if response and response.get("T") == 1002:
                    self.root.after(0, lambda r=response: self.update_data(r))
                    break
                elif response:
                    time.sleep(0.1)
                    response = self.read_response(timeout=0.3)
                    attempts += 1
                else:
                    break
            
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
    app = MagGyroMonitorGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
