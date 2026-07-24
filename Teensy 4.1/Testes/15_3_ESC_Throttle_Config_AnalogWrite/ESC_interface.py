import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports

class MotorControllerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Teensy ESC Motor Interface")
        self.root.geometry("800x600")
        
        # Variables
        self.serial_conn = None
        self.active_motor = tk.IntVar(value=0)
        self.pulse_us = tk.IntVar(value=1000)
        self.pwm_resolution = tk.IntVar(value=12)
        self.pwm_period_us = 20000  # 50Hz
        
        self.setup_ui()
        self.update_simulation()
        
        # Keyboard Bindings (Up/Down arrows)
        self.root.bind('<Up>', self.increment_pulse)
        self.root.bind('<Down>', self.decrement_pulse)

    def setup_ui(self):
        # --- Serial Connection Frame ---
        serial_frame = ttk.LabelFrame(self.root, text="Serial Connection", padding=10)
        serial_frame.pack(fill="x", padx=10, pady=5)
        
        self.port_cb = ttk.Combobox(serial_frame, values=[port.device for port in serial.tools.list_ports.comports()])
        self.port_cb.pack(side="left", padx=5)
        
        self.btn_connect = ttk.Button(serial_frame, text="Connect", command=self.toggle_connection)
        self.btn_connect.pack(side="left", padx=5)
        
        self.lbl_status = ttk.Label(serial_frame, text="Disconnected", foreground="red")
        self.lbl_status.pack(side="left", padx=10)

        # --- Controls Frame ---
        control_frame = ttk.LabelFrame(self.root, text="Motor & Signal Controls", padding=10)
        control_frame.pack(fill="x", padx=10, pady=5)
        
        # Motor Selection
        ttk.Label(control_frame, text="Active Motor:").grid(row=0, column=0, sticky="w", pady=5)
        motor_names = ["M0 (Front-Left)", "M1 (Front-Right)", "M2 (Back-Left)", "M3 (Back-Right)"]
        for i, name in enumerate(motor_names):
            ttk.Radiobutton(control_frame, text=name, variable=self.active_motor, value=i, command=self.send_motor_cmd).grid(row=0, column=i+1, padx=5)

        # Pulse Control
        ttk.Label(control_frame, text="Pulse (us):").grid(row=1, column=0, sticky="w", pady=10)
        self.entry_pulse = ttk.Entry(control_frame, textvariable=self.pulse_us, width=10)
        self.entry_pulse.grid(row=1, column=1, sticky="w")
        self.entry_pulse.bind('<Return>', lambda e: self.send_pulse_cmd())
        ttk.Label(control_frame, text="(Use Up/Down Arrows)").grid(row=1, column=2, columnspan=2, sticky="w")

        # PWM Resolution
        ttk.Label(control_frame, text="PWM Res (bits):").grid(row=2, column=0, sticky="w", pady=5)
        self.entry_res = ttk.Entry(control_frame, textvariable=self.pwm_resolution, width=10)
        self.entry_res.grid(row=2, column=1, sticky="w")
        self.entry_res.bind('<Return>', lambda e: self.update_simulation())

        # --- Simulation Frame ---
        sim_frame = ttk.LabelFrame(self.root, text="AnalogWrite PWM Simulation", padding=10)
        sim_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        self.lbl_duty = ttk.Label(sim_frame, text="Calculated Duty: ", font=('Courier', 10, 'bold'))
        self.lbl_duty.pack(anchor="w")
        
        self.canvas = tk.Canvas(sim_frame, bg="black", height=300)
        self.canvas.pack(fill="both", expand=True, pady=10, ipady=100)
        # self.canvas.

    def toggle_connection(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.btn_connect.config(text="Connect")
            self.lbl_status.config(text="Disconnected", foreground="red")
        else:
            port = self.port_cb.get()
            try:
                self.serial_conn = serial.Serial(port, 115200, timeout=1)
                self.btn_connect.config(text="Disconnect")
                self.lbl_status.config(text=f"Connected to {port}", foreground="green")
                # Send initial state
                self.send_motor_cmd()
                self.send_pulse_cmd()
            except Exception as e:
                messagebox.showerror("Connection Error", str(e))

    def send_motor_cmd(self):
        cmd = f"M {self.active_motor.get()}\n"
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.write(cmd.encode())
        self.update_simulation()

    def send_pulse_cmd(self):
        pulse = self.pulse_us.get()
        # Clamp between 1000 and 2000 for safety
        pulse = max(1000, min(2000, pulse))
        self.pulse_us.set(pulse)
        
        cmd = f"p {pulse}\n"
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.write(cmd.encode())
        self.update_simulation()

    def increment_pulse(self, event=None):
        self.pulse_us.set(self.pulse_us.get() + 5)
        self.send_pulse_cmd()

    def decrement_pulse(self, event=None):
        self.pulse_us.set(self.pulse_us.get() - 5)
        self.send_pulse_cmd()

    def update_simulation(self):
        self.canvas.delete("all")
        
        pulse = self.pulse_us.get()
        res_bits = self.pwm_resolution.get()
        max_val = (2 ** res_bits)
        
        # Applying the exact mapping from your Teensy code:
        # targetPulseUs = map(targetPulseUs, 1000, 2000, 2000, 1000)
        # duty = (max_val / PERIOD_US) * targetPulseUs
        mapped_pulse = 2000 - (pulse - 1000)  # equivalent to map(val, 1000, 2000, 2000, 1000)
        duty_val = int((max_val / self.pwm_period_us) * mapped_pulse)
        
        self.lbl_duty.config(text=f"Duty Value sent to analogWrite: {duty_val} / {max_val} (Inverted Logic)")
        
        # Drawing the wave
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()
        if width <= 1: width = 560  # default fallback
        if height <= 1: height = 200
        
        pad_x, pad_y = 20, 20
        eff_width = width - 2 * pad_x
        
        # Draw axes
        self.canvas.create_line(pad_x, height-pad_y, width-pad_x, height-pad_y, fill="gray")
        self.canvas.create_line(pad_x, pad_y, pad_x, height-pad_y, fill="gray")
        self.canvas.create_text(pad_x, height-pad_y+10, text="0ms", fill="white", anchor="n")
        self.canvas.create_text(width-pad_x, height-pad_y+10, text="20ms", fill="white", anchor="n")
        
        # Calculate pixel dimensions for HIGH and LOW states based on mapped_pulse
        high_ratio = mapped_pulse / self.pwm_period_us
        high_width = eff_width * high_ratio
        
        y_low = height - pad_y - 2
        y_high = pad_y + 20
        
        # Draw the square wave (Teensy Pin Output)
        self.canvas.create_line(pad_x, y_low, pad_x, y_high, fill="cyan", width=2) # Rise
        self.canvas.create_line(pad_x, y_high, pad_x + high_width, y_high, fill="cyan", width=2) # High State
        self.canvas.create_line(pad_x + high_width, y_high, pad_x + high_width, y_low, fill="cyan", width=2) # Fall
        self.canvas.create_line(pad_x + high_width, y_low, pad_x + eff_width, y_low, fill="cyan", width=2) # Low State
        
        self.canvas.create_text(pad_x + high_width/2, y_high - 10, text=f"{mapped_pulse}us HIGH", fill="cyan")

if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControllerApp(root)
    
    # Wait for window to render to get accurate canvas width for drawing
    root.update()
    app.update_simulation()
    
    root.mainloop()