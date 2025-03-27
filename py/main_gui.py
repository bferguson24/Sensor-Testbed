
import tkinter as tk
from scoop_control import Scoop
# Create the main window
root = tk.Tk()
root.title("Robot Control Panel")

scoop = Scoop()



angle_var = tk.StringVar()
vibration_var = tk.StringVar()

def update_scoop_values(*args):
    try:
        angle = float(angle_var.get())
    except ValueError:
        angle = 0.0

    try:
        vibration = float(vibration_var.get())
    except ValueError:
        vibration = 0.0

    scoop.update_values(angle, vibration)
    print(f"Updated Scoop values: angle={angle}, max vibration={vibration}")


def run_gui():

    angle_var.trace_add("write", update_scoop_values)
    vibration_var.trace_add("write", update_scoop_values)

    # Create buttons
    home_button = tk.Button(root, text="Home", command=scoop.home)
    home_button.grid(row=0, column=0, padx=10, pady=10)

    start_button = tk.Button(root, text="Start", command=scoop.start)
    start_button.grid(row=0, column=1, padx=10, pady=10)

    stop_button = tk.Button(root, text="Stop", command=scoop.stop)
    stop_button.grid(row=0, column=2, padx=10, pady=10)

    run_dig_button = tk.Button(root, text="Run Dig", command=scoop.dig_sequence)
    run_dig_button.grid(row=0, column=3, padx=10, pady=10)

    # Create labels and entry widgets for angle and max vibration
    angle_label = tk.Label(root, text="Angle:")
    angle_label.grid(row=1, column=0, padx=10, pady=10)

    angle_entry = tk.Entry(root, textvariable=angle_var)
    angle_entry.grid(row=1, column=1, padx=10, pady=10)

    vibration_label = tk.Label(root, text="Max Vibration:")
    vibration_label.grid(row=2, column=0, padx=10, pady=10)

    vibration_entry = tk.Entry(root, textvariable=vibration_var)
    vibration_entry.grid(row=2, column=1, padx=10, pady=10)

    # # Create the submit button
    # submit_button = tk.Button(root, text="Submit", command=lambda: submit_action(angle_entry.get(), vibration_entry.get()))
    # submit_button.grid(row=3, column=0, columnspan=4, pady=10)

    # Run the Tkinter event loop
    root.mainloop()

    
if __name__ == "__main__":
    run_gui()   