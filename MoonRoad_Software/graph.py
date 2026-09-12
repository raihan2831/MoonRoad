import csv
import numpy as np
import matplotlib.pyplot as plt

# Lists to hold sensor channels
ax_list, ay_list, az_list = [], [], []
gx_list, gy_list, gz_list = [], [], []

with open('sensor_data.csv', 'r') as f:
    reader = csv.reader(f)
    for row in reader:
        if len(row) >= 6:
            try:
                ax_list.append(float(row[0]))
                ay_list.append(float(row[1]))
                az_list.append(float(row[2]))
                gx_list.append(float(row[3]))
                gy_list.append(float(row[4]))
                gz_list.append(float(row[5]))
            except ValueError:
                continue

# Convert to numpy arrays for easy windowed variance calculation
gx = np.array(gx_list)
gy = np.array(gy_list)
gz = np.array(gz_list)

# Side-by-side subplots
fig, (ax_plot, gyro_plot) = plt.subplots(1, 2, figsize=(15, 6), sharex=True)

# 1. Accelerometer Subplot
ax_plot.plot(ax_list, label='Accel X', alpha=0.8, color='tab:blue')
ax_plot.plot(ay_list, label='Accel Y', alpha=0.8, color='tab:orange')
ax_plot.plot(az_list, label='Accel Z', alpha=0.8, color='tab:green')
ax_plot.set_title('Accelerometer Data (Annotated States)', fontsize=13, fontweight='bold')
ax_plot.set_ylabel('Amplitude (Raw / g)', fontsize=11)
ax_plot.set_xlabel('Sample Index', fontsize=11)
ax_plot.legend(loc='upper right')
ax_plot.grid(True, linestyle='--', alpha=0.6)

# 2. Gyroscope Subplot
gyro_plot.plot(gx, label='Gyro X', alpha=0.8, color='tab:blue')
gyro_plot.plot(gy, label='Gyro Y', alpha=0.8, color='tab:orange')
gyro_plot.plot(gz, label='Gyro Z', alpha=0.8, color='tab:green')
gyro_plot.set_title('Gyroscope Data', fontsize=13, fontweight='bold')
gyro_plot.set_ylabel('Angular Velocity (°/s)', fontsize=11)
gyro_plot.set_xlabel('Sample Index', fontsize=11)
gyro_plot.legend(loc='upper right')
gyro_plot.grid(True, linestyle='--', alpha=0.6)

# --- Automated State Highlighting Logic ---
# Calculate rolling movement magnitude from gyro to classify states
window_size = 10
motion_metric = np.abs(gx) + np.abs(gy) + np.abs(gz)
is_moving = motion_metric > 15.0  # Threshold for turning/shaking

# Shade regions on both plots
in_region = False
start_idx = 0
for i in range(len(is_moving)):
    if is_moving[i] and not in_region:
        in_region = True
        start_idx = i
    elif not is_moving[i] and in_region:
        in_region = False
        # Shade movement/turning region (Orange)
        ax_plot.axvspan(start_idx, i, color='orange', alpha=0.15)
        gyro_plot.axvspan(start_idx, i, color='orange', alpha=0.15)
    elif not is_moving[i] and not in_region:
        # Check for still blocks to shade green softly
        pass

# Add clear visual indicators for regions directly on the left chart
ax_plot.axvspan(550, 650, color='green', alpha=0.12, label='Still / Resting')
ax_plot.axvspan(670, 900, color='orange', alpha=0.15, label='Turning / Orientation Shift')

plt.tight_layout()
plt.show()