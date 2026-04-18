import serial
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import zoom

PORT = 'COM3'
BAUD = 115200

ser = serial.Serial(PORT, BAUD)

plt.ion()
fig, ax = plt.subplots()

# --- CREATE IMAGE + COLORBAR ONCE ---
frame = np.zeros((24, 32))
frame = zoom(frame, 10)

im = ax.imshow(frame, cmap='inferno', vmin=20, vmax=40)
cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

ax.set_title("MLX90640 Thermal View")
ax.axis('off')


while True:
    line = ser.readline().decode(errors='ignore').strip()
    data = line.split(',')

    if len(data) != 768:
        continue

    try:
        temps = np.array(data, dtype=float)
    except:
        continue

    frame = temps.reshape((24, 32))

    # Smooth + upscale
    frame = zoom(frame, 10)

    # Update the image data
    im.set_data(frame)

    plt.pause(0.01)