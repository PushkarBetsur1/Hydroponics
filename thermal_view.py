import serial
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import zoom

PORT = 'COM3'  # <-- CHANGE THIS
BAUD = 115200

ser = serial.Serial(PORT, BAUD)

plt.ion()
fig, ax = plt.subplots()

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

    # Fix orientation if needed
    #frame = np.flipud(frame)
    #frame = np.fliplr(frame)

    # Smooth + upscale
    frame = zoom(frame, 10)

    ax.clear()

    im = ax.imshow(
        frame,
        cmap='inferno',
        vmin=20,
        vmax=40
    )

    ax.set_title("MLX90640 Thermal View")
    ax.axis('off')

    plt.pause(0.01)