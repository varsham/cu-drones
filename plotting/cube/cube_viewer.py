# Draws a hollow cube that rotates with the board, using the "roll,pitch,yaw" lines cube.ino sends over usb.
#
# setup:  pip install pyserial matplotlib
# run:    python cube_viewer.py /dev/cu.usbmodem1101   (your port, find it in the arduino ide under Tools > Port)
#         python cube_viewer.py sim                     (no hardware: fakes the angles so you can see it work)
#
# close the arduino serial monitor first, only one program can hold the port at a time.

import sys
import math
import time
import matplotlib.pyplot as plt

port = sys.argv[1] if len(sys.argv) > 1 else "sim"

if port != "sim":
    import serial
    ser = serial.serial_for_url(port, 115200, timeout=1)  # serial_for_url also takes urls like rfc2217://localhost:4000 (for a simulator)

# the points we rotate. the first 8 are the cube's corners (every combo of -1 and +1),
# then the tips of two sticks so you can tell which way the board faces: red out the front (+x), blue out the top (+z)
points = [
    (-1, -1, -1),  # 0
    (-1, -1,  1),  # 1
    (-1,  1, -1),  # 2
    (-1,  1,  1),  # 3
    ( 1, -1, -1),  # 4
    ( 1, -1,  1),  # 5
    ( 1,  1, -1),  # 6
    ( 1,  1,  1),  # 7
    (1.8, 0,  0),  # 8: front stick tip (red)
    (0,  0, 1.8),  # 9: top stick tip (blue)
]

# the 12 edges of the cube, as pairs of corner numbers from above
edges = [
    (0, 1), (2, 3), (4, 5), (6, 7),  # the 4 edges running along z
    (0, 2), (1, 3), (4, 6), (5, 7),  # the 4 edges running along y
    (0, 4), (1, 5), (2, 6), (3, 7),  # the 4 edges running along x
]

# set up the 3d plot
fig = plt.figure()
ax = fig.add_subplot(projection="3d")
ax.set_xlim(-2, 2); ax.set_ylim(-2, 2); ax.set_zlim(-2, 2)
ax.set_box_aspect((1, 1, 1))  # equal scale on all axes, otherwise the cube looks squashed
ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
edge_lines = [ax.plot([0, 0], [0, 0], [0, 0], color="black")[0] for _ in edges]  # one line per edge, moved every frame
front_line = ax.plot([0, 0], [0, 0], [0, 0], color="red", linewidth=3)[0]
top_line = ax.plot([0, 0], [0, 0], [0, 0], color="blue", linewidth=3)[0]
title = ax.set_title("")

start = time.time()

while plt.fignum_exists(fig.number):  # keep going until the window is closed

    # get the angles (degrees)
    if port == "sim":
        t = time.time() - start
        roll = 30 * math.sin(t)          # rock side to side
        pitch = 20 * math.sin(0.7 * t)   # nod up and down, a bit slower
        yaw = (40 * t) % 360             # turn steadily
    else:
        text = ser.readline()            # wait for one line
        while ser.in_waiting:            # the board sends faster than we draw, so skip to the newest line or the cube lags behind
            text = ser.readline()
        parts = text.decode(errors="ignore").strip().split(",")
        if len(parts) != 3:
            continue                     # not an angle line (e.g. "calibrating..."), skip it
        try:
            roll, pitch, yaw = float(parts[0]), float(parts[1]), float(parts[2])
        except ValueError:
            continue                     # half a line or garbage, skip it

    # sin and cos of each angle. trig wants radians, the board sends degrees
    cr, sr = math.cos(math.radians(roll)), math.sin(math.radians(roll))
    cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))

    # rotate every point: first roll around x, then pitch around y, then yaw around z.
    # each step is the 2d rotation formula (a*cos - b*sin, a*sin + b*cos) applied to the two coordinates
    # that change when you spin around that axis. the third coordinate stays the same
    moved = []
    for (x, y, z) in points:
        # roll: spin around x, so y and z change
        y1 = y * cr - z * sr
        z1 = y * sr + z * cr
        x1 = x
        # pitch: spin around y, so z and x change
        x2 = x1 * cp + z1 * sp
        z2 = -x1 * sp + z1 * cp
        y2 = y1
        # yaw: spin around z, so x and y change
        x3 = x2 * cy - y2 * sy
        y3 = x2 * sy + y2 * cy
        z3 = z2
        moved.append((x3, y3, z3))

    # move each edge line to its two rotated corners
    for k in range(len(edges)):
        a, b = edges[k]
        edge_lines[k].set_data_3d([moved[a][0], moved[b][0]], [moved[a][1], moved[b][1]], [moved[a][2], moved[b][2]])

    # the two sticks go from the center of the cube to their rotated tips
    front_line.set_data_3d([0, moved[8][0]], [0, moved[8][1]], [0, moved[8][2]])
    top_line.set_data_3d([0, moved[9][0]], [0, moved[9][1]], [0, moved[9][2]])

    title.set_text(f"roll {roll:6.1f}   pitch {pitch:6.1f}   yaw {yaw:6.1f}")
    plt.pause(0.001)  # redraw the window
