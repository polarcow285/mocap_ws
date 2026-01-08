#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from scipy.spatial.transform import Rotation as R

# ----------------------------
# Load data
# ----------------------------
data = np.load("tracking_data.npz")

keypoints = data["keypoints"]        # (T, N, 3)
T, N, _ = keypoints.shape

# Detect rigid bodies automatically
rigid_bodies = {}
for k in data.files:
    if k.endswith("_pos"):
        rb_id = k[:-4]
        rigid_bodies[rb_id] = {
            "pos": data[k],
            "quat": data[f"{rb_id}_quat"]
        }

# ----------------------------
# Matplotlib setup
# ----------------------------
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")
plt.subplots_adjust(bottom=0.15)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)

ax.set_title("Tracking data viewer")

# Keypoints scatter
scatter = ax.scatter([], [], [], c="b", s=20)

# Rigid body quivers
rb_quivers = {}

axis_len = 0.1  # length of frame axes

# ----------------------------
# Update function
# ----------------------------
def update(frame):
    frame = int(frame)

    # --- keypoints ---
    kp = keypoints[frame]
    scatter._offsets3d = (kp[:, 0], kp[:, 1], kp[:, 2])

    # --- rigid bodies ---
    for rb_id, rb in rigid_bodies.items():
        pos = rb["pos"][frame]
        quat = rb["quat"][frame]

        rot = R.from_quat(quat).as_matrix()
        axes = rot * axis_len

        # Remove old quivers
        if rb_id in rb_quivers:
            for q in rb_quivers[rb_id]:
                q.remove()

        # Draw XYZ axes
        qx = ax.quiver(*pos, *axes[:, 0], color="r", linewidth=2)
        qy = ax.quiver(*pos, *axes[:, 1], color="g", linewidth=2)
        qz = ax.quiver(*pos, *axes[:, 2], color="b", linewidth=2)

        rb_quivers[rb_id] = (qx, qy, qz)

    fig.canvas.draw_idle()

# ----------------------------
# Slider
# ----------------------------
ax_slider = plt.axes([0.2, 0.05, 0.6, 0.03])
slider = Slider(ax_slider, "Frame", 0, T - 1, valinit=0, valstep=1)

slider.on_changed(update)

# ----------------------------
# Keyboard controls
# ----------------------------
def on_key(event):
    if event.key == "right":
        slider.set_val(min(slider.val + 1, T - 1))
    elif event.key == "left":
        slider.set_val(max(slider.val - 1, 0))

fig.canvas.mpl_connect("key_press_event", on_key)

# Initial draw
update(0)

plt.show()
