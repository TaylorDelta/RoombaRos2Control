import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Grid
x = 1
y = 1
X = 1
Y = 1

# Initial parameter
theta0 = 0.0

def vector_field(theta):
    U = np.cos(theta) * X - np.sin(theta) * Y
    V = np.sin(theta) * X + np.cos(theta) * Y
    return U, V

U, V = vector_field(theta0)
Z = 1
W = 1

# Figure and axis

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
plt.subplots_adjust(bottom=0.25)

q = ax.quiver(X, Y, Z, U, V, W)
ax.set_aspect("equal")
ax.set_title("Quiver + Slider")

# Slider axis
ax_slider = plt.axes([0.2, 0.1, 0.6, 0.03])
slider = Slider(
    ax=ax_slider,
    label="Theta",
    valmin=0,
    valmax=2 * np.pi,
    valinit=theta0,
)

# Update function
def update(val):
    theta = slider.val
    U, V = vector_field(theta)
    q.set_array(X, Y, Z, U, V, W)
    fig.canvas.draw_idle()

slider.on_changed(update)

plt.show()
