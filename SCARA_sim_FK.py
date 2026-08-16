import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider

# Link lengths
a1 = 4
a2 = 4

joint1 = 0
joint2 = 0
base_z = 0

fig = plt.figure(figsize=(5, 5))
ax = fig.add_subplot(111, projection='3d')

# Sliders
ax1Slider = plt.axes([0.2, 0.07,  0.65, 0.03])
ax2Slider = plt.axes([0.2, 0.04,  0.65, 0.03])
axZSlider = plt.axes([0.2, 0.01,  0.65, 0.03])

slider1 = Slider(ax1Slider, 't1 (deg)', -180, 180, valinit=0)
slider2 = Slider(ax2Slider, 't2 (deg)', -180, 180, valinit=0)
sliderZ = Slider(axZSlider, 'base Z',   -6,   6,   valinit=0)

def forwardKinematics(t1, t2, bz):
    t1 = np.radians(t1)
    t2 = np.radians(t2)

    # Setting up transformation matrices
    T_base = np.array([[1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 1, bz],
                       [0, 0, 0, 1]])

    
    A1 = np.array([[np.cos(t1), -np.sin(t1), 0, a1 * np.cos(t1)],
                   [np.sin(t1),  np.cos(t1), 0, a1 * np.sin(t1)],
                   [0,           0,           1, 0],
                   [0,           0,           0, 1]])

    
    A2 = np.array([[np.cos(t2), -np.sin(t2), 0, a2 * np.cos(t2)],
                   [np.sin(t2),  np.cos(t2), 0, a2 * np.sin(t2)],
                   [0,           0,           1, 0],
                   [0,           0,           0, 1]])

    T1 = T_base @ A1
    T2 = T_base @ A1 @ A2

    base_origin = T_base[:3, 3]  
    joint1_pos  = T1[:3, 3]
    tip         = T2[:3, 3]

    return base_origin, joint1_pos, tip

def plotUpdate(val=0):
    global joint1, joint2, base_z
    joint1 = slider1.val
    joint2 = slider2.val
    base_z = sliderZ.val

    ax.clear()
    base_origin, joint1_pos, tip = forwardKinematics(joint1, joint2, base_z)

    
    ax.plot([0, base_origin[0]],
            [0, base_origin[1]],
            [0, base_origin[2]], 'k--', linewidth=1.5, label='Base column')

    # Draw moving base
    ax.scatter(*base_origin, color='black', s=60, zorder=5)

    # Link 1
    ax.plot([base_origin[0], joint1_pos[0]],
            [base_origin[1], joint1_pos[1]],
            [base_origin[2], joint1_pos[2]], 'b-o', linewidth=3, label='Link 1')

    # Link 2
    ax.plot([joint1_pos[0], tip[0]],
            [joint1_pos[1], tip[1]],
            [joint1_pos[2], tip[2]], 'r-o', linewidth=3, label='Link 2')

    # World axes
    ax.plot([-10, 10], [0, 0], [0, 0], color='red',   linewidth=1)
    ax.plot([0, 0], [-10, 10], [0, 0], color='blue',  linewidth=1)
    ax.plot([0, 0], [0, 0], [-10, 10], color='green', linewidth=1)

    ax.set_xlim([-10, 10])
    ax.set_ylim([-10, 10])
    ax.set_zlim([-10, 10])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend(loc='upper left', fontsize=7)
    fig.canvas.draw_idle()

slider1.on_changed(plotUpdate)
slider2.on_changed(plotUpdate)
sliderZ.on_changed(plotUpdate)

plotUpdate()
plt.show()