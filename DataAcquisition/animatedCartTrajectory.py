import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

print("Circular path for cartesian data recording")


r = 300 # millimeter
center = (0, 0)

t = 12 # total time for one circle path in second
f = 1/t

omega = 2*np.pi*f

tvec = np.arange(0, t, 0.05)
xpoints = np.sin(omega*tvec)
ypoints = np.cos(omega*tvec)
ypoints = ypoints - 1
fig, ax = plt.subplots(figsize=(5, 5))
plt.axis('equal')
plt.xlim(-2, 2)
plt.ylim(-2, 2)
plt.xlabel('x-axis')
plt.ylabel('y-axis')

line, = ax.plot([], [], 'o', color='r')

def animate(i):
    line.set_data(xpoints[:i], ypoints[:i])
    return line,

ani = FuncAnimation(fig, animate, frames=len(tvec), interval=50, blit=True)
plt.show()