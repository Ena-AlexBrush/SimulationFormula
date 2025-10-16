import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from dataclasses import dataclass
import numpy as np

#Simulation Constants and State Class
@dataclass
class State:
    xpos : float
    ypos : float
    xvel : float
    yvel : float

deltaT = 1.0
bounceCoeff = 0.999

#Simulation Step Function
def step(state: State) -> State:
    new_xpos = state.xpos + state.xvel * deltaT
    new_ypos = state.ypos + state.yvel * deltaT

    new_yvel = state.yvel - 9.81 * deltaT

    if new_ypos < 0:
        new_yvel = new_yvel * -1 * bounceCoeff

    sNew = State(
        xpos = new_xpos,
        ypos = new_ypos,
        xvel = state.xvel,
        yvel = new_yvel
    )
    return sNew

#Animation Function
def animate(i):
    global s0
    s0 = step(s0) # Update the state for the next frame

    ax.clear() # Clear the previous frame

    # Plot the particle's new position
    ax.scatter([s0.xpos], [s0.ypos], s=200) # s=20 is the marker size

    # Set plot limits
    ax.set_xlim(-2, 2)
    ax.set_ylim(0, 1000)

    return ax, # Return the axis object for the animation

#Initial State and Plot Setup
s0 = State(
    xpos = 0,
    ypos = 950,
    xvel = 0,
    yvel = 0
)

# Setup the matplotlib figure and axes
fig = plt.figure(figsize=(3,3), dpi=150)
ax = fig.add_subplot(111)

ax.grid()
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
plt.pause(5)
ani = animation.FuncAnimation(fig, animate, interval=0)
plt.show()
