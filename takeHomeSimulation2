import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from dataclasses import dataclass

#constants and parameters
@dataclass
class State:
    pos: float
    vel: float

#time parameters
deltaT = 0.1
total_time_limit = 30.0  #increase limit to ensure the car stops
frame_counter = 0

# Car and Environment Constants
AIR_DENSITY = 1.225  # kg/m^3 (standard air density)
DRAG_TERM = 1.1  #cross Sectional Area * drag coeff = 1.1
CAR_MASS = 1000.0  #Arbitrary mass for calculating drag force component
M_PER_S_ACCEL = 2.0  #acceleration (m/s^2)
M_PER_S_BRAKE = 4.0  #deceleration (m/s^2)
ACCEL_DURATION = 10.0  # acceleration phase lasts 10 secs

#global simulation state variable
s0 = State(pos=0.0, vel=0.0)

#data History lists
time_history = []
velocity_history = []
position_history = []


#dynamics function
def calculate_acceleration(current_vel, current_time):
    #determine the applied force accel (from engine or brakes)
    if current_time <= ACCEL_DURATION:
        #accelerating
        applied_accel = M_PER_S_ACCEL
    else:
        #braking
        applied_accel = -M_PER_S_BRAKE
        #when stopped, no more accel or braking.
        if current_vel <= 0:
            return 0.0

    #calc the deceleration
    #crag Deceleration = (0.5 * DRAG_TERM * AIR_DENSITY * velocity^2) / CAR_MASS
    drag_force = 0.5 * DRAG_TERM * AIR_DENSITY * current_vel ** 2
    drag_decel = drag_force / CAR_MASS

    #net accel is applied minus drag
    net_accel = applied_accel - drag_decel

    return net_accel

#simulation step function
def step(state: State, current_time) -> State:
    acceleration = calculate_acceleration(state.vel, current_time)

    #euler integration
    new_vel = state.vel + acceleration * deltaT
    new_pos = state.pos + state.vel * deltaT

    #stop condition for braking phase
    if current_time > ACCEL_DURATION and new_vel < 0:
        new_vel = 0.0

    #create and return the new state object
    sNew = State(
        pos=new_pos,
        vel=new_vel
    )
    return sNew

#Animation Function (funn)
def animate(i):
    #Could be a better method to make these global
    global s0, frame_counter, deltaT, ax_pos, ax_vel, fig, total_time_limit
    global time_history, velocity_history, position_history

    current_time = frame_counter * deltaT

    #run the step & record data only if the car is still moving or time limit is not reached
    if current_time <= total_time_limit and s0.vel >= 0:
        s0 = step(s0, current_time)

        #record data history
        time_history.append(current_time)
        velocity_history.append(s0.vel)
        position_history.append(s0.pos)

        frame_counter += 1

    #Plotting :)

    #pos plot
    ax_pos.clear()
    ax_pos.plot(time_history, position_history, color='blue', linewidth=2)  # Plots a continuous line
    ax_pos.set_title("Car Position vs. Time")
    ax_pos.set_xlabel("Time (s)")
    ax_pos.set_ylabel("Position (m)")
    ax_pos.set_xlim(0, total_time_limit)
    ax_pos.set_ylim(0, max(10, max(position_history) * 1.1 if position_history else 1000))  # Dynamic y-limit
    ax_pos.grid(True)

    #Vel plot
    ax_vel.clear()
    ax_vel.plot(time_history, velocity_history, color='red', linewidth=2)  # Plots a continuous line
    ax_vel.set_title(f"Car Velocity (Current: {s0.vel:.2f} m/s)")
    ax_vel.set_xlabel("Time (s)")
    ax_vel.set_ylabel("Velocity (m/s)")
    ax_vel.set_xlim(0, total_time_limit)
    ax_vel.set_ylim(0, max(10, max(velocity_history) * 1.1 if velocity_history else 30))  # Dynamic y-limit
    ax_vel.grid(True)

    return ax_pos
    return ax_vel


#setup and run animation
fig, (ax_pos, ax_vel) = plt.subplots(2, 1, figsize=(8, 6), dpi=100)
fig.tight_layout(pad=3.0)

#interval=50 means update every 50ms, which is 20 frames per second.
ani = animation.FuncAnimation(fig, animate, interval=50, blit=False)

plt.show()
