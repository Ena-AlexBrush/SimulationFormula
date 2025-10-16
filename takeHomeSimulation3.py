import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from dataclasses import dataclass
import numpy as np

#simulation Constants and Parameters
@dataclass
class State:
    pos: float
    vel: float

#time and Initial State
deltaT = 0.1
total_time_limit = 30.0
frame_counter = 0
s0 = State(pos=0.0, vel=0.0)

#car and Environment sonstants
CAR_MASS = 1500.0  #(kg)
MAX_POWER = 150000.0  #(Watts = 150 kW)
MAX_TORQUE = 300.0  #(Nm)
RPM_PEAK_POWER = 5000  #RPM where max power is reached
RPM_MAX_TORQUE = 2000  #RPM where max torque is available

#MM (Motor Model) Approximation:
#Torque is MAX_TORQUE until RPM_MAX_TORQUE, then decreases,
#and is limited by MAX_POWER above RPM_PEAK_POWER.

#drivetrain/conversion constants
WHEEL_RADIUS = 0.3  #(meters)
GEAR_RATIO = 10.0  #fixed effective gear ratio
EFFICIENCY = 0.9  #drivetrain efficiency ( no unit)

#drag constants (simulation2.py)
AIR_DENSITY = 1.225  #kg/m^3
DRAG_TERM = 1.1  #cross sectional area * drag coeff

#data history lists for line plotting
time_history = []
velocity_history = []
torque_history = []

#helper functions for MM
def velocity_to_rpm(vel):
    #angular vel (rad/s) = vel/WHEEL_RADIUS
    #motor angular vel (rad/s) = (vel/WHEEL_RADIUS) * GEAR_RATIO
    #RPM = motor angular vel * 60 / (2*pi)
    return (vel / WHEEL_RADIUS) * GEAR_RATIO * 9.54929659  #conversion factor (60 / 2pi)


def get_motor_torque(rpm):
    #calc the motor torque based on RPM and power limits.
    global MAX_TORQUE, RPM_MAX_TORQUE, MAX_POWER, RPM_PEAK_POWER

    if rpm < 1.0:  #prevent division by zero
        return MAX_TORQUE
        #base torque: assume constant MAX_TORQUE up to RPM_MAX_torque
    if rpm <= RPM_MAX_TORQUE:
        T_base = MAX_TORQUE
    else:
        T_base = MAX_TORQUE * (1 - (rpm - RPM_MAX_TORQUE) / (RPM_PEAK_POWER - RPM_MAX_TORQUE))
        T_base = max(0.0, T_base)  #torque can't be negative

    T_limit_by_power = (MAX_POWER * 9549) / rpm

    T_actual = min(T_base, T_limit_by_power)

    return T_actual

#Simulation step function
def step(state: State) -> State:
    #calcs the next state based on motor toque, power and drag

    current_vel = state.vel

    #calc motor force (F_drive)
    rpm = velocity_to_rpm(current_vel)
    motor_torque = get_motor_torque(rpm)

    drive_force = (motor_torque * GEAR_RATIO * EFFICIENCY) / WHEEL_RADIUS

    #calc Drag Force (F_drag)
    drag_force = 0.5 * DRAG_TERM * AIR_DENSITY * current_vel ** 2

    #calc net force and accel
    net_force = drive_force - drag_force
    acceleration = net_force / CAR_MASS

    #euler integration
    new_vel = current_vel + acceleration * deltaT
    new_pos = state.pos + current_vel * deltaT

    #vel cant go < 0
    if new_vel < 0:
        new_vel = 0.0

    #store the calculated torque for plotting
    global s0
    s0.torque = motor_torque  #temp store torque in state for history

    #create & return the new state object
    sNew = State(
        pos=new_pos,
        vel=new_vel
    )
    return sNew


#animation function
def animate(i):
    #updates the global state, records data & plots the history as a line
    global s0, frame_counter, deltaT, ax_vel, ax_torque, total_time_limit
    global time_history, velocity_history, torque_history

    current_time = frame_counter * deltaT

    #stop condition: check if car is moving or time limit reached
    if current_time <= total_time_limit and s0.vel < 0.1 and frame_counter > 0:
        #stop once the car has slowed down to near zero after a few frames
        pass
    elif current_time <= total_time_limit:
        s0 = step(s0)

        #record data history
        time_history.append(current_time)
        velocity_history.append(s0.vel)

        #calc and store the torque separetely for plotting
        rpm = velocity_to_rpm(s0.vel)
        torque_history.append(get_motor_torque(rpm))

        frame_counter += 1

    #plotting

    #vel plot
    ax_vel.clear()
    ax_vel.plot(time_history, velocity_history, color='blue', linewidth=2)
    ax_vel.set_title(f"Velocity (Current: {s0.vel:.2f} m/s)")
    ax_vel.set_xlabel("Time (s)")
    ax_vel.set_ylabel("Velocity (m/s)")
    ax_vel.set_xlim(0, total_time_limit) #limit
    ax_vel.set_ylim(0, max(5, max(velocity_history) * 1.1 if velocity_history else 35))
    ax_vel.grid(True)

    #torque Plot
    ax_torque.clear()
    ax_torque.plot(time_history, torque_history, color='red', linewidth=2)
    ax_torque.set_title(f"Motor Torque (Current: {torque_history[-1]:.1f} Nm)" if torque_history else "Motor Torque")
    ax_torque.set_xlabel("Time (s)")
    ax_torque.set_ylabel("Torque (Nm)")
    ax_torque.set_xlim(0, total_time_limit)
    ax_torque.set_ylim(0, max(100, max(torque_history) * 1.1 if torque_history else 400))
    ax_torque.grid(True)

    return ax_vel, ax_torque

#setup & run animation

#setup the matplotlib figure and 2 subplots (vel & torque)
fig, (ax_vel, ax_torque) = plt.subplots(2, 1, figsize=(8, 6), dpi=100)
fig.tight_layout(pad=3.0)

#create the animation
ani = animation.FuncAnimation(fig, animate, interval=50, blit=False) #interval=50 means update every 50ms (20 FPS)

plt.show()
