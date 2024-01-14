from poliastro import iod
from astropy import units as u
from poliastro.bodies import Earth, Moon
from poliastro.twobody import Orbit
from poliastro.maneuver import Maneuver
import numpy as np

def solve_lambert_problem(start_orbit, end_orbit, time_of_flight):
    solutions = iod.lambert(Earth.k, start_orbit.r, end_orbit.r, time_of_flight)
    v0, vf = solutions
    man = Maneuver.impulse(v0.to(u.km / u.s) - start_orbit.v.to(u.km / u.s))
    return man


def plan_trajectory(current_position, target_position, current_velocity, total_time):
    direction = np.array(target_position) - np.array(current_position)
    distance = np.linalg.norm(direction)
    direction = direction / distance  

    required_velocity = direction * (distance / total_time)
    delta_v = required_velocity - np.array(current_velocity)

    return delta_v 

def update_navigation_data(sensor_data, current_state):
    updated_position = sensor_data.get('position', current_state['position'])
    updated_velocity = sensor_data.get('velocity', current_state['velocity'])

    return {'position': updated_position, 'velocity': updated_velocity}
