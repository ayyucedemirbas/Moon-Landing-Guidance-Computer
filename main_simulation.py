import numpy as np
from celestial_body import CelestialBody
from spacecraft import Spacecraft
from guidance import solve_lambert_problem, plan_trajectory, update_navigation_data
from poliastro.bodies import Earth, Moon
from poliastro.twobody import Orbit
from astropy import units as u
from astropy.coordinates import CartesianRepresentation, get_body_barycentric
from astropy.time import Time

time_step = 1 
total_simulation_time = 3600 
current_time = 0

moon = CelestialBody(name="Moon", mass=7.342e22, radius=1737.4e3, position=[0, 0, 0])

spacecraft = Spacecraft(position=[384400000, 0, 0], velocity=[0, -1, 0], mass=5000) 

moon_position = get_body_barycentric('moon', Time.now()).xyz.to(u.m)


start_orbit = Orbit.circular(Earth, alt=300 * u.km)

end_orbit = Orbit.from_vectors(Moon, r=moon_position, v=start_orbit.v.to(u.km / u.s))
time_of_flight = 72 * u.hour
maneuver = solve_lambert_problem(start_orbit, end_orbit, time_of_flight)

spacecraft.apply_thrust(maneuver.impulses[0][1].to(u.m / u.s).value)


while current_time < total_simulation_time:
    spacecraft.update_state(moon, time_step)
    if current_time % 300 == 0: 
        target_position = [moon.position[0], moon.position[1], moon.position[2] + moon.radius + 10000]  # 10 km above the moon's surface
        delta_v = plan_trajectory(spacecraft.position, target_position, spacecraft.velocity, 600)
        spacecraft.apply_directed_thrust(delta_v, delta_v)

    sensor_data = {'position': spacecraft.position, 'velocity': spacecraft.velocity}
    navigation_state = update_navigation_data(sensor_data, {'position': spacecraft.position, 'velocity': spacecraft.velocity})

    print(f"Time: {current_time}s, Spacecraft Position: {spacecraft.position}, Velocity: {spacecraft.velocity}")
    current_time += time_step
