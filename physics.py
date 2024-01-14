import numpy as np

def rk4_update(spacecraft, celestial_body, dt):
    def acceleration_due_to_gravity(spacecraft, celestial_body):
        G = 6.67430e-11
        distance_vector = celestial_body.position - spacecraft.position
        distance = np.linalg.norm(distance_vector)
        force_magnitude = G * spacecraft.mass * celestial_body.mass / distance**2
        return (distance_vector / distance) * force_magnitude / spacecraft.mass

    k1v = acceleration_due_to_gravity(spacecraft, celestial_body) * dt
    k1p = spacecraft.velocity * dt

    spacecraft_temp = spacecraft.copy()
    spacecraft_temp.velocity += k1v / 2
    k2v = acceleration_due_to_gravity(spacecraft_temp, celestial_body) * dt
    k2p = spacecraft_temp.velocity * dt

    spacecraft_temp.velocity += k2v / 2
    k3v = acceleration_due_to_gravity(spacecraft_temp, celestial_body) * dt
    k3p = spacecraft_temp.velocity * dt

    spacecraft_temp.velocity += k3v
    k4v = acceleration_due_to_gravity(spacecraft_temp, celestial_body) * dt
    k4p = spacecraft_temp.velocity * dt

    spacecraft.velocity += (k1v + 2 * k2v + 2 * k3v + k4v) / 6
    spacecraft.position += (k1p + 2 * k2p + 2 * k3p + k4p) / 6

def calculate_altitude(position, celestial_body_radius):
    return np.linalg.norm(position) - celestial_body_radius
