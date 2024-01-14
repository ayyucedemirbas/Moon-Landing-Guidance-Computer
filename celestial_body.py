import numpy as np

class CelestialBody:
    def __init__(self, name, mass, radius, position, gravitational_parameter=None):
        self.name = name
        self.mass = mass  # Mass in kilograms
        self.radius = radius  # Radius in meters
        self.position = np.array(position) 
        self.gravitational_parameter = gravitational_parameter if gravitational_parameter else self.compute_gravitational_parameter()

    def compute_gravitational_parameter(self):
        G = 6.67430e-11  # Universal gravitational constant in m^3 kg^-1 s^-2
        return G * self.mass

    def gravitational_force_on(self, spacecraft):
        distance_vector = spacecraft.position - self.position
        distance = np.linalg.norm(distance_vector)
        force_magnitude = self.gravitational_parameter * spacecraft.mass / distance**2
        force_vector = force_magnitude * (distance_vector / distance)
        return force_vector

