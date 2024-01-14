import numpy as np
from physics import rk4_update, calculate_altitude
from control_systems import PIDController

class Spacecraft:
    def __init__(self, position, velocity, mass, orientation=np.array([0, 0, 0])):
        self.position = np.array(position, dtype=float)
        self.velocity = np.array(velocity, dtype=float)  
        self.mass = mass
        self.orientation = np.array(orientation, dtype=float)
        self.pitch_control = PIDController(1.0, 0.1, 0.5, setpoint=0)
        self.yaw_control = PIDController(1.0, 0.1, 0.5, setpoint=0)
        self.roll_control = PIDController(1.0, 0.1, 0.5, setpoint=0)

    def copy(self):
        return Spacecraft(self.position.copy(), self.velocity.copy(), self.mass, self.orientation.copy())

    def apply_thrust(self, thrust):
        delta_v = thrust / self.mass
        self.velocity += delta_v

    def apply_directed_thrust(self, thrust, direction):
        direction = np.array(direction)
        direction = direction / np.linalg.norm(direction)
        delta_v = (thrust / self.mass) * direction
        self.velocity += delta_v

    def adjust_attitude(self, target_orientation, dt):
        self.pitch_control.set_setpoint(target_orientation[0])
        self.yaw_control.set_setpoint(target_orientation[1])
        self.roll_control.set_setpoint(target_orientation[2])

        pitch_adjustment = self.pitch_control.update(self.orientation[0], dt)
        yaw_adjustment = self.yaw_control.update(self.orientation[1], dt)
        roll_adjustment = self.roll_control.update(self.orientation[2], dt)

        self.orientation += np.array([pitch_adjustment, yaw_adjustment, roll_adjustment])
        self.orientation = np.mod(self.orientation, 2 * np.pi)

    def compute_altitude(self, celestial_body):
        return calculate_altitude(self.position, celestial_body.radius)

    def update_state(self, celestial_body, dt):
        rk4_update(self, celestial_body, dt)
