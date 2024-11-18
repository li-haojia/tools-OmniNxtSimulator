import numpy as np

class Quadrotor:
    # Configuration
    alpha0_ = 45.0 # deg
    mass_ = 1.0 # kg
    gravity_ = 9.81 # m/s^2
    prop_radius_ = 0.045 # m
    arm_length_ = 0.09 # m
    min_rpm_ = 1200.0 # rpm
    max_rpm_ = 35000.0 # rpm

    # State
    position_ = np.zeros(3)
    rotation_ = np.zeros((3, 3))
    quaternion_ = np.zeros(4) # [w, x, y, z]
    linear_velocity_ = np.zeros(3)
    angular_velocity_ = np.zeros(3)
    motor_rpms_ = np.zeros(4)
    motor_forces_ = np.zeros(4)
    motor_torques_ = np.zeros(4)

    def __init__(self, config):
        self.alpha0_ = config.get("alpha0", self.alpha0_)
        self.mass_ = config.get("mass", self.mass_)
        self.gravity_ = config.get("gravity", self.gravity_)
        self.prop_radius_ = config.get("prop_radius", self.prop_radius_)
        self.arm_length_ = config.get("arm_length", self.arm_length_)
        self.min_rpm_ = config.get("min_rpm", self.min_rpm_)
        self.max_rpm_ = config.get("max_rpm", self.max_rpm_)

    def quaternion_to_rotation(self, quaternion):
        q0, q1, q2, q3 = quaternion
        r00 = 1 - 2 * (q2**2 + q3**2)
        r01 = 2 * (q1*q2 - q0*q3)
        r02 = 2 * (q0*q2 + q1*q3)
        r10 = 2 * (q1*q2 + q0*q3)
        r11 = 1 - 2 * (q1**2 + q3**2)
        r12 = 2 * (q2*q3 - q0*q1)
        r20 = 2 * (q1*q3 - q0*q2)
        r21 = 2 * (q0*q1 + q2*q3)
        r22 = 1 - 2 * (q1**2 + q2**2)
        return np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])

    def update_state(self, position, quaternion, linear_velocity, angular_velocity):
        self.position_ = position
        self.quaternion_ = quaternion
        self.rotation_ = self.quaternion_to_rotation(quaternion)
        self.linear_velocity_ = linear_velocity
        self.angular_velocity_ = angular_velocity