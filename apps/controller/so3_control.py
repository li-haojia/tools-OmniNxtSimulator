import numpy as np
import math

class SO3Control:
    def __init__(self, mass=0.5, gravity=9.81):
        """
        Initialize the SO3Control controller with default mass and gravitational acceleration.

        :param mass: Object mass (kg), default is 0.5
        :param gravity: Gravitational acceleration (m/s²), default is 9.81
        """
        self.mass = mass
        self.gravity = gravity
        self.position = np.zeros(3)
        self.quaternion = np.array([0.0, 0.0, 0.0, 1.0]) # x, y, z, w
        self.velocity = np.zeros(3)
        self.acceleration = np.zeros(3)
        self.throttle = 0.0
        self.angular_velocity = np.zeros(3)

    def set_mass(self, mass):
        """Set the object's mass."""
        if mass <= 0:
            raise ValueError("Mass must be positive.")
        self.mass = mass

    def set_gravity(self, gravity):
        """Set gravitational acceleration."""
        self.gravity = gravity

    def set_position(self, position):
        """Set current position."""
        self._validate_vector(position, 'position')
        self.position = np.array(position, dtype=float)

    def set_quaternion(self, x, y, z, w):
        """Set current quaternion."""
        self.quaternion = np.array([x, y, z, w], dtype=float)

    def set_velocity(self, velocity):
        """Set current velocity."""
        self._validate_vector(velocity, 'velocity')
        self.velocity = np.array(velocity, dtype=float)

    def set_angular_velocity(self, angular_velocity):
        """Set current angular velocity."""
        self._validate_vector(angular_velocity, 'angular_velocity')
        self.angular_velocity = np.array(angular_velocity, dtype=float)

    def set_acceleration(self, acceleration):
        """Set current acceleration."""
        self._validate_vector(acceleration, 'acceleration')
        self.acceleration = np.array(acceleration, dtype=float)

    def calculate_control(self, des_pos, des_vel, des_yaw, kx, kv):
        """
        Calculate throttle and angular velocity based on desired state and gains.

        :param des_pos: Desired position (3,)
        :param des_vel: Desired velocity (3,)
        :param des_acc: Desired acceleration (3,)
        :param des_yaw: Desired yaw angle (radians)
        :param des_yaw_dot: Desired yaw rate (not used in current implementation)
        :param kx: Position control gain (3,)
        :param kv: Velocity control gain (3,)
        """
        # Validate inputs
        self._validate_vector(des_pos, 'des_pos')
        self._validate_vector(des_vel, 'des_vel')
        self._validate_vector(kx, 'kx')
        self._validate_vector(kv, 'kv')

        # Determine usage of position, velocity, and acceleration
        flag_use_pos = not np.isnan(des_pos).any()
        flag_use_vel = not np.isnan(des_vel).any()

        # Calculate total error
        total_error = np.zeros(3)
        if flag_use_pos:
            total_error += des_pos - self.position
        if flag_use_vel:
            total_error += des_vel - self.velocity

        # Calculate acceleration gain (ka)
        ka = np.where(np.abs(total_error) > 3, 0, np.abs(total_error) * 0.2)

        # Calculate required total force
        total_force = self.mass * self.gravity * np.array([0, 0, 1])
        if flag_use_pos:
            total_force += np.diag(kx) @ (des_pos - self.position)
        if flag_use_vel:
            total_force += np.diag(kv) @ (des_vel - self.velocity)

        # Calculate throttle (assume throttle relates to z-axis component of total force)
        self.throttle = total_force[2]

        # Clamp throttle to ensure reasonable range (adjustable as needed)
        self.throttle = np.clip(self.throttle, 0, self.mass * self.gravity * 2)

        # Calculate desired attitude to achieve desired direction
        theta = math.pi / 4  # Control angle limited to 45 degrees
        c = math.cos(theta)
        f = total_force - self.mass * self.gravity * np.array([0, 0, 1])

        force_norm = np.linalg.norm(total_force)
        if force_norm > 1e-6:
            force_direction = total_force / force_norm
            if force_direction[2] < c:
                nf = np.linalg.norm(f)
                A = (c**2) * (nf**2) - (f[2]**2)
                B = 2 * (c**2 - 1) * f[2] * self.mass * self.gravity
                C = (c**2 - 1) * (self.mass * self.gravity)**2
                discriminant = B**2 - 4 * A * C
                if discriminant >= 0:
                    s = (-B + math.sqrt(discriminant)) / (2 * A)
                    total_force = s * f + self.mass * self.gravity * np.array([0, 0, 1])
                else:
                    # Reset to default force in case of numerical issues
                    total_force = self.mass * self.gravity * np.array([0, 0, 1])

        # Calculate desired attitude matrix
        b1d = np.array([math.cos(des_yaw), math.sin(des_yaw), 0])

        if np.linalg.norm(total_force) > 1e-6:
            b3c = total_force / np.linalg.norm(total_force)
        else:
            b3c = np.array([0, 0, 1])

        b2c = np.cross(b3c, b1d)
        norm_b2c = np.linalg.norm(b2c)
        if norm_b2c < 1e-6:
            # Avoid zero vector by selecting an arbitrary perpendicular vector
            b2c = np.array([0, 1, 0])
        else:
            b2c /= norm_b2c

        b1c = np.cross(b2c, b3c)

        # Construct rotation matrix from body frame to world frame
        R_matrix = np.column_stack((b1c, b2c, b3c))

        # Convert rotation matrix to quaternion
        desired_quaternion = self._rotation_matrix_to_quaternion(R_matrix)
        desired_quaternion = self._normalize_quaternion(desired_quaternion)
        # desired_quaternion = self._identity_quaternion()

        # Calculate current quaternion
        current_quaternion = self.quaternion

        # Calculate quaternion error
        quaternion_error = self._quaternion_multiply(desired_quaternion, self._quaternion_conjugate(current_quaternion))

        # Ensure quaternion_error takes the shortest path
        if quaternion_error[3] < 0:
            quaternion_error = -quaternion_error

        # Convert quaternion error to rotation vector
        rotation_vector = quaternion_error[:3] * 2.0

        # Set angular velocity as proportional to rotation vector (simple PD control, adjustable as needed)
        kp = 10.0  # Proportional gain
        kd = kp * 0.05  # Derivative gain (if angular velocity feedback available)

        self.angular_velocity = kp * rotation_vector - kd * (self.angular_velocity)

    def get_computed_throttle(self):
        """Return the computed throttle value."""
        return self.throttle

    def get_computed_angular_velocity(self):
        """Return the computed angular velocity (x, y, z)."""
        return self.angular_velocity.copy()

    @staticmethod
    def _validate_vector(vector, name):
        """Validate that the input is a 3D vector."""
        if not isinstance(vector, (list, tuple, np.ndarray)):
            raise TypeError(f"{name} must be a list, tuple, or numpy array.")
        if len(vector) != 3:
            raise ValueError(f"{name} must be a 3-dimensional vector.")
        if not all(isinstance(x, (int, float, np.floating, np.integer)) for x in vector):
            raise TypeError(f"All elements in {name} must be numeric.")

    @staticmethod
    def _identity_quaternion():
        """Return the identity quaternion."""
        return np.array([0.0, 0.0, 0.0, 1.0])

    @staticmethod
    def _quaternion_multiply(q1, q2):
        """Quaternion multiplication."""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        return np.array([x, y, z, w])

    @staticmethod
    def _quaternion_conjugate(q):
        """Compute the quaternion conjugate."""
        return np.array([-q[0], -q[1], -q[2], q[3]])

    @staticmethod
    def _rotation_matrix_to_quaternion(R):
        """
        Convert rotation matrix to quaternion.

        :param R: 3x3 rotation matrix
        :return: Quaternion in the format [x, y, z, w]
        """
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2,1] - R[1,2]) * s
            y = (R[0,2] - R[2,0]) * s
            z = (R[1,0] - R[0,1]) * s
        else:
            if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
                s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
                w = (R[2,1] - R[1,2]) / s
                x = 0.25 * s
                y = (R[0,1] + R[1,0]) / s
                z = (R[0,2] + R[2,0]) / s
            elif R[1,1] > R[2,2]:
                s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
                w = (R[0,2] - R[2,0]) / s
                x = (R[0,1] + R[1,0]) / s
                y = 0.25 * s
                z = (R[1,2] + R[2,1]) / s
            else:
                s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
                w = (R[1,0] - R[0,1]) / s
                x = (R[0,2] + R[2,0]) / s
                y = (R[1,2] + R[2,1]) / s
                z = 0.25 * s
        quaternion = np.array([x, y, z, w])
        return SO3Control._normalize_quaternion(quaternion)

    @staticmethod
    def _normalize_quaternion(q):
        """Normalize the quaternion."""
        if q[3] == 0:
            q[3] = 1.0
        norm = np.linalg.norm(q)
        if norm == 0:
            raise ValueError("Cannot normalize a zero quaternion.")
        return q / norm