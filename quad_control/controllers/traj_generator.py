import numpy as np
import matplotlib.pyplot as plt

class TrajectoryGenerator:
    def __init__(self, start, end, duration, order=7):
        """
        Initialize the TrajectoryGenerator.

        :param start: List of boundary conditions at t=0 [x(0), y(0), z(0), roll(0), pitch(0), yaw(0)].
        :param end: List of boundary conditions at t=T [x(T), y(T), z(T), roll(T), pitch(T), yaw(T)].
        :param duration: Time duration of the trajectory (T).
        :param order: Polynomial order (default is 7 for a minimum snap trajectory).
        """
        self.start = start
        self.end = end
        self.duration = duration
        self.order = order
        self.coefficients = self._compute_coefficients()

    def _construct_matrix(self, T):
        """Construct the boundary condition matrix."""
        A = np.array([
            [0, 0, 0, 0, 0, 0, 0, 1],           # x(0) = a
            [0, 0, 0, 0, 0, 0, 1, 0],           # x_dot(0) = 0
            [0, 0, 0, 0, 0, 1, 0, 0],           # x_ddot(0) = 0
            [0, 0, 0, 0, 1, 0, 0, 0],           # x_dddot(0) = 0
            [T**7, T**6, T**5, T**4, T**3, T**2, T, 1],  # x(T) = b
            [7*T**6, 6*T**5, 5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0],  # x_dot(T) = 0
            [42*T**5, 30*T**4, 20*T**3, 12*T**2, 6*T, 2, 0, 0],   # x_ddot(T) = 0
            [210*T**4, 120*T**3, 60*T**2, 24*T, 6, 0, 0, 0]       # x_dddot(T) = 0
        ])
        return A

    def _compute_coefficients(self):
        """Compute the polynomial coefficients for all dimensions."""
        coefficients = []
        T = self.duration
        for i in range(6):
            A = self._construct_matrix(T)
            B = np.array([self.start[i], 0, 0, 0, self.end[i], 0, 0, 0])
            coeffs = np.linalg.solve(A, B)
            coefficients.append(coeffs)
        return np.array(coefficients)
    
    def update_cofficients(self, start, duration):
        """Compute the polynomial coefficients for all dimensions."""
        coefficients = []
        T = duration
        for i in range(6):
            A = self._construct_matrix(T)
            B = np.array([start[i], 0, 0, 0, self.end[i], 0, 0, 0])
            coeffs = np.linalg.solve(A, B)
            coefficients.append(coeffs)
        return np.array(coefficients)
        

    def evaluate(self, t, order=0):
        """
        Evaluate the trajectory or its derivatives at time t.

        :param t: Time points to evaluate.
        :param order: Derivative order (0=position, 1=velocity, 2=acceleration, 3=jerk).
        :return: List of evaluated values for all dimensions.
        """
        trajectories = []
        for coeffs in self.coefficients:
            deriv_coeffs = coeffs
            for _ in range(order):
                deriv_coeffs = np.polyder(deriv_coeffs)
            trajectories.append(np.polyval(deriv_coeffs, t))
        return np.array(trajectories)

    def plot(self, freq=100):
        """Plot the trajectory and its derivatives."""
        t = np.linspace(0, self.duration , freq * self.duration)
        print(t)
        positions = self.evaluate(t, order=0)
        velocities = self.evaluate(t, order=1)
        accelerations = self.evaluate(t, order=2)
        jerks = self.evaluate(t, order=3)
        snaps = self.evaluate(t, order=4)
        crackles = self.evaluate(t, order=5)

        labels = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        fig, axs = plt.subplots(6, 1, figsize=(10, 30))

        for i, label in enumerate(labels):
            axs[0].plot(t, positions[i], label=f'{label}(t)')
            axs[1].plot(t, velocities[i], label=f"{label}'(t)")
            axs[2].plot(t, accelerations[i], label=f"{label}''(t)")
            axs[3].plot(t, jerks[i], label=f"{label}'''(t)")
            axs[4].plot(t, snaps[i], label=f"{label}''''(t)")
            axs[5].plot(t, crackles[i], label=f"{label}'''''(t)")

        for ax in axs:
            ax.legend()
            ax.set_xlabel('t')
            ax.grid(True)

        axs[0].set_ylabel('Position')
        axs[1].set_ylabel('Velocity')
        axs[2].set_ylabel('Acceleration')
        axs[3].set_ylabel('Jerk')
        axs[4].set_ylabel('Snap')
        axs[5].set_ylabel('Crackle')

        plt.tight_layout()

        # 3D plot for x, y, z
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(positions[0], positions[1], positions[2], label='Trajectory')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Trajectory of Position')
        ax.legend()
        plt.show()



# Boundary conditions
start = [0, 0, 0, 0, 0, 0]  # Initial conditions
end = [-1.5, 1, 1.5, 0.0, 0.0, 0.0]  # Final conditions
T = 4  # Time duration

# Create the trajectory generator
traj_gen = TrajectoryGenerator(start, end, T)

# Plot the trajectory and its derivatives
traj_gen.plot()