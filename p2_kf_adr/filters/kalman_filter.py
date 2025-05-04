import numpy as np 

from ..motion_models import velocity_motion_model, velocity_motion_model_2
from ..observation_models import odometry_observation_model, odometry_observation_model_2

class KalmanFilter:

    def __init__(self, initial_state, initial_covariance, proc_noise_std = [0.02, 0.02, 0.01], obs_noise_std = [0.02, 0.02, 0.01]):
        self.mu = initial_state # Initial state estimate [x, y, theta]
        self.Sigma = initial_covariance # Initial uncertainty

        self.A, self.B = velocity_motion_model() # The action model to use. Returns A and B matrices

        # Standard deviations for the noise in x, y, and theta (process or action model noise)
        self.proc_noise_std = np.array(proc_noise_std)
        # Process noise covariance (R)
        self.R = np.diag(self.proc_noise_std ** 2)  # process noise covariance

        # Observation model (C)
        self.C = odometry_observation_model() # The observation model to use

        # Standard deviations for the noise in x, y, theta (observation or sensor model noise)
        self.obs_noise_std = np.array(obs_noise_std)
        # Observation noise covariance (Q)
        self.Q = np.diag(self.obs_noise_std ** 2)
            
    def predict(self, u, dt):
        A = self.A()  # llamar a la función para obtener la matriz
        B = self.B(self.mu, dt)  # Matriz B con orientación actual
        self.mu = A @ self.mu + B @ u
        self.Sigma = A @ self.Sigma @ A.T + self.R

    def update(self, z):
        y = z - self.C @ self.mu                          # Residual
        S = self.C @ self.Sigma @ self.C.T + self.Q       # Innovación
        K = self.Sigma @ self.C.T @ np.linalg.inv(S)      # Ganancia de Kalman
        self.mu = self.mu + K @ y                         # Corrección de estado
        I = np.eye(self.Sigma.shape[0])
        self.Sigma = (I - K @ self.C) @ self.Sigma        # Corrección de covarianza

    def get_state(self):
        return self.mu

    def get_covariance(self):
        return self.Sigma

class KalmanFilter_2:
    def __init__(self, initial_state, initial_covariance,
                 proc_noise_std=[0.02]*6, obs_noise_std=[0.02]*6):

        self.mu = initial_state  # Initial state estimate [x, y, theta, vx, vy, omega]
        self.Sigma = initial_covariance  # Initial uncertainty

        self.A_fn, self.B = velocity_motion_model_2()  # Motion model matrices

        self.proc_noise_std = np.array(proc_noise_std)
        self.R = np.diag(self.proc_noise_std ** 2)  # Process noise covariance

        self.C = odometry_observation_model_2()  # Observation matrix
        self.obs_noise_std = np.array(obs_noise_std)
        self.Q = np.diag(self.obs_noise_std ** 2)  # Observation noise covariance

    def predict(self, u=None, dt=1.0):
        A = self.A_fn(dt)
        self.mu = A @ self.mu
        self.Sigma = A @ self.Sigma @ A.T + self.R

    def update(self, z):
        S = self.C @ self.Sigma @ self.C.T + self.Q  # Cov. de innovación
        K = self.Sigma @ self.C.T @ np.linalg.inv(S)  # Ganancia de Kalman
        y = z - self.C @ self.mu  # Innovación
        self.mu = self.mu + K @ y
        self.Sigma = (np.eye(6) - K @ self.C) @ self.Sigma

    def get_state(self):
        return self.mu

    def get_covariance(self):
        return self.Sigma
 