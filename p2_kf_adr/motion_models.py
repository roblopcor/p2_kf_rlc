import numpy as np

def velocity_motion_model():

    def state_transition_matrix_A():
        return np.eye(3)

    def control_input_matrix_B(mu, delta_t):
        theta = mu[2]
        return np.array([
            [np.cos(theta) * delta_t, 0],
            [np.sin(theta) * delta_t, 0],
            [0, delta_t]
        ])
    
    return state_transition_matrix_A, control_input_matrix_B

def velocity_motion_model_2():
    def A():
        def compute(dt):
            return np.array([
                [1, 0, 0, dt,  0,  0],
                [0, 1, 0, 0,  dt, 0],
                [0, 0, 1, 0,  0,  dt],
                [0, 0, 0, 1,  0,  0],
                [0, 0, 0, 0,  1,  0],
                [0, 0, 0, 0,  0,  1],
            ])
        return compute

    def B(mu, dt):
        # No usamos entradas de control (filtro puro)
        return np.zeros((6, 2))
    return A(), B