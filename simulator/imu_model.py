import numpy as np

class IMUModel:
    def __init__(
        self,
        gyro_noise_std=0.002,      # rad/s
        accel_noise_std=0.05,      # m/s²
        gyro_bias_walk_std=0.00005,
        accel_bias_walk_std=0.001
    ):
        self.gyro_noise_std = gyro_noise_std
        self.accel_noise_std = accel_noise_std

        self.gyro_bias_walk_std = gyro_bias_walk_std
        self.accel_bias_walk_std = accel_bias_walk_std

        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)

        self.g = 9.81

    def update(self, truth_state, dt):
        """
        truth_state:
        {
            "pos": ...,
            "vel": ...,
            "acc_world": ...,
            "omegas": ...,
            "q": ...
        }

        returns:
        {
            "gyro": ...,
            "accel": ...
        }
        """

        self.gyro_bias += (np.random.normal(0.0, self.gyro_bias_walk_std, 3)
            * np.sqrt(dt)
        )

        self.accel_bias += (
            np.random.normal(0.0, self.accel_bias_walk_std, 3)
            * np.sqrt(dt))

        gyro_true = truth_state["omegas"]

        gyro_measured = (gyro_true+ self.gyro_bias
            + np.random.normal(0.0,self.gyro_noise_std,3))

        q0, q1, q2, q3 = truth_state["q"]

        rotation_matrix = np.array([
            [
                2*(q0*q0 + q1*q1) - 1,
                2*(q1*q2 - q0*q3),
                2*(q1*q3 + q0*q2)
            ],
            [
                2*(q1*q2 + q0*q3),
                2*(q0*q0 + q2*q2) - 1,
                2*(q2*q3 - q0*q1)
            ],
            [
                2*(q1*q3 - q0*q2),
                2*(q2*q3 + q0*q1),
                2*(q0*q0 + q3*q3) - 1
            ]
        ])

        gravity_world = np.array([0.0,0.0,self.g])

        specific_force_world = truth_state["acc_world"] - gravity_world

        accel_true = rotation_matrix.T @ specific_force_world

        accel_measured = (accel_true + self.accel_bias + 
                np.random.normal(0.0,self.accel_noise_std,3)
            )
        

        return {
            "gyro": gyro_measured,
            "accel": accel_measured
        }