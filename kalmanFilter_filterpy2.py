from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

import numpy as np


# YOU MUST SET f.x to initial state matrix after calling this function
# states are as follows [[x_pos], [y_pos], [x_vel], [y_vel]]
def init_filter(dt):
    # Establish Kalman Filter with 4 dimensions (x, xdot, y, ydot) and 2 measurements (x, y)
    f = KalmanFilter(dim_x=4, dim_z=2)

    # set state transition matrix (our best linear model for the system)
    f.F = np.array([[1., 0., dt, 0.], [0., 1., 0., dt], [0., 0., 1., 0.], [0., 0., 0., 1.]])

    # set control transition matrix
    f.B = np.array([[0], [-.5*pow(dt, 2)], [0], [-dt]])

    # define the measurement function (what our measurement, z measures)
    f.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

    # define initial covariance. We are not confident in our initial state, so we make it relatively high
    # The initial position x,y have a much lower variance then the initial velocity
    # no covariance between x and y directions
    sigma_sq_cov = 0.1
    f.P = sigma_sq_cov * np.array([[.1, 0, 0.4472, 0], [0, .1, 0,0.4472], [0.4472, 0, 2, 0], [0, 0.4472, 0, 2]])

    # define process covariance matrix (covariance of the error in our model)
    # Set covariance between x and y dimensional variables to 0. Calculate other covariances as normal
    f.Q = Q_discrete_white_noise(dim=4, dt=dt, var=0.1)

    # define measurement covariance matrix (covariance of the error in our measurement)
    sigma_sq_measurement = .0001
    f.R = sigma_sq_measurement * np.array([[1, 0], [0, 1]])

    return f

