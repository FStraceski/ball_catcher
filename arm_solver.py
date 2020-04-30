import numpy as np


# link lengths in meters
l1 = .17
l2 = .13
r_max = l1 + l2


# this uses the closed geometric solution to determine the angles needed to move the end affector to x,y
# return 1 if successful, 0 if failed
# Source: https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
def get_angle(x, y):
    if x == 0:
        return 0, 0, 0

    # angle between l1 and l2, ignore the negative solution
    num = np.power(x, 2) + np.power(y, 2) - np.power(l1, 2) - np.power(l2, 2)
    den = 2 * l1 * l2
    if num/den > 1:
        return False, 0, 0
    a2 = np.arccos(num / den)

    # angle between l1 and the ground
    num2 = l2 * np.sin(a2)
    den2 = l1 + l2*np.sin(a2)
    a1 = np.rad2deg(np.arctan(y / x) - np.arctan(num2 / den2))
    a2 = np.rad2deg(a2)

    if (np.isnan(a1) == True or np.isnan(a2) == True):
        return False, 0, 0

    return True, int(np.around(a1)), int(np.around(a2))
