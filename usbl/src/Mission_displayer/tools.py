import numpy as np
from numpy import cos,sin


def R(theta):
    return np.array([[cos(theta),-sin(theta)],[sin(theta),cos(theta)]])