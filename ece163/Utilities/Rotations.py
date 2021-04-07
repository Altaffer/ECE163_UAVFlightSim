"""
Author: Luca Altaffer. ECE 163
Defining Rotation functions for lab 0
"""
import math
from . import MatrixMath

def c(angle):
    return math.cos(angle)
def s(angle):
    return math.sin(angle)
def euler2DCM(yaw,pitch,roll):
    """ function to get the DCM matrix using the euler angles
    -use the rotation matrix to input the euler angle to get my DCM
    """
    dcm = [[c(pitch)*c(yaw), c(pitch)*s(yaw), -s(pitch)], [s(roll)*s(pitch)*c(yaw)-(c(roll)*s(yaw)), s(roll)*s(pitch)*s(yaw)+(c(roll)*c(yaw)), s(roll)*c(picth)], [c(roll)*s(pitch)*c(yaw)+(s(roll)*s(yaw)), c(roll)*s(pitch)*s(yaw)-(s(roll)*c(yaw)), c(roll)*c(pitch)]]
    return dcm

def dcm2euler(dcm):
    """function to get the euler
    - use the defined formulas of yaw, pitch, and roll
    - assigned the correct
    """
    if dcm[0][2]
    pitch = -math.asin(dcm[0][2])
    yaw = math.atan2(dcm[1][2],dcm[2][2])
    roll = math.atan2(dcm[1][2],dcm[2][2])
    euler = [yaw, pitch, roll]
    return euler

def ned2enu(points):
    """function to get enu points from ned points.
    - use the rotation matrix previously define to rotate given ned point to enu
    """
    rot_n2e = [[0,1,0], [1,0,0], [0,0,-1]]
    enu = matrixMultiply(rot_n2e,points)
    return enu
