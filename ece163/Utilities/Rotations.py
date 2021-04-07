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
    """ for this function I:
    -use the rotation matrix to input the euler angle to get my DCM
    """
    dcm = [[c(pitch)*c(yaw), c(pitch)*s(yaw), -s(pitch)], [s(roll)*s(pitch)*c(yaw)-(c(roll)*s(yaw)), s(roll)*s(pitch)*s(yaw)+(c(roll)*c(yaw)), s(roll)*c(picth)], [c(roll)*s(pitch)*c(yaw)+(s(roll)*s(yaw)), c(roll)*s(pitch)*s(yaw)-(s(roll)*c(yaw)), c(roll)*c(pitch)]]
    return dcm

def dcm2euler(dcm):
    """for this function I:
    - use the defined formulas of yaw, pitch, and roll
    - assigned the correct
    """
    pitch = math.asin(dcm[0][2])
    yaw = math.atan2(dcm[1][2],dcm[2][2])
    roll = c(2*dcm[1][2])/s(2*dcm[2][2])
    euler = [yaw, pitch, roll]
    return euler

def ned2enu(points):
    """for this function I:
    -define ned and the rotation matrix as individual matrices
    -multiply each section of the matrix using for loops
    """
    rot_n2e = [[0,1,0], [1,0,0], [0,0,-1]]
    for x in range(len(points)):
        for y in range(len(rot_n2e[0])):
            for z in range(len(rot_n23)):
                enu[x][y] += points[x][z]*rot_n2e[z][y]
    return enu
