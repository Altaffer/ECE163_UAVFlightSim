"""
Author: Luca Altaffer (taltaffe@ucsc.edu)
This file contains tools to perform rotations by producing rotation matrices.
"""
import math
#import MatrixMath as mm
from ..Utilities import MatrixMath as mm

def c(angle):
    return math.cos(angle)
def s(angle):
    return math.sin(angle)
def euler2DCM(yaw,pitch,roll):
    """ function to get the DCM matrix using the euler angles
    -use the rotation matrix to input the euler angle to get my DCM
    """
    dcm = [[c(pitch)*c(yaw), c(pitch)*s(yaw), -s(pitch)], [s(roll)*s(pitch)*c(yaw)-(c(roll)*s(yaw)), s(roll)*s(pitch)*s(yaw)+(c(roll)*c(yaw)), s(roll)*c(pitch)], [c(roll)*s(pitch)*c(yaw)+(s(roll)*s(yaw)), c(roll)*s(pitch)*s(yaw)-(s(roll)*c(yaw)), c(roll)*c(pitch)]]
    return dcm

def dcm2euler(dcm):
    """function to get the euler
    - use the defined formulas of yaw, pitch, and roll
    - assigned the correct
    """
    if (dcm[0][2] >1):
        dcm = 1
    if (dcm[0][2] < -1):
        dcm = -1
    pitch = -math.asin(dcm[0][2])
    yaw = math.atan2(dcm[0][1],dcm[0][0])
    roll = math.atan2(dcm[1][2],dcm[2][2])
    euler = [yaw, pitch, roll]
    return euler

def ned2enu(points):
    """function to get enu points from ned points.
    - use the rotation matrix previously define to rotate given ned point to enu
    """
    rot_n2e = [[0,1,0], [1,0,0], [0,0,-1]]
    enu = mm.multiply(points,rot_n2e)
    return enu
