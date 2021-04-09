"""This file is a test harness for the module ece163.Utilities.Rotations,
and for the method ece163.Modeling.VehicleGeometry.getNewPoints(). 

 It is meant to be run from the root directory of the repo with:

python testChapter2.py

at which point it will execute various tests on the Rotations module"""

#%% Initialization of test harness and helpers:

import math
import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleGeometry as VG


"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-12)

def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
	return all(el_close)

#of course, you should test your testing tools too:
assert(compareVectors([[0], [0], [-1]],[[1e-13], [0], [-1+1e-9]]))
assert(not compareVectors([[0], [0], [-1]],[[1e-11], [0], [-1]]))
assert(not compareVectors([[1e8], [0], [-1]],[[1e8+1], [0], [-1]]))



failed = []
passed = []
def evaluateTest(test_name, boolean):
	"""evaluateTest prints the output of a test and adds it to one of two 
	global lists, passed and failed, which can be printed later"""
	if boolean:
		print(f"   passed {test_name}")
		passed.append(test_name)
	else:
		print(f"   failed {test_name}")
		failed.append(test_name)
	return boolean


#%% Euler2dcm():
print("Beginning testing of Rotations.Euler2dcm()")

cur_test = "Euler2dcm yaw test 1"
#we know that rotating [1,0,0] by 90 degrees about Z should produce [0,-1,0], so
R = Rotations.euler2DCM(90*math.pi/180, 0, 0)
orig_vec = [[1],[0],[0]]
expected_vec = [[0],[-1],[0]]
actual_vec = mm.matrixMultiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")
cur_test = "Euler2dcm yaw test 2"
#we know that rotating [1,0,0] by -90 degrees about Z should produce [0,1,0], so
R = Rotations.euler2DCM(-90*math.pi/180, 0, 0)
orig_vec = [[1],[0],[0]]
expected_vec = [[0],[1],[0]]
actual_vec = mm.matrixMultiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")
cur_test = "Euler2dcm yaw test 3"
#we know that rotating [1,0,0] by 45 degrees about Z should produce [math.sqrt(2)/2,math.sqrt(2)/2,0], so
R = Rotations.euler2DCM(45*math.pi/180, 0, 0)
orig_vec = [[1],[0],[0]]
expected_vec = [[math.sqrt(2)/2],[-math.sqrt(2)/2],[0]]
actual_vec = mm.matrixMultiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")


#%%  
cur_test = "Euler2dcm pitch test 1"
#we know that rotating [1,0,0] by 90 degrees about Y should produce [0,0,1], so
R = Rotations.euler2DCM(0,90*math.pi/180,0)
orig_vec = [[1],[0],[0]]
expected_vec = [[0],[0],[1]]
actual_vec = mm.matrixMultiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")
cur_test = "Euler2dcm pitch test 2"
#we know that rotating [1,0,0] by -90 degrees about Y should produce [0,0,-1], so
R = Rotations.euler2DCM(0,-90*math.pi/180,0)
orig_vec = [[1],[0],[0]]
expected_vec = [[0],[0],[-1]]
actual_vec = mm.matrixMultiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")
cur_test = "Euler2dcm pitch test 3"
#we know that rotating [1,0,0] by 45 degrees about Y should produce [math.sqrt(2)/2,0,-math.sqrt(2)/2], so
R = Rotations.euler2DCM(0,45*math.pi/180,0)
orig_vec = [[1],[0],[0]]
expected_vec = [[math.sqrt(2)/2],[0],[math.sqrt(2)/2]]
actual_vec = mm.matrixMultiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")


cur_test = "Euler2dcm roll test 1"
#we know that rotating [0,0,1] by 90 degrees about X should produce [0,1,0], so
R = Rotations.euler2DCM(0,0,90*math.pi/180)
orig_vec = [[0],[0],[1]]
expected_vec = [[0],[1],[0]]
actual_vec = mm.matrixMultiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")
cur_test = "Euler2dcm roll test 2"
#we know that rotating [0,0,1] by -90 degrees about X should produce [0,0,-1], so
R = Rotations.euler2DCM(0,0,-90*math.pi/180)
orig_vec = [[0],[-1],[0]]
expected_vec = [[0],[0],[-1]]
actual_vec = mm.matrixMultiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")
cur_test = "Euler2dcm roll test 3"
#we know that rotating [0,1,0] by 45 degrees about X should produce [0,math.sqrt(2)/s,math.sqrt(2)/2], so
R = Rotations.euler2DCM(0,0,45*math.pi/180)
orig_vec = [[0],[-1],[0]]
expected_vec = [[0],[-math.sqrt(2)/2],[math.sqrt(2)/2]]
actual_vec = mm.matrixMultiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")


cur_test = "dcm2euler test 1"
#We know that by using our previously defined function for dcm2euler in terms of R it should be equal to R*(the actual vector), so
R = Rotations.euler2DCM(0,0,90*math.pi/180)
expected_vec = Rotations.dcm2euler(R)
actual_vec = [0,0,90*math.pi/180]
if not evaluateTest(cur_test, isclose(math.pi/2, actual_vec[2])):
	print(f"{expected_vec} != {actual_vec}")
cur_test = "dcm2euler test 2"
#We know that by using our previously defined function for dcm2euler in terms of R it should be equal to R*(the actual vector), so
R = Rotations.euler2DCM(0,45*math.pi/180,0)
expected_vec = Rotations.dcm2euler(R)
actual_vec = [0,45*math.pi/180,0]
if not evaluateTest(cur_test, isclose(45*math.pi/180, actual_vec[1])):
	print(f"{expected_vec} != {actual_vec}")
cur_test = "dcm2euler test 3"
#We know that by using our previously defined function for dcm2euler in terms of R it should be equal to R*(the actual vector), so
R = Rotations.euler2DCM(-90*math.pi/180,0,0)
expected_vec = Rotations.dcm2euler(R)
actual_vec = [-90*math.pi/180,0,0]
if not evaluateTest(cur_test, isclose(-math.pi/2, actual_vec[0])):
	print(f"{expected_vec} != {actual_vec}")


cur_test = "ned2enu test 1"
#By applying the previosuly defined ned2enu function to [[0,2,4]] we expect to get [[2,0,-4]]
R = mm.matrixTranspose(Rotations.ned2enu([[0,2,4]]))
expected_vec = mm.matrixTranspose([[2,0,-4]])
if not evaluateTest(cur_test, compareVectors(expected_vec, R) ):
	print(f"{expected_vec} != {actual_vec}")
cur_test = "ned2enu test 2"
#By applying the previosuly defined ned2enu function to [[1,6,7]] we expect to get [[6,1,-7]]
R = mm.matrixTranspose(Rotations.ned2enu([[1,6,7]]))
expected_vec = mm.matrixTranspose([[6,1,-7]])
if not evaluateTest(cur_test, compareVectors(expected_vec, R) ):
	print(f"{expected_vec} != {actual_vec}")
cur_test = "ned2enu test 3"
#By applying the previosuly defined ned2enu function to [[-3,1,-4]] we expect to get [[1,-3,4]]
R = mm.matrixTranspose(Rotations.ned2enu([[-3,1,-4]]))
expected_vec = mm.matrixTranspose([[1,-3,4]])
if not evaluateTest(cur_test, compareVectors(expected_vec, R) ):
	print(f"{expected_vec} != {actual_vec}")


cur_test = "vehicle geometry test 1"
vned = VG.VehicleGeometry
vned.__init__(vned)
newpoints = (vned.getNewPoints(vned, 0, 0, 0, 90*math.pi/180, 0, 0))
point = newpoints[0]
expected_vec = [-0.9652,0,0]
if not evaluateTest(cur_test, point[0]<-0.95 and point[0]>-0.97 and point[1]<0.01 and point[1]>-0.01 and point[2]<0.01 and point[2]>-0.01):
	print(f"{expected_vec} != {point}")
cur_test = "vehicle geometry test 2"
vned = VG.VehicleGeometry
vned.__init__(vned)
newpoints = (vned.getNewPoints(vned, 0, 0, 0, 0, 90*math.pi/180, 0))
point = newpoints[0]
expected_vec = [0,0,-0.9652]
if not evaluateTest(cur_test, point[2]<-0.95 and point[2]>-0.97 and point[1]<0.01 and point[1]>-0.01 and point[0]<0.01 and point[0]>-0.01):
	print(f"{expected_vec} != {point}")
cur_test = "vehicle geometry test 3"
vned = VG.VehicleGeometry
vned.__init__(vned)
newpoints = (vned.getNewPoints(vned, 0, 0, 0, 0, 0, 90*math.pi/180))
point = newpoints[0]
expected_vec = [0,0.9652,0]
if not evaluateTest(cur_test, point[1]>0.95 and point[1]<0.97 and point[2]<0.01 and point[2]>-0.01 and point[0]<0.01 and point[0]>-0.01):
	print(f"{expected_vec} != {point}")
cur_test = "vehicle geometry test 4"
vned = VG.VehicleGeometry
vned.__init__(vned)
newpoints = (vned.getNewPoints(vned, 1, 0, 0, 0, 0, 0))
point = newpoints[0]
expected_vec = [0,1.9652,0]
if not evaluateTest(cur_test, point[1]>1.95 and point[1]<1.97 and point[2]<0.01 and point[2]>-0.01 and point[0]<0.01 and point[0]>-0.01):
	print(f"{expected_vec} != {point}")
cur_test = "vehicle geometry test 5"
vned = VG.VehicleGeometry
vned.__init__(vned)
newpoints = (vned.getNewPoints(vned, 0, 1, 0, 0, 0, 0))
point = newpoints[0]
expected_vec = [1,0.9652,0]
if not evaluateTest(cur_test, point[1]>0.95 and point[1]<0.97 and point[2]<0.01 and point[2]>-0.01 and point[0]<1.1 and point[0]>0.9):
	print(f"{expected_vec} != {point}")
cur_test = "vehicle geometry test 6"
vned = VG.VehicleGeometry
vned.__init__(vned)
newpoints = (vned.getNewPoints(vned, 0, 0, 1, 0, 0, 0))
point = newpoints[0]
expected_vec = [0,0.9652,1]
if not evaluateTest(cur_test, point[1]>0.95 and point[1]<0.97 and point[2]<0.01 and point[2]>-1.01 and point[0]<0.01 and point[0]>-0.01):
	print(f"{expected_vec} != {point}")
#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]