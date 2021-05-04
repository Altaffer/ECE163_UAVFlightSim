"""This file is a test harness for the module ece163.Utilities.Rotations,
and for the method ece163.Modeling.VehicleGeometry.getNewPoints(). 

 It is meant to be run from the root directory of the repo with:

python testChapter5.py

at which point it will execute various tests on the Rotations module"""

#%% Initialization of test harness and helpers:

import math
import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Modeling.VehicleAerodynamicsModel as VAM
import ece163.Modeling.WindModel as WM
import ece163.Controls.VehiclePerturbationModels as VPM
import ece163.Containers.States as States
import ece163.Containers.Inputs as Inputs


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


print("Beginning testing of VehicleAerodynamicsModel()")


def getWindModeltest():
	"""Testing the getWindModel in aerodynamic forces against windmodel"""
	cur_test = "getVehicleState test 1"
	Am = VAM.VehicleAerodynamicsModel()
	actual = WM.WindModel()


def setWindModeltest():
	"""Testing setWindModel against windmodel"""

def CalculateAirspeedtest():
	"""Calculating the airspeed, angle of attck, and side-slip angle with wind in mind"""

def updateForcestest():
	"""testing the wind contribution on Va, alpha, and beta"""


print("Beginning testing of WindModel()")


def __init__test():
	"""Testig the init for WindModel"""

def resettest():
	"""Validating the parameter reset of WindModel"""

def setWindtest():
	"""Testing setWind against windState"""

def getWindtest():
	"""Testing getWind against windState"""

def setWindModelParameterstest():
	"""Testing WindModel Parameters against predefined values for, Wn, We, Wd, drydenParameters  """

def CreateDrydenTransferFnstest():
	"""Testing define values for Phi, gamma, and H and their components"""

def getDrydenTransferFnstest():
	"""Testing the data from CreateDrydenTransferFns that are stored in init"""

def Updatetest():
	"""Testing whether x updates a step accordingly"""


print("Beginning testing of VehiclePerturbationModels")


def CreateTransferFunctiontest():
	"""Testing all the outputted components of TransferFunction and their predefined components"""

def dThrust_dThrottletest():
	"""Testing the dThrust/dThrottle derivative using CalculatePropForces and defined values for Va and Throttle"""

def dThrust_dVatest():
	"""Testing the dThrust/dVa derivative using CalculatePropForces and define values for Va and Throttle"""


#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]