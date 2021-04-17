"""This file is a test harness for the module ece163.Utilities.Rotations,
and for the method ece163.Modeling.VehicleGeometry.getNewPoints().

 It is meant to be run from the root directory of the repo with:

python testChapter3.py

at which point it will execute various tests on the Rotations module"""

#%% Initialization of test harness and helpers:

import math
import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleGeometry as VG
import ece163.Containers.States as States
import ece163.Modeling.VehicleDynamicsModel as VDM
from ece163.Constants import VehiclePhysicalConstants as VPC


"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-12)
def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
	return all(el_close)

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


cur_test = "init test 1"
#init test for state
p1 = VDM.VehicleDynamicsModel(0.01)
if not evaluateTest(cur_test, (p1.state == States.vehicleState()) ):
	print(f"{p1.state} != {q}")
cur_test = "init test 2"
#init test for dot
p1 = VDM.VehicleDynamicsModel(0.01)
if not evaluateTest(cur_test, (p1.dot == States.vehicleState()) ):
	print(f"{p1.dot} != {q}")
cur_test = "init test 3"
#init test for dt
p1 = VDM.VehicleDynamicsModel(0.01)
if not evaluateTest(cur_test, (p1.dT == VPC.dT) ):
	print(f"{p1.dot} != {VPC.dT}")

cur_test = "setVehicleState test 1"
#test for setting the vehicle state
p1 = VDM.VehicleDynamicsModel(0.01)
#if not evaluateTest(cur_test, (p1.state == States.vehicleState()) ):
	#print(f"{p1.dot} != {VPC.dT}")

cur_test = "getVehicleState test 1"
#test for getting the vehicle state
p1 = VDM.VehicleDynamicsModel(0.01)
if not evaluateTest(cur_test, (p1.state == States.vehicleState()) ):
	print(f"{p1.state} != {States.vehicleState()}")

cur_test = "setVehicleDerivative test 1"
#test for setting the vehicle Derivative

cur_test = "getVehicleDerivative test 1"
#test for getting the vehicle Derivative
p1 = VDM.VehicleDynamicsModel(0.01)
if not evaluateTest(cur_test, (p1.dot == States.vehicleState()) ):
	print(f"{p1.dot} != {VPC.dT}")

cur_test = "reset test 1"
#testing if the reset brings components back to the origin

cur_test = "derivative test 1"
#testing the derivative of pn
p1 = VDM.VehicleDynamicsModel(0.01)
p2 = VDM.VehicleDynamicsModel(0.01)
p2.dot.pn = 1
p1.dot = p2.getVehicleDerivative()
if not evaluateTest(cur_test, isclose(p1.dot.pn, p2.dot.pn) ):
	print(f"{p1.dot.pn} != {p2.dot.pn}")
p2.reset()
cur_test = "derivative test 2"
#testing the derivative of pe
p1 = VDM.VehicleDynamicsModel(0.01)
p2 = VDM.VehicleDynamicsModel(0.01)
p2.dot.pe = 2
p1.dot = p2.getVehicleDerivative()
if not evaluateTest(cur_test, isclose(p1.dot.pe, p2.dot.pe) ):
	print(f"{p1.dot.pe} != {p2.dot.pe}")
cur_test = "derivative test 3"
#testing the derivative of pd
p1 = VDM.VehicleDynamicsModel(0.01)
p2 = VDM.VehicleDynamicsModel(0.01)
p2.dot.pd = 3
p1.dot = p2.getVehicleDerivative()
if not evaluateTest(cur_test, isclose(p1.dot.pd, p2.dot.pd) ):
	print(f"{p1.dot.pd} != {p2.dot.pd}")
p2.reset()
cur_test = "derivative test 4"
#testing the derivative of u
p1 = VDM.VehicleDynamicsModel(0.01)
p2 = VDM.VehicleDynamicsModel(0.01)
p2.dot.u = 4
p1.dot = p2.getVehicleDerivative()
if not evaluateTest(cur_test, isclose(p1.dot.u, p2.dot.u) ):
	print(f"{p1.dot.u} != {p2.dot.u}")
cur_test = "derivative test 5"
#testing the derivative of v
p1 = VDM.VehicleDynamicsModel(0.01)
p2 = VDM.VehicleDynamicsModel(0.01)
p2.dot.v = 5
p1.dot = p2.getVehicleDerivative()
if not evaluateTest(cur_test, isclose(p1.dot.v, p2.dot.v) ):
	print(f"{p1.dot.v} != {p2.dot.v}")
p2.reset()
cur_test = "derivative test 6"
#testing the derivative of w
p1 = VDM.VehicleDynamicsModel(0.01)
p2 = VDM.VehicleDynamicsModel(0.01)
p2.dot.w = 6
p1.dot = p2.getVehicleDerivative()
if not evaluateTest(cur_test, isclose(p1.dot.w, p2.dot.w) ):
	print(f"{p1.dot.w} != {p2.dot.w}")
cur_test = "derivative test 7"
#testing the derivative of yaw
p1 = VDM.VehicleDynamicsModel(0.01)
p2 = VDM.VehicleDynamicsModel(0.01)
p2.dot.yaw = 7
p1.dot = p2.getVehicleDerivative()
if not evaluateTest(cur_test, isclose(p1.dot.yaw, p2.dot.yaw) ):
	print(f"{p1.dot.yaw} != {p2.dot.yaw}")
cur_test = "derivative test 8"
#testing the derivative of pitch
p1 = VDM.VehicleDynamicsModel(0.01)
p2 = VDM.VehicleDynamicsModel(0.01)
p2.dot.pitch = 8
p1.dot = p2.getVehicleDerivative()
if not evaluateTest(cur_test, isclose(p1.dot.pitch, p2.dot.pitch) ):
	print(f"{p1.dot.pitch} != {p2.dot.pitch}")
p2.reset()
cur_test = "derivative test 9"
#testing the derivative of roll
p1 = VDM.VehicleDynamicsModel(0.01)
p2 = VDM.VehicleDynamicsModel(0.01)
p2.dot.roll = 9
p1.dot = p2.getVehicleDerivative()
if not evaluateTest(cur_test, isclose(p1.dot.roll, p2.dot.roll) ):
	print(f"{p1.dot.roll} != {p2.dot.roll}")
cur_test = "derivative test 10"
#testing the derivative of p
p1 = VDM.VehicleDynamicsModel(0.01)
p2 = VDM.VehicleDynamicsModel(0.01)
p2.dot.p = 10
p1.dot = p2.getVehicleDerivative()
if not evaluateTest(cur_test, isclose(p1.dot.p, p2.dot.p) ):
	print(f"{p1.dot.p} != {p2.dot.p}")
cur_test = "derivative test 11"
#testing the derivative of q
p1 = VDM.VehicleDynamicsModel(0.01)
p2 = VDM.VehicleDynamicsModel(0.01)
p2.dot.q = 11
p1.dot = p2.getVehicleDerivative()
if not evaluateTest(cur_test, isclose(p1.dot.q, p2.dot.q) ):
	print(f"{p1.dot.q} != {p2.dot.q}")
p2.reset()
cur_test = "derivative test 12"
#testing the derivative of r
p1 = VDM.VehicleDynamicsModel(0.01)
p2 = VDM.VehicleDynamicsModel(0.01)
p2.dot.r = 12
p1.dot = p2.getVehicleDerivative()
if not evaluateTest(cur_test, isclose(p1.dot.r, p2.dot.r) ):
	print(f"{p1.dot.r} != {p2.dot.r}")
cur_test = "derivative test 13"
#testing the derivative of R
p1 = VDM.VehicleDynamicsModel(0.01)
p2 = VDM.VehicleDynamicsModel(0.01)
p2.dot.R = 12
p1.dot = p2.getVehicleDerivative()
if not evaluateTest(cur_test, isclose(p1.dot.R, p2.dot.R) ):
	print(f"{p1.dot.R} != {p2.dot.R}")

cur_test = "Rexp test 1"
p1 = VDM.VehicleDynamicsModel()
p1.state.p = 1
p1.dot.p = 2
p1.state.q = 3
p1.dot.q = 4
p1.state.r=5
p1.dot.r=6
Rexp_r = p1.Rexp(p1.dT, p1.state, p1.dot)
expec = [[0.9963024056253, 0.0704087104055, -0.049235480468299994],[-0.06940936057449999, 0.99735172294785, 0.021722872344249997],[0.050634570231699996, -0.01822514793575, 0.99855094274505]]
if not evaluateTest(cur_test, (Rexp_r==expec)):
	print(f"{Rexp_r} != {expec}")

cur_test = "IntegrateState test 1"
statex = States.vehicleState()
p1 = VDM.VehicleDynamicsModel()
p2 = VDM.VehicleDynamicsModel()
dT= VPC.dT
p2.state.pn = 2
p2.state.yaw = 45
p2.state.p = 5
p2.state.u = 3
expec_pn = 2
statex = p2.IntegrateState(dT, p2.state, p2.dot)
if not evaluateTest(cur_test, (statex.pn == expec_pn) ):
	print(f"{statex.pn} != {expec_pn}")
cur_test = "IntegrateState test 2"
statex = States.vehicleState()
p1 = VDM.VehicleDynamicsModel()
p2 = VDM.VehicleDynamicsModel()
dT= VPC.dT
p2.state.pe = 2
p2.state.pitch = 45
p2.state.q = 5
p2.state.v = 3
expec_v = 3
statex = p2.IntegrateState(dT, p2.state, p2.dot)
if not evaluateTest(cur_test, (statex.v == expec_v) ):
	print(f"{statex.v} != {expec_v}")
cur_test = "IntegrateState test 3"
statex = States.vehicleState()
p1 = VDM.VehicleDynamicsModel()
p2 = VDM.VehicleDynamicsModel()
dT= VPC.dT
p2.state.pd = 2
p2.state.roll = 45
p2.state.r = 5
p2.state.w = 3
expec_roll = 45
statex = p2.IntegrateState(dT, p2.state, p2.dot)
if not evaluateTest(cur_test, (statex.roll == expec_roll) ):
	print(f"{statex.roll} != {expec_roll}")
cur_test = "ForwardEuler test 1"
statex = States.vehicleState()
stated = States.vehicleState()
statex.pn = 1
stated.pn = 1
statex.v = 1
stated.v = 1
statex.r = 1
stated.r=1
p1 = VDM.VehicleDynamicsModel()
p2=p1.getVehicleDerivative()
fEuler =p1.ForwardEuler(0.01,statex, stated)
a = [p2.pn,p2.pe,p2.pd,p2.u,p2.v,p2.w,p2.p,p2.q,p2.r]
expec = [0,0,0,0,0,0,0,0,0]
if not evaluateTest(cur_test, (expec == a) ):
	print(f"{expec} != {a}")

#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]