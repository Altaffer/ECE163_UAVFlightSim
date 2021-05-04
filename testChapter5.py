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

cur_test = "reset test 1"
#we know that rotating [1,0,0] by 90 degrees about Z should produce [0,-1,0], so
#R = Rotations.euler2DCM(0*math.pi/180, 0*math.pi/180, -1*math.pi/180)
#orig_vec = [[0],[0],[107.91]]
#expected_vec = [[0],[-1],[0]]
#actual_vec = mm.matrixMultiply(R, orig_vec)
#if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	#print(f"{expected_vec} != {actual_vec}")

def getWindModel():
	""""""

def setWindModel():
	""""""
cur_test = "Update test 1"
Am = VAM.VehicleAerodynamicsModel()
Am2 = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
control = Inputs.controlInputs(Throttle = 0, Aileron = 0, Elevator = 0, Rudder = 0)
Am.Update(control)
force = Am2.updateForces(Am2.vehicleDynamicsModel.state, control)
Am2.vehicleDynamicsModel.Update(force)
actualstate = [Am.vehicleDynamicsModel.state.pn, Am.vehicleDynamicsModel.state.pe, Am.vehicleDynamicsModel.state.pd,
			   Am.vehicleDynamicsModel.state.u, Am.vehicleDynamicsModel.state.v, Am.vehicleDynamicsModel.state.w,
			   Am.vehicleDynamicsModel.state.p, Am.vehicleDynamicsModel.state.q, Am.vehicleDynamicsModel.state.r]
actualR = Am.vehicleDynamicsModel.state.R
expectedstate = [Am2.vehicleDynamicsModel.state.pn, Am2.vehicleDynamicsModel.state.pe, Am2.vehicleDynamicsModel.state.pd,
				 Am2.vehicleDynamicsModel.state.u, Am2.vehicleDynamicsModel.state.v, Am2.vehicleDynamicsModel.state.w,
				 Am2.vehicleDynamicsModel.state.p, Am2.vehicleDynamicsModel.state.q, Am2.vehicleDynamicsModel.state.r]
expectedR = Am2.vehicleDynamicsModel.state.R
if not evaluateTest(cur_test, (actualstate == expectedstate) and (actualR == expectedR)):
	print(f"{actualstate} != {expectedstate}")
	print(f"{actualR} != {exectedR}")

cur_test = "Update test 2"
Am = VAM.VehicleAerodynamicsModel()
Am2 = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
control = Inputs.controlInputs(Throttle = 1, Aileron = 0, Elevator = 0, Rudder = 0)
Am.Update(control)
force = Am2.updateForces(Am2.vehicleDynamicsModel.state, control)
Am2.vehicleDynamicsModel.Update(force)
actualstate = [Am.vehicleDynamicsModel.state.pn, Am.vehicleDynamicsModel.state.pe, Am.vehicleDynamicsModel.state.pd,
			   Am.vehicleDynamicsModel.state.u, Am.vehicleDynamicsModel.state.v, Am.vehicleDynamicsModel.state.w,
			   Am.vehicleDynamicsModel.state.p, Am.vehicleDynamicsModel.state.q, Am.vehicleDynamicsModel.state.r]
actualR = Am.vehicleDynamicsModel.state.R
expectedstate = [Am2.vehicleDynamicsModel.state.pn, Am2.vehicleDynamicsModel.state.pe, Am2.vehicleDynamicsModel.state.pd,
				 Am2.vehicleDynamicsModel.state.u, Am2.vehicleDynamicsModel.state.v, Am2.vehicleDynamicsModel.state.w,
				 Am2.vehicleDynamicsModel.state.p, Am2.vehicleDynamicsModel.state.q, Am2.vehicleDynamicsModel.state.r]
expectedR = Am2.vehicleDynamicsModel.state.R
if not evaluateTest(cur_test, (actualstate == expectedstate) and (actualR == expectedR)):
	print(f"{actualstate} != {expectedstate}")
	print(f"{actualR} != {exectedR}")

cur_test = "Update test 3"
Am = VAM.VehicleAerodynamicsModel()
Am2 = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
control = Inputs.controlInputs(Throttle = 0, Aileron = 1, Elevator = 0, Rudder = 0)
Am.Update(control)
force = Am2.updateForces(Am2.vehicleDynamicsModel.state, control)
Am2.vehicleDynamicsModel.Update(force)
actualstate = [Am.vehicleDynamicsModel.state.pn, Am.vehicleDynamicsModel.state.pe, Am.vehicleDynamicsModel.state.pd,
			   Am.vehicleDynamicsModel.state.u, Am.vehicleDynamicsModel.state.v, Am.vehicleDynamicsModel.state.w,
			   Am.vehicleDynamicsModel.state.p, Am.vehicleDynamicsModel.state.q, Am.vehicleDynamicsModel.state.r]
actualR = Am.vehicleDynamicsModel.state.R
expectedstate = [Am2.vehicleDynamicsModel.state.pn, Am2.vehicleDynamicsModel.state.pe, Am2.vehicleDynamicsModel.state.pd,
				 Am2.vehicleDynamicsModel.state.u, Am2.vehicleDynamicsModel.state.v, Am2.vehicleDynamicsModel.state.w,
				 Am2.vehicleDynamicsModel.state.p, Am2.vehicleDynamicsModel.state.q, Am2.vehicleDynamicsModel.state.r]
expectedR = Am2.vehicleDynamicsModel.state.R
if not evaluateTest(cur_test, (actualstate == expectedstate) and (actualR == expectedR)):
	print(f"{actualstate} != {expectedstate}")
	print(f"{actualR} != {exectedR}")

cur_test = "Update test 4"
Am = VAM.VehicleAerodynamicsModel()
Am2 = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
control = Inputs.controlInputs(Throttle = 0, Aileron = 0, Elevator = 1, Rudder = 0)
Am.Update(control)
force = Am2.updateForces(Am2.vehicleDynamicsModel.state, control)
Am2.vehicleDynamicsModel.Update(force)
actualstate = [Am.vehicleDynamicsModel.state.pn, Am.vehicleDynamicsModel.state.pe, Am.vehicleDynamicsModel.state.pd,
			   Am.vehicleDynamicsModel.state.u, Am.vehicleDynamicsModel.state.v, Am.vehicleDynamicsModel.state.w,
			   Am.vehicleDynamicsModel.state.p, Am.vehicleDynamicsModel.state.q, Am.vehicleDynamicsModel.state.r]
actualR = Am.vehicleDynamicsModel.state.R
expectedstate = [Am2.vehicleDynamicsModel.state.pn, Am2.vehicleDynamicsModel.state.pe, Am2.vehicleDynamicsModel.state.pd,
				 Am2.vehicleDynamicsModel.state.u, Am2.vehicleDynamicsModel.state.v, Am2.vehicleDynamicsModel.state.w,
				 Am2.vehicleDynamicsModel.state.p, Am2.vehicleDynamicsModel.state.q, Am2.vehicleDynamicsModel.state.r]
expectedR = Am2.vehicleDynamicsModel.state.R
if not evaluateTest(cur_test, (actualstate == expectedstate) and (actualR == expectedR)):
	print(f"{actualstate} != {expectedstate}")
	print(f"{actualR} != {exectedR}")

cur_test = "Update test 5"
Am = VAM.VehicleAerodynamicsModel()
Am2 = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
control = Inputs.controlInputs(Throttle = 0, Aileron = 0, Elevator = 0, Rudder = 1)
Am.Update(control)
force = Am2.updateForces(Am2.vehicleDynamicsModel.state, control)
Am2.vehicleDynamicsModel.Update(force)
actualstate = [Am.vehicleDynamicsModel.state.pn, Am.vehicleDynamicsModel.state.pe, Am.vehicleDynamicsModel.state.pd,
			   Am.vehicleDynamicsModel.state.u, Am.vehicleDynamicsModel.state.v, Am.vehicleDynamicsModel.state.w,
			   Am.vehicleDynamicsModel.state.p, Am.vehicleDynamicsModel.state.q, Am.vehicleDynamicsModel.state.r]
actualR = Am.vehicleDynamicsModel.state.R
expectedstate = [Am2.vehicleDynamicsModel.state.pn, Am2.vehicleDynamicsModel.state.pe, Am2.vehicleDynamicsModel.state.pd,
				 Am2.vehicleDynamicsModel.state.u, Am2.vehicleDynamicsModel.state.v, Am2.vehicleDynamicsModel.state.w,
				 Am2.vehicleDynamicsModel.state.p, Am2.vehicleDynamicsModel.state.q, Am2.vehicleDynamicsModel.state.r]
expectedR = Am2.vehicleDynamicsModel.state.R
if not evaluateTest(cur_test, (actualstate == expectedstate) and (actualR == expectedR)):
	print(f"{actualstate} != {expectedstate}")
	print(f"{actualR} != {exectedR}")

cur_test = "Update test 6"
Am = VAM.VehicleAerodynamicsModel()
Am2 = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
control = Inputs.controlInputs(Throttle = 1, Aileron = 2, Elevator = 3, Rudder = 4)
Am.Update(control)
force = Am2.updateForces(Am2.vehicleDynamicsModel.state, control)
Am2.vehicleDynamicsModel.Update(force)
actualstate = [Am.vehicleDynamicsModel.state.pn, Am.vehicleDynamicsModel.state.pe, Am.vehicleDynamicsModel.state.pd,
			   Am.vehicleDynamicsModel.state.u, Am.vehicleDynamicsModel.state.v, Am.vehicleDynamicsModel.state.w,
			   Am.vehicleDynamicsModel.state.p, Am.vehicleDynamicsModel.state.q, Am.vehicleDynamicsModel.state.r]
actualR = Am.vehicleDynamicsModel.state.R
expectedstate = [Am2.vehicleDynamicsModel.state.pn, Am2.vehicleDynamicsModel.state.pe, Am2.vehicleDynamicsModel.state.pd,
				 Am2.vehicleDynamicsModel.state.u, Am2.vehicleDynamicsModel.state.v, Am2.vehicleDynamicsModel.state.w,
				 Am2.vehicleDynamicsModel.state.p, Am2.vehicleDynamicsModel.state.q, Am2.vehicleDynamicsModel.state.r]
expectedR = Am2.vehicleDynamicsModel.state.R
if not evaluateTest(cur_test, (actualstate == expectedstate) and (actualR == expectedR)):
	print(f"{actualstate} != {expectedstate}")
	print(f"{actualR} != {exectedR}")

#testing getVehicleState
cur_test = "getVehicleState test 1"
#testting pn
Am.vehicleDynamicsModel.state.pn = 5		#setting pn to be 5
actual = Am.getVehicleState()				#putting pn into getVehicleState
state.pn = 5									#testing against pn from States.vehicleState = 5
if not evaluateTest(cur_test, (actual.pn == state.pn)):
	print(f"{actual.pn} != {state.pn}")

cur_test = "getVehicleState test 2"
#testting pe
Am.vehicleDynamicsModel.state.pe = 4		#setting pe to be 4
actual = Am.getVehicleState()				#putting pe into getVehicleState
state.pe = 4									#testing against pe from States.vehicleState = 4
if not evaluateTest(cur_test, (actual.pe == state.pe)):
	print(f"{actual.pe} != {state.pe}")

cur_test = "getVehicleState test 3"
#testting pd
Am.vehicleDynamicsModel.state.pd = 3		#setting pd to be 3
actual = Am.getVehicleState()				#putting pd into getVehicleState
state.pd = 3									#testing against pd from States.vehicleState = 3
if not evaluateTest(cur_test, (actual.pd == state.pd)):
	print(f"{actual.pd} != {state.pd}")

cur_test = "getVehicleState test 4"
#testting u
Am.vehicleDynamicsModel.state.u = 2			#setting u to be 2
actual = Am.getVehicleState()				#putting u into getVehicleState
state.u = 2									#testing against u from States.vehicleState = 2
if not evaluateTest(cur_test, (actual.u == state.u)):
	print(f"{actual.u} != {state.u}")

cur_test = "getVehicleState test 5"
#testting v
Am.vehicleDynamicsModel.state.v = 1			#setting v to be 1
actual = Am.getVehicleState()				#putting v into getVehicleState
state.v = 1									#testing against v from States.vehicleState = 1
if not evaluateTest(cur_test, (actual.v == state.v)):
	print(f"{actual.v} != {state.v}")

cur_test = "getVehicleState test 6"
#testting w
Am.vehicleDynamicsModel.state.w = 6			#setting w to be 6
actual = Am.getVehicleState()				#putting w into getVehicleState
state.w = 6									#testing against w from States.vehicleState = 6
if not evaluateTest(cur_test, (actual.w == state.w)):
	print(f"{actual.w} != {state.w}")

cur_test = "getVehicleState test 7"
#testting yaw
Am.vehicleDynamicsModel.state.yaw = 7		#setting yaw to be 7
actual = Am.getVehicleState()				#putting yaw into getVehicleState
state.yaw = 7									#testing against yaw from States.vehicleState = 7
if not evaluateTest(cur_test, (actual.yaw == state.yaw)):
	print(f"{actual.yaw} != {state.yaw}")

cur_test = "getVehicleState test 8"
#testting pitch
Am.vehicleDynamicsModel.state.pitch = 8		#setting pitch to be 8
actual = Am.getVehicleState()				#putting pitch into getVehicleState
state.pitch = 8								#testing against pitch from States.vehicleState = 8
if not evaluateTest(cur_test, (actual.pitch == state.pitch)):
	print(f"{actual.pitch} != {state.pitch}")

cur_test = "getVehicleState test 9"
#testting roll
Am.vehicleDynamicsModel.state.roll = 9		#setting roll to be 9
actual = Am.getVehicleState()				#putting roll into getVehicleState
state.roll = 9									#testing against roll from States.vehicleState = 9
if not evaluateTest(cur_test, (actual.roll == state.roll)):
	print(f"{actual.roll} != {state.roll}")

cur_test = "getVehicleState test 10"
#testting p
Am.vehicleDynamicsModel.state.p = 10		#setting p to be 10
actual = Am.getVehicleState()				#putting p into getVehicleState
state.p = 10									#testing against p from States.vehicleState = 10
if not evaluateTest(cur_test, (actual.p == state.p)):
	print(f"{actual.p} != {state.p}")

cur_test = "getVehicleState test 11"
#testting q
Am.vehicleDynamicsModel.state.q = 11		#setting q to be 11
actual = Am.getVehicleState()				#putting q into getVehicleState
state.q = 11									#testing against q from States.vehicleState = 11
if not evaluateTest(cur_test, (actual.q == state.q)):
	print(f"{actual.q} != {state.q}")

cur_test = "getVehicleState test 12"
#testting r
Am.vehicleDynamicsModel.state.r = 12		#setting r to be 12
actual = Am.getVehicleState()				#putting r into getVehicleState
state.r = 12									#testing against r from States.vehicleState = 12
if not evaluateTest(cur_test, (actual.r == state.r)):
	print(f"{actual.r} != {state.r}")


#testing setVehicleState
cur_test = "setVehicleState test 1"
#testting pn
Am.vehicleDynamicsModel.state.pn = 0			#setting pn to be 0
state = States.vehicleState()
state.pn = 0									#putting pn into getVehicleState
Am.setVehicleState(state)						#testing against pn from States.vehicleState = 0
if not evaluateTest(cur_test, (Am.vehicleDynamicsModel.state.pn == state.pn)):
	print(f"{Am.vehicleDynamicsModel.state.pn} != {state.pn}")

cur_test = "setVehicleState test 2"
#testting pe
Am.vehicleDynamicsModel.state.pe = 1			#setting pe to be 1
state = States.vehicleState()
state.pe = 1									#putting pe into getVehicleState
Am.setVehicleState(state)						#testing against pe from States.vehicleState = 1
if not evaluateTest(cur_test, (Am.vehicleDynamicsModel.state.pe == state.pe)):
	print(f"{Am.vehicleDynamicsModel.state.pe} != {state.pe}")

cur_test = "setVehicleState test 3"
#testting pe
#Am.vehicleDynamicsModel.state.pd = 3			#setting pd to be 3
state = States.vehicleState()
state.pd = 3									#putting pd into getVehicleState
Am.setVehicleState(state)						#testing against pd from States.vehicleState = 3
if not evaluateTest(cur_test, (Am.vehicleDynamicsModel.state.pd == state.pd)):
	print(f"{Am.vehicleDynamicsModel.state.pd} != {state.pd}")

cur_test = "setVehicleState test 4"
#testting u
Am.vehicleDynamicsModel.state.u = 4				#setting u to be 4
state = States.vehicleState()
state.u = 4										#putting u into getVehicleState
Am.setVehicleState(state)						#testing against u from States.vehicleState = 4
if not evaluateTest(cur_test, (Am.vehicleDynamicsModel.state.u == state.u)):
	print(f"{Am.vehicleDynamicsModel.state.pe} != {state.pe}")

cur_test = "setVehicleState test 5"
#testting v
Am.vehicleDynamicsModel.state.v = 5			#setting v to be 5
state = States.vehicleState()
state.v = 5										#putting v into getVehicleState
Am.setVehicleState(state)						#testing against v from States.vehicleState = 5
if not evaluateTest(cur_test, (Am.vehicleDynamicsModel.state.v == state.v)):
	print(f"{Am.vehicleDynamicsModel.state.v} != {state.v}")

cur_test = "setVehicleState test 6"
#testting w
Am.vehicleDynamicsModel.state.w = 6			#setting w to be 6
state = States.vehicleState()
state.w = 6									#putting w into getVehicleState
Am.setVehicleState(state)						#testing against w from States.vehicleState = 6
if not evaluateTest(cur_test, (Am.vehicleDynamicsModel.state.w == state.w)):
	print(f"{Am.vehicleDynamicsModel.state.w} != {state.w}")

cur_test = "setVehicleState test 7"
#testting yaw
Am.vehicleDynamicsModel.state.yaw = 7			#setting yaw to be 7
state = States.vehicleState()
state.yaw = 7									#putting yaw into getVehicleState
Am.setVehicleState(state)						#testing against yaw from States.vehicleState = 7
if not evaluateTest(cur_test, (Am.vehicleDynamicsModel.state.yaw == state.yaw)):
	print(f"{Am.vehicleDynamicsModel.state.yaw} != {state.yaw}")

cur_test = "setVehicleState test 8"
#testting pitch
Am.vehicleDynamicsModel.state.pitch = 8			#setting pitch to be 8
state = States.vehicleState()
state.pitch = 8									#putting pitch into getVehicleState
Am.setVehicleState(state)						#testing against pitch from States.vehicleState = 8
if not evaluateTest(cur_test, (Am.vehicleDynamicsModel.state.pitch == state.pitch)):
	print(f"{Am.vehicleDynamicsModel.state.pitch} != {state.pitch}")

cur_test = "setVehicleState test 9"
#testting roll
Am.vehicleDynamicsModel.state.roll = 9			#setting roll to be 9
state = States.vehicleState()
state.roll = 9									#putting roll into getVehicleState
Am.setVehicleState(state)						#testing against roll from States.vehicleState = 9
if not evaluateTest(cur_test, (Am.vehicleDynamicsModel.state.roll == state.roll)):
	print(f"{Am.vehicleDynamicsModel.state.roll} != {state.roll}")

cur_test = "setVehicleState test 10"
#testting p
Am.vehicleDynamicsModel.state.p = 10			#setting p to be 10
state = States.vehicleState()
state.p = 10									#putting p into getVehicleState
Am.setVehicleState(state)						#testing against p from States.vehicleState = 10
if not evaluateTest(cur_test, (Am.vehicleDynamicsModel.state.p == state.p)):
	print(f"{Am.vehicleDynamicsModel.state.p} != {state.p}")

cur_test = "setVehicleState test 11"
#testting q
Am.vehicleDynamicsModel.state.pe = 11			#setting q to be 11
state = States.vehicleState()
state.q = 11									#putting q into getVehicleState
Am.setVehicleState(state)						#testing against q from States.vehicleState = 11
if not evaluateTest(cur_test, (Am.vehicleDynamicsModel.state.q == state.q)):
	print(f"{Am.vehicleDynamicsModel.state.q} != {state.q}")

cur_test = "setVehicleState test 12"
#testting r
Am.vehicleDynamicsModel.state.r = 12			#setting r to be 12
state = States.vehicleState()
state.r = 12									#putting r into getVehicleState
Am.setVehicleState(state)						#testing against r from States.vehicleState = 12
if not evaluateTest(cur_test, (Am.vehicleDynamicsModel.state.r == state.r)):
	print(f"{Am.vehicleDynamicsModel.state.r} != {state.r}")


cur_test = "getVehicleDynamicsModel test 1"
#testting pn
state = States.vehicleState()
dot = States.vehicleState()
Am.vehicleDynamicsModel.state.pn = 1				#setting pn from state to be 1
Am.vehicleDynamicsModel.dot.pn = 2					#setting pn from dot to 2
actual = Am.getVehicleDynamicsModel()				#putting pn into getVehicleDynamicsModel
state.pn = 1										#testing against state.pn
dot.pn = 2											#testing aginst dot.pn
if not evaluateTest(cur_test, (actual.state.pn == state.pn and actual.dot.pn == dot.pn)):
	print(f"{actual.state.pn} != {state.pn}")
	print(f"{actual.dot.pn} != {dot.pn}")

cur_test = "getVehicleDynamicsModel test 2"
#testting pe
state = States.vehicleState()
dot = States.vehicleState()
Am.vehicleDynamicsModel.state.pe = 3				#setting pe from state to be 3
Am.vehicleDynamicsModel.dot.pe = 4					#setting pe from dot to 4
actual = Am.getVehicleDynamicsModel()				#putting pe into getVehicleDynamicsModel
state.pe = 3										#testing against state.pe
dot.pe = 4											#testing aginst dot.pe
if not evaluateTest(cur_test, (actual.state.pe == state.pe and actual.dot.pe == dot.pe)):
	print(f"{actual.state.pe} != {state.pe}")
	print(f"{actual.dot.pe} != {dot.pe}")

cur_test = "getVehicleDynamicsModel test 3"
#testting pd
state = States.vehicleState()
dot = States.vehicleState()
Am.vehicleDynamicsModel.state.pd = 5				#setting pn from state to be 5
Am.vehicleDynamicsModel.dot.pd = 6					#setting pn from dot to 6
actual = Am.getVehicleDynamicsModel()				#putting pn into getVehicleDynamicsModel
state.pd = 5										#testing against state.pd
dot.pd = 6											#testing aginst dot.pd
if not evaluateTest(cur_test, (actual.state.pd == state.pd and actual.dot.pd == dot.pd)):
	print(f"{actual.state.pd} != {state.pd}")
	print(f"{actual.dot.pd} != {dot.pd}")

cur_test = "getVehicleDynamicsModel test 4"
#testting u
state = States.vehicleState()
dot = States.vehicleState()
Am.vehicleDynamicsModel.state.u = 7				#setting u from state to be 7
Am.vehicleDynamicsModel.dot.u = 8					#setting u from dot to 6
actual = Am.getVehicleDynamicsModel()				#putting u into getVehicleDynamicsModel
state.u = 7										#testing against state.u
dot.u = 8											#testing aginst dot.u
if not evaluateTest(cur_test, (actual.state.u == state.u and actual.dot.u == dot.u)):
	print(f"{actual.state.u} != {state.u}")
	print(f"{actual.dot.u} != {dot.u}")

cur_test = "getVehicleDynamicsModel test 5"
#testting v
state = States.vehicleState()
dot = States.vehicleState()
Am.vehicleDynamicsModel.state.v = 9				#setting v from state to be 9
Am.vehicleDynamicsModel.dot.v = 10					#setting v from dot to 10
actual = Am.getVehicleDynamicsModel()				#putting v into getVehicleDynamicsModel
state.v = 9										#testing against state.v
dot.v = 10											#testing aginst dot.v
if not evaluateTest(cur_test, (actual.state.v == state.v and actual.dot.v == dot.v)):
	print(f"{actual.state.v} != {state.v}")
	print(f"{actual.dot.v} != {dot.v}")

cur_test = "getVehicleDynamicsModel test 6"
#testting r
state = States.vehicleState()
dot = States.vehicleState()
Am.vehicleDynamicsModel.state.r = 11				#setting r from state to be 11
Am.vehicleDynamicsModel.dot.r = 12					#setting r from dot to 12
actual = Am.getVehicleDynamicsModel()				#putting r into getVehicleDynamicsModel
state.r = 11										#testing against state.r
dot.r = 12											#testing aginst dot.r
if not evaluateTest(cur_test, (actual.state.r == state.r and actual.dot.r == dot.r)):
	print(f"{actual.state.r} != {state.r}")
	print(f"{actual.dot.r} != {dot.r}")

cur_test = "getVehicleDynamicsModel test 7"
#testting yaw
state = States.vehicleState()
dot = States.vehicleState()
Am.vehicleDynamicsModel.state.yaw = 13				#setting yaw from state to be 13
Am.vehicleDynamicsModel.dot.yaw = 14					#setting yaw from dot to 14
actual = Am.getVehicleDynamicsModel()				#putting yaw into getVehicleDynamicsModel
state.yaw = 13										#testing against state.yaw
dot.yaw = 14											#testing aginst dot.yaw
if not evaluateTest(cur_test, (actual.state.yaw == state.yaw and actual.dot.yaw == dot.yaw)):
	print(f"{actual.state.yaw} != {state.yaw}")
	print(f"{actual.dot.yaw} != {dot.yaw}")

cur_test = "getVehicleDynamicsModel test 8"
#testting pitch
state = States.vehicleState()
dot = States.vehicleState()
Am.vehicleDynamicsModel.state.pitch = 15				#setting pitch from state to be 15
Am.vehicleDynamicsModel.dot.pitch = 16					#setting pn from dot to 16
actual = Am.getVehicleDynamicsModel()				#putting pn into getVehicleDynamicsModel
state.pitch = 15										#testing against state.pitch
dot.pitch = 16											#testing aginst dot.pitch
if not evaluateTest(cur_test, (actual.state.pitch == state.pitch and actual.dot.pitch == dot.pitch)):
	print(f"{actual.state.pitch} != {state.pitch}")
	print(f"{actual.dot.pitch} != {dot.pitch}")

cur_test = "getVehicleDynamicsModel test 9"
#testting roll
state = States.vehicleState()
dot = States.vehicleState()
Am.vehicleDynamicsModel.state.roll = 17				#setting roll from state to be 17
Am.vehicleDynamicsModel.dot.roll = 18					#setting roll from dot to 18
actual = Am.getVehicleDynamicsModel()				#putting pn into getVehicleDynamicsModel
state.roll = 17											#testing against state.roll
dot.roll = 18											#testing aginst dot.roll
if not evaluateTest(cur_test, (actual.state.roll == state.roll and actual.dot.roll == dot.roll)):
	print(f"{actual.state.roll} != {state.roll}")
	print(f"{actual.dot.roll} != {dot.roll}")

cur_test = "getVehicleDynamicsModel test 10"
#testting p
state = States.vehicleState()
dot = States.vehicleState()
Am.vehicleDynamicsModel.state.p = 19				#setting p from state to be 19
Am.vehicleDynamicsModel.dot.p = 20					#setting p from dot to 20
actual = Am.getVehicleDynamicsModel()				#putting p into getVehicleDynamicsModel
state.p = 19										#testing against state.p
dot.p = 20											#testing aginst dot.p
if not evaluateTest(cur_test, (actual.state.p == state.p and actual.dot.p == dot.p)):
	print(f"{actual.state.p} != {state.p}")
	print(f"{actual.dot.p} != {dot.p}")

cur_test = "getVehicleDynamicsModel test 11"
#testting q
state = States.vehicleState()
dot = States.vehicleState()
Am.vehicleDynamicsModel.state.q = 21				#setting q from state to be 21
Am.vehicleDynamicsModel.dot.q = 22					#setting q from dot to 22
actual = Am.getVehicleDynamicsModel()				#putting q into getVehicleDynamicsModel
state.q = 21										#testing against state.q
dot.q = 22											#testing aginst dot.q
if not evaluateTest(cur_test, (actual.state.q == state.q and actual.dot.q == dot.q)):
	print(f"{actual.state.q} != {state.q}")
	print(f"{actual.dot.q} != {dot.q}")

cur_test = "getVehicleDynamicsModel test 12"
#testting r
state = States.vehicleState()
dot = States.vehicleState()
Am.vehicleDynamicsModel.state.r = 23				#setting r from state to be 23
Am.vehicleDynamicsModel.dot.r = 24					#setting r from dot to 24
actual = Am.getVehicleDynamicsModel()				#putting r into getVehicleDynamicsModel
state.r = 23										#testing against state.r
dot.r = 24											#testing aginst dot.r
if not evaluateTest(cur_test, (actual.state.r == state.r and actual.dot.r == dot.r)):
	print(f"{actual.state.r} != {state.r}")
	print(f"{actual.dot.r} != {dot.r}")


cur_test = "gravityForces test 1"
#testing gravity changing all three Euler Angles
state.yaw = 90*math.pi/180
state.pitch = 45*math.pi/180
state.roll = -90*math.pi/180
#plugging in new euler angles into the state
state.R = Rotations.euler2DCM(state.yaw, state.pitch, state.roll)
#initiating the function gravityForces
x = Am.gravityForces(state)
#setting the original vector to the mass times gravity and multiply times R
orig_vec = [[0],[0],[11*9.81]]
expected_vec = mm.matrixMultiply(state.R,orig_vec)
#testing against the vector created by gravityForces()
actual_vec = [[x.Fx], [x.Fy], [x.Fz]]
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")

cur_test = "gravityForces test 2"
#testing gravity changing all three Euler Angles
state.yaw = 37*math.pi/180
state.pitch = 0*math.pi/180
state.roll = -24*math.pi/180
#plugging in new euler angles into the state
state.R = Rotations.euler2DCM(state.yaw, state.pitch, state.roll)
#initiating the function gravityForces
x = Am.gravityForces(state)
#setting the original vector to the mass times gravity and multiply times R
orig_vec = [[0],[0],[11*9.81]]
expected_vec = mm.matrixMultiply(state.R,orig_vec)
#testing against the vector created by gravityForces()
actual_vec = [[x.Fx], [x.Fy], [x.Fz]]
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")

cur_test = "gravityForces test 3"
#testing gravity changing all three Euler Angles
state.yaw = 0*math.pi/180
state.pitch = 0*math.pi/180
state.roll = -1*math.pi/180
#plugging in new euler angles into the state
state.R = Rotations.euler2DCM(state.yaw, state.pitch, state.roll)
#initiating the function gravityForces
x = Am.gravityForces(state)
#setting the original vector to the mass times gravity and multiply times R
orig_vec = [[0],[0],[11*9.81]]
expected_vec = mm.matrixMultiply(state.R,orig_vec)
#testing against the vector created by gravityForces()
actual_vec = [[x.Fx], [x.Fy], [x.Fz]]
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")


cur_test = "CalculateCoeff_alpha test 1"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
#seting alpha to 0
state.alpha = 0
#the actual values
C_L, C_D, C_M = Am.CalculateCoeff_alpha(state.alpha)
#expected values that were calculated by hand
expected_CL = 0.229999999973088
expected_CD = 0.0012272946567217947
expected_CM = 0.0135
if not evaluateTest(cur_test, isclose(C_L, expected_CL) and isclose(C_D, expected_CD) and isclose(C_M, expected_CM)):
	print(f"{C_L} != {expected_CL}")
	print(f"{C_D} != {expected_CD}")
	print(f"{C_M} != {expected_CM}")

cur_test = "CalculateCoeff_alpha test 2"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
#seting alpha to 27
state.alpha = (27*math.pi/180)
#the actual values
C_L, C_D, C_M = Am.CalculateCoeff_alpha(state.alpha)
#expected values that were calculated by hand
expected_CL = 1.8413336061853793
expected_CD = 0.3018997585209241
expected_CM = -1.277694580625405
if not evaluateTest(cur_test, isclose(C_L, expected_CL) and isclose(C_D, expected_CD) and isclose(C_M, expected_CM)):
	print(f"{C_L} != {expected_CL}")
	print(f"{C_D} != {expected_CD}")
	print(f"{C_M} != {expected_CM}")

cur_test = "CalculateCoeff_alpha test 3"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
#seting alpha to 27
state.alpha = (90*math.pi/180)
#the actual values
C_L, C_D, C_M = Am.CalculateCoeff_alpha(state.alpha)
#expected values that were calculated by hand
expected_CL = 0
expected_CD = 2
expected_CM = -4.2904819354180175
if not evaluateTest(cur_test, isclose(C_L, expected_CL) and isclose(C_D, expected_CD) and isclose(C_M, expected_CM)):
	print(f"{C_L} != {expected_CL}")
	print(f"{C_D} != {expected_CD}")
	print(f"{C_M} != {expected_CM}")

cur_test = "CalculateCoeff_alpha test 4"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
#seting alpha to 27
state.alpha = (-27*math.pi/180)
#the actual values
C_L, C_D, C_M = Am.CalculateCoeff_alpha(state.alpha)
#expected values that were calculated by hand
expected_CL = -1.6113336061853794
expected_CD = 0.2736863864585998
expected_CM = 1.304694580625405
if not evaluateTest(cur_test, isclose(C_L, expected_CL) and isclose(C_D, expected_CD) and isclose(C_M, expected_CM)):
	print(f"{C_L} != {expected_CL}")
	print(f"{C_D} != {expected_CD}")
	print(f"{C_M} != {expected_CM}")

cur_test = "CalculateCoeff_alpha test 5"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
#seting alpha to 27
state.alpha = (-15*math.pi/180)
#the actual values
C_L, C_D, C_M = Am.CalculateCoeff_alpha(state.alpha)
#expected values that were calculated by hand
expected_CL = -1.2386736470105295
expected_CD = 0.035600461045418856
expected_CM = 0.7308303225696694
if not evaluateTest(cur_test, isclose(C_L, expected_CL) and isclose(C_D, expected_CD) and isclose(C_M, expected_CM)):
	print(f"{C_L} != {expected_CL}")
	print(f"{C_D} != {expected_CD}")
	print(f"{C_M} != {expected_CM}")

cur_test = "CalculateCoeff_alpha test 6"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
#seting alpha to 27
state.alpha = (45*math.pi/180)
#the actual values
C_L, C_D, C_M = Am.CalculateCoeff_alpha(state.alpha)
#expected values that were calculated by hand
expected_CL = 1.0000005479640122
expected_CD = 0.9999999244456799
expected_CM = -2.1384909677090085
if not evaluateTest(cur_test, isclose(C_L, expected_CL) and isclose(C_D, expected_CD) and isclose(C_M, expected_CM)):
	print(f"{C_L} != {expected_CL}")
	print(f"{C_D} != {expected_CD}")
	print(f"{C_M} != {expected_CM}")


cur_test = "aeroforces test 1"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
#setting values for p,q,r,alpha,beta,Va
state.q = 0
state.p = 4
state.r = 8
state.Va = 5
state.alpha = 0
state.beta = 6
#creating an instance of aeroForces with the components define above
expected = Am.aeroForces(state)
#calculated vlaues that were calculated by hand
actualFx = -0.010700628700125238
actualFy = -51.266985000000005
actualFz = -2.005341249765364
actualMx = -19.984585678429678
actualMy = 0.02235685208625
actualMz = 7.519707359200871
if not evaluateTest(cur_test, isclose(actualFx, expected.Fx) and isclose(actualFy, expected.Fy) and isclose(actualFz, expected.Fz)
							  and isclose(actualMx, expected.Mx) and isclose(actualMy, expected.My) and isclose(actualMz, expected.Mz)):
	print(f"{actualFx} != {expected.Fx}")
	print(f"{actualFy} != {expected.Fy}")
	print(f"{actualFz} != {expected.Fz}")
	print(f"{actualMx} != {expected.Mx}")
	print(f"{actualMy} != {expected.My}")
	print(f"{actualMz} != {expected.Mz}")

cur_test = "aeroforces test 2"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
#setting values for p,q,r,alpha,beta,Va
state.q = 5
state.p = 1
state.r = 0
state.Va = 0
state.alpha = 45*math.pi/180
state.beta = 0
#creating an instance of aeroForces with the components define above
expected = Am.aeroForces(state)
#calculated vlaues that were calculated by hand
actualFx = 0
actualFy = 0
actualFz = 0
actualMx = 0
actualMy = 0
actualMz = 0
if not evaluateTest(cur_test, isclose(actualFx, expected.Fx) and isclose(actualFy, expected.Fy) and isclose(actualFz, expected.Fz)
							  and isclose(actualMx, expected.Mx) and isclose(actualMy, expected.My) and isclose(actualMz, expected.Mz)):
	print(f"{actualFx} != {expected.Fx}")
	print(f"{actualFy} != {expected.Fy}")
	print(f"{actualFz} != {expected.Fz}")
	print(f"{actualMx} != {expected.Mx}")
	print(f"{actualMy} != {expected.My}")
	print(f"{actualMz} != {expected.Mz}")

cur_test = "aeroforces test 3"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
#setting values for p,q,r,alpha,beta,Va
state.q = 7
state.p = 1
state.r = 1
state.Va = 1
state.alpha = 90*math.pi/180
state.beta = 8
#creating an instance of aeroForces with the components define above
expected = Am.aeroForces(state)
#calculated vlaues that were calculated by hand
actualFx = 1.8431982497775004
actualFy = -2.7342392
actualFz = -0.6975100000000002
actualMx = -1.4303868667785842
actualMy = -1.9668801866827414
actualMz = 0.5517415381861417
if not evaluateTest(cur_test, isclose(actualFx, expected.Fx) and isclose(actualFy, expected.Fy) and isclose(actualFz, expected.Fz)
							  and isclose(actualMx, expected.Mx) and isclose(actualMy, expected.My) and isclose(actualMz, expected.Mz)):
	print(f"{actualFx} != {expected.Fx}")
	print(f"{actualFy} != {expected.Fy}")
	print(f"{actualFz} != {expected.Fz}")
	print(f"{actualMx} != {expected.Mx}")
	print(f"{actualMy} != {expected.My}")
	print(f"{actualMz} != {expected.Mz}")


cur_test = "CalculatePropForce test 1"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
#creating an instance of calculatePropForces
F, M = Am.CalculatePropForces(0, 0)
#expected values calculated by hand
expF = 0.00018320594739300043
expM = -5.201975946047016e-06
if not evaluateTest(cur_test, isclose(expF, F) and isclose(expM, M)):
	print(f"{expF} != {F}")
	print(f"{expM} != {M}")

cur_test = "CalculatePropForce test 2"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
#creating an instance of calculatePropForces
F, M = Am.CalculatePropForces(1, 0)
#expected values calculated by hand
expF = -0.033654137677839
expM = 0.0028236828549583586
if not evaluateTest(cur_test, isclose(expF, F) and isclose(expM, M)):
	print(f"{expF} != {F}")
	print(f"{expM} != {M}")

cur_test = "CalculatePropForce test 3"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
#creating an instance of calculatePropForces
F, M = Am.CalculatePropForces(0, 1)
#expected values calculated by hand
expF = 84.56952909917005
expM = -2.401279338375964
if not evaluateTest(cur_test, isclose(expF, F) and isclose(expM, M)):
	print(f"{expF} != {F}")
	print(f"{expM} != {M}")

cur_test = "CalculatePropForce test 4"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
#creating an instance of calculatePropForces
F, M = Am.CalculatePropForces(5, 5)
#expected values calculated by hand
expF = 1658.0088332902185
expM = -48.66009127352194
if not evaluateTest(cur_test, isclose(expF, F) and isclose(expM, M)):
	print(f"{expF} != {F}")
	print(f"{expM} != {M}")

cur_test = "CalculatePropForce test 5"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
#creating an instance of calculatePropForces
F, M = Am.CalculatePropForces(-2, .7)
#expected values calculated by hand
expF = 43.69322756427685
expM = -1.13033097294558
if not evaluateTest(cur_test, isclose(expF, F) and isclose(expM, M)):
	print(f"{expF} != {F}")
	print(f"{expM} != {M}")


cur_test = "controlForces test 1"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
state.Va = 0
state.alpha = 0
control = Inputs.controlInputs(Throttle = 0, Aileron = 0, Elevator = 0, Rudder = 0)
force = Am.controlForces(state, control)
F, M = Am.CalculatePropForces(state.Va, control.Throttle)
actualFx = force.Fx - F
actualFz = force.Fz
actualFy = force.Fy
actualMx = force.Mx - M
actualMy = force.My
actualMz = force.Mz
expectedFx = 0
expectedFz = 0
expectedFy = 0
expectedMx = 0
expectedMy = 0
expectedMz = 0
if not evaluateTest(cur_test, isclose(actualFx, expectedFx) and isclose(actualFy, expectedFy) and isclose(actualFz, expectedFz)
							  and isclose(actualMx, expectedMx) and isclose(actualMy, expectedMy) and isclose(actualMz, expectedMz)):
	print(f"{actualFx} != {expectedFx}")
	print(f"{actualFy} != {expectedFy}")
	print(f"{actualFz} != {expectedFz}")
	print(f"{actualMx} != {expectedMx}")
	print(f"{actualMy} != {expectedMy}")
	print(f"{actualMz} != {expectedMz}")

cur_test = "controlForces test 2"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
state.Va = 1
state.alpha = 0
control = Inputs.controlInputs(Throttle = 0, Aileron = 0, Elevator = 0, Rudder = 0)
force = Am.controlForces(state, control)
F, M = Am.CalculatePropForces(state.Va, control.Throttle)
actualFx = force.Fx - F
actualFz = force.Fz
actualFy = force.Fy
actualMx = force.Mx - M
actualMy = force.My
actualMz = force.Mz
expectedFx = 0
expectedFz = 0
expectedFy = 0
expectedMx = 0
expectedMy = 0
expectedMz = 0
if not evaluateTest(cur_test, isclose(actualFx, expectedFx) and isclose(actualFy, expectedFy) and isclose(actualFz, expectedFz)
							  and isclose(actualMx, expectedMx) and isclose(actualMy, expectedMy) and isclose(actualMz, expectedMz)):
	print(f"{actualFx} != {expectedFx}")
	print(f"{actualFy} != {expectedFy}")
	print(f"{actualFz} != {expectedFz}")
	print(f"{actualMx} != {expectedMx}")
	print(f"{actualMy} != {expectedMy}")
	print(f"{actualMz} != {expectedMz}")

cur_test = "controlForces test 3"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
state.Va = 0
state.alpha = 1
control = Inputs.controlInputs(Throttle = 0, Aileron = 0, Elevator = 0, Rudder = 0)
force = Am.controlForces(state, control)
F, M = Am.CalculatePropForces(state.Va, control.Throttle)
actualFx = force.Fx - F
actualFz = force.Fz
actualFy = force.Fy
actualMx = force.Mx - M
actualMy = force.My
actualMz = force.Mz
expectedFx = 0
expectedFz = 0
expectedFy = 0
expectedMx = 0
expectedMy = 0
expectedMz = 0
if not evaluateTest(cur_test, isclose(actualFx, expectedFx) and isclose(actualFy, expectedFy) and isclose(actualFz, expectedFz)
							  and isclose(actualMx, expectedMx) and isclose(actualMy, expectedMy) and isclose(actualMz, expectedMz)):
	print(f"{actualFx} != {expectedFx}")
	print(f"{actualFy} != {expectedFy}")
	print(f"{actualFz} != {expectedFz}")
	print(f"{actualMx} != {expectedMx}")
	print(f"{actualMy} != {expectedMy}")
	print(f"{actualMz} != {expectedMz}")

cur_test = "controlForces test 4"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
state.Va = 1
state.alpha = 1
control = Inputs.controlInputs(Throttle = 0, Aileron = 0, Elevator = 0, Rudder = 0)
force = Am.controlForces(state, control)
F, M = Am.CalculatePropForces(state.Va, control.Throttle)
actualFx = force.Fx - F
actualFz = force.Fz
actualFy = force.Fy
actualMx = force.Mx - M
actualMy = force.My
actualMz = force.Mz
expectedFx = 0
expectedFz = 0
expectedFy = 0
expectedMx = 0
expectedMy = 0
expectedMz = 0
if not evaluateTest(cur_test, isclose(actualFx, expectedFx) and isclose(actualFy, expectedFy) and isclose(actualFz, expectedFz)
							  and isclose(actualMx, expectedMx) and isclose(actualMy, expectedMy) and isclose(actualMz, expectedMz)):
	print(f"{actualFx} != {expectedFx}")
	print(f"{actualFy} != {expectedFy}")
	print(f"{actualFz} != {expectedFz}")
	print(f"{actualMx} != {expectedMx}")
	print(f"{actualMy} != {expectedMy}")
	print(f"{actualMz} != {expectedMz}")

cur_test = "controlForces test 5"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
state.Va = 1
state.alpha = 0
control = Inputs.controlInputs(Throttle = 0, Aileron = 1, Elevator = 0, Rudder = 0)
force = Am.controlForces(state, control)
F, M = Am.CalculatePropForces(state.Va, control.Throttle)
actualFx = force.Fx - F
actualFz = force.Fz
actualFy = force.Fy
actualMx = force.Mx - M
actualMy = force.My
actualMz = force.Mz
expectedFx = 0
expectedFy = 0.026156625000000003
expectedFz = 0
expectedMx = 0.17167534626000003
expectedMy = 0
expectedMz = -0.011108404758
if not evaluateTest(cur_test, isclose(actualFx, expectedFx) and isclose(actualFy, expectedFy) and isclose(actualFz, expectedFz)
							  and isclose(actualMx, expectedMx) and isclose(actualMy, expectedMy) and isclose(actualMz, expectedMz)):
	print(f"{actualFx} != {expectedFx}")
	print(f"{actualFy} != {expectedFy}")
	print(f"{actualFz} != {expectedFz}")
	print(f"{actualMx} != {expectedMx}")
	print(f"{actualMy} != {expectedMy}")
	print(f"{actualMz} != {expectedMz}")

cur_test = "controlForces test 6"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
state.Va = 0
state.alpha = 1
control = Inputs.controlInputs(Throttle = 1, Aileron = 0, Elevator = 0, Rudder = 0)
force = Am.controlForces(state, control)
F, M = Am.CalculatePropForces(state.Va, control.Throttle)
actualFx = force.Fx - F
actualFz = force.Fz
actualFy = force.Fy
actualMx = force.Mx - M
actualMy = force.My
actualMz = force.Mz
expectedFx = 0
expectedFy = 0
expectedFz = 0
expectedMx = 0
expectedMy = 0
expectedMz = 0
if not evaluateTest(cur_test, isclose(actualFx, expectedFx) and isclose(actualFy, expectedFy) and isclose(actualFz, expectedFz)
							  and isclose(actualMx, expectedMx) and isclose(actualMy, expectedMy) and isclose(actualMz, expectedMz)):
	print(f"{actualFx} != {expectedFx}")
	print(f"{actualFy} != {expectedFy}")
	print(f"{actualFz} != {expectedFz}")
	print(f"{actualMx} != {expectedMx}")
	print(f"{actualMy} != {expectedMy}")
	print(f"{actualMz} != {expectedMz}")

cur_test = "controlForces test 7"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
state.Va = 1
state.alpha = 1
control = Inputs.controlInputs(Throttle = 0, Aileron = 1, Elevator = 1, Rudder = 0)
force = Am.controlForces(state, control)
F, M = Am.CalculatePropForces(state.Va, control.Throttle)
actualFx = force.Fx - F
actualFz = force.Fz
actualFy = force.Fy
actualMx = force.Mx - M
actualMy = force.My
actualMz = force.Mz
expectedFx = 0.03560689046564706
expectedFy = 0.026156625000000003
expectedFz = -0.028458114368435757
expectedMx = 0.17167534626000003
expectedMy = -0.065580099453
expectedMz = -0.011108404758
if not evaluateTest(cur_test, isclose(actualFx, expectedFx) and isclose(actualFy, expectedFy) and isclose(actualFz, expectedFz)
							  and isclose(actualMx, expectedMx) and isclose(actualMy, expectedMy) and isclose(actualMz, expectedMz)):
	print(f"{actualFx} != {expectedFx}")
	print(f"{actualFy} != {expectedFy}")
	print(f"{actualFz} != {expectedFz}")
	print(f"{actualMx} != {expectedMx}")
	print(f"{actualMy} != {expectedMy}")
	print(f"{actualMz} != {expectedMz}")

cur_test = "controlForces test 8"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
state.Va = 6
state.alpha = 5
control = Inputs.controlInputs(Throttle = 4, Aileron = 3, Elevator = 2, Rudder = 1)
force = Am.controlForces(state, control)
F, M = Am.CalculatePropForces(state.Va, control.Throttle)
actualFx = force.Fx - F
actualFz = force.Fz
actualFy = force.Fy
actualMx = force.Mx - M
actualMy = force.My
actualMz = force.Mz
expectedFx = -3.2264199919763996
expectedFy = 5.210399700000001
expectedFz = -0.6009061417792305
expectedMx = 18.6281888661792
expectedMy = -4.721767160616
expectedMz = -3.708187479216001
if not evaluateTest(cur_test, isclose(actualFx, expectedFx) and isclose(actualFy, expectedFy) and isclose(actualFz, expectedFz)
							  and isclose(actualMx, expectedMx) and isclose(actualMy, expectedMy) and isclose(actualMz, expectedMz)):
	print(f"{actualFx} != {expectedFx}")
	print(f"{actualFy} != {expectedFy}")
	print(f"{actualFz} != {expectedFz}")
	print(f"{actualMx} != {expectedMx}")
	print(f"{actualMy} != {expectedMy}")
	print(f"{actualMz} != {expectedMz}")


cur_test = "updateForces test 1"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
state.u = 0
state.v = 0
state.w = 0
control = Inputs.controlInputs(Throttle = 0, Aileron = 0, Elevator = 0, Rudder = 0)
Am.updateForces(state, control)
actualVa = state.Va
expectedVa = 0
actualalpha = state.alpha
expectedalpha = 0
actualbeta = state.beta
expectedbeta = 0
x = Am.updateForces(state, control)
g = Am.gravityForces(state)
a = Am.aeroForces(state)
cf = Am.controlForces(state, control)
expectedFx = g.Fx + a.Fx + cf.Fx
actualFx = x.Fx
expectedFy = g.Fy + a.Fy + cf.Fy
actualFy = x.Fy
expectedFz = g.Fz + a.Fz + cf.Fz
actualFz = x.Fz
expectedMx = g.Mx + a.Mx + cf.Mx
actualMx = x.Mx
expectedMy = g.My + a.My + cf.My
actualMy = x.My
expectedMz = g.Mz + a.Mz + cf.Mz
actualMz = x.Mz
if not evaluateTest(cur_test, isclose(actualFx, expectedFx) and isclose(actualFy, expectedFy) and isclose(actualFz, expectedFz)
							  and isclose(actualMx, expectedMx) and isclose(actualMy, expectedMy) and isclose(actualMz, expectedMz) and
							  isclose(actualVa, expectedVa) and isclose(actualalpha, expectedalpha) and isclose(actualbeta, expectedbeta)):
	print(f"{actualFx} != {expectedFx}")
	print(f"{actualFy} != {expectedFy}")
	print(f"{actualFz} != {expectedFz}")
	print(f"{actualMx} != {expectedMx}")
	print(f"{actualMy} != {expectedMy}")
	print(f"{actualMz} != {expectedMz}")
	print(f"{actualVa} != {expectedVa}")
	print(f"{actualalpha} != {expectedalpha}")
	print(f"{actualbeta} != {expectedbeta}")

cur_test = "updateForces test 2"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
state.u = 1
state.v = 1
state.w = 1
control = Inputs.controlInputs(Throttle = 0, Aileron = 0, Elevator = 0, Rudder = 0)
Am.updateForces(state, control)
actualVa = state.Va
expectedVa = 1.7320508075688772
actualalpha = state.alpha
expectedalpha = 0.7853981633974483
actualbeta = state.beta
expectedbeta = 0.6154797086703875
x = Am.updateForces(state, control)
g = Am.gravityForces(state)
a = Am.aeroForces(state)
cf = Am.controlForces(state, control)
expectedFx = g.Fx + a.Fx + cf.Fx
actualFx = x.Fx
expectedFy = g.Fy + a.Fy + cf.Fy
actualFy = x.Fy
expectedFz = g.Fz + a.Fz + cf.Fz
actualFz = x.Fz
expectedMx = g.Mx + a.Mx + cf.Mx
actualMx = x.Mx
expectedMy = g.My + a.My + cf.My
actualMy = x.My
expectedMz = g.Mz + a.Mz + cf.Mz
actualMz = x.Mz
if not evaluateTest(cur_test, isclose(actualFx, expectedFx) and isclose(actualFy, expectedFy) and isclose(actualFz, expectedFz)
							  and isclose(actualMx, expectedMx) and isclose(actualMy, expectedMy) and isclose(actualMz, expectedMz) and
							  isclose(actualVa, expectedVa) and isclose(actualalpha, expectedalpha) and isclose(actualbeta, expectedbeta)):
	print(f"{actualFx} != {expectedFx}")
	print(f"{actualFy} != {expectedFy}")
	print(f"{actualFz} != {expectedFz}")
	print(f"{actualMx} != {expectedMx}")
	print(f"{actualMy} != {expectedMy}")
	print(f"{actualMz} != {expectedMz}")
	print(f"{actualVa} != {expectedVa}")
	print(f"{actualalpha} != {expectedalpha}")
	print(f"{actualbeta} != {expectedbeta}")

cur_test = "updateForces test 3"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
state.u = 1
state.v = 1
state.w = 1
control = Inputs.controlInputs(Throttle = 0, Aileron = 1, Elevator = 0, Rudder = 0)
Am.updateForces(state, control)
actualVa = state.Va
expectedVa = 1.7320508075688772
actualalpha = state.alpha
expectedalpha = 0.7853981633974483
actualbeta = state.beta
expectedbeta = 0.6154797086703875
x = Am.updateForces(state, control)
g = Am.gravityForces(state)
a = Am.aeroForces(state)
cf = Am.controlForces(state, control)
expectedFx = g.Fx + a.Fx + cf.Fx
actualFx = x.Fx
expectedFy = g.Fy + a.Fy + cf.Fy
actualFy = x.Fy
expectedFz = g.Fz + a.Fz + cf.Fz
actualFz = x.Fz
expectedMx = g.Mx + a.Mx + cf.Mx
actualMx = x.Mx
expectedMy = g.My + a.My + cf.My
actualMy = x.My
expectedMz = g.Mz + a.Mz + cf.Mz
actualMz = x.Mz
if not evaluateTest(cur_test, isclose(actualFx, expectedFx) and isclose(actualFy, expectedFy) and isclose(actualFz, expectedFz)
							  and isclose(actualMx, expectedMx) and isclose(actualMy, expectedMy) and isclose(actualMz, expectedMz) and
							  isclose(actualVa, expectedVa) and isclose(actualalpha, expectedalpha) and isclose(actualbeta, expectedbeta)):
	print(f"{actualFx} != {expectedFx}")
	print(f"{actualFy} != {expectedFy}")
	print(f"{actualFz} != {expectedFz}")
	print(f"{actualMx} != {expectedMx}")
	print(f"{actualMy} != {expectedMy}")
	print(f"{actualMz} != {expectedMz}")
	print(f"{actualVa} != {expectedVa}")
	print(f"{actualalpha} != {expectedalpha}")
	print(f"{actualbeta} != {expectedbeta}")

cur_test = "updateForces test 4"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
state.u = 1
state.v = 1
state.w = 1
control = Inputs.controlInputs(Throttle = 1, Aileron = 1, Elevator = 0, Rudder = 0)
Am.updateForces(state, control)
actualVa = state.Va
expectedVa = 1.7320508075688772
actualalpha = state.alpha
expectedalpha = 0.7853981633974483
actualbeta = state.beta
expectedbeta = 0.6154797086703875
x = Am.updateForces(state, control)
g = Am.gravityForces(state)
a = Am.aeroForces(state)
cf = Am.controlForces(state, control)
expectedFx = g.Fx + a.Fx + cf.Fx
actualFx = x.Fx
expectedFy = g.Fy + a.Fy + cf.Fy
actualFy = x.Fy
expectedFz = g.Fz + a.Fz + cf.Fz
actualFz = x.Fz
expectedMx = g.Mx + a.Mx + cf.Mx
actualMx = x.Mx
expectedMy = g.My + a.My + cf.My
actualMy = x.My
expectedMz = g.Mz + a.Mz + cf.Mz
actualMz = x.Mz
if not evaluateTest(cur_test, isclose(actualFx, expectedFx) and isclose(actualFy, expectedFy) and isclose(actualFz, expectedFz)
							  and isclose(actualMx, expectedMx) and isclose(actualMy, expectedMy) and isclose(actualMz, expectedMz) and
							  isclose(actualVa, expectedVa) and isclose(actualalpha, expectedalpha) and isclose(actualbeta, expectedbeta)):
	print(f"{actualFx} != {expectedFx}")
	print(f"{actualFy} != {expectedFy}")
	print(f"{actualFz} != {expectedFz}")
	print(f"{actualMx} != {expectedMx}")
	print(f"{actualMy} != {expectedMy}")
	print(f"{actualMz} != {expectedMz}")
	print(f"{actualVa} != {expectedVa}")
	print(f"{actualalpha} != {expectedalpha}")
	print(f"{actualbeta} != {expectedbeta}")

cur_test = "updateForces test 5"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
state.u = 1
state.v = 1
state.w = 1
control = Inputs.controlInputs(Throttle = 1, Aileron = 1, Elevator = 1, Rudder = 1)
Am.updateForces(state, control)
actualVa = state.Va
expectedVa = 1.7320508075688772
actualalpha = state.alpha
expectedalpha = 0.7853981633974483
actualbeta = state.beta
expectedbeta = 0.6154797086703875
x = Am.updateForces(state, control)
g = Am.gravityForces(state)
a = Am.aeroForces(state)
cf = Am.controlForces(state, control)
expectedFx = g.Fx + a.Fx + cf.Fx
actualFx = x.Fx
expectedFy = g.Fy + a.Fy + cf.Fy
actualFy = x.Fy
expectedFz = g.Fz + a.Fz + cf.Fz
actualFz = x.Fz
expectedMx = g.Mx + a.Mx + cf.Mx
actualMx = x.Mx
expectedMy = g.My + a.My + cf.My
actualMy = x.My
expectedMz = g.Mz + a.Mz + cf.Mz
actualMz = x.Mz
if not evaluateTest(cur_test, isclose(actualFx, expectedFx) and isclose(actualFy, expectedFy) and isclose(actualFz, expectedFz)
							  and isclose(actualMx, expectedMx) and isclose(actualMy, expectedMy) and isclose(actualMz, expectedMz) and
							  isclose(actualVa, expectedVa) and isclose(actualalpha, expectedalpha) and isclose(actualbeta, expectedbeta)):
	print(f"{actualFx} != {expectedFx}")
	print(f"{actualFy} != {expectedFy}")
	print(f"{actualFz} != {expectedFz}")
	print(f"{actualMx} != {expectedMx}")
	print(f"{actualMy} != {expectedMy}")
	print(f"{actualMz} != {expectedMz}")
	print(f"{actualVa} != {expectedVa}")
	print(f"{actualalpha} != {expectedalpha}")
	print(f"{actualbeta} != {expectedbeta}")

cur_test = "updateForces test 2"
#creating instances of AerodynamicsModel and States
Am = VAM.VehicleAerodynamicsModel()
state = States.vehicleState()
state.u = 1
state.v = 2
state.w = 3
control = Inputs.controlInputs(Throttle = 4, Aileron = 5, Elevator = 6, Rudder = 7)
Am.updateForces(state, control)
actualVa = state.Va
expectedVa = 3.741657386773941
actualalpha = state.alpha
expectedalpha = 1.2490457723982544
actualbeta = state.beta
expectedbeta = 0.5639426413606289
x = Am.updateForces(state, control)
g = Am.gravityForces(state)
a = Am.aeroForces(state)
cf = Am.controlForces(state, control)
expectedFx = g.Fx + a.Fx + cf.Fx
actualFx = x.Fx
expectedFy = g.Fy + a.Fy + cf.Fy
actualFy = x.Fy
expectedFz = g.Fz + a.Fz + cf.Fz
actualFz = x.Fz
expectedMx = g.Mx + a.Mx + cf.Mx
actualMx = x.Mx
expectedMy = g.My + a.My + cf.My
actualMy = x.My
expectedMz = g.Mz + a.Mz + cf.Mz
actualMz = x.Mz
if not evaluateTest(cur_test, isclose(actualFx, expectedFx) and isclose(actualFy, expectedFy) and isclose(actualFz, expectedFz)
							  and isclose(actualMx, expectedMx) and isclose(actualMy, expectedMy) and isclose(actualMz, expectedMz) and
							  isclose(actualVa, expectedVa) and isclose(actualalpha, expectedalpha) and isclose(actualbeta, expectedbeta)):
	print(f"{actualFx} != {expectedFx}")
	print(f"{actualFy} != {expectedFy}")
	print(f"{actualFz} != {expectedFz}")
	print(f"{actualMx} != {expectedMx}")
	print(f"{actualMy} != {expectedMy}")
	print(f"{actualMz} != {expectedMz}")
	print(f"{actualVa} != {expectedVa}")
	print(f"{actualalpha} != {expectedalpha}")
	print(f"{actualbeta} != {expectedbeta}")

#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]