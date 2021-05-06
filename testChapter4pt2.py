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
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Containers.States as States
import ece163.Containers.Inputs as Inputs
import ece163.Controls.VehicleTrim as VT


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
	"""Testing what was added from WindModel inside of Aerodynamics Model"""
	def getWindModeltest1():
		cur_test = "getWindModel test 1"

		#Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()
		wind = States.windState()

		#setting numerical values to components of state and wind
		Am.WindModel.windState.pn = 1

		#Calling get WindModel()
		actual = Am.getWindModel()

		#expected values
		wind.pn = 1

		if not evaluateTest(cur_test, (actual.windState.pn == wind.pn)):
			print(f"{actual.windState.pn} != {wind.pn}")
	getWindModeltest1()

	def getWindModeltest2():
		cur_test = "getWindModel test 2"

		#Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()
		wind = States.windState()

		#setting numerical values to components of state and wind
		Am.WindModel.windState.pe = 2

		#Calling get WindModel()
		actual = Am.getWindModel()

		#expected values
		wind.pe = 2

		if not evaluateTest(cur_test, (actual.windState.pe == wind.pe)):
			print(f"{actual.windState.pe} != {wind.pe}")
	getWindModeltest2()

	def getWindModeltest3():
		cur_test = "getWindModel test 3"

		#Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()
		wind = States.windState()

		#setting numerical values to components of state and wind
		Am.WindModel.windState.pd = 3

		#Calling get WindModel()
		actual = Am.getWindModel()

		#expected values
		wind.pd = 3

		if not evaluateTest(cur_test, (actual.windState.pd == wind.pd)):
			print(f"{actual.windState.pd} != {wind.pd}")
	getWindModeltest3()

	def getWindModeltest4():
		cur_test = "getWindModel test 4"

		#Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()
		wind = States.windState()

		#setting numerical values to components of state and wind
		Am.WindModel.windState.u = 4

		#Calling get WindModel()
		actual = Am.getWindModel()

		#expected values
		wind.u = 4

		if not evaluateTest(cur_test, (actual.windState.u == wind.u)):
			print(f"{actual.windState.u} != {wind.u}")
	getWindModeltest4()

	def getWindModeltest5():
		cur_test = "getWindModel test 5"

		#Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()
		wind = States.windState()

		#setting numerical values to components of state and wind
		Am.WindModel.windState.v = 5

		#Calling get WindModel()
		actual = Am.getWindModel()

		#expected values
		wind.v = 5

		if not evaluateTest(cur_test, (actual.windState.v == wind.v)):
			print(f"{actual.windState.v} != {wind.v}")
	getWindModeltest5()

	def getWindModeltest6():
		cur_test = "getWindModel test 6"

		#Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()
		wind = States.windState()

		#setting numerical values to components of state and wind
		Am.WindModel.windState.w = 6

		#Calling get WindModel()
		actual = Am.getWindModel()

		#expected values
		wind.w = 6

		if not evaluateTest(cur_test, (actual.windState.w == wind.w)):
			print(f"{actual.windState.w} != {wind.w}")
	getWindModeltest6()

	def getWindModeltest7():
		cur_test = "getWindModel test 7"

		#Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()
		wind = States.windState()

		#setting numerical values to components of state and wind
		Am.WindModel.windState.yaw = 7

		#Calling get WindModel()
		actual = Am.getWindModel()

		#expected values
		wind.yaw = 7

		if not evaluateTest(cur_test, (actual.windState.yaw == wind.yaw)):
			print(f"{actual.windState.yaw} != {wind.yaw}")
	getWindModeltest7()

	def getWindModeltest8():
		cur_test = "getWindModel test 8"

		#Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()
		wind = States.windState()

		#setting numerical values to components of state and wind
		Am.WindModel.windState.pitch = 8

		#Calling get WindModel()
		actual = Am.getWindModel()

		#expected values
		wind.pitch = 8

		if not evaluateTest(cur_test, (actual.windState.pitch == wind.pitch)):
			print(f"{actual.windState.pitch} != {wind.pitch}")
	getWindModeltest8()

	def getWindModeltest9():
		cur_test = "getWindModel test 9"

		#Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()
		wind = States.windState()

		#setting numerical values to components of state and wind
		Am.WindModel.windState.roll = 9

		#Calling get WindModel()
		actual = Am.getWindModel()

		#expected values
		wind.roll = 9

		if not evaluateTest(cur_test, (actual.windState.roll == wind.roll)):
			print(f"{actual.windState.roll} != {wind.roll}")
	getWindModeltest9()

	def getWindModeltest10():
		cur_test = "getWindModel test 10"

		#Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()
		wind = States.windState()

		#setting numerical values to components of state and wind
		Am.WindModel.windState.p = 10

		#Calling get WindModel()
		actual = Am.getWindModel()

		#expected values
		wind.p = 10

		if not evaluateTest(cur_test, (actual.windState.p == wind.p)):
			print(f"{actual.windState.p} != {wind.p}")
	getWindModeltest10()

	def getWindModeltest11():
		cur_test = "getWindModel test 11"

		#Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()
		wind = States.windState()

		#setting numerical values to components of state and wind
		Am.WindModel.windState.q = 11

		#Calling get WindModel()
		actual = Am.getWindModel()

		#expected values
		wind.q = 11

		if not evaluateTest(cur_test, (actual.windState.q == wind.q)):
			print(f"{actual.windState.q} != {wind.q}")
	getWindModeltest11()

	def getWindModeltest12():
		cur_test = "getWindModel test 12"

		#Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()
		wind = States.windState()

		#setting numerical values to components of state and wind
		Am.WindModel.windState.r = 12

		#Calling get WindModel()
		actual = Am.getWindModel()

		#expected values
		wind.r = 12

		if not evaluateTest(cur_test, (actual.windState.r == wind.r)):
			print(f"{actual.windState.r} != {wind.r}")
	getWindModeltest12()
getWindModeltest()

def setWindModeltest():
	"""Testing setWindModel against windmodel"""
	def setWindModeltest1():
		cur_test = "setWindModel test 1"

		#Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()

		#setting numerical values to components of state and wind
		windModel = WM.WindModel()
		windModel.windState.dT = 0
		windModel.windState.Va = 0
		windModel.windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)

		#Calling get WindModel()
		Am.setWindModel(windModel)

		if not evaluateTest(cur_test, (Am.WindModel.windState.dT == windModel.windState.dT and Am.WindModel.windState.Va == windModel.windState.Va
		and Am.WindModel.windState.drydenParamters == windModel.windState.drydenParamters)):
			print(f"{Am.WindModel.windState.dT} != {windModel.windState.dT}")
			print(f"{Am.WindModel.windState.Va} != {windModel.windState.Va}")
			print(f"{Am.WindModel.windState.drydenParamters} != {windModel.windState.drydenParamters}")
	setWindModeltest1()

	def setWindModeltest2():
		cur_test = "setWindModel test 2"

		#Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()

		#setting numerical values to components of state and wind
		windModel = WM.WindModel()
		windModel.windState.dT = 1
		windModel.windState.Va = 0
		windModel.windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)

		#Calling get WindModel()
		Am.setWindModel(windModel)

		if not evaluateTest(cur_test, (Am.WindModel.windState.dT == windModel.windState.dT and Am.WindModel.windState.Va == windModel.windState.Va
		and Am.WindModel.windState.drydenParamters == windModel.windState.drydenParamters)):
			print(f"{Am.WindModel.windState.dT} != {windModel.windState.dT}")
			print(f"{Am.WindModel.windState.Va} != {windModel.windState.Va}")
			print(f"{Am.WindModel.windState.drydenParamters} != {windModel.windState.drydenParamters}")
	setWindModeltest2()

	def setWindModeltest3():
		cur_test = "setWindModel test 3"

		# Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()

		# setting numerical values to components of state and wind
		windModel = WM.WindModel()
		windModel.windState.dT = 0
		windModel.windState.Va = 1
		windModel.windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)

		# Calling get WindModel()
		Am.setWindModel(windModel)

		if not evaluateTest(cur_test, (Am.WindModel.windState.dT == windModel.windState.dT and Am.WindModel.windState.Va == windModel.windState.Va
									   and Am.WindModel.windState.drydenParamters == windModel.windState.drydenParamters)):
			print(f"{Am.WindModel.windState.dT} != {windModel.windState.dT}")
			print(f"{Am.WindModel.windState.Va} != {windModel.windState.Va}")
			print(f"{Am.WindModel.windState.drydenParamters} != {windModel.windState.drydenParamters}")
	setWindModeltest3()

	def setWindModeltest4():
		cur_test = "setWindModel test 4"

		#Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()

		#setting numerical values to components of state and wind
		windModel = WM.WindModel()
		windModel.windState.dT = 0
		windModel.windState.Va = 0
		windModel.windState.drydenParamters = Inputs.drydenParameters(Lu=1, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)

		#Calling get WindModel()
		Am.setWindModel(windModel)

		if not evaluateTest(cur_test, (Am.WindModel.windState.dT == windModel.windState.dT and Am.WindModel.windState.Va == windModel.windState.Va
		and Am.WindModel.windState.drydenParamters == windModel.windState.drydenParamters)):
			print(f"{Am.WindModel.windState.dT} != {windModel.windState.dT}")
			print(f"{Am.WindModel.windState.Va} != {windModel.windState.Va}")
			print(f"{Am.WindModel.windState.drydenParamters} != {windModel.windState.drydenParamters}")
	setWindModeltest4()

	def setWindModeltest5():
		cur_test = "setWindModel test 5"

		#Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()

		#setting numerical values to components of state and wind
		windModel = WM.WindModel()
		windModel.windState.dT = 0
		windModel.windState.Va = 0
		windModel.windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=1, Lw=0, sigmau=0, sigmav=0, sigmaw=0)

		#Calling get WindModel()
		Am.setWindModel(windModel)

		if not evaluateTest(cur_test, (Am.WindModel.windState.dT == windModel.windState.dT and Am.WindModel.windState.Va == windModel.windState.Va
		and Am.WindModel.windState.drydenParamters == windModel.windState.drydenParamters)):
			print(f"{Am.WindModel.windState.dT} != {windModel.windState.dT}")
			print(f"{Am.WindModel.windState.Va} != {windModel.windState.Va}")
			print(f"{Am.WindModel.windState.drydenParamters} != {windModel.windState.drydenParamters}")
	setWindModeltest5()

	def setWindModeltest6():
		cur_test = "setWindModel test 6"

		#Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()

		#setting numerical values to components of state and wind
		windModel = WM.WindModel()
		windModel.windState.dT = 0
		windModel.windState.Va = 0
		windModel.windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=1, sigmau=0, sigmav=0, sigmaw=0)

		#Calling get WindModel()
		Am.setWindModel(windModel)

		if not evaluateTest(cur_test, (Am.WindModel.windState.dT == windModel.windState.dT and Am.WindModel.windState.Va == windModel.windState.Va
		and Am.WindModel.windState.drydenParamters == windModel.windState.drydenParamters)):
			print(f"{Am.WindModel.windState.dT} != {windModel.windState.dT}")
			print(f"{Am.WindModel.windState.Va} != {windModel.windState.Va}")
			print(f"{Am.WindModel.windState.drydenParamters} != {windModel.windState.drydenParamters}")
	setWindModeltest6()

	def setWindModeltest7():
		cur_test = "setWindModel test 7"

		#Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()

		#setting numerical values to components of state and wind
		windModel = WM.WindModel()
		windModel.windState.dT = 0
		windModel.windState.Va = 0
		windModel.windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=1, sigmav=0, sigmaw=0)

		#Calling get WindModel()
		Am.setWindModel(windModel)

		if not evaluateTest(cur_test, (Am.WindModel.windState.dT == windModel.windState.dT and Am.WindModel.windState.Va == windModel.windState.Va
		and Am.WindModel.windState.drydenParamters == windModel.windState.drydenParamters)):
			print(f"{Am.WindModel.windState.dT} != {windModel.windState.dT}")
			print(f"{Am.WindModel.windState.Va} != {windModel.windState.Va}")
			print(f"{Am.WindModel.windState.drydenParamters} != {windModel.windState.drydenParamters}")
	setWindModeltest7()

	def setWindModeltest8():
		cur_test = "setWindModel test 8"

		#Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()

		#setting numerical values to components of state and wind
		windModel = WM.WindModel()
		windModel.windState.dT = 0
		windModel.windState.Va = 0
		windModel.windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=1, sigmaw=0)

		#Calling get WindModel()
		Am.setWindModel(windModel)

		if not evaluateTest(cur_test, (Am.WindModel.windState.dT == windModel.windState.dT and Am.WindModel.windState.Va == windModel.windState.Va
		and Am.WindModel.windState.drydenParamters == windModel.windState.drydenParamters)):
			print(f"{Am.WindModel.windState.dT} != {windModel.windState.dT}")
			print(f"{Am.WindModel.windState.Va} != {windModel.windState.Va}")
			print(f"{Am.WindModel.windState.drydenParamters} != {windModel.windState.drydenParamters}")
	setWindModeltest8()

	def setWindModeltest9():
		cur_test = "setWindModel test 9"

		#Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()

		#setting numerical values to components of state and wind
		windModel = WM.WindModel()
		windModel.windState.dT = 0
		windModel.windState.Va = 0
		windModel.windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=1)

		#Calling get WindModel()
		Am.setWindModel(windModel)

		if not evaluateTest(cur_test, (Am.WindModel.windState.dT == windModel.windState.dT and Am.WindModel.windState.Va == windModel.windState.Va
		and Am.WindModel.windState.drydenParamters == windModel.windState.drydenParamters)):
			print(f"{Am.WindModel.windState.dT} != {windModel.windState.dT}")
			print(f"{Am.WindModel.windState.Va} != {windModel.windState.Va}")
			print(f"{Am.WindModel.windState.drydenParamters} != {windModel.windState.drydenParamters}")
	setWindModeltest9()

	def setWindModeltest10():
		cur_test = "setWindModel test 10"

		#Initializing functions and classes
		Am = VAM.VehicleAerodynamicsModel()

		#setting numerical values to components of state and wind
		windModel = WM.WindModel()
		windModel.windState.dT = 1
		windModel.windState.Va = 2
		windModel.windState.drydenParamters = Inputs.drydenParameters(Lu=3, Lv=4, Lw=5, sigmau=6, sigmav=7, sigmaw=8)

		#Calling get WindModel()
		Am.setWindModel(windModel)

		if not evaluateTest(cur_test, (Am.WindModel.windState.dT == windModel.windState.dT and Am.WindModel.windState.Va == windModel.windState.Va
		and Am.WindModel.windState.drydenParamters == windModel.windState.drydenParamters)):
			print(f"{Am.WindModel.windState.dT} != {windModel.windState.dT}")
			print(f"{Am.WindModel.windState.Va} != {windModel.windState.Va}")
			print(f"{Am.WindModel.windState.drydenParamters} != {windModel.windState.drydenParamters}")
	setWindModeltest10()
setWindModeltest()

def CalculateAirspeedtest():
	"""Calculating the airspeed, angle of attck, and side-slip angle with wind in mind"""
	def calculateAirspeedtest1():
		cur_test = "CalculateAirspeed test 1"
		#initializing class and states
		state = States.vehicleState()
		windModel = WM.WindModel()
		Am = VAM.VehicleAerodynamicsModel()

		#creating values for state and wind
		state.u = 0
		state.v = 0
		state.w = 0
		windModel.windState.Wn = 0
		windModel.windState.We = 0
		windModel.windState.Wd = 0
		windModel.windState.Wu = 0
		windModel.windState.Wv = 0
		windModel.windState.Ww = 0

		#expected values for Va, alpha, beta
		expected_Va = 0
		expected_alpha = 0
		expected_beta = 0

		#Creating the actual values based on the function
		Vamag, alpha, beta = Am.CalculateAirspeed(state, windModel.windState)

		if not evaluateTest(cur_test, (isclose(expected_Va, Vamag) and (isclose(expected_alpha, alpha)) and (isclose(expected_beta, beta)))):
			print(f"{expected_Va} != {Vamag}")
			print(f"{expected_alpha} != {alpha}")
			print(f"{expected_beta} != {beta}")
	calculateAirspeedtest1()

	def calculateAirspeedtest2():
		cur_test = "CalculateAirspeed test 2"
		#initializing class and states
		state = States.vehicleState()
		windModel = WM.WindModel()
		Am = VAM.VehicleAerodynamicsModel()

		#creating values for state and wind
		state.u = 1
		state.v = 0
		state.w = 0
		windModel.windState.Wn = 0
		windModel.windState.We = 0
		windModel.windState.Wd = 0
		windModel.windState.Wu = 0
		windModel.windState.Wv = 0
		windModel.windState.Ww = 0

		#expected values for Va, alpha, beta
		expected_Va = 1
		expected_alpha = 0
		expected_beta = 0

		#Creating the actual values based on the function
		Vamag, alpha, beta = Am.CalculateAirspeed(state, windModel.windState)

		if not evaluateTest(cur_test, (isclose(expected_Va, Vamag) and (isclose(expected_alpha, alpha)) and (isclose(expected_beta, beta)))):
			print(f"{expected_Va} != {Vamag}")
			print(f"{expected_alpha} != {alpha}")
			print(f"{expected_beta} != {beta}")
	calculateAirspeedtest2()

	def calculateAirspeedtest3():
		cur_test = "CalculateAirspeed test 3"
		#initializing class and states
		state = States.vehicleState()
		windModel = WM.WindModel()
		Am = VAM.VehicleAerodynamicsModel()

		#creating values for state and wind
		state.u = 0
		state.v = 1
		state.w = 0
		windModel.windState.Wn = 0
		windModel.windState.We = 0
		windModel.windState.Wd = 0
		windModel.windState.Wu = 0
		windModel.windState.Wv = 0
		windModel.windState.Ww = 0

		#expected values for Va, alpha, beta
		expected_Va = 1
		expected_alpha = 0
		expected_beta = 1.5707963267948966

		#Creating the actual values based on the function
		Vamag, alpha, beta = Am.CalculateAirspeed(state, windModel.windState)

		if not evaluateTest(cur_test, (isclose(expected_Va, Vamag) and (isclose(expected_alpha, alpha)) and (isclose(expected_beta, beta)))):
			print(f"{expected_Va} != {Vamag}")
			print(f"{expected_alpha} != {alpha}")
			print(f"{expected_beta} != {beta}")
	calculateAirspeedtest3()

	def calculateAirspeedtest4():
		cur_test = "CalculateAirspeed test 4"
		#initializing class and states
		state = States.vehicleState()
		windModel = WM.WindModel()
		Am = VAM.VehicleAerodynamicsModel()

		#creating values for state and wind
		state.u = 0
		state.v = 0
		state.w = 1
		windModel.windState.Wn = 0
		windModel.windState.We = 0
		windModel.windState.Wd = 0
		windModel.windState.Wu = 0
		windModel.windState.Wv = 0
		windModel.windState.Ww = 0

		#expected values for Va, alpha, beta
		expected_Va = 1
		expected_alpha = 1.5707963267948966
		expected_beta = 0

		#Creating the actual values based on the function
		Vamag, alpha, beta = Am.CalculateAirspeed(state, windModel.windState)

		if not evaluateTest(cur_test, (isclose(expected_Va, Vamag) and (isclose(expected_alpha, alpha)) and (isclose(expected_beta, beta)))):
			print(f"{expected_Va} != {Vamag}")
			print(f"{expected_alpha} != {alpha}")
			print(f"{expected_beta} != {beta}")
	calculateAirspeedtest4()

	def calculateAirspeedtest5():
		cur_test = "CalculateAirspeed test 5"
		#initializing class and states
		state = States.vehicleState()
		windModel = WM.WindModel()
		Am = VAM.VehicleAerodynamicsModel()

		#creating values for state and wind
		state.u = 0
		state.v = 0
		state.w = 0
		windModel.windState.Wn = 1
		windModel.windState.We = 0
		windModel.windState.Wd = 0
		windModel.windState.Wu = 0
		windModel.windState.Wv = 0
		windModel.windState.Ww = 0

		#expected values for Va, alpha, beta
		expected_Va = 1
		expected_alpha = 3.141592653589793
		expected_beta = 0

		#Creating the actual values based on the function
		Vamag, alpha, beta = Am.CalculateAirspeed(state, windModel.windState)

		if not evaluateTest(cur_test, (isclose(expected_Va, Vamag) and (isclose(expected_alpha, alpha)) and (isclose(expected_beta, beta)))):
			print(f"{expected_Va} != {Vamag}")
			print(f"{expected_alpha} != {alpha}")
			print(f"{expected_beta} != {beta}")
	calculateAirspeedtest5()

	def calculateAirspeedtest6():
		cur_test = "CalculateAirspeed test 6"
		#initializing class and states
		state = States.vehicleState()
		windModel = WM.WindModel()
		Am = VAM.VehicleAerodynamicsModel()

		#creating values for state and wind
		state.u = 0
		state.v = 0
		state.w = 0
		windModel.windState.Wn = 0
		windModel.windState.We = 1
		windModel.windState.Wd = 0
		windModel.windState.Wu = 0
		windModel.windState.Wv = 0
		windModel.windState.Ww = 0

		#expected values for Va, alpha, beta
		expected_Va = 1
		expected_alpha = 0
		expected_beta = -1.5707963267948966

		#Creating the actual values based on the function
		Vamag, alpha, beta = Am.CalculateAirspeed(state, windModel.windState)

		if not evaluateTest(cur_test, (isclose(expected_Va, Vamag) and (isclose(expected_alpha, alpha)) and (isclose(expected_beta, beta)))):
			print(f"{expected_Va} != {Vamag}")
			print(f"{expected_alpha} != {alpha}")
			print(f"{expected_beta} != {beta}")
	calculateAirspeedtest6()

	def calculateAirspeedtest7():
		cur_test = "CalculateAirspeed test 7"
		#initializing class and states
		state = States.vehicleState()
		windModel = WM.WindModel()
		Am = VAM.VehicleAerodynamicsModel()

		#creating values for state and wind
		state.u = 0
		state.v = 0
		state.w = 0
		windModel.windState.Wn = 0
		windModel.windState.We = 0
		windModel.windState.Wd = 1
		windModel.windState.Wu = 0
		windModel.windState.Wv = 0
		windModel.windState.Ww = 0

		#expected values for Va, alpha, beta
		expected_Va = 1
		expected_alpha = -1.5707963267948966
		expected_beta = 0

		#Creating the actual values based on the function
		Vamag, alpha, beta = Am.CalculateAirspeed(state, windModel.windState)

		if not evaluateTest(cur_test, (isclose(expected_Va, Vamag) and (isclose(expected_alpha, alpha)) and (isclose(expected_beta, beta)))):
			print(f"{expected_Va} != {Vamag}")
			print(f"{expected_alpha} != {alpha}")
			print(f"{expected_beta} != {beta}")
	calculateAirspeedtest7()

	def calculateAirspeedtest8():
		cur_test = "CalculateAirspeed test 8"
		#initializing class and states
		state = States.vehicleState()
		windModel = WM.WindModel()
		Am = VAM.VehicleAerodynamicsModel()

		#creating values for state and wind
		state.u = 0
		state.v = 0
		state.w = 0
		windModel.windState.Wn = 0
		windModel.windState.We = 0
		windModel.windState.Wd = 0
		windModel.windState.Wu = 1
		windModel.windState.Wv = 0
		windModel.windState.Ww = 0

		#expected values for Va, alpha, beta
		expected_Va = 1
		expected_alpha = 3.141592653589793
		expected_beta = 0

		#Creating the actual values based on the function
		Vamag, alpha, beta = Am.CalculateAirspeed(state, windModel.windState)

		if not evaluateTest(cur_test, (isclose(expected_Va, Vamag) and (isclose(expected_alpha, alpha)) and (isclose(expected_beta, beta)))):
			print(f"{expected_Va} != {Vamag}")
			print(f"{expected_alpha} != {alpha}")
			print(f"{expected_beta} != {beta}")
	calculateAirspeedtest8()

	def calculateAirspeedtest9():
		cur_test = "CalculateAirspeed test 9"
		#initializing class and states
		state = States.vehicleState()
		windModel = WM.WindModel()
		Am = VAM.VehicleAerodynamicsModel()

		#creating values for state and wind
		state.u = 0
		state.v = 0
		state.w = 0
		windModel.windState.Wn = 0
		windModel.windState.We = 0
		windModel.windState.Wd = 0
		windModel.windState.Wu = 0
		windModel.windState.Wv = 1
		windModel.windState.Ww = 0

		#expected values for Va, alpha, beta
		expected_Va = 1
		expected_alpha = 0
		expected_beta = -1.5707963267948966

		#Creating the actual values based on the function
		Vamag, alpha, beta = Am.CalculateAirspeed(state, windModel.windState)

		if not evaluateTest(cur_test, (isclose(expected_Va, Vamag) and (isclose(expected_alpha, alpha)) and (isclose(expected_beta, beta)))):
			print(f"{expected_Va} != {Vamag}")
			print(f"{expected_alpha} != {alpha}")
			print(f"{expected_beta} != {beta}")
	calculateAirspeedtest9()

	def calculateAirspeedtest10():
		cur_test = "CalculateAirspeed test 10"
		#initializing class and states
		state = States.vehicleState()
		windModel = WM.WindModel()
		Am = VAM.VehicleAerodynamicsModel()

		#creating values for state and wind
		state.u = 0
		state.v = 0
		state.w = 0
		windModel.windState.Wn = 0
		windModel.windState.We = 0
		windModel.windState.Wd = 0
		windModel.windState.Wu = 0
		windModel.windState.Wv = 0
		windModel.windState.Ww = 1

		#expected values for Va, alpha, beta
		expected_Va = 1
		expected_alpha = -1.5707963267948966
		expected_beta = 0

		#Creating the actual values based on the function
		Vamag, alpha, beta = Am.CalculateAirspeed(state, windModel.windState)

		if not evaluateTest(cur_test, (isclose(expected_Va, Vamag) and (isclose(expected_alpha, alpha)) and (isclose(expected_beta, beta)))):
			print(f"{expected_Va} != {Vamag}")
			print(f"{expected_alpha} != {alpha}")
			print(f"{expected_beta} != {beta}")
	calculateAirspeedtest10()

	def calculateAirspeedtest11():
		cur_test = "CalculateAirspeed test 11"
		#initializing class and states
		state = States.vehicleState()
		windModel = WM.WindModel()
		Am = VAM.VehicleAerodynamicsModel()

		#creating values for state and wind
		state.u = 1
		state.v = 2
		state.w = 3
		windModel.windState.Wn = 4
		windModel.windState.We = 5
		windModel.windState.Wd = 6
		windModel.windState.Wu = 7
		windModel.windState.Wv = 8
		windModel.windState.Ww = 9

		#expected values for Va, alpha, beta
		expected_Va = 16.516805712430187
		expected_alpha = -1.3054712970268652
		expected_beta = -0.4497696089483118

		#Creating the actual values based on the function
		Vamag, alpha, beta = Am.CalculateAirspeed(state, windModel.windState)

		if not evaluateTest(cur_test, (isclose(expected_Va, Vamag) and (isclose(expected_alpha, alpha)) and (isclose(expected_beta, beta)))):
			print(f"{expected_Va} != {Vamag}")
			print(f"{expected_alpha} != {alpha}")
			print(f"{expected_beta} != {beta}")
	calculateAirspeedtest11()
CalculateAirspeedtest()


print("Beginning testing of WindModel()")


def setWindtest():
	"""Testing setWind against windState"""
	def setWindtest1():
		cur_test = "setWind test 1"

		# Initializing functions and classes
		Wm = WM.WindModel()
		windState = States.windState

		# setting numerical values to components of state and wind
		windState.dT = 0
		windState.Va = 0
		windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)

		# Calling get WindModel()
		Wm.setWind(Wm.windState)

		if not evaluateTest(cur_test, (
				Wm.windState.dT == windState.dT and Wm.windState.Va == windState.Va
				and Wm.windState.drydenParamters == windState.drydenParamters)):
			print(f"{Wm.windState.dT} != {windState.dT}")
			print(f"{Wm.windState.Va} != {windState.Va}")
			print(f"{Wm.windState.drydenParamters} != {windState.drydenParamters}")
	setWindtest1()

	def setWindtest2():
		cur_test = "setWind test 2"

		# Initializing functions and classes
		Wm = WM.WindModel()
		windState = States.windState

		# setting numerical values to components of state and wind
		windState.dT = 1
		windState.Va = 0
		windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)

		# Calling get WindModel()
		Wm.setWind(Wm.windState)

		if not evaluateTest(cur_test, (
				Wm.windState.dT == windState.dT and Wm.windState.Va == windState.Va
				and Wm.windState.drydenParamters == windState.drydenParamters)):
			print(f"{Wm.windState.dT} != {windState.dT}")
			print(f"{Wm.windState.Va} != {windState.Va}")
			print(f"{Wm.windState.drydenParamters} != {windState.drydenParamters}")
	setWindtest2()

	def setWindtest3():
		cur_test = "setWind test 3"

		# Initializing functions and classes
		Wm = WM.WindModel()
		windState = States.windState

		# setting numerical values to components of state and wind
		windState.dT = 0
		windState.Va = 1
		windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)

		# Calling get WindModel()
		Wm.setWind(Wm.windState)

		if not evaluateTest(cur_test, (
				Wm.windState.dT == windState.dT and Wm.windState.Va == windState.Va
				and Wm.windState.drydenParamters == windState.drydenParamters)):
			print(f"{Wm.windState.dT} != {windState.dT}")
			print(f"{Wm.windState.Va} != {windState.Va}")
			print(f"{Wm.windState.drydenParamters} != {windState.drydenParamters}")
	setWindtest3()

	def setWindtest4():
		cur_test = "setWind test 4"

		# Initializing functions and classes
		Wm = WM.WindModel()
		windState = States.windState

		# setting numerical values to components of state and wind
		windState.dT = 0
		windState.Va = 0
		windState.drydenParamters = Inputs.drydenParameters(Lu=1, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)

		# Calling get WindModel()
		Wm.setWind(Wm.windState)

		if not evaluateTest(cur_test, (
				Wm.windState.dT == windState.dT and Wm.windState.Va == windState.Va
				and Wm.windState.drydenParamters == windState.drydenParamters)):
			print(f"{Wm.windState.dT} != {windState.dT}")
			print(f"{Wm.windState.Va} != {windState.Va}")
			print(f"{Wm.windState.drydenParamters} != {windState.drydenParamters}")
	setWindtest4()

	def setWindtest5():
		cur_test = "setWind test 5"

		# Initializing functions and classes
		Wm = WM.WindModel()
		windState = States.windState

		# setting numerical values to components of state and wind
		windState.dT = 0
		windState.Va = 0
		windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=1, Lw=0, sigmau=0, sigmav=0, sigmaw=0)

		# Calling get WindModel()
		Wm.setWind(Wm.windState)

		if not evaluateTest(cur_test, (
				Wm.windState.dT == windState.dT and Wm.windState.Va == windState.Va
				and Wm.windState.drydenParamters == windState.drydenParamters)):
			print(f"{Wm.windState.dT} != {windState.dT}")
			print(f"{Wm.windState.Va} != {windState.Va}")
			print(f"{Wm.windState.drydenParamters} != {windState.drydenParamters}")
	setWindtest5()

	def setWindtest6():
		cur_test = "setWind test 6"

		# Initializing functions and classes
		Wm = WM.WindModel()
		windState = States.windState

		# setting numerical values to components of state and wind
		windState.dT = 0
		windState.Va = 0
		windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=1, sigmau=0, sigmav=0, sigmaw=0)

		# Calling get WindModel()
		Wm.setWind(Wm.windState)

		if not evaluateTest(cur_test, (
				Wm.windState.dT == windState.dT and Wm.windState.Va == windState.Va
				and Wm.windState.drydenParamters == windState.drydenParamters)):
			print(f"{Wm.windState.dT} != {windState.dT}")
			print(f"{Wm.windState.Va} != {windState.Va}")
			print(f"{Wm.windState.drydenParamters} != {windState.drydenParamters}")
	setWindtest6()

	def setWindtest7():
		cur_test = "setWind test 7"

		# Initializing functions and classes
		Wm = WM.WindModel()
		windState = States.windState

		# setting numerical values to components of state and wind
		windState.dT = 0
		windState.Va = 0
		windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=1, sigmav=0, sigmaw=0)

		# Calling get WindModel()
		Wm.setWind(Wm.windState)

		if not evaluateTest(cur_test, (
				Wm.windState.dT == windState.dT and Wm.windState.Va == windState.Va
				and Wm.windState.drydenParamters == windState.drydenParamters)):
			print(f"{Wm.windState.dT} != {windState.dT}")
			print(f"{Wm.windState.Va} != {windState.Va}")
			print(f"{Wm.windState.drydenParamters} != {windState.drydenParamters}")
	setWindtest7()

	def setWindtest8():
		cur_test = "setWind test 8"

		# Initializing functions and classes
		Wm = WM.WindModel()
		windState = States.windState

		# setting numerical values to components of state and wind
		windState.dT = 0
		windState.Va = 0
		windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=1, sigmaw=0)

		# Calling get WindModel()
		Wm.setWind(Wm.windState)

		if not evaluateTest(cur_test, (
				Wm.windState.dT == windState.dT and Wm.windState.Va == windState.Va
				and Wm.windState.drydenParamters == windState.drydenParamters)):
			print(f"{Wm.windState.dT} != {windState.dT}")
			print(f"{Wm.windState.Va} != {windState.Va}")
			print(f"{Wm.windState.drydenParamters} != {windState.drydenParamters}")
	setWindtest8()

	def setWindtest9():
		cur_test = "setWind test 9"

		# Initializing functions and classes
		Wm = WM.WindModel()
		windState = States.windState

		# setting numerical values to components of state and wind
		windState.dT = 0
		windState.Va = 0
		windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=1)

		# Calling get WindModel()
		Wm.setWind(Wm.windState)

		if not evaluateTest(cur_test, (
				Wm.windState.dT == windState.dT and Wm.windState.Va == windState.Va
				and Wm.windState.drydenParamters == windState.drydenParamters)):
			print(f"{Wm.windState.dT} != {windState.dT}")
			print(f"{Wm.windState.Va} != {windState.Va}")
			print(f"{Wm.windState.drydenParamters} != {windState.drydenParamters}")
	setWindtest9()

	def setWindtest10():
		cur_test = "setWind test 10"

		# Initializing functions and classes
		Wm = WM.WindModel()
		windState = States.windState

		# setting numerical values to components of state and wind
		windState.dT = 1
		windState.Va = 2
		windState.drydenParamters = Inputs.drydenParameters(Lu=3, Lv=4, Lw=5, sigmau=6, sigmav=7, sigmaw=8)

		# Calling get WindModel()
		Wm.setWind(Wm.windState)

		if not evaluateTest(cur_test, (
				Wm.windState.dT == windState.dT and Wm.windState.Va == windState.Va
				and Wm.windState.drydenParamters == windState.drydenParamters)):
			print(f"{Wm.windState.dT} != {windState.dT}")
			print(f"{Wm.windState.Va} != {windState.Va}")
			print(f"{Wm.windState.drydenParamters} != {windState.drydenParamters}")
	setWindtest10()
setWindtest()

def getWindtest():
	"""Testing getWind against windState"""
	def getwindtest1():
		cur_test = "setVehicleState test 1"
		#Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState

		#Wind definition inside of class
		Wm.windState.dT = 0
		Wm.windState.Va = 0
		Wm.windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)

		#instance of getwind
		actual = Wm.getWind()

		#setting define variables
		windState.dT = 0
		windState.Va = 0
		windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)

		if not evaluateTest(cur_test, (actual.dT == windState.dT and actual.Va == windState.Va and actual.drydenParamters == windState.drydenParamters)):
			print(f"{actual.dT} != {windState.dT}")
			print(f"{actual.Va} != {windState.Va}")
			print(f"{actual.drydenParamters} != {windState.drydenParamters}")
	getwindtest1()

	def getwindtest2():
		cur_test = "setVehicleState test 2"
		#Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState

		#Wind definition inside of class
		Wm.windState.dT = 1
		Wm.windState.Va = 0
		Wm.windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)

		#instance of getwind
		actual = Wm.getWind()

		#setting define variables
		windState.dT = 1
		windState.Va = 0
		windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)

		if not evaluateTest(cur_test, (actual.dT == windState.dT and actual.Va == windState.Va and actual.drydenParamters == windState.drydenParamters)):
			print(f"{actual.dT} != {windState.dT}")
			print(f"{actual.Va} != {windState.Va}")
			print(f"{actual.drydenParamters} != {windState.drydenParamters}")
	getwindtest2()

	def getwindtest3():
		cur_test = "setVehicleState test 3"
		#Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState

		#Wind definition inside of class
		Wm.windState.dT = 0
		Wm.windState.Va = 1
		Wm.windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)

		#instance of getwind
		actual = Wm.getWind()

		#setting define variables
		windState.dT = 0
		windState.Va = 1
		windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)

		if not evaluateTest(cur_test, (actual.dT == windState.dT and actual.Va == windState.Va and actual.drydenParamters == windState.drydenParamters)):
			print(f"{actual.dT} != {windState.dT}")
			print(f"{actual.Va} != {windState.Va}")
			print(f"{actual.drydenParamters} != {windState.drydenParamters}")
	getwindtest3()

	def getwindtest4():
		cur_test = "setVehicleState test 4"
		#Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState

		#Wind definition inside of class
		Wm.windState.dT = 0
		Wm.windState.Va = 0
		Wm.windState.drydenParamters = Inputs.drydenParameters(Lu=1, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)

		#instance of getwind
		actual = Wm.getWind()

		#setting define variables
		windState.dT = 0
		windState.Va = 0
		windState.drydenParamters = Inputs.drydenParameters(Lu=1, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)

		if not evaluateTest(cur_test, (actual.dT == windState.dT and actual.Va == windState.Va and actual.drydenParamters == windState.drydenParamters)):
			print(f"{actual.dT} != {windState.dT}")
			print(f"{actual.Va} != {windState.Va}")
			print(f"{actual.drydenParamters} != {windState.drydenParamters}")
	getwindtest4()

	def getwindtest5():
		cur_test = "setVehicleState test 5"
		#Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState

		#Wind definition inside of class
		Wm.windState.dT = 0
		Wm.windState.Va = 0
		Wm.windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=1, Lw=0, sigmau=0, sigmav=0, sigmaw=0)

		#instance of getwind
		actual = Wm.getWind()

		#setting define variables
		windState.dT = 0
		windState.Va = 0
		windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=1, Lw=0, sigmau=0, sigmav=0, sigmaw=0)

		if not evaluateTest(cur_test, (actual.dT == windState.dT and actual.Va == windState.Va and actual.drydenParamters == windState.drydenParamters)):
			print(f"{actual.dT} != {windState.dT}")
			print(f"{actual.Va} != {windState.Va}")
			print(f"{actual.drydenParamters} != {windState.drydenParamters}")
	getwindtest5()

	def getwindtest6():
		cur_test = "setVehicleState test 6"
		#Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState

		#Wind definition inside of class
		Wm.windState.dT = 0
		Wm.windState.Va = 0
		Wm.windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=1, sigmau=0, sigmav=0, sigmaw=0)

		#instance of getwind
		actual = Wm.getWind()

		#setting define variables
		windState.dT = 0
		windState.Va = 0
		windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=1, sigmau=0, sigmav=0, sigmaw=0)

		if not evaluateTest(cur_test, (actual.dT == windState.dT and actual.Va == windState.Va and actual.drydenParamters == windState.drydenParamters)):
			print(f"{actual.dT} != {windState.dT}")
			print(f"{actual.Va} != {windState.Va}")
			print(f"{actual.drydenParamters} != {windState.drydenParamters}")
	getwindtest6()

	def getwindtest7():
		cur_test = "setVehicleState test 7"
		#Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState

		#Wind definition inside of class
		Wm.windState.dT = 0
		Wm.windState.Va = 0
		Wm.windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=1, sigmav=0, sigmaw=0)

		#instance of getwind
		actual = Wm.getWind()

		#setting define variables
		windState.dT = 0
		windState.Va = 0
		windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=1, sigmav=0, sigmaw=0)

		if not evaluateTest(cur_test, (actual.dT == windState.dT and actual.Va == windState.Va and actual.drydenParamters == windState.drydenParamters)):
			print(f"{actual.dT} != {windState.dT}")
			print(f"{actual.Va} != {windState.Va}")
			print(f"{actual.drydenParamters} != {windState.drydenParamters}")
	getwindtest7()

	def getwindtest8():
		cur_test = "setVehicleState test 8"
		#Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState

		#Wind definition inside of class
		Wm.windState.dT = 0
		Wm.windState.Va = 0
		Wm.windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=1, sigmaw=0)

		#instance of getwind
		actual = Wm.getWind()

		#setting define variables
		windState.dT = 0
		windState.Va = 0
		windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=1, sigmaw=0)

		if not evaluateTest(cur_test, (actual.dT == windState.dT and actual.Va == windState.Va and actual.drydenParamters == windState.drydenParamters)):
			print(f"{actual.dT} != {windState.dT}")
			print(f"{actual.Va} != {windState.Va}")
			print(f"{actual.drydenParamters} != {windState.drydenParamters}")
	getwindtest8()

	def getwindtest9():
		cur_test = "setVehicleState test 9"
		#Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState

		#Wind definition inside of class
		Wm.windState.dT = 0
		Wm.windState.Va = 0
		Wm.windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=1)

		#instance of getwind
		actual = Wm.getWind()

		#setting define variables
		windState.dT = 0
		windState.Va = 0
		windState.drydenParamters = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=1)

		if not evaluateTest(cur_test, (actual.dT == windState.dT and actual.Va == windState.Va and actual.drydenParamters == windState.drydenParamters)):
			print(f"{actual.dT} != {windState.dT}")
			print(f"{actual.Va} != {windState.Va}")
			print(f"{actual.drydenParamters} != {windState.drydenParamters}")
	getwindtest9()

	def getwindtest10():
		cur_test = "getwindtest10"
		#Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState

		#Wind definition inside of class
		Wm.windState.dT = 1
		Wm.windState.Va = 2
		Wm.windState.drydenParamters = Inputs.drydenParameters(Lu=3, Lv=4, Lw=5, sigmau=6, sigmav=7, sigmaw=8)

		#instance of getwind
		actual = Wm.getWind()

		#setting define variables
		windState.dT = 1
		windState.Va = 2
		windState.drydenParamters = Inputs.drydenParameters(Lu=3, Lv=4, Lw=5, sigmau=6, sigmav=7, sigmaw=8)

		if not evaluateTest(cur_test, (actual.dT == windState.dT and actual.Va == windState.Va and actual.drydenParamters == windState.drydenParamters)):
			print(f"{actual.dT} != {windState.dT}")
			print(f"{actual.Va} != {windState.Va}")
			print(f"{actual.drydenParamters} != {windState.drydenParamters}")
	getwindtest10()
getWindtest()

def setWindModelParameterstest():
	"""Testing WindModel Parameters against predefined values for, Wn, We, Wd, drydenParameters  """
	def setwindmodelparatemterstest1():
		cur_test = "setwindmodelparam test 1"

		# Initializing class and states
		Wm = WM.WindModel()

		#instance of function
		Wn = 0
		We = 0
		Wd = 0
		drydenParamters2 = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)
		Wm.setWindModelParameters(Wn, We, Wd, drydenParamters2)
		if not evaluateTest(cur_test, (Wn == Wm.windState.Wn  and We == Wm.windState.We and Wd == Wm.windState.Wd) and drydenParamters2 == Wm.drydenParamters):
			print(f"{Wn} != {Wm.windState.Wn}")
			print(f"{We} != {Wm.windState.We}")
			print(f"{Wd} != {Wm.windState.Wd}")
			print(f"{drydenParamters2} != {Wm.drydenParamters}")
	setwindmodelparatemterstest1()

	def setwindmodelparatemterstest2():
		cur_test = "setwindmodelparam test 2"

		# Initializing class and states
		Wm = WM.WindModel()

		#instance of function
		Wn = 1
		We = 0
		Wd = 0
		drydenParamters2 = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)
		Wm.setWindModelParameters(Wn, We, Wd, drydenParamters2)
		if not evaluateTest(cur_test, (Wn == Wm.windState.Wn  and We == Wm.windState.We and Wd == Wm.windState.Wd) and drydenParamters2 == Wm.drydenParamters):
			print(f"{Wn} != {Wm.windState.Wn}")
			print(f"{We} != {Wm.windState.We}")
			print(f"{Wd} != {Wm.windState.Wd}")
			print(f"{drydenParamters2} != {Wm.drydenParamters}")
	setwindmodelparatemterstest2()

	def setwindmodelparatemterstest3():
		cur_test = "setwindmodelparam test 3"

		# Initializing class and states
		Wm = WM.WindModel()

		#instance of function
		Wn = 0
		We = 1
		Wd = 0
		drydenParamters2 = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)
		Wm.setWindModelParameters(Wn, We, Wd, drydenParamters2)
		if not evaluateTest(cur_test, (Wn == Wm.windState.Wn  and We == Wm.windState.We and Wd == Wm.windState.Wd) and drydenParamters2 == Wm.drydenParamters):
			print(f"{Wn} != {Wm.windState.Wn}")
			print(f"{We} != {Wm.windState.We}")
			print(f"{Wd} != {Wm.windState.Wd}")
			print(f"{drydenParamters2} != {Wm.drydenParamters}")
	setwindmodelparatemterstest3()

	def setwindmodelparatemterstest4():
		cur_test = "setwindmodelparam test 4"

		# Initializing class and states
		Wm = WM.WindModel()

		#instance of function
		Wn = 0
		We = 0
		Wd = 1
		drydenParamters2 = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)
		Wm.setWindModelParameters(Wn, We, Wd, drydenParamters2)
		if not evaluateTest(cur_test, (Wn == Wm.windState.Wn  and We == Wm.windState.We and Wd == Wm.windState.Wd) and drydenParamters2 == Wm.drydenParamters):
			print(f"{Wn} != {Wm.windState.Wn}")
			print(f"{We} != {Wm.windState.We}")
			print(f"{Wd} != {Wm.windState.Wd}")
			print(f"{drydenParamters2} != {Wm.drydenParamters}")
	setwindmodelparatemterstest4()

	def setwindmodelparatemterstest5():
		cur_test = "setwindmodelparam test 5"

		# Initializing class and states
		Wm = WM.WindModel()

		#instance of function
		Wn = 0
		We = 0
		Wd = 0
		drydenParamters2 = Inputs.drydenParameters(Lu=1, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)
		Wm.setWindModelParameters(Wn, We, Wd, drydenParamters2)
		if not evaluateTest(cur_test, (Wn == Wm.windState.Wn  and We == Wm.windState.We and Wd == Wm.windState.Wd) and drydenParamters2 == Wm.drydenParamters):
			print(f"{Wn} != {Wm.windState.Wn}")
			print(f"{We} != {Wm.windState.We}")
			print(f"{Wd} != {Wm.windState.Wd}")
			print(f"{drydenParamters2} != {Wm.drydenParamters}")
	setwindmodelparatemterstest5()

	def setwindmodelparatemterstest6():
		cur_test = "setwindmodelparam test 6"

		# Initializing class and states
		Wm = WM.WindModel()

		#instance of function
		Wn = 1
		We = 0
		Wd = 0
		drydenParamters2 = Inputs.drydenParameters(Lu=0, Lv=1, Lw=0, sigmau=0, sigmav=0, sigmaw=0)
		Wm.setWindModelParameters(Wn, We, Wd, drydenParamters2)
		if not evaluateTest(cur_test, (Wn == Wm.windState.Wn  and We == Wm.windState.We and Wd == Wm.windState.Wd) and drydenParamters2 == Wm.drydenParamters):
			print(f"{Wn} != {Wm.windState.Wn}")
			print(f"{We} != {Wm.windState.We}")
			print(f"{Wd} != {Wm.windState.Wd}")
			print(f"{drydenParamters2} != {Wm.drydenParamters}")
	setwindmodelparatemterstest6()

	def setwindmodelparatemterstest7():
		cur_test = "setwindmodelparam test 7"

		# Initializing class and states
		Wm = WM.WindModel()

		#instance of function
		Wn = 0
		We = 0
		Wd = 0
		drydenParamters2 = Inputs.drydenParameters(Lu=0, Lv=0, Lw=1, sigmau=0, sigmav=0, sigmaw=0)
		Wm.setWindModelParameters(Wn, We, Wd, drydenParamters2)
		if not evaluateTest(cur_test, (Wn == Wm.windState.Wn  and We == Wm.windState.We and Wd == Wm.windState.Wd) and drydenParamters2 == Wm.drydenParamters):
			print(f"{Wn} != {Wm.windState.Wn}")
			print(f"{We} != {Wm.windState.We}")
			print(f"{Wd} != {Wm.windState.Wd}")
			print(f"{drydenParamters2} != {Wm.drydenParamters}")
	setwindmodelparatemterstest7()

	def setwindmodelparatemterstest8():
		cur_test = "setwindmodelparam test 8"

		# Initializing class and states
		Wm = WM.WindModel()

		#instance of function
		Wn = 0
		We = 0
		Wd = 0
		drydenParamters2 = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=1, sigmav=0, sigmaw=0)
		Wm.setWindModelParameters(Wn, We, Wd, drydenParamters2)
		if not evaluateTest(cur_test, (Wn == Wm.windState.Wn  and We == Wm.windState.We and Wd == Wm.windState.Wd) and drydenParamters2 == Wm.drydenParamters):
			print(f"{Wn} != {Wm.windState.Wn}")
			print(f"{We} != {Wm.windState.We}")
			print(f"{Wd} != {Wm.windState.Wd}")
			print(f"{drydenParamters2} != {Wm.drydenParamters}")
	setwindmodelparatemterstest8()

	def setwindmodelparatemterstest9():
		cur_test = "setwindmodelparam test 9"

		# Initializing class and states
		Wm = WM.WindModel()

		#instance of function
		Wn = 0
		We = 0
		Wd = 0
		drydenParamters2 = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=1, sigmaw=0)
		Wm.setWindModelParameters(Wn, We, Wd, drydenParamters2)
		if not evaluateTest(cur_test, (Wn == Wm.windState.Wn  and We == Wm.windState.We and Wd == Wm.windState.Wd) and drydenParamters2 == Wm.drydenParamters):
			print(f"{Wn} != {Wm.windState.Wn}")
			print(f"{We} != {Wm.windState.We}")
			print(f"{Wd} != {Wm.windState.Wd}")
			print(f"{drydenParamters2} != {Wm.drydenParamters}")
	setwindmodelparatemterstest9()

	def setwindmodelparatemterstest10():
		cur_test = "setwindmodelparam test 10"

		# Initializing class and states
		Wm = WM.WindModel()

		#instance of function
		Wn = 0
		We = 0
		Wd = 0
		drydenParamters2 = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=1)
		Wm.setWindModelParameters(Wn, We, Wd, drydenParamters2)
		if not evaluateTest(cur_test, (Wn == Wm.windState.Wn  and We == Wm.windState.We and Wd == Wm.windState.Wd) and drydenParamters2 == Wm.drydenParamters):
			print(f"{Wn} != {Wm.windState.Wn}")
			print(f"{We} != {Wm.windState.We}")
			print(f"{Wd} != {Wm.windState.Wd}")
			print(f"{drydenParamters2} != {Wm.drydenParamters}")
	setwindmodelparatemterstest10()

	def setwindmodelparatemterstest11():
		cur_test = "setwindmodelparam test 11"

		# Initializing class and states
		Wm = WM.WindModel()

		#instance of function
		Wn = 1
		We = 2
		Wd = 3
		drydenParamters2 = Inputs.drydenParameters(Lu=4, Lv=5, Lw=6, sigmau=7, sigmav=8, sigmaw=9)
		Wm.setWindModelParameters(Wn, We, Wd, drydenParamters2)
		if not evaluateTest(cur_test, (Wn == Wm.windState.Wn  and We == Wm.windState.We and Wd == Wm.windState.Wd) and drydenParamters2 == Wm.drydenParamters):
			print(f"{Wn} != {Wm.windState.Wn}")
			print(f"{We} != {Wm.windState.We}")
			print(f"{Wd} != {Wm.windState.Wd}")
			print(f"{drydenParamters2} != {Wm.drydenParamters}")
	setwindmodelparatemterstest11()
setWindModelParameterstest()

def CreateDrydenTransferFnstest():
	"""Testing define values for Phi, gamma, and H and their components"""
	def createdrydentransfnstest1():
		cur_test = "create dryden transfer function test 1"
		# Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState()
		drydenParamters3 = Inputs.drydenParameters(Lu=0, Lv=0, Lw=0, sigmau=0, sigmav=0, sigmaw=0)
		dT = 0
		Va = 0
		Wm.CreateDrydenTransferFns(dT, Va, drydenParamters3)
		expectedPhi_u = [[1]]
		expectedPhi_v = [[1, 0], [0, 1]]
		expectedPhi_w = [[1, 0], [0, 1]]
		expectedGamma_u = [[0]]
		expectedGamma_v = [[0], [0]]
		expectedGamma_w = [[0], [0]]
		expectedH_u = [[1]]
		expectedH_v = [[1, 1]]
		expectedH_w = [[1, 1]]
		if not evaluateTest(cur_test, (expectedPhi_u == Wm.Phi_u  and expectedPhi_v == Wm.Phi_v and expectedPhi_w == Wm.Phi_w and
									   expectedGamma_u == Wm.Gamma_u and expectedGamma_v == Wm.Gamma_v and expectedGamma_w == Wm.Gamma_w and
									   expectedH_u == Wm.H_u and expectedH_v == Wm.H_v and expectedH_w == Wm.H_w)):
			print(f"{expectedPhi_u} != {Wm.Phi_u}")
			print(f"{expectedPhi_v} != {Wm.Phi_v}")
			print(f"{expectedPhi_w} != {Wm.Phi_w}")
			print(f"{expectedGamma_u} != {Wm.Gamma_u}")
			print(f"{expectedGamma_v} != {Wm.Gamma_v}")
			print(f"{expectedGamma_w} != {Wm.Gamma_w}")
			print(f"{expectedH_u} != {Wm.H_u}")
			print(f"{expectedH_v} != {Wm.H_v}")
			print(f"{expectedH_w} != {Wm.H_w}")
	createdrydentransfnstest1()

	def createdrydentransfnstest2():
		cur_test = "create dryden transfer function test 2"
		# Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState()
		drydenParamters3 = Inputs.drydenParameters(Lu=1, Lv=2, Lw=3, sigmau=4, sigmav=5, sigmaw=6)
		dT = 0
		Va = 1
		Wm.CreateDrydenTransferFns(dT, Va, drydenParamters3)
		expectedPhi_u = [[1]]
		expectedPhi_v = [[1, 0], [0, 1]]
		expectedPhi_w = [[1, 0], [0, 1]]
		expectedGamma_u = [[0]]
		expectedGamma_v = [[0], [0]]
		expectedGamma_w = [[0], [0]]
		expectedH_u = [[3.1915382432114616]]
		expectedH_v = [[3.454941494713355, 0.9973557010035818]]
		expectedH_w = [[3.385137501286538, 0.65147001587056]]
		if not evaluateTest(cur_test, (expectedPhi_u == Wm.Phi_u  and expectedPhi_v == Wm.Phi_v and expectedPhi_w == Wm.Phi_w and
									   expectedGamma_u == Wm.Gamma_u and expectedGamma_v == Wm.Gamma_v and expectedGamma_w == Wm.Gamma_w and
									   expectedH_u == Wm.H_u and expectedH_v == Wm.H_v and expectedH_w == Wm.H_w)):
			print(f"{expectedPhi_u} != {Wm.Phi_u}")
			print(f"{expectedPhi_v} != {Wm.Phi_v}")
			print(f"{expectedPhi_w} != {Wm.Phi_w}")
			print(f"{expectedGamma_u} != {Wm.Gamma_u}")
			print(f"{expectedGamma_v} != {Wm.Gamma_v}")
			print(f"{expectedGamma_w} != {Wm.Gamma_w}")
			print(f"{expectedH_u} != {Wm.H_u}")
			print(f"{expectedH_v} != {Wm.H_v}")
			print(f"{expectedH_w} != {Wm.H_w}")
	createdrydentransfnstest2()

	def createdrydentransfnstest3():
		cur_test = "create dryden transfer function test 3"
		# Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState()
		drydenParamters3 = Inputs.drydenParameters(Lu=1, Lv=2, Lw=3, sigmau=4, sigmav=5, sigmaw=6)
		dT = 7
		Va = 8
		Wm.CreateDrydenTransferFns(dT, Va, drydenParamters3)
		expectedPhi_u = [[4.780892883885469e-25]]
		expectedPhi_v = [[-1.8668880288738548e-11, -7.744128119773028e-11], [4.840080074858142e-12, 2.005176031012659e-11]]
		expectedPhi_w = [[-1.381415377143731e-07, -3.8922898676754174e-07], [5.4735326264185564e-08, 1.5378020236128323e-07]]
		expectedGamma_u = [[0.125]]
		expectedGamma_v = [[4.840080074858142e-12], [0.06249999999874676]]
		expectedGamma_w = [[5.4735326264185564e-08], [0.14062497837465907]]
		expectedH_u = [[9.0270333367641]]
		expectedH_v = [[9.772050238058398, 22.567583341910254]]
		expectedH_w = [[9.574614729634385, 14.741083710776982]]
		if not evaluateTest(cur_test, ((expectedPhi_u == Wm.Phi_u)  and expectedPhi_v == Wm.Phi_v and expectedPhi_w == Wm.Phi_w and
									   expectedGamma_u == Wm.Gamma_u and expectedGamma_v == Wm.Gamma_v and expectedGamma_w == Wm.Gamma_w and
									   expectedH_u == Wm.H_u and expectedH_v == Wm.H_v and expectedH_w == Wm.H_w)):
			print(f"{expectedPhi_u} != {Wm.Phi_u}")
			print(f"{expectedPhi_v} != {Wm.Phi_v}")
			print(f"{expectedPhi_w} != {Wm.Phi_w}")
			print(f"{expectedGamma_u} != {Wm.Gamma_u}")
			print(f"{expectedGamma_v} != {Wm.Gamma_v}")
			print(f"{expectedGamma_w} != {Wm.Gamma_w}")
			print(f"{expectedH_u} != {Wm.H_u}")
			print(f"{expectedH_v} != {Wm.H_v}")
			print(f"{expectedH_w} != {Wm.H_w}")
	createdrydentransfnstest3()
CreateDrydenTransferFnstest()

def getDrydenTransferFnstest():
	"""Testing the data from CreateDrydenTransferFns that are stored in init"""
	def getdrydentransfunstest1():
		cur_test = "get Dryden Transfer Fns test 1"
		# Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState()

		# creating values
		Wm.Phi_u = 0
		Wm.Phi_v = 0
		Wm.Phi_w = 0
		Wm.Gamma_u = 0
		Wm.Gamma_v = 0
		Wm.Gamma_w = 0
		Wm.H_u = 0
		Wm.H_v = 0
		Wm.H_w = 0

		# Instance of getDrydenTransferFns
		Phi_u, Gamma_u, H_u, Phi_v, Gamma_v, H_v, Phi_w, Gamma_w, H_w = Wm.getDrydenTransferFns()

		if not evaluateTest(cur_test,
							(Phi_u == Wm.Phi_u and Phi_v == Wm.Phi_v and Phi_w == Wm.Phi_w and
							 Gamma_u == Wm.Gamma_u and Gamma_v == Wm.Gamma_v and Gamma_w == Wm.Gamma_w and
							 H_u == Wm.H_u and H_v == Wm.H_v and H_w == Wm.H_w)):
			print(f"{Phi_u} != {Wm.Phi_u}")
			print(f"{Phi_v} != {Wm.Phi_v}")
			print(f"{Phi_w} != {Wm.Phi_w}")
			print(f"{Gamma_u} != {Wm.Gamma_u}")
			print(f"{Gamma_v} != {Wm.Gamma_v}")
			print(f"{Gamma_w} != {Wm.Gamma_w}")
			print(f"{H_u} != {Wm.H_u}")
			print(f"{H_v} != {Wm.H_v}")
			print(f"{H_w} != {Wm.H_w}")
	getdrydentransfunstest1()

	def getdrydentransfunstest2():
		cur_test = "get Dryden Transfer Fns test 2"
		# Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState()

		# creating values
		Wm.Phi_u = 1
		Wm.Phi_v = 0
		Wm.Phi_w = 0
		Wm.Gamma_u = 0
		Wm.Gamma_v = 0
		Wm.Gamma_w = 0
		Wm.H_u = 0
		Wm.H_v = 0
		Wm.H_w = 0

		# Instance of getDrydenTransferFns
		Phi_u, Gamma_u, H_u, Phi_v, Gamma_v, H_v, Phi_w, Gamma_w, H_w = Wm.getDrydenTransferFns()

		if not evaluateTest(cur_test,
							(Phi_u == Wm.Phi_u and Phi_v == Wm.Phi_v and Phi_w == Wm.Phi_w and
							 Gamma_u == Wm.Gamma_u and Gamma_v == Wm.Gamma_v and Gamma_w == Wm.Gamma_w and
							 H_u == Wm.H_u and H_v == Wm.H_v and H_w == Wm.H_w)):
			print(f"{Phi_u} != {Wm.Phi_u}")
			print(f"{Phi_v} != {Wm.Phi_v}")
			print(f"{Phi_w} != {Wm.Phi_w}")
			print(f"{Gamma_u} != {Wm.Gamma_u}")
			print(f"{Gamma_v} != {Wm.Gamma_v}")
			print(f"{Gamma_w} != {Wm.Gamma_w}")
			print(f"{H_u} != {Wm.H_u}")
			print(f"{H_v} != {Wm.H_v}")
			print(f"{H_w} != {Wm.H_w}")
	getdrydentransfunstest2()

	def getdrydentransfunstest3():
		cur_test = "get Dryden Transfer Fns test 3"
		# Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState()

		# creating values
		Wm.Phi_u = 0
		Wm.Phi_v = 1
		Wm.Phi_w = 0
		Wm.Gamma_u = 0
		Wm.Gamma_v = 0
		Wm.Gamma_w = 0
		Wm.H_u = 0
		Wm.H_v = 0
		Wm.H_w = 0

		# Instance of getDrydenTransferFns
		Phi_u, Gamma_u, H_u, Phi_v, Gamma_v, H_v, Phi_w, Gamma_w, H_w = Wm.getDrydenTransferFns()

		if not evaluateTest(cur_test,
							(Phi_u == Wm.Phi_u and Phi_v == Wm.Phi_v and Phi_w == Wm.Phi_w and
							 Gamma_u == Wm.Gamma_u and Gamma_v == Wm.Gamma_v and Gamma_w == Wm.Gamma_w and
							 H_u == Wm.H_u and H_v == Wm.H_v and H_w == Wm.H_w)):
			print(f"{Phi_u} != {Wm.Phi_u}")
			print(f"{Phi_v} != {Wm.Phi_v}")
			print(f"{Phi_w} != {Wm.Phi_w}")
			print(f"{Gamma_u} != {Wm.Gamma_u}")
			print(f"{Gamma_v} != {Wm.Gamma_v}")
			print(f"{Gamma_w} != {Wm.Gamma_w}")
			print(f"{H_u} != {Wm.H_u}")
			print(f"{H_v} != {Wm.H_v}")
			print(f"{H_w} != {Wm.H_w}")
	getdrydentransfunstest3()

	def getdrydentransfunstest4():
		cur_test = "get Dryden Transfer Fns test 4"
		# Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState()

		# creating values
		Wm.Phi_u = 0
		Wm.Phi_v = 0
		Wm.Phi_w = 1
		Wm.Gamma_u = 0
		Wm.Gamma_v = 0
		Wm.Gamma_w = 0
		Wm.H_u = 0
		Wm.H_v = 0
		Wm.H_w = 0

		# Instance of getDrydenTransferFns
		Phi_u, Gamma_u, H_u, Phi_v, Gamma_v, H_v, Phi_w, Gamma_w, H_w = Wm.getDrydenTransferFns()

		if not evaluateTest(cur_test,
							(Phi_u == Wm.Phi_u and Phi_v == Wm.Phi_v and Phi_w == Wm.Phi_w and
							 Gamma_u == Wm.Gamma_u and Gamma_v == Wm.Gamma_v and Gamma_w == Wm.Gamma_w and
							 H_u == Wm.H_u and H_v == Wm.H_v and H_w == Wm.H_w)):
			print(f"{Phi_u} != {Wm.Phi_u}")
			print(f"{Phi_v} != {Wm.Phi_v}")
			print(f"{Phi_w} != {Wm.Phi_w}")
			print(f"{Gamma_u} != {Wm.Gamma_u}")
			print(f"{Gamma_v} != {Wm.Gamma_v}")
			print(f"{Gamma_w} != {Wm.Gamma_w}")
			print(f"{H_u} != {Wm.H_u}")
			print(f"{H_v} != {Wm.H_v}")
			print(f"{H_w} != {Wm.H_w}")
	getdrydentransfunstest4()

	def getdrydentransfunstest5():
		cur_test = "get Dryden Transfer Fns test 5"
		# Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState()

		# creating values
		Wm.Phi_u = 0
		Wm.Phi_v = 0
		Wm.Phi_w = 0
		Wm.Gamma_u = 1
		Wm.Gamma_v = 0
		Wm.Gamma_w = 0
		Wm.H_u = 0
		Wm.H_v = 0
		Wm.H_w = 0

		# Instance of getDrydenTransferFns
		Phi_u, Gamma_u, H_u, Phi_v, Gamma_v, H_v, Phi_w, Gamma_w, H_w = Wm.getDrydenTransferFns()

		if not evaluateTest(cur_test,
							(Phi_u == Wm.Phi_u and Phi_v == Wm.Phi_v and Phi_w == Wm.Phi_w and
							 Gamma_u == Wm.Gamma_u and Gamma_v == Wm.Gamma_v and Gamma_w == Wm.Gamma_w and
							 H_u == Wm.H_u and H_v == Wm.H_v and H_w == Wm.H_w)):
			print(f"{Phi_u} != {Wm.Phi_u}")
			print(f"{Phi_v} != {Wm.Phi_v}")
			print(f"{Phi_w} != {Wm.Phi_w}")
			print(f"{Gamma_u} != {Wm.Gamma_u}")
			print(f"{Gamma_v} != {Wm.Gamma_v}")
			print(f"{Gamma_w} != {Wm.Gamma_w}")
			print(f"{H_u} != {Wm.H_u}")
			print(f"{H_v} != {Wm.H_v}")
			print(f"{H_w} != {Wm.H_w}")
	getdrydentransfunstest5()

	def getdrydentransfunstest6():
		cur_test = "get Dryden Transfer Fns test 6"
		# Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState()

		# creating values
		Wm.Phi_u = 0
		Wm.Phi_v = 0
		Wm.Phi_w = 0
		Wm.Gamma_u = 0
		Wm.Gamma_v = 1
		Wm.Gamma_w = 0
		Wm.H_u = 0
		Wm.H_v = 0
		Wm.H_w = 0

		# Instance of getDrydenTransferFns
		Phi_u, Gamma_u, H_u, Phi_v, Gamma_v, H_v, Phi_w, Gamma_w, H_w = Wm.getDrydenTransferFns()

		if not evaluateTest(cur_test,
							(Phi_u == Wm.Phi_u and Phi_v == Wm.Phi_v and Phi_w == Wm.Phi_w and
							 Gamma_u == Wm.Gamma_u and Gamma_v == Wm.Gamma_v and Gamma_w == Wm.Gamma_w and
							 H_u == Wm.H_u and H_v == Wm.H_v and H_w == Wm.H_w)):
			print(f"{Phi_u} != {Wm.Phi_u}")
			print(f"{Phi_v} != {Wm.Phi_v}")
			print(f"{Phi_w} != {Wm.Phi_w}")
			print(f"{Gamma_u} != {Wm.Gamma_u}")
			print(f"{Gamma_v} != {Wm.Gamma_v}")
			print(f"{Gamma_w} != {Wm.Gamma_w}")
			print(f"{H_u} != {Wm.H_u}")
			print(f"{H_v} != {Wm.H_v}")
			print(f"{H_w} != {Wm.H_w}")
	getdrydentransfunstest6()

	def getdrydentransfunstest7():
		cur_test = "get Dryden Transfer Fns test 7"
		# Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState()

		# creating values
		Wm.Phi_u = 0
		Wm.Phi_v = 0
		Wm.Phi_w = 0
		Wm.Gamma_u = 0
		Wm.Gamma_v = 0
		Wm.Gamma_w = 1
		Wm.H_u = 0
		Wm.H_v = 0
		Wm.H_w = 0

		# Instance of getDrydenTransferFns
		Phi_u, Gamma_u, H_u, Phi_v, Gamma_v, H_v, Phi_w, Gamma_w, H_w = Wm.getDrydenTransferFns()

		if not evaluateTest(cur_test,
							(Phi_u == Wm.Phi_u and Phi_v == Wm.Phi_v and Phi_w == Wm.Phi_w and
							 Gamma_u == Wm.Gamma_u and Gamma_v == Wm.Gamma_v and Gamma_w == Wm.Gamma_w and
							 H_u == Wm.H_u and H_v == Wm.H_v and H_w == Wm.H_w)):
			print(f"{Phi_u} != {Wm.Phi_u}")
			print(f"{Phi_v} != {Wm.Phi_v}")
			print(f"{Phi_w} != {Wm.Phi_w}")
			print(f"{Gamma_u} != {Wm.Gamma_u}")
			print(f"{Gamma_v} != {Wm.Gamma_v}")
			print(f"{Gamma_w} != {Wm.Gamma_w}")
			print(f"{H_u} != {Wm.H_u}")
			print(f"{H_v} != {Wm.H_v}")
			print(f"{H_w} != {Wm.H_w}")
	getdrydentransfunstest7()

	def getdrydentransfunstest8():
		cur_test = "get Dryden Transfer Fns test 8"
		# Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState()

		# creating values
		Wm.Phi_u = 0
		Wm.Phi_v = 0
		Wm.Phi_w = 0
		Wm.Gamma_u = 0
		Wm.Gamma_v = 0
		Wm.Gamma_w = 0
		Wm.H_u = 1
		Wm.H_v = 0
		Wm.H_w = 0

		# Instance of getDrydenTransferFns
		Phi_u, Gamma_u, H_u, Phi_v, Gamma_v, H_v, Phi_w, Gamma_w, H_w = Wm.getDrydenTransferFns()

		if not evaluateTest(cur_test,
							(Phi_u == Wm.Phi_u and Phi_v == Wm.Phi_v and Phi_w == Wm.Phi_w and
							 Gamma_u == Wm.Gamma_u and Gamma_v == Wm.Gamma_v and Gamma_w == Wm.Gamma_w and
							 H_u == Wm.H_u and H_v == Wm.H_v and H_w == Wm.H_w)):
			print(f"{Phi_u} != {Wm.Phi_u}")
			print(f"{Phi_v} != {Wm.Phi_v}")
			print(f"{Phi_w} != {Wm.Phi_w}")
			print(f"{Gamma_u} != {Wm.Gamma_u}")
			print(f"{Gamma_v} != {Wm.Gamma_v}")
			print(f"{Gamma_w} != {Wm.Gamma_w}")
			print(f"{H_u} != {Wm.H_u}")
			print(f"{H_v} != {Wm.H_v}")
			print(f"{H_w} != {Wm.H_w}")
	getdrydentransfunstest8()

	def getdrydentransfunstest9():
		cur_test = "get Dryden Transfer Fns test 9"
		# Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState()

		# creating values
		Wm.Phi_u = 0
		Wm.Phi_v = 0
		Wm.Phi_w = 0
		Wm.Gamma_u = 0
		Wm.Gamma_v = 0
		Wm.Gamma_w = 0
		Wm.H_u = 0
		Wm.H_v = 1
		Wm.H_w = 0

		# Instance of getDrydenTransferFns
		Phi_u, Gamma_u, H_u, Phi_v, Gamma_v, H_v, Phi_w, Gamma_w, H_w = Wm.getDrydenTransferFns()

		if not evaluateTest(cur_test,
							(Phi_u == Wm.Phi_u and Phi_v == Wm.Phi_v and Phi_w == Wm.Phi_w and
							 Gamma_u == Wm.Gamma_u and Gamma_v == Wm.Gamma_v and Gamma_w == Wm.Gamma_w and
							 H_u == Wm.H_u and H_v == Wm.H_v and H_w == Wm.H_w)):
			print(f"{Phi_u} != {Wm.Phi_u}")
			print(f"{Phi_v} != {Wm.Phi_v}")
			print(f"{Phi_w} != {Wm.Phi_w}")
			print(f"{Gamma_u} != {Wm.Gamma_u}")
			print(f"{Gamma_v} != {Wm.Gamma_v}")
			print(f"{Gamma_w} != {Wm.Gamma_w}")
			print(f"{H_u} != {Wm.H_u}")
			print(f"{H_v} != {Wm.H_v}")
			print(f"{H_w} != {Wm.H_w}")
	getdrydentransfunstest9()

	def getdrydentransfunstest10():
		cur_test = "get Dryden Transfer Fns test 10"
		# Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState()

		# creating values
		Wm.Phi_u = 0
		Wm.Phi_v = 0
		Wm.Phi_w = 0
		Wm.Gamma_u = 0
		Wm.Gamma_v = 0
		Wm.Gamma_w = 0
		Wm.H_u = 0
		Wm.H_v = 1
		Wm.H_w = 0

		# Instance of getDrydenTransferFns
		Phi_u, Gamma_u, H_u, Phi_v, Gamma_v, H_v, Phi_w, Gamma_w, H_w = Wm.getDrydenTransferFns()

		if not evaluateTest(cur_test,
							(Phi_u == Wm.Phi_u and Phi_v == Wm.Phi_v and Phi_w == Wm.Phi_w and
							 Gamma_u == Wm.Gamma_u and Gamma_v == Wm.Gamma_v and Gamma_w == Wm.Gamma_w and
							 H_u == Wm.H_u and H_v == Wm.H_v and H_w == Wm.H_w)):
			print(f"{Phi_u} != {Wm.Phi_u}")
			print(f"{Phi_v} != {Wm.Phi_v}")
			print(f"{Phi_w} != {Wm.Phi_w}")
			print(f"{Gamma_u} != {Wm.Gamma_u}")
			print(f"{Gamma_v} != {Wm.Gamma_v}")
			print(f"{Gamma_w} != {Wm.Gamma_w}")
			print(f"{H_u} != {Wm.H_u}")
			print(f"{H_v} != {Wm.H_v}")
			print(f"{H_w} != {Wm.H_w}")
	getdrydentransfunstest10()

	def getdrydentransfunstest11():
		cur_test = "get Dryden Transfer Fns test 11"
		# Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState()

		# creating values
		Wm.Phi_u = 0
		Wm.Phi_v = 0
		Wm.Phi_w = 0
		Wm.Gamma_u = 0
		Wm.Gamma_v = 0
		Wm.Gamma_w = 0
		Wm.H_u = 0
		Wm.H_v = 0
		Wm.H_w = 1

		# Instance of getDrydenTransferFns
		Phi_u, Gamma_u, H_u, Phi_v, Gamma_v, H_v, Phi_w, Gamma_w, H_w = Wm.getDrydenTransferFns()

		if not evaluateTest(cur_test,
							(Phi_u == Wm.Phi_u and Phi_v == Wm.Phi_v and Phi_w == Wm.Phi_w and
							 Gamma_u == Wm.Gamma_u and Gamma_v == Wm.Gamma_v and Gamma_w == Wm.Gamma_w and
							 H_u == Wm.H_u and H_v == Wm.H_v and H_w == Wm.H_w)):
			print(f"{Phi_u} != {Wm.Phi_u}")
			print(f"{Phi_v} != {Wm.Phi_v}")
			print(f"{Phi_w} != {Wm.Phi_w}")
			print(f"{Gamma_u} != {Wm.Gamma_u}")
			print(f"{Gamma_v} != {Wm.Gamma_v}")
			print(f"{Gamma_w} != {Wm.Gamma_w}")
			print(f"{H_u} != {Wm.H_u}")
			print(f"{H_v} != {Wm.H_v}")
			print(f"{H_w} != {Wm.H_w}")
	getdrydentransfunstest11()

	def getdrydentransfunstest12():
		cur_test = "get Dryden Transfer Fns test 12"
		# Initializing class and states
		Wm = WM.WindModel()
		windState = States.windState()

		# creating values
		Wm.Phi_u = 1
		Wm.Phi_v = 2
		Wm.Phi_w = 3
		Wm.Gamma_u = 4
		Wm.Gamma_v = 5
		Wm.Gamma_w = 6
		Wm.H_u = 7
		Wm.H_v = 8
		Wm.H_w = 9

		# Instance of getDrydenTransferFns
		Phi_u, Gamma_u, H_u, Phi_v, Gamma_v, H_v, Phi_w, Gamma_w, H_w = Wm.getDrydenTransferFns()

		if not evaluateTest(cur_test,
							(Phi_u == Wm.Phi_u and Phi_v == Wm.Phi_v and Phi_w == Wm.Phi_w and
							 Gamma_u == Wm.Gamma_u and Gamma_v == Wm.Gamma_v and Gamma_w == Wm.Gamma_w and
							 H_u == Wm.H_u and H_v == Wm.H_v and H_w == Wm.H_w)):
			print(f"{Phi_u} != {Wm.Phi_u}")
			print(f"{Phi_v} != {Wm.Phi_v}")
			print(f"{Phi_w} != {Wm.Phi_w}")
			print(f"{Gamma_u} != {Wm.Gamma_u}")
			print(f"{Gamma_v} != {Wm.Gamma_v}")
			print(f"{Gamma_w} != {Wm.Gamma_w}")
			print(f"{H_u} != {Wm.H_u}")
			print(f"{H_v} != {Wm.H_v}")
			print(f"{H_w} != {Wm.H_w}")
	getdrydentransfunstest12()
getDrydenTransferFnstest()

def Updatetest():
	"""Testing whether x updates a step accordingly"""
	def updatetestWu():
		cur_test = "Update test Wu"
		Wm = WM.WindModel(drydenParamters=VPC.DrydenLowAltitudeLight)
		listWu = []
		listWu2 = []
		for i in range(1000):
			Wm.Update()
			listWu.append(Wm.windState.Wu)
		Wusum = sum(listWu)
		Wumean = Wusum/1000
		for i in range(1000):
			Wm.Update()
			listWu2.append(Wm.windState.Wu - Wumean)
		SD = sum(listWu2)/1000
		if not evaluateTest(cur_test, (Wumean <= 0.1 ) and (SD <= 0.1)):
			print(f"{Wumean} = {0.1}")
			print(f"{SD} = {0.1}")
	updatetestWu()

	def updatetestWv():
		cur_test = "Update test Wv"
		Wm = WM.WindModel(drydenParamters=VPC.DrydenLowAltitudeLight)
		listWv = []
		listWv2 = []
		for i in range(1000):
			Wm.Update()
			listWv.append(Wm.windState.Wv)
		Wvsum = sum(listWv)
		Wvmean = Wvsum/1000
		for i in range(1000):
			Wm.Update()
			listWv2.append(Wm.windState.Wu - Wvmean)
		SD = sum(listWv2)/1000
		if not evaluateTest(cur_test, (Wvmean <= 0.1 ) and (SD <= 0.1)):
			print(f"{Wvmean} = {0.1}")
			print(f"{SD} = {0.1}")
	updatetestWv()

	def updatetestWw():
		cur_test = "Update test Ww"
		Wm = WM.WindModel(drydenParamters=VPC.DrydenLowAltitudeLight)
		listWw = []
		listWw2 = []
		for i in range(1000):
			Wm.Update()
			listWw.append(Wm.windState.Ww)
		Wwsum = sum(listWw)
		Wwmean = Wwsum/1000
		for i in range(1000):
			Wm.Update()
			listWw2.append(Wm.windState.Ww - Wwmean)
		SD = sum(listWw2)/1000
		if not evaluateTest(cur_test, (Wwmean <= 0.1 ) and (SD <= 0.1)):
			print(f"{Wwmean} = {0.1}")
			print(f"{SD} = {0.1}")
	updatetestWw()
Updatetest()


#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]