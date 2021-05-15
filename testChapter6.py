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
import ece163.Controls.VehicleClosedLoopControl as VCLC
import ece163.Containers.States as States
import ece163.Containers.Inputs as Inputs
import ece163.Controls.VehicleTrim as VT
import ece163.Containers.Controls as Controls
import ece163.Controls.VehicleControlGains as VCG
import ece163.Containers.Linearized as Linearized
from matplotlib import pyplot as plt



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


print("Beginning testing of VehicleClosedLoop")

def PDControltest():
	print("Beginning testing of PDControl")

	def updatetest():
		def updatetest1():
			"""Testing update from PDControl"""
			cur_test = "update test 1"
			# creating istances
			Vcl = VCLC.PDControl()

			# Inputing vaues into function
			Vcl.kp = 13
			Vcl.kd = 4
			Vcl.trim = 77
			Vcl.highLimit = 1000
			Vcl.lowLimit = -1000
			actual = Vcl.Update(command = 60, current = 2, derivative = 1)

			# expected u
			expected_u = 827

			if not evaluateTest(cur_test, (actual == expected_u)):
				print(f"{actual} != {expected_u}")
		updatetest1()

		def updatetest2():
			"""Testing update from PDControl"""
			cur_test = "update test 2"
			# creating istances
			Vcl = VCLC.PDControl()

			# Inputing vaues into function
			Vcl.kp = 1
			Vcl.kd = 0.66
			Vcl.trim = -5
			Vcl.highLimit = 100
			Vcl.lowLimit = -10
			actual = Vcl.Update(command = 20, current = -2, derivative = 0.77)

			# expected u
			expected_u = 16.4918

			if not evaluateTest(cur_test, (actual == expected_u)):
				print(f"{actual} != {expected_u}")
		updatetest2()
	updatetest()

	def setPDGainstest():
		"""testing setPDGains from PDControls"""
		def setPDGaintest1():
			#"""Testing setPDGain from PDControl"""
			cur_test = "setPDGain test 1"
			# creating instances
			Vcl = VCLC.PDControl()

			#inputting valeus into function
			Vcl.setPDGains(kp = 1, kd = 2, trim = 3, lowLimit = 4, highLimit = 5)

			#expected values
			expected_kp = 1
			expected_kd = 2
			expected_trim = 3
			expected_lowLimit = 4
			expected_highLimit = 5

			if not evaluateTest(cur_test, (Vcl.kp == expected_kp) and
										  (Vcl.kd == expected_kd) and
										  (Vcl.trim == expected_trim) and
										  (Vcl.lowLimit == expected_lowLimit) and
										  (Vcl.highLimit == expected_highLimit)):
				print(f"{Vcl.kp} != {expected_kp}")
				print(f"{Vcl.kd} != {expected_kd}")
				print(f"{Vcl.trim} != {expected_trim}")
				print(f"{Vcl.lowLimit} != {expected_lowLimit}")
				print(f"{Vcl.highLimit} != {expected_highLimit}")
		setPDGaintest1()
	setPDGainstest()
PDControltest()

def PIControltest():
	print("Beginning testing of PIControl")

	def updatetest():
		"""Testing update from PIControl"""

		def updatetest1():
			"""Testing update from PDControl"""
			cur_test = "update test 1"
			# creating istances
			Vcl = VCLC.PIControl()

			# Inputing vaues into function
			Vcl.kp = 13
			Vcl.ki = 4
			Vcl.trim = 77
			Vcl.highLimit = 1000
			Vcl.lowLimit = -1000
			actual = Vcl.Update(command = 60, current = 2)

			# expected u
			expected_u = 832.16

			if not evaluateTest(cur_test, (actual == expected_u)):
				print(f"{actual} != {expected_u}")
		updatetest1()

		def updatetest2():
			"""Testing update from PDControl"""
			cur_test = "update test 2"
			# creating istances
			Vcl = VCLC.PIControl()

			# Inputing vaues into function
			Vcl.kp = 0.01
			Vcl.ki = -5
			Vcl.trim = -.88
			Vcl.highLimit = 50
			Vcl.lowLimit = -240
			actual = Vcl.Update(command = 0.88, current = -1)

			# expected u
			expected_u = -0.9082

			if not evaluateTest(cur_test, (actual == expected_u)):
				print(f"{actual} != {expected_u}")
		updatetest2()
	updatetest()

	def resetIntegratortest():
		"""Testing resetIntegrator from PIControl"""
		def resetIntegratortest1():
			cur_test = "resetIntegrator test q"
			# creating istances
			Vcl = VCLC.PIControl()

			#creating values
			Vcl.accumulator = 1
			Vcl.error = 2
			Vcl.preverror = 3

			#instance of reset
			Vcl.resetIntegrator()

			if not evaluateTest(cur_test, (Vcl.accumulator == 0) and
										  (Vcl.error == 0) and
										  (Vcl.preverror ==0)):
				print(f"{Vcl.accumulator} != {0}")
				print(f"{Vcl.error} != {0}")
				print(f"{Vcl.preverror} != {0}")
		resetIntegratortest1()
	resetIntegratortest()

	def setPIControltest():
		"""testing setPIControl from PIControl"""
		def setPIGaintest1():
			#"""Testing setPDGain from PDControl"""
			cur_test = "setPIGain test 1"
			# creating instances
			Vcl = VCLC.PIControl()

			#inputting valeus into function
			Vcl.setPIGains(kp = 1, ki = 2, trim = 3, lowLimit = 4, highLimit = 5)

			#expected values
			expected_kp = 1
			expected_ki = 2
			expected_trim = 3
			expected_lowLimit = 4
			expected_highLimit = 5

			if not evaluateTest(cur_test, (Vcl.kp == expected_kp) and
										  (Vcl.ki == expected_ki) and
										  (Vcl.trim == expected_trim) and
										  (Vcl.lowLimit == expected_lowLimit) and
										  (Vcl.highLimit == expected_highLimit)):
				print(f"{Vcl.kp} != {expected_kp}")
				print(f"{Vcl.ki} != {expected_ki}")
				print(f"{Vcl.trim} != {expected_trim}")
				print(f"{Vcl.lowLimit} != {expected_lowLimit}")
				print(f"{Vcl.highLimit} != {expected_highLimit}")
		setPIGaintest1()
	setPIControltest()
PIControltest()

def PIDControltests():
	print("Beginning testing of PIDControl")

	def Updatetest():
		"""Test Update from PIDControl"""
		def updatetest1():
			"""Testing update from PIDControl"""
			cur_test = "update test 1"
			# creating istances
			Vcl = VCLC.PIDControl()

			# Inputing vaues into function
			Vcl.kp = 13
			Vcl.kd = 1
			Vcl.ki = 4
			Vcl.trim = 77
			Vcl.highLimit = 1000
			Vcl.lowLimit = -1000
			actual = Vcl.Update(command = 60, current = 2, derivative = -8)

			# expected u
			expected_u = 840.16

			if not evaluateTest(cur_test, (actual == expected_u)):
				print(f"{actual} != {expected_u}")
		updatetest1()

		def updatetest2():
			"""Testing update from PIDControl"""
			cur_test = "update test 2"
			# creating instances
			Vcl = VCLC.PIDControl()

			# Inputing vaues into function
			Vcl.kp = -0.001
			Vcl.kd = 100
			Vcl.ki = -3
			Vcl.trim = 5.5
			Vcl.highLimit = 66
			Vcl.lowLimit = -10
			actual = Vcl.Update(command = 3, current = 2, derivative = -1)

			# expected u
			expected_u = 66

			if not evaluateTest(cur_test, (actual == expected_u)):
				print(f"{actual} != {expected_u}")
		updatetest2()
	Updatetest()

	def resetIntegratortest():
		"""Testing resetIntegrator"""
		def resetIntegratortest1():
			cur_test = "resetIntegrator test 1"
			# creating istances
			Vcl = VCLC.PIDControl()

			#creating values
			Vcl.accumulator = 1
			Vcl.error = 2
			Vcl.preverror = 3

			#instance of reset
			Vcl.resetIntegrator()

			if not evaluateTest(cur_test, (Vcl.accumulator == 0) and
										  (Vcl.error == 0) and
										  (Vcl.preverror ==0)):
				print(f"{Vcl.accumulator} != {0}")
				print(f"{Vcl.error} != {0}")
				print(f"{Vcl.preverror} != {0}")
		resetIntegratortest1()
	resetIntegratortest()

	def setPIDGainstest():
		"""Testing setPIDgains"""
		def setPIDGaintest1():
			# """Testing setPIDGain from PIDControl"""
			cur_test = "setPIDGain test 1"
			# creating instances
			Vcl = VCLC.PIDControl()

			# inputting valeus into function
			Vcl.setPIDGains(kp=1, kd=6, ki=2, trim=3, lowLimit=4, highLimit=5)

			# expected values
			expected_kp = 1
			expected_kd = 6
			expected_ki = 2
			expected_trim = 3
			expected_lowLimit = 4
			expected_highLimit = 5

			if not evaluateTest(cur_test, (Vcl.kp == expected_kp) and
										  (Vcl.ki == expected_ki) and
										  ((Vcl.kd == expected_kd)) and
										  (Vcl.trim == expected_trim) and
										  (Vcl.lowLimit == expected_lowLimit) and
										  (Vcl.highLimit == expected_highLimit)):
				print(f"{Vcl.kp} != {expected_kp}")
				print(f"{Vcl.ki} != {expected_ki}")
				print(f"{Vcl.kd} != {expected_kd}")
				print(f"{Vcl.trim} != {expected_trim}")
				print(f"{Vcl.lowLimit} != {expected_lowLimit}")
				print(f"{Vcl.highLimit} != {expected_highLimit}")
		setPIDGaintest1()
	setPIDGainstest()

PIDControltests()

def VehicleClosedLoopControltests():
	print("Beginning testing of VehicleClosedLoopControl")

	def getControlGainstest():
		"""testing getCOntrolGains from VehicleClosedLoopControl"""
		def getControlGainstest1():
			cur_test = "getControlGain test 1"
			# instances of necessary functions
			Vcl = VCLC.VehicleClosedLoopControl()

			# Lateral Gains inputs
			Vcl.controlGains.kp_roll = 1
			Vcl.controlGains.kd_roll = -2
			Vcl.controlGains.ki_roll = .3
			Vcl.controlGains.kp_sideslip = 4
			Vcl.controlGains.ki_sideslip = -.5
			Vcl.controlGains.kp_course = 6
			Vcl.controlGains.ki_course = 7
			# Longitudinal Gains inputs
			Vcl.controlGains.kp_pitch = .008
			Vcl.controlGains.kd_pitch = -900
			Vcl.controlGains.kp_altitude = 10
			Vcl.controlGains.ki_altitude = 11
			Vcl.controlGains.kp_SpeedfromThrottle = 12
			Vcl.controlGains.ki_SpeedfromThrottle = 13
			Vcl.controlGains.kp_SpeedfromElevator = 14
			Vcl.controlGains.ki_SpeedfromElevator = 15

			#actual value
			actual = Vcl.getControlGains()
			if not evaluateTest(cur_test, (actual.kp_roll == Vcl.controlGains.kp_roll) and
										  (actual.kd_roll == Vcl.controlGains.kd_roll) and
										  (actual.ki_roll == Vcl.controlGains.ki_roll) and
										  (actual.kp_sideslip == Vcl.controlGains.kp_sideslip) and
										  (actual.ki_sideslip == Vcl.controlGains.ki_sideslip) and
										  (actual.kp_course == Vcl.controlGains.kp_course) and
										  (actual.ki_course == Vcl.controlGains.ki_course) and
										  (actual.kp_pitch == Vcl.controlGains.kp_pitch) and
										  (actual.kd_pitch == Vcl.controlGains.kd_pitch) and
										  (actual.kp_altitude == Vcl.controlGains.kp_altitude) and
										  (actual.ki_altitude == Vcl.controlGains.ki_altitude) and
										  (actual.kp_SpeedfromThrottle == Vcl.controlGains.kp_SpeedfromThrottle) and
										  (actual.ki_SpeedfromThrottle == Vcl.controlGains.ki_SpeedfromThrottle) and
										  (actual.kp_SpeedfromElevator == Vcl.controlGains.kp_SpeedfromElevator) and
										  (actual.ki_SpeedfromElevator == Vcl.controlGains.ki_SpeedfromElevator)):
				print(f"{actual.kp_roll} != {Vcl.controlGains.kp_roll}")
				print(f"{actual.kd_roll} != {Vcl.controlGains.kd_roll}")
				print(f"{actual.ki_roll} != {Vcl.controlGains.ki_roll}")
				print(f"{actual.kp_sideslip} != {Vcl.controlGains.kp_sideslip}")
				print(f"{actual.ki_sideslip} != {Vcl.controlGains.ki_sideslip}")
				print(f"{actual.kp_course} != {Vcl.controlGains.kp_course}")
				print(f"{actual.ki_course} != {Vcl.controlGains.ki_course}")
				print(f"{actual.kp_pitch} != {Vcl.controlGains.kp_pitch}")
				print(f"{actual.kd_pitch} != {Vcl.controlGains.kd_pitch}")
				print(f"{actual.kp_altitude} != {Vcl.controlGains.kp_altitude}")
				print(f"{actual.ki_altitude} != {Vcl.controlGains.ki_altitude}")
				print(f"{actual.kp_SpeedfromThrottle} != {Vcl.controlGains.kp_SpeedFromThrottle}")
				print(f"{actual.ki_SpeedfromThrottle} != {Vcl.controlGains.ki_SpeedfromThrottle}")
				print(f"{actual.kp_SpeedfromElevator} != {Vcl.controlGains.kp_SpeedfromElevator}")
				print(f"{actual.ki_SpeedfromElevator} != {Vcl.controlGains.ki_SpeedfromElevator}")
		getControlGainstest1()
	getControlGainstest()

	def getTrimInputstest():
		"""Testing getTrimInputs from VehicleClosedLoopControl"""
		def getTrimInputstest1():
			cur_test = "getTrimInputs test 1"
			# instances of necessary functions
			Vcl = VCLC.VehicleClosedLoopControl()
			expected = Inputs.controlInputs()

			#inputting to function
			Vcl.trimInputs.Throttle = 0
			Vcl.trimInputs.Aileron = 0
			Vcl.trimInputs.Elevator = 0
			Vcl.trimInputs.Rudder = 0
			actual = Vcl.getTrimInputs()

			#expected values
			expected.Throttle = Vcl.trimInputs.Throttle
			expected.Aileron = Vcl.trimInputs.Aileron
			expected.Elevator = Vcl.trimInputs.Elevator
			expected.Rudder = Vcl.trimInputs.Rudder
			if not evaluateTest(cur_test, (actual.Throttle == expected.Throttle) and
										  (actual.Aileron == expected.Aileron) and
										  (actual.Elevator == expected.Elevator) and
										  (actual.Rudder == expected.Rudder)):
				print(f"{actual.Throttle} != {expected.Throttle}")
				print(f"{actual.Aileron} != {expected.Aileron}")
				print(f"{actual.Elevator} != {expected.Elevator}")
				print(f"{actual.Rudder} != {expected.Rudder}")
		getTrimInputstest1()

		def getTrimInputstest2():
			cur_test = "getTrimInputs test 2"
			# instances of necessary functions
			Vcl = VCLC.VehicleClosedLoopControl()
			expected = Inputs.controlInputs()

			#inputting to function
			Vcl.trimInputs.Throttle = 1
			Vcl.trimInputs.Aileron = 0
			Vcl.trimInputs.Elevator = 0
			Vcl.trimInputs.Rudder = 0
			actual = Vcl.getTrimInputs()

			#expected values
			expected.Throttle = Vcl.trimInputs.Throttle
			expected.Aileron = Vcl.trimInputs.Aileron
			expected.Elevator = Vcl.trimInputs.Elevator
			expected.Rudder = Vcl.trimInputs.Rudder
			if not evaluateTest(cur_test, (actual.Throttle == expected.Throttle) and
										  (actual.Aileron == expected.Aileron) and
										  (actual.Elevator == expected.Elevator) and
										  (actual.Rudder == expected.Rudder)):
				print(f"{actual.Throttle} != {expected.Throttle}")
				print(f"{actual.Aileron} != {expected.Aileron}")
				print(f"{actual.Elevator} != {expected.Elevator}")
				print(f"{actual.Rudder} != {expected.Rudder}")
		getTrimInputstest2()

		def getTrimInputstest3():
			cur_test = "getTrimInputs test 3"
			# instances of necessary functions
			Vcl = VCLC.VehicleClosedLoopControl()
			expected = Inputs.controlInputs()

			#inputting to function
			Vcl.trimInputs.Throttle = 0
			Vcl.trimInputs.Aileron = -1
			Vcl.trimInputs.Elevator = 0
			Vcl.trimInputs.Rudder = 0
			actual = Vcl.getTrimInputs()

			#expected values
			expected.Throttle = Vcl.trimInputs.Throttle
			expected.Aileron = Vcl.trimInputs.Aileron
			expected.Elevator = Vcl.trimInputs.Elevator
			expected.Rudder = Vcl.trimInputs.Rudder
			if not evaluateTest(cur_test, (actual.Throttle == expected.Throttle) and
										  (actual.Aileron == expected.Aileron) and
										  (actual.Elevator == expected.Elevator) and
										  (actual.Rudder == expected.Rudder)):
				print(f"{actual.Throttle} != {expected.Throttle}")
				print(f"{actual.Aileron} != {expected.Aileron}")
				print(f"{actual.Elevator} != {expected.Elevator}")
				print(f"{actual.Rudder} != {expected.Rudder}")
		getTrimInputstest3()

		def getTrimInputstest4():
			cur_test = "getTrimInputs test 4"
			# instances of necessary functions
			Vcl = VCLC.VehicleClosedLoopControl()
			expected = Inputs.controlInputs()

			#inputting to function
			Vcl.trimInputs.Throttle = 0
			Vcl.trimInputs.Aileron = 0
			Vcl.trimInputs.Elevator = 100
			Vcl.trimInputs.Rudder = 0
			actual = Vcl.getTrimInputs()

			#expected values
			expected.Throttle = Vcl.trimInputs.Throttle
			expected.Aileron = Vcl.trimInputs.Aileron
			expected.Elevator = Vcl.trimInputs.Elevator
			expected.Rudder = Vcl.trimInputs.Rudder
			if not evaluateTest(cur_test, (actual.Throttle == expected.Throttle) and
										  (actual.Aileron == expected.Aileron) and
										  (actual.Elevator == expected.Elevator) and
										  (actual.Rudder == expected.Rudder)):
				print(f"{actual.Throttle} != {expected.Throttle}")
				print(f"{actual.Aileron} != {expected.Aileron}")
				print(f"{actual.Elevator} != {expected.Elevator}")
				print(f"{actual.Rudder} != {expected.Rudder}")
		getTrimInputstest4()

		def getTrimInputstest5():
			cur_test = "getTrimInputs test 5"
			# instances of necessary functions
			Vcl = VCLC.VehicleClosedLoopControl()
			expected = Inputs.controlInputs()

			#inputting to function
			Vcl.trimInputs.Throttle = 0
			Vcl.trimInputs.Aileron = 0
			Vcl.trimInputs.Elevator = 0
			Vcl.trimInputs.Rudder = -35
			actual = Vcl.getTrimInputs()

			#expected values
			expected.Throttle = Vcl.trimInputs.Throttle
			expected.Aileron = Vcl.trimInputs.Aileron
			expected.Elevator = Vcl.trimInputs.Elevator
			expected.Rudder = Vcl.trimInputs.Rudder
			if not evaluateTest(cur_test, (actual.Throttle == expected.Throttle) and
										  (actual.Aileron == expected.Aileron) and
										  (actual.Elevator == expected.Elevator) and
										  (actual.Rudder == expected.Rudder)):
				print(f"{actual.Throttle} != {expected.Throttle}")
				print(f"{actual.Aileron} != {expected.Aileron}")
				print(f"{actual.Elevator} != {expected.Elevator}")
				print(f"{actual.Rudder} != {expected.Rudder}")
		getTrimInputstest5()

		def getTrimInputstest6():
			cur_test = "getTrimInputs test 1"
			# instances of necessary functions
			Vcl = VCLC.VehicleClosedLoopControl()
			expected = Inputs.controlInputs()

			#inputting to function
			Vcl.trimInputs.Throttle = 12
			Vcl.trimInputs.Aileron = -000.1
			Vcl.trimInputs.Elevator = -44
			Vcl.trimInputs.Rudder = 1000
			actual = Vcl.getTrimInputs()

			#expected values
			expected.Throttle = Vcl.trimInputs.Throttle
			expected.Aileron = Vcl.trimInputs.Aileron
			expected.Elevator = Vcl.trimInputs.Elevator
			expected.Rudder = Vcl.trimInputs.Rudder
			if not evaluateTest(cur_test, (actual.Throttle == expected.Throttle) and
										  (actual.Aileron == expected.Aileron) and
										  (actual.Elevator == expected.Elevator) and
										  (actual.Rudder == expected.Rudder)):
				print(f"{actual.Throttle} != {expected.Throttle}")
				print(f"{actual.Aileron} != {expected.Aileron}")
				print(f"{actual.Elevator} != {expected.Elevator}")
				print(f"{actual.Rudder} != {expected.Rudder}")
		getTrimInputstest6()
	getTrimInputstest()

	def getVehicleAerodynamicsModeltest():
		"""Testing getVehicleAerodynamicsModel from VehicleCLosedLoopControl"""
		def getVehicleAerodynamicsModeltest1():
			cur_test = "getVehcileAerodynamicsModel test 1"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()
			wind = States.windState()

			# assigning values
			Vcl.VAmodel.vehicleDynamicsModel.state.pn = 1
			Vcl.VAmodel.WindModel.windState.Wn = 1
			actual = Vcl.getVehicleAerodynamicsModel()
			state.pn = 1
			wind.Wn = 1

			if not evaluateTest(cur_test, (actual.WindModel.windState.Wn == wind.Wn) and (
					actual.vehicleDynamicsModel.state.pn == state.pn)):
				print(f"{actual.WindModel.windState.Wn} != {wind.Wn}")
				print(f"{actual.vehicleDynamicsModel.state.pn} != {state.pn}")
		getVehicleAerodynamicsModeltest1()

		def getVehicleAerodynamicsModeltest2():
			cur_test = "getVehcileAerodynamicsModel test 2"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()
			wind = States.windState()

			# assigning values
			Vcl.VAmodel.vehicleDynamicsModel.state.pe = 2
			Vcl.VAmodel.WindModel.windState.We = 2
			actual = Vcl.getVehicleAerodynamicsModel()
			state.pe = 2
			wind.We = 2

			if not evaluateTest(cur_test, (actual.WindModel.windState.We == wind.We) and (
					actual.vehicleDynamicsModel.state.pe == state.pe)):
				print(f"{actual.WindModel.windState.We} != {wind.We}")
				print(f"{actual.vehicleDynamicsModel.state.pe} != {state.pe}")
		getVehicleAerodynamicsModeltest2()

		def getVehicleAerodynamicsModeltest3():
			cur_test = "getVehcileAerodynamicsModel test 3"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()
			wind = States.windState()

			# assigning values
			Vcl.VAmodel.vehicleDynamicsModel.state.pd = 3
			Vcl.VAmodel.WindModel.windState.Wd = 3
			actual = Vcl.getVehicleAerodynamicsModel()
			state.pd = 3
			wind.Wd = 3

			if not evaluateTest(cur_test, (actual.WindModel.windState.Wd == wind.Wd) and (
					actual.vehicleDynamicsModel.state.pd == state.pd)):
				print(f"{actual.WindModel.windState.Wd} != {wind.Wd}")
				print(f"{actual.vehicleDynamicsModel.state.pd} != {state.pd}")
		getVehicleAerodynamicsModeltest3()

		def getVehicleAerodynamicsModeltest4():
			cur_test = "getVehcileAerodynamicsModel test 4"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()
			wind = States.windState()

			# assigning values
			Vcl.VAmodel.vehicleDynamicsModel.state.u = 4
			Vcl.VAmodel.WindModel.windState.Wu = 4
			actual = Vcl.getVehicleAerodynamicsModel()
			state.u = 4
			wind.Wu = 4

			if not evaluateTest(cur_test, (actual.WindModel.windState.Wu == wind.Wu) and (
					actual.vehicleDynamicsModel.state.u == state.u)):
				print(f"{actual.WindModel.windState.Wu} != {wind.Wu}")
				print(f"{actual.vehicleDynamicsModel.state.u} != {state.u}")
		getVehicleAerodynamicsModeltest4()

		def getVehicleAerodynamicsModeltest5():
			cur_test = "getVehcileAerodynamicsModel test 5"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()
			wind = States.windState()

			# assigning values
			Vcl.VAmodel.vehicleDynamicsModel.state.v = 5
			Vcl.VAmodel.WindModel.windState.Wv = 5
			actual = Vcl.getVehicleAerodynamicsModel()
			state.v = 5
			wind.Wv = 5

			if not evaluateTest(cur_test, (actual.WindModel.windState.Wv == wind.Wv) and (
					actual.vehicleDynamicsModel.state.v == state.v)):
				print(f"{actual.WindModel.windState.Wv} != {wind.Wv}")
				print(f"{actual.vehicleDynamicsModel.state.v} != {state.v}")
		getVehicleAerodynamicsModeltest5()

		def getVehicleAerodynamicsModeltest6():
			cur_test = "getVehcileAerodynamicsModel test 6"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()
			wind = States.windState()

			# assigning values
			Vcl.VAmodel.vehicleDynamicsModel.state.w = 6
			Vcl.VAmodel.WindModel.windState.Ww = 6
			actual = Vcl.getVehicleAerodynamicsModel()
			state.w = 6
			wind.Ww = 6

			if not evaluateTest(cur_test, (actual.WindModel.windState.Ww == wind.Ww) and (
					actual.vehicleDynamicsModel.state.w == state.w)):
				print(f"{actual.WindModel.windState.Ww} != {wind.Ww}")
				print(f"{actual.vehicleDynamicsModel.state.w} != {state.w}")
		getVehicleAerodynamicsModeltest6()

		def getVehicleAerodynamicsModeltest7():
			cur_test = "getVehcileAerodynamicsModel test 7"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()
			wind = States.windState()

			# assigning values
			Vcl.VAmodel.vehicleDynamicsModel.state.yaw = 7
			actual = Vcl.getVehicleAerodynamicsModel()
			state.yaw = 7

			if not evaluateTest(cur_test, (actual.vehicleDynamicsModel.state.yaw == state.yaw)):
				print(f"{actual.vehicleDynamicsModel.state.yaw} != {state.yaw}")
		getVehicleAerodynamicsModeltest7()

		def getVehicleAerodynamicsModeltest8():
			cur_test = "getVehcileAerodynamicsModel test 8"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()
			wind = States.windState()

			# assigning values
			Vcl.VAmodel.vehicleDynamicsModel.state.pitch = 8
			actual = Vcl.getVehicleAerodynamicsModel()
			state.pitch = 8

			if not evaluateTest(cur_test, (actual.vehicleDynamicsModel.state.pitch == state.pitch)):
				print(f"{actual.vehicleDynamicsModel.state.pitch} != {state.pitch}")
		getVehicleAerodynamicsModeltest8()

		def getVehicleAerodynamicsModeltest9():
			cur_test = "getVehcileAerodynamicsModel test 9"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()
			wind = States.windState()

			# assigning values
			Vcl.VAmodel.vehicleDynamicsModel.state.roll = 9
			actual = Vcl.getVehicleAerodynamicsModel()
			state.roll = 9

			if not evaluateTest(cur_test, (actual.vehicleDynamicsModel.state.roll == state.roll)):
				print(f"{actual.vehicleDynamicsModel.state.roll} != {state.roll}")
		getVehicleAerodynamicsModeltest9()

		def getVehicleAerodynamicsModeltest10():
			cur_test = "getVehcileAerodynamicsModel test 10"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()
			wind = States.windState()

			# assigning values
			Vcl.VAmodel.vehicleDynamicsModel.state.p = 10
			actual = Vcl.getVehicleAerodynamicsModel()
			state.p = 10

			if not evaluateTest(cur_test, (actual.vehicleDynamicsModel.state.p == state.p)):
				print(f"{actual.vehicleDynamicsModel.state.p} != {state.p}")
		getVehicleAerodynamicsModeltest10()

		def getVehicleAerodynamicsModeltest11():
			cur_test = "getVehcileAerodynamicsModel test 11"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()
			wind = States.windState()

			# assigning values
			Vcl.VAmodel.vehicleDynamicsModel.state.q = 11
			actual = Vcl.getVehicleAerodynamicsModel()
			state.q = 11

			if not evaluateTest(cur_test, (actual.vehicleDynamicsModel.state.q == state.q)):
				print(f"{actual.vehicleDynamicsModel.state.q} != {state.q}")
		getVehicleAerodynamicsModeltest11()

		def getVehicleAerodynamicsModeltest12():
			cur_test = "getVehcileAerodynamicsModel test 12"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()
			wind = States.windState()

			# assigning values
			Vcl.VAmodel.vehicleDynamicsModel.state.r = 12
			actual = Vcl.getVehicleAerodynamicsModel()
			state.r = 12

			if not evaluateTest(cur_test, (actual.vehicleDynamicsModel.state.r == state.r)):
				print(f"{actual.vehicleDynamicsModel.state.r} != {state.r}")
		getVehicleAerodynamicsModeltest12()
	getVehicleAerodynamicsModeltest()

	def getVehicleControlSurfacestest():
		"""Testing getVehicleControlSurface from VehicleClosedLoopControl"""
		def getVehicleControlSurfacetest1():
			cur_test = "getVehcileControlSurfaces test 1"
			# instances of necessary functions
			Vcl = VCLC.VehicleClosedLoopControl()
			expected = Inputs.controlInputs()

			#inputting to function
			Vcl.trimOutputs.Throttle = 0
			Vcl.trimOutputs.Aileron = 0
			Vcl.trimOutputs.Elevator = 0
			Vcl.trimOutputs.Rudder = 0
			actual = Vcl.getVehicleControlSurfaces()

			#expected values
			expected.Throttle = Vcl.trimOutputs.Throttle
			expected.Aileron = Vcl.trimOutputs.Aileron
			expected.Elevator = Vcl.trimOutputs.Elevator
			expected.Rudder = Vcl.trimOutputs.Rudder
			if not evaluateTest(cur_test, (actual.Throttle == expected.Throttle) and
										  (actual.Aileron == expected.Aileron) and
										  (actual.Elevator == expected.Elevator) and
										  (actual.Rudder == expected.Rudder)):
				print(f"{actual.Throttle} != {expected.Throttle}")
				print(f"{actual.Aileron} != {expected.Aileron}")
				print(f"{actual.Elevator} != {expected.Elevator}")
				print(f"{actual.Rudder} != {expected.Rudder}")
		getVehicleControlSurfacetest1()

		def getVehicleControlSurfacetest2():
			cur_test = "getVehcileControlSurfaces test 2"
			# instances of necessary functions
			Vcl = VCLC.VehicleClosedLoopControl()
			expected = Inputs.controlInputs()

			#inputting to function
			Vcl.trimOutputs.Throttle = 1
			Vcl.trimOutputs.Aileron = 0
			Vcl.trimOutputs.Elevator = 0
			Vcl.trimOutputs.Rudder = 0
			actual = Vcl.getVehicleControlSurfaces()

			#expected values
			expected.Throttle = Vcl.trimOutputs.Throttle
			expected.Aileron = Vcl.trimOutputs.Aileron
			expected.Elevator = Vcl.trimOutputs.Elevator
			expected.Rudder = Vcl.trimOutputs.Rudder
			if not evaluateTest(cur_test, (actual.Throttle == expected.Throttle) and
										  (actual.Aileron == expected.Aileron) and
										  (actual.Elevator == expected.Elevator) and
										  (actual.Rudder == expected.Rudder)):
				print(f"{actual.Throttle} != {expected.Throttle}")
				print(f"{actual.Aileron} != {expected.Aileron}")
				print(f"{actual.Elevator} != {expected.Elevator}")
				print(f"{actual.Rudder} != {expected.Rudder}")
		getVehicleControlSurfacetest2()

		def getVehicleControlSurfacetest3():
			cur_test = "getVehcileControlSurfaces test 3"
			# instances of necessary functions
			Vcl = VCLC.VehicleClosedLoopControl()
			expected = Inputs.controlInputs()

			#inputting to function
			Vcl.trimOutputs.Throttle = 0
			Vcl.trimOutputs.Aileron = 1
			Vcl.trimOutputs.Elevator = 0
			Vcl.trimOutputs.Rudder = 0
			actual = Vcl.getVehicleControlSurfaces()

			#expected values
			expected.Throttle = Vcl.trimOutputs.Throttle
			expected.Aileron = Vcl.trimOutputs.Aileron
			expected.Elevator = Vcl.trimOutputs.Elevator
			expected.Rudder = Vcl.trimOutputs.Rudder
			if not evaluateTest(cur_test, (actual.Throttle == expected.Throttle) and
										  (actual.Aileron == expected.Aileron) and
										  (actual.Elevator == expected.Elevator) and
										  (actual.Rudder == expected.Rudder)):
				print(f"{actual.Throttle} != {expected.Throttle}")
				print(f"{actual.Aileron} != {expected.Aileron}")
				print(f"{actual.Elevator} != {expected.Elevator}")
				print(f"{actual.Rudder} != {expected.Rudder}")
		getVehicleControlSurfacetest3()

		def getVehicleControlSurfacetest4():
			cur_test = "getVehcileControlSurfaces test 4"
			# instances of necessary functions
			Vcl = VCLC.VehicleClosedLoopControl()
			expected = Inputs.controlInputs()

			#inputting to function
			Vcl.trimOutputs.Throttle = 0
			Vcl.trimOutputs.Aileron = 0
			Vcl.trimOutputs.Elevator = 1
			Vcl.trimOutputs.Rudder = 0
			actual = Vcl.getVehicleControlSurfaces()

			#expected values
			expected.Throttle = Vcl.trimOutputs.Throttle
			expected.Aileron = Vcl.trimOutputs.Aileron
			expected.Elevator = Vcl.trimOutputs.Elevator
			expected.Rudder = Vcl.trimOutputs.Rudder
			if not evaluateTest(cur_test, (actual.Throttle == expected.Throttle) and
										  (actual.Aileron == expected.Aileron) and
										  (actual.Elevator == expected.Elevator) and
										  (actual.Rudder == expected.Rudder)):
				print(f"{actual.Throttle} != {expected.Throttle}")
				print(f"{actual.Aileron} != {expected.Aileron}")
				print(f"{actual.Elevator} != {expected.Elevator}")
				print(f"{actual.Rudder} != {expected.Rudder}")
		getVehicleControlSurfacetest4()

		def getVehicleControlSurfacetest5():
			cur_test = "getVehcileControlSurfaces test 5"
			# instances of necessary functions
			Vcl = VCLC.VehicleClosedLoopControl()
			expected = Inputs.controlInputs()

			#inputting to function
			Vcl.trimOutputs.Throttle = 0
			Vcl.trimOutputs.Aileron = 0
			Vcl.trimOutputs.Elevator = 0
			Vcl.trimOutputs.Rudder = 1
			actual = Vcl.getVehicleControlSurfaces()

			#expected values
			expected.Throttle = Vcl.trimOutputs.Throttle
			expected.Aileron = Vcl.trimOutputs.Aileron
			expected.Elevator = Vcl.trimOutputs.Elevator
			expected.Rudder = Vcl.trimOutputs.Rudder
			if not evaluateTest(cur_test, (actual.Throttle == expected.Throttle) and
										  (actual.Aileron == expected.Aileron) and
										  (actual.Elevator == expected.Elevator) and
										  (actual.Rudder == expected.Rudder)):
				print(f"{actual.Throttle} != {expected.Throttle}")
				print(f"{actual.Aileron} != {expected.Aileron}")
				print(f"{actual.Elevator} != {expected.Elevator}")
				print(f"{actual.Rudder} != {expected.Rudder}")
		getVehicleControlSurfacetest5()

		def getVehicleControlSurfacetest6():
			cur_test = "getVehcileControlSurfaces test 6"
			# instances of necessary functions
			Vcl = VCLC.VehicleClosedLoopControl()
			expected = Inputs.controlInputs()

			#inputting to function
			Vcl.trimOutputs.Throttle = 1
			Vcl.trimOutputs.Aileron = 2
			Vcl.trimOutputs.Elevator = 3
			Vcl.trimOutputs.Rudder = 4
			actual = Vcl.getVehicleControlSurfaces()

			#expected values
			expected.Throttle = Vcl.trimOutputs.Throttle
			expected.Aileron = Vcl.trimOutputs.Aileron
			expected.Elevator = Vcl.trimOutputs.Elevator
			expected.Rudder = Vcl.trimOutputs.Rudder
			if not evaluateTest(cur_test, (actual.Throttle == expected.Throttle) and
										  (actual.Aileron == expected.Aileron) and
										  (actual.Elevator == expected.Elevator) and
										  (actual.Rudder == expected.Rudder)):
				print(f"{actual.Throttle} != {expected.Throttle}")
				print(f"{actual.Aileron} != {expected.Aileron}")
				print(f"{actual.Elevator} != {expected.Elevator}")
				print(f"{actual.Rudder} != {expected.Rudder}")
		getVehicleControlSurfacetest6()
	getVehicleControlSurfacestest()

	def getVehicleStatetest():
		"""Testing getVehicleState from the VehicleClosedLoopControl class"""
		def getVehicleStatetest1():
			cur_test = "getVehicleState test 1"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			Vcl.VAmodel.vehicleDynamicsModel.state.pn = 1
			actual = Vcl.getVehicleState()
			state.pn = 1

			if not evaluateTest(cur_test, (actual.pn == state.pn)):
				print(f"{actual.pn} != {state.pn}")
		getVehicleStatetest1()

		def getVehicleStatetest2():
			cur_test = "getVehicleState test 2"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			Vcl.VAmodel.vehicleDynamicsModel.state.pe = 2
			actual = Vcl.getVehicleState()
			state.pe = 2

			if not evaluateTest(cur_test, (actual.pe == state.pe)):
				print(f"{actual.pe} != {state.pe}")
		getVehicleStatetest2()

		def getVehicleStatetest3():
			cur_test = "getVehicleState test 3"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			Vcl.VAmodel.vehicleDynamicsModel.state.pd = 3
			actual = Vcl.getVehicleState()
			state.pd = 3

			if not evaluateTest(cur_test, (actual.pd == state.pd)):
				print(f"{actual.pd} != {state.pd}")
		getVehicleStatetest3()

		def getVehicleStatetest4():
			cur_test = "getVehicleState test 4"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			Vcl.VAmodel.vehicleDynamicsModel.state.p = 4
			actual = Vcl.getVehicleState()
			state.p = 4

			if not evaluateTest(cur_test, (actual.p == state.p)):
				print(f"{actual.p} != {state.p}")
		getVehicleStatetest4()

		def getVehicleStatetest5():
			cur_test = "getVehicleState test 5"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			Vcl.VAmodel.vehicleDynamicsModel.state.q = 5
			actual = Vcl.getVehicleState()
			state.q = 5

			if not evaluateTest(cur_test, (actual.q == state.q)):
				print(f"{actual.q} != {state.q}")
		getVehicleStatetest5()

		def getVehicleStatetest6():
			cur_test = "getVehicleState test 6"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			Vcl.VAmodel.vehicleDynamicsModel.state.r = 6
			actual = Vcl.getVehicleState()
			state.r = 6

			if not evaluateTest(cur_test, (actual.r == state.r)):
				print(f"{actual.r} != {state.r}")
		getVehicleStatetest6()

		def getVehicleStatetest7():
			cur_test = "getVehicleState test 7"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			Vcl.VAmodel.vehicleDynamicsModel.state.u = 7
			actual = Vcl.getVehicleState()
			state.u = 7

			if not evaluateTest(cur_test, (actual.u == state.u)):
				print(f"{actual.u} != {state.u}")
		getVehicleStatetest7()

		def getVehicleStatetest8():
			cur_test = "getVehicleState test 8"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			Vcl.VAmodel.vehicleDynamicsModel.state.v = 8
			actual = Vcl.getVehicleState()
			state.v = 8

			if not evaluateTest(cur_test, (actual.v == state.v)):
				print(f"{actual.v} != {state.v}")
		getVehicleStatetest8()

		def getVehicleStatetest9():
			cur_test = "getVehicleState test 9"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			Vcl.VAmodel.vehicleDynamicsModel.state.w = 9
			actual = Vcl.getVehicleState()
			state.w = 9

			if not evaluateTest(cur_test, (actual.w == state.w)):
				print(f"{actual.w} != {state.w}")
		getVehicleStatetest9()

		def getVehicleStatetest10():
			cur_test = "getVehicleState test 10"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			Vcl.VAmodel.vehicleDynamicsModel.state.yaw = 10
			actual = Vcl.getVehicleState()
			state.yaw = 10

			if not evaluateTest(cur_test, (actual.yaw == state.yaw)):
				print(f"{actual.yaw} != {state.yaw}")
		getVehicleStatetest10()

		def getVehicleStatetest11():
			cur_test = "getVehicleState test 11"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			Vcl.VAmodel.vehicleDynamicsModel.state.pitch = 11
			actual = Vcl.getVehicleState()
			state.pitch = 11

			if not evaluateTest(cur_test, (actual.pitch == state.pitch)):
				print(f"{actual.pitch} != {state.pitch}")
		getVehicleStatetest11()

		def getVehicleStatetest12():
			cur_test = "getVehicleState test 12"
			# creating istances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			Vcl.VAmodel.vehicleDynamicsModel.state.roll = 12
			actual = Vcl.getVehicleState()
			state.roll = 12

			if not evaluateTest(cur_test, (actual.roll == state.roll)):
				print(f"{actual.roll} != {state.roll}")
		getVehicleStatetest12()
	getVehicleStatetest()

	def setTrimInputstest():
		"""Testing setTrimInputs from VehicleClosedLoopControls"""
		def setTrimInputstest1():
			cur_test = "setTrimInputs test 1"
			# creating istances
			Vcl = VCLC.VehicleClosedLoopControl()

			#assigning values
			trimInputs = Inputs.controlInputs(Throttle=0.5, Aileron=-100, Elevator=50, Rudder=-.008)

			#outputted values
			Vcl.setTrimInputs(trimInputs)

			if not evaluateTest(cur_test, (Vcl.trimInputs.Throttle == trimInputs.Throttle) and
										  (Vcl.trimInputs.Aileron == trimInputs.Aileron) and
										  (Vcl.trimInputs.Elevator == trimInputs.Elevator) and
										  (Vcl.trimInputs.Rudder == trimInputs.Rudder)):
				print(f"{Vcl.trimInputs.Throttle} != {trimInputs.Throttle}")
				print(f"{Vcl.trimInputs.Aileron} != {trimInputs.Aileron}")
				print(f"{Vcl.trimInputs.Elevator} != {trimInputs.Elevator}")
				print(f"{Vcl.trimInputs.Rudder} != {trimInputs.Rudder}")
		setTrimInputstest1()
	setTrimInputstest()

	def setVehicleStatetest():
		"""testing setVehicleState from VehicleClosedLoopControl"""
		def setVehicleStatetest1():
			cur_test = "setVehicleState test 1"
			# creating instances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			state.pn = 1
			Vcl.setVehicleState(state)

			if not evaluateTest(cur_test, (Vcl.VAmodel.vehicleDynamicsModel.state.pn == state.pn)):
				print(f"{Vcl.VAmodel.vehicleDynamicsModel.state.pn} != {state.pn}")
		setVehicleStatetest1()

		def setVehicleStatetest2():
			cur_test = "setVehicleState test 2"
			# creating instances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			state.pe = 2
			Vcl.setVehicleState(state)

			if not evaluateTest(cur_test, (Vcl.VAmodel.vehicleDynamicsModel.state.pe == state.pe)):
				print(f"{Vcl.VAmodel.vehicleDynamicsModel.state.pe} != {state.pe}")
		setVehicleStatetest2()

		def setVehicleStatetest3():
			cur_test = "setVehicleState test 3"
			# creating instances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			state.pd = 3
			Vcl.setVehicleState(state)

			if not evaluateTest(cur_test, (Vcl.VAmodel.vehicleDynamicsModel.state.pd == state.pd)):
				print(f"{Vcl.VAmodel.vehicleDynamicsModel.state.pd} != {state.pd}")
		setVehicleStatetest3()

		def setVehicleStatetest4():
			cur_test = "setVehicleState test 4"
			# creating instances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			state.u = 4
			Vcl.setVehicleState(state)

			if not evaluateTest(cur_test, (Vcl.VAmodel.vehicleDynamicsModel.state.u == state.u)):
				print(f"{Vcl.VAmodel.vehicleDynamicsModel.state.u} != {state.u}")
		setVehicleStatetest4()

		def setVehicleStatetest5():
			cur_test = "setVehicleState test 5"
			# creating instances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			state.v = 5
			Vcl.setVehicleState(state)

			if not evaluateTest(cur_test, (Vcl.VAmodel.vehicleDynamicsModel.state.v == state.v)):
				print(f"{Vcl.VAmodel.vehicleDynamicsModel.state.v} != {state.v}")
		setVehicleStatetest5()

		def setVehicleStatetest6():
			cur_test = "setVehicleState test 6"
			# creating instances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			state.w = 6
			Vcl.setVehicleState(state)

			if not evaluateTest(cur_test, (Vcl.VAmodel.vehicleDynamicsModel.state.w == state.w)):
				print(f"{Vcl.VAmodel.vehicleDynamicsModel.state.w} != {state.w}")
		setVehicleStatetest6()

		def setVehicleStatetest7():
			cur_test = "setVehicleState test 7"
			# creating instances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			state.p = 7
			Vcl.setVehicleState(state)

			if not evaluateTest(cur_test, (Vcl.VAmodel.vehicleDynamicsModel.state.p == state.p)):
				print(f"{Vcl.VAmodel.vehicleDynamicsModel.state.p} != {state.p}")
		setVehicleStatetest7()

		def setVehicleStatetest8():
			cur_test = "setVehicleState test 8"
			# creating instances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			state.q = 8
			Vcl.setVehicleState(state)

			if not evaluateTest(cur_test, (Vcl.VAmodel.vehicleDynamicsModel.state.q == state.q)):
				print(f"{Vcl.VAmodel.vehicleDynamicsModel.state.q} != {state.q}")
		setVehicleStatetest8()

		def setVehicleStatetest9():
			cur_test = "setVehicleState test 9"
			# creating instances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			state.r = 9
			Vcl.setVehicleState(state)

			if not evaluateTest(cur_test, (Vcl.VAmodel.vehicleDynamicsModel.state.r == state.r)):
				print(f"{Vcl.VAmodel.vehicleDynamicsModel.state.r} != {state.r}")
		setVehicleStatetest9()

		def setVehicleStatetest10():
			cur_test = "setVehicleState test 10"
			# creating instances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			state.yaw = 10
			Vcl.setVehicleState(state)

			if not evaluateTest(cur_test, (Vcl.VAmodel.vehicleDynamicsModel.state.yaw == state.yaw)):
				print(f"{Vcl.VAmodel.vehicleDynamicsModel.state.yaw} != {state.yaw}")
		setVehicleStatetest10()

		def setVehicleStatetest11():
			cur_test = "setVehicleState test 11"
			# creating instances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			state.pitch = 11
			Vcl.setVehicleState(state)

			if not evaluateTest(cur_test, (Vcl.VAmodel.vehicleDynamicsModel.state.pitch == state.pitch)):
				print(f"{Vcl.VAmodel.vehicleDynamicsModel.state.pitch} != {state.pitch}")
		setVehicleStatetest11()

		def setVehicleStatetest12():
			cur_test = "setVehicleState test 12"
			# creating instances
			VAmodel = VAM.VehicleAerodynamicsModel()
			Vcl = VCLC.VehicleClosedLoopControl()
			state = States.vehicleState()

			# inputting values
			state.roll = 12
			Vcl.setVehicleState(state)

			if not evaluateTest(cur_test, (Vcl.VAmodel.vehicleDynamicsModel.state.roll == state.roll)):
				print(f"{Vcl.VAmodel.vehicleDynamicsModel.state.roll} != {state.roll}")
		setVehicleStatetest12()
	setVehicleStatetest()

VehicleClosedLoopControltests()

print("Beginning testing of VehicleControlGains")
def computeGainstests():
	"""testing computeGains from VehicleControlGains"""
	def computeGainstest1():
		cur_test = "computeGains test 1"
		# instances of necessary functions
		tuningParameters = Controls.controlTuning()
		linearizedModel = Linearized.transferFunctions()

		#inputs for tuning parameters
		tuningParameters.Wn_roll = 1
		tuningParameters.Zeta_roll = 2
		tuningParameters.Wn_course = 3
		tuningParameters.Zeta_course = 4
		tuningParameters.Wn_sideslip = 5
		tuningParameters.Zeta_sideslip = 6
		# tuning knobs for longitudinal control
		tuningParameters.Wn_pitch = 7
		tuningParameters.Zeta_pitch = 8
		tuningParameters.Wn_altitude = 9
		tuningParameters.Zeta_altitude = 10
		tuningParameters.Wn_SpeedfromThrottle = 11
		tuningParameters.Zeta_SpeedfromThrottle = 12
		tuningParameters.Wn_SpeedfromElevator = 13
		tuningParameters.Zeta_SpeedfromElevator = 14

		#inputs linearized model
		linearizedModel.Va_trim = 15
		linearizedModel.alpha_trim = 16
		linearizedModel.beta_trim = 17
		linearizedModel.gamma_trim = 18
		linearizedModel.theta_trim = 19
		linearizedModel.phi_trim = 20
		linearizedModel.a_phi1 = 21
		linearizedModel.a_phi2 = 22
		linearizedModel.a_beta1 = 23
		linearizedModel.a_beta2 = 24
		linearizedModel.a_theta1 = 25
		linearizedModel.a_theta2 = 26
		linearizedModel.a_theta3 = 27
		linearizedModel.a_V1 = 28
		linearizedModel.a_V2 = 29
		linearizedModel.a_V3 = 30

		#istance of the computeGains
		actual = VCG.computeGains(tuningParameters, linearizedModel)

		#expected values
		expected_kp_roll = 0.045454545454545456
		expected_kd_roll = -0.7727272727272727
		expected_ki_roll = 0.001
		expected_kp_sideslip = 1.5416666666666667
		expected_ki_sideslip = 1.0416666666666667
		expected_kp_course = 36.69724770642202
		expected_ki_course = 13.761467889908257
		# Longitudinal Gains
		expected_kp_pitch = 0.8518518518518519
		expected_kd_pitch = 3.2222222222222223
		expected_kp_altitude = 25.565217391304348
		expected_ki_altitude = 11.504347826086956
		expected_kp_SpeedfromThrottle = 8.137931034482758
		expected_ki_SpeedfromThrottle = 4.172413793103448
		expected_kp_SpeedfromElevator = -72.9690200771174
		expected_ki_SpeedfromElevator = -36.70167974116917

		if not evaluateTest(cur_test, (actual.kp_roll == expected_kp_roll) and
									  (actual.kd_roll == expected_kd_roll) and
									  (actual.ki_roll == expected_ki_roll) and
									  (actual.kp_sideslip == expected_kp_sideslip) and
									  (actual.ki_sideslip == expected_ki_sideslip) and
									  (actual.kp_course == expected_kp_course) and
									  (actual.ki_course == expected_ki_course) and
									  (actual.kp_pitch == expected_kp_pitch) and
									  (actual.kd_pitch == expected_kd_pitch) and
									  (actual.kp_altitude == expected_kp_altitude) and
									  (actual.ki_altitude == expected_ki_altitude) and
									  (actual.kp_SpeedfromThrottle == expected_kp_SpeedfromThrottle) and
									  (actual.ki_SpeedfromThrottle == expected_ki_SpeedfromThrottle) and
									  (actual.kp_SpeedfromElevator == expected_kp_SpeedfromElevator) and
									  (actual.ki_SpeedfromElevator == expected_ki_SpeedfromElevator)):
			print(f"{actual.kp_roll} != {expected_kp_roll}")
			print(f"{actual.kd_roll} != {expected_kd_roll}")
			print(f"{actual.ki_roll} != {expected_ki_roll}")
			print(f"{actual.kp_sideslip} != {expected_kp_sideslip}")
			print(f"{actual.ki_sideslip} != {expected_ki_sideslip}")
			print(f"{actual.kp_course} != {expected_kp_course}")
			print(f"{actual.ki_course} != {expected_ki_course}")
			print(f"{actual.kp_pitch} != {expected_kp_pitch}")
			print(f"{actual.kd_pitch} != {expected_kd_pitch}")
			print(f"{actual.kp_altitude} != {expected_kp_altitude}")
			print(f"{actual.ki_altitude} != {expected_ki_altitude}")
			print(f"{actual.kp_SpeedfromThrottle} != {expected_kp_SpeedfromThrottle}")
			print(f"{actual.ki_SpeedfromThrottle} != {expected_ki_SpeedfromThrottle}")
			print(f"{actual.kp_SpeedfromElevator} != {expected_kp_SpeedfromElevator}")
			print(f"{actual.ki_SpeedfromElevator} != {expected_ki_SpeedfromElevator}")
	computeGainstest1()
computeGainstests()

def computeTuningParameterstests():
	"""Testing computeTuningParameters from Vehicle Control Gains"""
	def computeTuningParameterstest1():
		cur_test = "computeTuningParameters test 1"
		# instances of necessary functions
		controlGains = Controls.controlGains()
		linearizedModel = Linearized.transferFunctions()
		# expected values
		controlGains.kp_roll = 1
		controlGains.kd_roll = 2
		controlGains.ki_roll = 3
		controlGains.kp_sideslip = 4
		controlGains.ki_sideslip = 5
		controlGains.kp_course = 6
		controlGains.ki_course = 7
		# Longitudinal Gains
		controlGains.kp_pitch = 8
		controlGains.kd_pitch = 9
		controlGains.kp_altitude = 10
		controlGains.ki_altitude = 11
		controlGains.kp_SpeedfromThrottle = 12
		controlGains.ki_SpeedfromThrottle = 13
		controlGains.kp_SpeedfromElevator = 14
		controlGains.ki_SpeedfromElevator = -15

		#inputs linearized model
		linearizedModel.Va_trim = 15
		linearizedModel.alpha_trim = 16
		linearizedModel.beta_trim = 17
		linearizedModel.gamma_trim = 18
		linearizedModel.theta_trim = 19
		linearizedModel.phi_trim = 20
		linearizedModel.a_phi1 = 21
		linearizedModel.a_phi2 = 22
		linearizedModel.a_beta1 = 23
		linearizedModel.a_beta2 = 24
		linearizedModel.a_theta1 = 25
		linearizedModel.a_theta2 = 26
		linearizedModel.a_theta3 = 27
		linearizedModel.a_V1 = 28
		linearizedModel.a_V2 = 29
		linearizedModel.a_V3 = 30

		#istance of the computeGains
		actual = VCG.computeTuningParameters(controlGains, linearizedModel)

		# expected values for tuning parameters
		expected_Wn_roll = 4.69041575982343
		expected_Zeta_roll = 6.929023281557339
		expected_Wn_course = 2.139626135566679
		expected_Zeta_course = 0.9169826295285766
		expected_Wn_sideslip = 10.954451150103322
		expected_Zeta_sideslip = 5.4315820285928975
		# tuning knobs for longitudinal control
		expected_Wn_pitch = 15.556349186104045
		expected_Zeta_pitch = 8.613846243545215
		expected_Wn_altitude = 12.135597524338358
		expected_Zeta_altitude = 5.516180692881072
		expected_Wn_SpeedfromThrottle = 19.4164878389476
		expected_Zeta_SpeedfromThrottle = 9.68249260934257
		expected_Wn_SpeedfromElevator = 11.460388120293684
		expected_Zeta_SpeedfromElevator = -4.126582006521676

		if not evaluateTest(cur_test, (actual.Wn_roll == expected_Wn_roll) and
									  (actual.Zeta_roll == expected_Zeta_roll) and
									  (actual.Wn_course == expected_Wn_course) and
									  (actual.Zeta_course == expected_Zeta_course) and
									  (actual.Wn_sideslip == expected_Wn_sideslip) and
									  (actual.Zeta_sideslip == expected_Zeta_sideslip) and
									  (actual.Wn_pitch == expected_Wn_pitch) and
									  (actual.Zeta_pitch == expected_Zeta_pitch) and
									  (actual.Wn_altitude == expected_Wn_altitude) and
									  (actual.Zeta_altitude == expected_Zeta_altitude) and
									  (actual.Wn_SpeedfromThrottle == expected_Wn_SpeedfromThrottle) and
									  (actual.Zeta_SpeedfromThrottle == expected_Zeta_SpeedfromThrottle) and
									  (actual.Wn_SpeedfromElevator == expected_Wn_SpeedfromElevator) and
									  (actual.Zeta_SpeedfromElevator == expected_Zeta_SpeedfromElevator)):
			print(f"{actual.Wn_roll} != {expected_Wn_roll}")
			print(f"{actual.Zeta_roll} != {expected_Zeta_roll}")
			print(f"{actual.Wn_course} != {expected_Wn_course}")
			print(f"{actual.Zeta_course} != {expected_Zeta_course}")
			print(f"{actual.Wn_sideslip} != {expected_Wn_sideslip}")
			print(f"{actual.Zeta_sideslip} != {expected_Zeta_sideslip}")
			print(f"{actual.Wn_pitch} != {expected_Wn_pitch}")
			print(f"{actual.Zeta_pitch} != {expected_Zeta_pitch}")
			print(f"{actual.Wn_altitude} != {expected_Wn_altitude}")
			print(f"{actual.Zeta_altitude} != {expected_Zeta_altitude}")
			print(f"{actual.Wn_SpeedfromThrottle} != {expected_Wn_SpeedfromThrottle}")
			print(f"{actual.Zeta_SpeedfromThrottle} != {expected_Zeta_SpeedfromThrottle}")
			print(f"{actual.Wn_SpeedfromElevator} != {expected_Wn_SpeedfromElevator}")
			print(f"{actual.Zeta_SpeedfromElevator} != {expected_Zeta_SpeedfromElevator}")
	computeTuningParameterstest1()
computeTuningParameterstests()



def Simulationtest():
	print("Begining testing of the simulation")
	def Simulationest1():
		# instance of VehicleClosedLoopControl
		Vcl = VCLC.VehicleClosedLoopControl()

		# setting dT
		Vcl.dT = 0.001

		# setting rudderControl source
		Vcl.rudderControlSource = 'SIDESLIP'

		# setting VehicleAerodynamicsModel
		Vcl.VAmodel.InitialSpeed = VPC.InitialSpeed  # Initial Speed
		Vcl.VAmodel.InitialHeight = VPC.InitialDownPosition  # Initial Height
		Vcl.VAmodel.WindModel.dT = 0.001  # WindModel time step
		Vcl.VAmodel.WindModel.Va = 3  # WindModel airspeed
		Vcl.VAmodel.WindModel.drydenParameters = VPC.DrydenLowAltitudeLight  # Windmodel DrydenParameters
		Vcl.VAmodel.vehicleDynamicsModel.dT = 0.001

		# setting control gains
		# lateral Gains
		Vcl.controlGains = Inputs.controlInputs()
		Vcl.controlGains.kp_roll = 3
		Vcl.controlGains.kd_roll = 0.04
		Vcl.controlGains.ki_roll = 0.001
		Vcl.controlGains.kp_sideslip = 2
		Vcl.controlGains.ki_sideslip = 2
		Vcl.controlGains.kp_course = 5
		Vcl.controlGains.ki_course = 2
		# Longitudinal Gains
		Vcl.controlGains.kp_pitch = -10
		Vcl.controlGains.kd_pitch = -0.8
		Vcl.controlGains.kp_altitude = 0.08
		Vcl.controlGains.ki_altitude = 0.03
		Vcl.controlGains.kp_SpeedfromThrottle = 2.0
		Vcl.controlGains.ki_SpeedfromThrottle = 1.0
		Vcl.controlGains.kp_SpeedfromElevator = -0.5
		Vcl.controlGains.ki_SpeedfromElevator = -0.1

		# setting trim Inputs
		Vcl.trimInputs = Inputs.controlInputs()
		Vcl.trimInputs.Throttle = 0.8
		Vcl.trimInputs.Aileron = 0
		Vcl.trimInputs.Elevator = 0
		Vcl.trimInputs.Rudder = 0

		# setting mode
		Vcl.mode = Controls.AltitudeStates.HOLDING

		# graphing
		# Time
		T_tot = 20  # a total of 20 seconds for flight
		n_step = int(T_tot / Vcl.dT)
		t_data = [i * Vcl.dT for i in range(n_step)]

		# lists for function
		pd_data = [0 for i in range(n_step)]  # pd
		pe_data = [0 for i in range(n_step)]  # pe
		pn_data = [0 for i in range(n_step)]  # pd
		u_data = [0 for i in range(n_step)]  # u
		v_data = [0 for i in range(n_step)]  # v
		w_data = [0 for i in range(n_step)]  # w
		yaw_data = [0 for i in range(n_step)]  # yaw
		pitch_data = [0 for i in range(n_step)]  # pitch
		roll_data = [0 for i in range(n_step)]  # roll
		p_data = [0 for i in range(n_step)]  # p
		q_data = [0 for i in range(n_step)]  # q
		r_data = [0 for i in range(n_step)]  # r

		# Reference Commands
		ReferenceCommands = Controls.referenceCommands()
		ReferenceCommands.courseCommand = VPC.InitialYawAngle
		ReferenceCommands.altitudeCommand = -VPC.InitialDownPosition
		ReferenceCommands.airspeedCommand = VPC.InitialSpeed

		# loop for many instances
		for i in range(n_step):
			# record our data
			pd_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.pd
			pe_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.pe
			pn_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.pn
			u_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.u
			v_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.v
			w_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.w
			yaw_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.yaw
			pitch_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.pitch
			roll_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.roll
			p_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.p
			q_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.q
			r_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.r

			# Update
			Vcl.Update(ReferenceCommands)
		# plotting
		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, pn_data, label="pn response")
		ax.set_title("state: pn")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("pn ")
		ax.legend()
		plt.show("pngraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, pe_data, label="pe response")
		ax.set_title("state: pe")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("pe ")
		ax.legend()
		plt.show("pegraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, pd_data, label="pd response")
		ax.set_title("state: pd")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("pd ")
		ax.legend()
		plt.show("pdgraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, u_data, label="u response")
		ax.set_title("state: u")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("u ")
		ax.legend()
		plt.show("ugraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, v_data, label="v response")
		ax.set_title("state: v")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("v ")
		ax.legend()
		plt.show("vgraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, w_data, label="w response")
		ax.set_title("state: w")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("w ")
		ax.legend()
		plt.saveshow("wgraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, yaw_data, label="yaw response")
		ax.set_title("state: yaw")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("yaw (rad) ")
		ax.legend()
		plt.show("yawgraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, pitch_data, label="pitch response")
		ax.set_title("state: pitch")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("pitch (rad) ")
		ax.legend()
		plt.show("pitchgraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, roll_data, label="roll response")
		ax.set_title("state: roll")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("roll (rad) ")
		ax.legend()
		plt.saveshow("roll.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, p_data, label="p response")
		ax.set_title("state: p")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("p ")
		ax.legend()
		plt.show("pgraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, q_data, label="q response")
		ax.set_title("state: q")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("q ")
		ax.legend()
		plt.show("qgraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, r_data, label="r response")
		ax.set_title("state: r")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("r ")
		ax.legend()
		plt.show("rgraph.png")
	Simulationest1()

	def Simulationstest2():
		# instance of VehicleClosedLoopControl
		Vcl = VCLC.VehicleClosedLoopControl()

		# setting dT
		Vcl.dT = 0.001

		# setting rudderControl source
		Vcl.rudderControlSource = 'SIDESLIP'

		# setting VehicleAerodynamicsModel
		Vcl.VAmodel.InitialSpeed = 150  # Initial Speed
		Vcl.VAmodel.InitialHeight = 100  # Initial Height
		Vcl.VAmodel.WindModel.dT = 0.001  # WindModel time step
		Vcl.VAmodel.WindModel.Va = 10  # WindModel airspeed
		Vcl.VAmodel.WindModel.drydenParameters = VPC.DrydenLowAltitudeModerate  # Windmodel DrydenParameters
		Vcl.VAmodel.vehicleDynamicsModel.dT = 0.001

		# setting control gains
		# lateral Gains
		Vcl.controlGains = Inputs.controlInputs()
		Vcl.controlGains.kp_roll = 3
		Vcl.controlGains.kd_roll = 0.04
		Vcl.controlGains.ki_roll = 0.001
		Vcl.controlGains.kp_sideslip = 2
		Vcl.controlGains.ki_sideslip = 2
		Vcl.controlGains.kp_course = 5
		Vcl.controlGains.ki_course = 2
		# Longitudinal Gains
		Vcl.controlGains.kp_pitch = -10
		Vcl.controlGains.kd_pitch = -0.8
		Vcl.controlGains.kp_altitude = 0.08
		Vcl.controlGains.ki_altitude = 0.03
		Vcl.controlGains.kp_SpeedfromThrottle = 2.0
		Vcl.controlGains.ki_SpeedfromThrottle = 1.0
		Vcl.controlGains.kp_SpeedfromElevator = -0.5
		Vcl.controlGains.ki_SpeedfromElevator = -0.1

		# setting trim Inputs
		Vcl.trimInputs = Inputs.controlInputs()
		Vcl.trimInputs.Throttle = 0.8
		Vcl.trimInputs.Aileron = 3
		Vcl.trimInputs.Elevator = 2
		Vcl.trimInputs.Rudder = 3

		# setting mode
		Vcl.mode = Controls.AltitudeStates.CLIMBING

		# graphing
		# Time
		T_tot = 20  # a total of 20 seconds for flight
		n_step = int(T_tot / Vcl.dT)
		t_data = [i * Vcl.dT for i in range(n_step)]

		# lists for function
		pd_data = [0 for i in range(n_step)]  # pd
		pe_data = [0 for i in range(n_step)]  # pe
		pn_data = [0 for i in range(n_step)]  # pd
		u_data = [0 for i in range(n_step)]  # u
		v_data = [0 for i in range(n_step)]  # v
		w_data = [0 for i in range(n_step)]  # w
		yaw_data = [0 for i in range(n_step)]  # yaw
		pitch_data = [0 for i in range(n_step)]  # pitch
		roll_data = [0 for i in range(n_step)]  # roll
		p_data = [0 for i in range(n_step)]  # p
		q_data = [0 for i in range(n_step)]  # q
		r_data = [0 for i in range(n_step)]  # r

		# Reference Commands
		ReferenceCommands = Controls.referenceCommands()
		ReferenceCommands.courseCommand = VPC.InitialYawAngle
		ReferenceCommands.altitudeCommand = -VPC.InitialDownPosition
		ReferenceCommands.airspeedCommand = VPC.InitialSpeed

		# loop for many instances
		for i in range(n_step):
			# record our data
			pd_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.pd
			pe_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.pe
			pn_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.pn
			u_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.u
			v_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.v
			w_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.w
			yaw_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.yaw
			pitch_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.pitch
			roll_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.roll
			p_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.p
			q_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.q
			r_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.r

			# Update
			Vcl.Update(ReferenceCommands)
		# plotting
		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, pn_data, label="pn response")
		ax.set_title("state: pn")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("pn ")
		ax.legend()
		plt.show("pngraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, pe_data, label="pe response")
		ax.set_title("state: pe")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("pe ")
		ax.legend()
		plt.show("pegraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, pd_data, label="pd response")
		ax.set_title("state: pd")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("pd ")
		ax.legend()
		plt.show("pdgraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, u_data, label="u response")
		ax.set_title("state: u")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("u ")
		ax.legend()
		plt.show("ugraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, v_data, label="v response")
		ax.set_title("state: v")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("v ")
		ax.legend()
		plt.show("vgraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, w_data, label="w response")
		ax.set_title("state: w")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("w ")
		ax.legend()
		plt.saveshow("wgraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, yaw_data, label="yaw response")
		ax.set_title("state: yaw")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("yaw (rad) ")
		ax.legend()
		plt.show("yawgraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, pitch_data, label="pitch response")
		ax.set_title("state: pitch")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("pitch (rad) ")
		ax.legend()
		plt.show("pitchgraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, roll_data, label="roll response")
		ax.set_title("state: roll")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("roll (rad) ")
		ax.legend()
		plt.saveshow("roll.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, p_data, label="p response")
		ax.set_title("state: p")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("p ")
		ax.legend()
		plt.show("pgraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, q_data, label="q response")
		ax.set_title("state: q")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("q ")
		ax.legend()
		plt.show("qgraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, r_data, label="r response")
		ax.set_title("state: r")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("r ")
		ax.legend()
		plt.show("rgraph.png")
	Simulationstest2()

	def Simulationtest3():
		# instance of VehicleClosedLoopControl
		Vcl = VCLC.VehicleClosedLoopControl()

		# setting dT
		Vcl.dT = 0.001

		# setting rudderControl source
		Vcl.rudderControlSource = 'SIDESLIP'

		# setting VehicleAerodynamicsModel
		Vcl.VAmodel.InitialSpeed = 50  # Initial Speed
		Vcl.VAmodel.InitialHeight = 70  # Initial Height
		Vcl.VAmodel.WindModel.dT = 0.001  # WindModel time step
		Vcl.VAmodel.WindModel.Va = 50  # WindModel airspeed
		Vcl.VAmodel.WindModel.drydenParameters = VPC.DrydenHighAltitudeLight  # Windmodel DrydenParameters
		Vcl.VAmodel.vehicleDynamicsModel.dT = 0.001

		# setting control gains
		# lateral Gains
		Vcl.controlGains = Inputs.controlInputs()
		Vcl.controlGains.kp_roll = 3
		Vcl.controlGains.kd_roll = 0.04
		Vcl.controlGains.ki_roll = 0.001
		Vcl.controlGains.kp_sideslip = 2
		Vcl.controlGains.ki_sideslip = 2
		Vcl.controlGains.kp_course = 5
		Vcl.controlGains.ki_course = 2
		# Longitudinal Gains
		Vcl.controlGains.kp_pitch = -10
		Vcl.controlGains.kd_pitch = -0.8
		Vcl.controlGains.kp_altitude = 0.08
		Vcl.controlGains.ki_altitude = 0.03
		Vcl.controlGains.kp_SpeedfromThrottle = 2.0
		Vcl.controlGains.ki_SpeedfromThrottle = 1.0
		Vcl.controlGains.kp_SpeedfromElevator = -0.5
		Vcl.controlGains.ki_SpeedfromElevator = -0.1

		# setting trim Inputs
		Vcl.trimInputs = Inputs.controlInputs()
		Vcl.trimInputs.Throttle = 0.2
		Vcl.trimInputs.Aileron = 10
		Vcl.trimInputs.Elevator = 0
		Vcl.trimInputs.Rudder = 0

		# setting mode
		Vcl.mode = Controls.AltitudeStates.DESCENDING

		# graphing
		# Time
		T_tot = 20  # a total of 20 seconds for flight
		n_step = int(T_tot / Vcl.dT)
		t_data = [i * Vcl.dT for i in range(n_step)]

		# lists for function
		pd_data = [0 for i in range(n_step)]  # pd
		pe_data = [0 for i in range(n_step)]  # pe
		pn_data = [0 for i in range(n_step)]  # pd
		u_data = [0 for i in range(n_step)]  # u
		v_data = [0 for i in range(n_step)]  # v
		w_data = [0 for i in range(n_step)]  # w
		yaw_data = [0 for i in range(n_step)]  # yaw
		pitch_data = [0 for i in range(n_step)]  # pitch
		roll_data = [0 for i in range(n_step)]  # roll
		p_data = [0 for i in range(n_step)]  # p
		q_data = [0 for i in range(n_step)]  # q
		r_data = [0 for i in range(n_step)]  # r

		# Reference Commands
		ReferenceCommands = Controls.referenceCommands()
		ReferenceCommands.courseCommand = VPC.InitialYawAngle
		ReferenceCommands.altitudeCommand = -VPC.InitialDownPosition
		ReferenceCommands.airspeedCommand = VPC.InitialSpeed

		# loop for many instances
		for i in range(n_step):
			# record our data
			pd_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.pd
			pe_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.pe
			pn_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.pn
			u_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.u
			v_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.v
			w_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.w
			yaw_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.yaw
			pitch_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.pitch
			roll_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.roll
			p_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.p
			q_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.q
			r_data[i] = Vcl.VAmodel.vehicleDynamicsModel.state.r

			# Update
			Vcl.Update(ReferenceCommands)
		# plotting
		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, pn_data, label="pn response")
		ax.set_title("state: pn")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("pn ")
		ax.legend()
		plt.show("pngraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, pe_data, label="pe response")
		ax.set_title("state: pe")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("pe ")
		ax.legend()
		plt.show("pegraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, pd_data, label="pd response")
		ax.set_title("state: pd")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("pd ")
		ax.legend()
		plt.show("pdgraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, u_data, label="u response")
		ax.set_title("state: u")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("u ")
		ax.legend()
		plt.show("ugraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, v_data, label="v response")
		ax.set_title("state: v")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("v ")
		ax.legend()
		plt.show("vgraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, w_data, label="w response")
		ax.set_title("state: w")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("w ")
		ax.legend()
		plt.saveshow("wgraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, yaw_data, label="yaw response")
		ax.set_title("state: yaw")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("yaw (rad) ")
		ax.legend()
		plt.show("yawgraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, pitch_data, label="pitch response")
		ax.set_title("state: pitch")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("pitch (rad) ")
		ax.legend()
		plt.show("pitchgraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, roll_data, label="roll response")
		ax.set_title("state: roll")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("roll (rad) ")
		ax.legend()
		plt.saveshow("roll.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, p_data, label="p response")
		ax.set_title("state: p")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("p ")
		ax.legend()
		plt.show("pgraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, q_data, label="q response")
		ax.set_title("state: q")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("q ")
		ax.legend()
		plt.show("qgraph.png")

		plt.close("all")
		fig, ax = plt.subplots()
		ax.plot(t_data, r_data, label="r response")
		ax.set_title("state: r")
		ax.set_xlabel("time (s)")
		ax.set_ylabel("r ")
		ax.legend()
		plt.show("rgraph.png")
	Simulationtest3()

Simulationtest()



#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]