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


print("Beginning testing of VehiclePerturbationModels")

def CreateTransferFunctiontest():
	"""Testing all the outputted components of TransferFunction and their predefined components"""
	def createtransferfunctest1():
		cur_test = "Create Transfer Function Test 1"
		vTrim = VT.VehicleTrim()
		Vastar = 25
		Gammastar = math.radians(6.0)
		Kappastar = -1.0 / 150

		check = vTrim.computeTrim(Vastar, Kappastar, Gammastar)
		if check:
			print("Optimization successful")
		else:
			print("Model converged outside of valid inputs, change parameters and try again")
		tF = VPM.CreateTransferFunction(
			vTrim.getTrimState(),
			vTrim.getTrimControls())
		expectedVa = 25.000000000000036
		expectedalpha = 0.05707917211405147
		expectedgamma = 0.10003714446720471
		expectedtheta = 0.1571163165812562
		expectedphi = -0.39967477522059336
		expecteda_phi1 = 22.628850693257796
		expecteda_phi2 = 130.88367819945077
		expecteda_beta1 = 0.7767725000000012
		expecteda_beta2 = 0.15059875000000023
		expecteda_theta1 = 5.294738297989025
		expecteda_theta2 = 99.94742162885493
		expecteda_theta3 = -36.112389566630064
		expecteda_v1 =  0.29167223914686624
		expecteda_v2 = 9.812166264574964
		expecteda_v3 = 9.760954476622285
		if not evaluateTest(cur_test, (expectedVa == tF.Va_trim  and expectedalpha == tF.alpha_trim and expectedgamma == tF.gamma_trim and
									   expectedtheta == tF.theta_trim and expectedphi == tF.phi_trim and expecteda_phi1 == tF.a_phi1 and
									   expecteda_phi2 == tF.a_phi2 and expecteda_beta1 == tF.a_beta1 and expecteda_beta2 == tF.a_beta2 and
									   expecteda_theta1 == tF.a_theta1 and expecteda_theta2 == tF.a_theta2 and expecteda_theta3 == tF.a_theta3 and
									   expecteda_v1 == tF.a_V1 and expecteda_v2 == tF.a_V2 and expecteda_v3 == tF.a_V3)):
			print(f"{expectedVa} != {tF.Va_trim}")
			print(f"{expectedalpha} != {tF.alpha_trim}")
			print(f"{ expectedgamma} != {tF.gamma_trim}")
			print(f"{expectedtheta} != {tF.theta_trim}")
			print(f"{expectedphi} != {tF.phi_trim}")
			print(f"{expecteda_phi1} != {tF.a_phi1}")
			print(f"{expecteda_phi2} != {tF.a_phi2}")
			print(f"{expecteda_beta1} != {tF.a_beta1}")
			print(f"{expecteda_beta2} != {tF.a_beta2}")
			print(f"{expecteda_theta1} != {tF.a_theta1}")
			print(f"{expecteda_theta2} != {tF.a_theta2}")
			print(f"{expecteda_theta3} != {tF.a_theta3}")
			print(f"{ expecteda_v1} != {tF.a_V1}")
			print(f"{expecteda_v2} != {tF.a_V2}")
			print(f"{expecteda_v3} != {tF.a_V3}")

	createtransferfunctest1()
CreateTransferFunctiontest()

def dThrust_dThrottletest():
	"""Testing the dThrust/dThrottle derivative using CalculatePropForces and defined values for Va and Throttle"""
	def dthrust_dTtest1():
		cur_test = "dThrust_dThrottle Test 1"
		actual = VPM.dThrust_dThrottle(Va = 0, Throttle = 0, epsilon=0.01)
		expected = 0.651234363961867
		if not evaluateTest(cur_test, (expected == actual)):
			print(f"{expected} != {actual}")
	dthrust_dTtest1()

	def dthrust_dTtest2():
		cur_test = "dThrust_dThrottle Test 2"
		actual = VPM.dThrust_dThrottle(Va = 1, Throttle = 0, epsilon=0.01)
		expected = -0.41961411722243186
		if not evaluateTest(cur_test, (expected == actual)):
			print(f"{expected} != {actual}")
	dthrust_dTtest2()

	def dthrust_dTtest3():
		cur_test = "dThrust_dThrottle Test 3"
		actual = VPM.dThrust_dThrottle(Va = 0, Throttle = 1, epsilon=0.01)
		expected = 164.46338009925228
		if not evaluateTest(cur_test, (expected == actual)):
			print(f"{expected} != {actual}")
	dthrust_dTtest3()

	def dthrust_dTtest4():
		cur_test = "dThrust_dThrottle Test 4"
		actual = VPM.dThrust_dThrottle(Va = 3, Throttle = 4, epsilon=0.01)
		expected = 506.09635134906057
		if not evaluateTest(cur_test, (expected == actual)):
			print(f"{expected} != {actual}")
	dthrust_dTtest4()
dThrust_dThrottletest()

def dThrust_dVatest():
	"""Testing the dThrust/dVa derivative using CalculatePropForces and define values for Va and Throttle"""
	def dthrust_dVatest1():
		cur_test = "dThrust_dVa Test 1"
		actual = VPM.dThrust_dVa(Va=0, Throttle=0, epsilon=0.5)
		expected = -0.01614315967103503
		if not evaluateTest(cur_test, (expected == actual)):
			print(f"{expected} != {actual}")
	dthrust_dVatest1()

	def dthrust_dVatest2():
		cur_test = "dThrust_dVa Test 2"
		actual = VPM.dThrust_dVa(Va=1, Throttle=0, epsilon=0.5)
		expected = -0.08698337848202185
		if not evaluateTest(cur_test, (expected == actual)):
			print(f"{expected} != {actual}")
	dthrust_dVatest2()

	def dthrust_dVatest3():
		cur_test = "dThrust_dVa Test 3"
		actual = VPM.dThrust_dVa(Va=0, Throttle=1, epsilon=0.5)
		expected = -1.1556631995518387
		if not evaluateTest(cur_test, (expected == actual)):
			print(f"{expected} != {actual}")
	dthrust_dVatest3()

	def dthrust_dVatest4():
		cur_test = "dThrust_dVa Test 4"
		actual = VPM.dThrust_dVa(Va=5, Throttle=6, epsilon=0.5)
		expected = -7.686388708960294
		if not evaluateTest(cur_test, (expected == actual)):
			print(f"{expected} != {actual}")
	dthrust_dVatest4()
dThrust_dVatest()


#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]