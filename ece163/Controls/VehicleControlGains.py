import math
import pickle
from ece163.Modeling import VehicleAerodynamicsModel as VAM
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Controls
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath
from ece163.Utilities import Rotations

def computeGains(tuningParameters=Controls.controlTuning(), linearizedModel=Linearized.transferFunctions()):
    """Function to compute the control gains using the tuning parameters outlined in Beard Chapter 6. Both the lateral
    and longitudinal gains are calculated. No check is made for frequency separation. Transfer function input comes form
    the VehiclePerturbationModels which rely on the VehicleTrim.py (provided) to compute the trim values.
    """
    #instance of controlGains
    gains = Controls.controlGains()

    #Lateral direwctional autopilot
    #phi
    gains.kp_roll = (tuningParameters.Wn_roll**2) / linearizedModel.a_phi2
    gains.kd_roll = ((2 * tuningParameters.Zeta_roll * tuningParameters.Wn_roll) - linearizedModel.a_phi1) / linearizedModel.a_phi2
    gains.ki_roll = 0.001

    #course
    gains.ki_course = ((tuningParameters.Wn_course**2) * linearizedModel.Va_trim) / VPC.g0
    gains.kp_course = (2 * tuningParameters.Zeta_course * tuningParameters.Wn_course * linearizedModel.Va_trim) / VPC.g0

    #sideslip
    gains.ki_sideslip = ((tuningParameters.Wn_sideslip**2) / linearizedModel.a_beta2)
    gains.kp_sideslip = ((2 * tuningParameters.Zeta_sideslip * tuningParameters.Wn_sideslip) - linearizedModel.a_beta1) / linearizedModel.a_beta2

    # Longitudinal directional autopilot
    #pitch
    gains.kp_pitch = ((tuningParameters.Wn_pitch**2) - linearizedModel.a_theta2) / linearizedModel.a_theta3
    gains.kd_pitch = ((2 * tuningParameters.Zeta_pitch * tuningParameters.Wn_pitch) - linearizedModel.a_theta1) / linearizedModel.a_theta3

    #altitude
    K_thetaDC = (gains.kp_pitch * linearizedModel.a_theta3) / (linearizedModel.a_theta2 + (gains.kp_pitch * linearizedModel.a_theta3))
    gains.ki_altitude = ((tuningParameters.Wn_altitude**2)) / (K_thetaDC * linearizedModel.Va_trim)
    gains.kp_altitude = (2 * tuningParameters.Zeta_altitude * tuningParameters.Wn_altitude) / (K_thetaDC * linearizedModel.Va_trim)

    #airspeed from pitch
    gains.ki_SpeedfromElevator = -(tuningParameters.Wn_SpeedfromElevator**2) / (K_thetaDC * VPC.g0)
    gains.kp_SpeedfromElevator = (linearizedModel.a_V1 - (2 * tuningParameters.Zeta_SpeedfromElevator * tuningParameters.Wn_SpeedfromElevator)) / (K_thetaDC * VPC.g0)

    # airspeed from throttle
    gains.ki_SpeedfromThrottle = (tuningParameters.Wn_SpeedfromThrottle**2) / linearizedModel.a_V2
    gains.kp_SpeedfromThrottle = ((2 * tuningParameters.Zeta_SpeedfromThrottle * tuningParameters.Wn_SpeedfromThrottle) - linearizedModel.a_V1) / linearizedModel.a_V2

    return gains


def computeTuningParameters(controlGains=Controls.controlGains(), linearizedModel=Linearized.transferFunctions()):
    """Function to compute the tuning parameters given the gains in the successive loop closure, needs a try block to
    deal with taking square root of negative number. Function should never fail, if an exception occurs, return an empty
    (inited) turningParameters class. Transfer function input comes form the VehiclePerturbationModels which rely on the
    VehicleTrim.py (provided) to compute the trim value
    """
    #instance of control tuning
    tune = Controls.controlTuning()

    #Lateral directional autopilot
    #phi
    tune.Wn_roll = math.sqrt(controlGains.kp_roll * linearizedModel.a_phi2)
    tune.Zeta_roll = (linearizedModel.a_phi1 + (linearizedModel.a_phi2 * controlGains.kd_roll)) / (2 * tune.Wn_roll)

    #course
    tune.Wn_course = math.sqrt((VPC.g0 * controlGains.ki_course) / linearizedModel.Va_trim)
    tune.Zeta_course = ((VPC.g0 * controlGains.kp_course) / linearizedModel.Va_trim) / (2 * tune.Wn_course)

    #sideslip
    tune.Wn_sideslip = math.sqrt(linearizedModel.a_beta2 * controlGains.ki_sideslip)
    tune.Zeta_sideslip = (linearizedModel.a_beta1 + (linearizedModel.a_beta2 * controlGains.kp_sideslip)) / (2 * tune.Wn_sideslip)

    #Longitudinal directional autopilot
    #pitch
    tune.Wn_pitch = math.sqrt(linearizedModel.a_theta2 + (controlGains.kp_pitch * linearizedModel.a_theta3))
    tune.Zeta_pitch = (linearizedModel.a_theta1 + (controlGains.kd_pitch * linearizedModel.a_theta3)) / (2 * tune.Wn_pitch)

    #altitude
    K_thetaDC = (controlGains.kp_pitch * linearizedModel.a_theta3) / (linearizedModel.a_theta2 + (controlGains.kp_pitch * linearizedModel.a_theta3))     #K_thetaDC for altitude and airspeed
    tune.Wn_altitude = math.sqrt(K_thetaDC * linearizedModel.Va_trim * controlGains.ki_altitude)
    tune.Zeta_altitude = (K_thetaDC * linearizedModel.Va_trim * controlGains.kp_altitude) / (2 * tune.Wn_altitude)

    #airspeed from pitch
    tune.Wn_SpeedfromElevator = math.sqrt(-K_thetaDC * VPC.g0 * controlGains.ki_SpeedfromElevator)
    tune.Zeta_SpeedfromElevator = (linearizedModel.a_V1 - (K_thetaDC * VPC.g0 * controlGains.kp_SpeedfromElevator)) / (2 * tune.Wn_SpeedfromElevator)

    #airspeed from throttle
    tune.Wn_SpeedfromThrottle = math.sqrt(linearizedModel.a_V2 * controlGains.ki_SpeedfromThrottle)
    tune.Zeta_SpeedfromThrottle = (linearizedModel.a_V1 + (linearizedModel.a_V2 * controlGains.kp_SpeedfromThrottle)) / (2 * tune.Wn_SpeedfromThrottle)

    return tune


