import math
import pickle
from ece163.Modeling import VehicleAerodynamicsModel
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

def computeTuningParameters(controlGains=Controls.controlGains(), linearizedModel=Linearized.transferFunctions()):
    """Function to compute the tuning parameters given the gains in the successive loop closure, needs a try block to
    deal with taking square root of negative number. Function should never fail, if an exception occurs, return an empty
    (inited) turningParameters class. Transfer function input comes form the VehiclePerturbationModels which rely on the
    VehicleTrim.py (provided) to compute the trim value
    """