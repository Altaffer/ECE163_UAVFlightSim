import math
from ece163.Modeling import VehicleAerodynamicsModel as VAM
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath

def CreateTransferFunction(trimState, trimInputs):
    """Function to fill the transfer function parameters used for the successive loop closure from the given trim state
    and trim inputs. Note that these parameters will be later used to generate actual control loops. Vehicle Perturbation
    models are developed using the trim state and inputs. Models for transfer function parameters and state space
    implementations are calculated using constants in VehiclePhysicalParameters and the input trim state and trim control
    inputs. Results are returned as a Linearized.transferFunction class.
    """

    #initializing transferFunction class and derivative functions
    tF = Linearized.transferFunctions()
    dTdThrottle = dThrust_dThrottle(trimState.Va, trimInputs.Throttle, epsilon=0.01)
    dTdVa = dThrust_dVa(trimState.Va, trimInputs.Throttle, epsilon=0.5)

    #Setting the trimstate values to the transferFunctions
    tF.Va_trim = trimState.Va
    tF.alpha_trim = trimState.alpha
    tF.beta_trim = trimState.beta
    tF.gamma_trim =  trimState.pitch - trimState.alpha
    tF.theta_trim = trimState.pitch
    tF.phi_trim = trimState.roll

    #setting the other variables to the transferFunctions
    tF.a_phi1 = -(1/2) * VPC.rho * (trimState.Va**2) * VPC.S * VPC.b * VPC.Cpp * (VPC.b/(2 * trimState.Va))
    tF.a_phi2 = (1/2) * VPC.rho * (trimState.Va**2) * VPC.S * VPC.b * VPC.CpdeltaA
    tF.a_beta1 = -(VPC.rho * trimState.Va * VPC.S * VPC.CYbeta)/(2 * VPC.mass)
    tF.a_beta2 = (VPC.rho * trimState.Va * VPC.S * VPC.CYdeltaR)/(2 * VPC.mass)
    tF.a_theta1 = -((VPC.rho * (trimState.Va**2) * VPC.c * VPC.S * VPC.CMq)/(2 * VPC.Jyy)) * (VPC.c/(2 * trimState.Va))
    tF.a_theta2 = -((VPC.rho * (trimState.Va**2) * VPC.c * VPC.S * VPC.CMalpha)/(2 * VPC.Jyy))
    tF.a_theta3 = ((VPC.rho * (trimState.Va**2) * VPC.c * VPC.S * VPC.CMdeltaE)/(2 * VPC.Jyy))
    tF.a_V1 = ((VPC.rho * (trimState.Va) * VPC.S)/VPC.mass) * (VPC.CD0 + (VPC.CDalpha * trimState.alpha) + (VPC.CDdeltaE * trimInputs.Elevator)) -(dTdVa/VPC.mass)
    tF.a_V2 = dTdThrottle/VPC.mass
    tF.a_V3 = VPC.g0 * math.cos(trimState.pitch - trimState.alpha)

    return tF

def dThrust_dThrottle(Va, Throttle, epsilon=0.01):
    """def dThrust_dThrottle(Va, Throttle, epsilon=0.01): Function to calculate the numerical partial derivative of
    propeller thrust to change in throttle setting using the actual prop function from complex propeller model (inside
    the VehicleAerodynamicsModel class)
    """

    #creating instance of VehicleAerodynamicsModel and CalculatePropForces
    am = VAM.VehicleAerodynamicsModel()
    Fp1, Mp1 = am.CalculatePropForces(Va, Throttle + epsilon)
    Fp2, Mp2 = am.CalculatePropForces(Va, Throttle)

    #Calculating the derivative
    dTdDeltaT = (Fp1 - Fp2)/epsilon
    return dTdDeltaT

def dThrust_dVa(Va, Throttle, epsilon=0.5):
    """def dThrust_dVa(Va, Throttle, epsilon=0.5): Function to calculate the numerical partial derivative of propeller
    thrust to change in airspeed using the actual prop function from complex propeller model (inside the
    VehicleAerodynamicsModel class)
    """
    am = VAM.VehicleAerodynamicsModel()
    Fp1, Mp1 = am.CalculatePropForces(Va + epsilon, Throttle)
    Fp2, Mp2 = am.CalculatePropForces(Va, Throttle)

    # Calculating the derivative
    dTdVa = (Fp1 - Fp2)/epsilon
    return dTdVa