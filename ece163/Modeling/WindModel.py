import math
import random
from ..Containers import States
from ..Utilities import MatrixMath as mm
from ..Constants import VehiclePhysicalConstants as VPC
from ..Controls import VehiclePerturbationModels as VPM

class WindModel():
    """Implements the Dryden gust model and allows for an update of the wind at every time step when driven by white noise.
    """

    def __init__(self, dT=VPC.dT, Va=VPC.InitialSpeed, drydenParamters = VPC.DrydenNoWind):
        """initialize the wind model code. Will load the appropriate constants that parameterize the wind gusts from the
        Dryden gust model. Creates the discrete transfer functions for the gust models that are used to update the local
        wind gusts in the wind frame. These are added to the inertial wind (Wn, We, Wd) that are simply constants.
        Discrete models are held in self and used in the Update function
        """

        #defineing the attributes for Va and dT
        self.dT = dT
        self.Va = Va

        #attribute for the dryden parameters
        self.drydenParamters = drydenParamters

        #attribute for the transfer functions
        self.Phi_u = 0
        self.Phi_v = 0
        self.Phi_w = 0
        self.Gamma_u = 0
        self.Gamma_v = 0
        self.Gamma_w = 0
        self.H_u = 0
        self.H_v = 0
        self.H_w = 0
        self.CreateDrydenTransferFns(self.dT, self.Va, self.drydenParamters)

        #attribute for windState
        self.windState = States.windState()

        #attribute for xu, xv, xw
        self.x_u = [[0]]
        self.x_v = [[0], [0]]
        self.x_w = [[0], [0]]

        return

    def reset(self):
        """Wrapper function that resets the wind model code (but does not reset the model chosen for wind. To change the
        model transfer functions you need to use CreateDrydenTranferFns.
        """
        self.dT = VPC.dT
        self.Va = VPC.InitialSpeed
        self.drydenParamters = VPC.DrydenNoWind
        self.windState = States.windState()
        self.CreateDrydenTransferFns(self.dT, self.Va, self.drydenParamters)
        return

    def setWind(self, windState):
        """Wrapper function that allows for injecting constant wind and gust values into the class :param windState:
        class from vehicleStates with inertial constant wind and wind frame gusts
        """
        self.windState = windState
        return

    def getWind(self):
        """Wrapper function to return the wind state from the module
        """
        return self.windState

    def setWindModelParameters(self,Wn=0.0, We=0.0, Wd=0.0, drydenParamters=VPC.DrydenNoWind):
        """def setWindModel(self, Wn=0.0, We=0.0, Wd=0.0, drydenParamters=VPC.DrydenNoWind): Wrapper function that will
        inject constant winds and gust parameters into the wind model using the constant wind in the inertial frame
        (steady wind) and gusts that are stochastically derived in the body frame using the Dryden wind gust models.
        """
        self.windState.Wn = Wn
        self.windState.We = We
        self.windState.Wd = Wd
        self.drydenParamters = drydenParamters
        self.CreateDrydenTransferFns(self.dT, VPC.InitialSpeed, drydenParamters)
        return

    def CreateDrydenTransferFns(self,dT, Va, drydenParamters):
        """def CreateDrydenTransferFns(self, dT, Va, drydenParamters): Function creates the Dryden transfer functions in
        discrete form. These are used in generating the gust models for wind gusts (in wind frame).
        """

        if drydenParamters.Lu == 0.0:
            self.Phi_u = [[1]]
            self.Gamma_u = [[0]]
            self.H_u = [[1]]
        else:
            self.Phi_u = [[math.exp(-Va * dT / drydenParamters.Lu)]]
            self.Gamma_u = [[(drydenParamters.Lu / Va) * (1 - math.exp(-Va * dT / drydenParamters.Lu))]]
            self.H_u = [[drydenParamters.sigmau * math.sqrt((2 * Va) / (math.pi * drydenParamters.Lu))]]

        # attributes for Psi, gamma, H for the v component
        if drydenParamters.Lv == 0.0:
            self.Phi_v = [[1, 0], [0, 1]]
            self.Gamma_v = [[0], [0]]
            self.H_v = [[1, 1]]
        else:
            self.Phi_v = mm.scalarMultiply(math.exp(-Va * dT / drydenParamters.Lv), [[1 - (Va * dT / drydenParamters.Lv), - ((Va / drydenParamters.Lv)**2) * dT],
                                                                       [dT, 1 + ((Va / drydenParamters.Lv) * dT)]])
            self.Gamma_v = mm.scalarMultiply(math.exp(-Va * dT / drydenParamters.Lv), [[dT],
                                                                         [((drydenParamters.Lv / Va)**2) * (math.exp(Va * dT/drydenParamters.Lv) - 1) - ((drydenParamters.Lv * dT)/Va)]])
            self.H_v = mm.scalarMultiply(drydenParamters.sigmav * math.sqrt((3 * Va) / (math.pi * drydenParamters.Lv)), [[1, Va/(math.sqrt(3) * drydenParamters.Lv)]])

        # attributes for Psi, gamma, H for the w component
        if drydenParamters.Lw == 0.0:
            self.Phi_w = [[1, 0], [0, 1]]
            self.Gamma_w = [[0], [0]]
            self.H_w = [[1, 1]]
        else:
            self.Phi_w =mm.scalarMultiply(math.exp(-Va * dT / drydenParamters.Lw), [[1 - (Va * dT / drydenParamters.Lw), - ((Va / drydenParamters.Lw)**2) * dT],
                                                                      [dT, 1 + ((Va / drydenParamters.Lw) * dT)]])
            self.Gamma_w = mm.scalarMultiply(math.exp(-Va * dT / drydenParamters.Lw), [[dT],
                                                                         [((drydenParamters.Lw / Va)**2) * (math.exp(Va * dT/drydenParamters.Lw) - 1) - ((drydenParamters.Lw * dT)/Va)]])
            self.H_w = mm.scalarMultiply(drydenParamters.sigmaw * math.sqrt((3 * Va) / (math.pi * drydenParamters.Lw)), [[1, Va/(math.sqrt(3) * drydenParamters.Lw)]])

        return

    def getDrydenTransferFns(self):
        """Wrapper function to return the internals of the Dryden Transfer function in order to be able to test the code
        without requiring consistent internal names. Returns the discretized version of the Drydem gust model as outlined
        in the ECE163_DrydenWindModel handout (Phi_u, Gamma_u, H_u, Phi_v, Gamma_v, H_v, Phi_w, Gamma_w, H_w).
        """
        return self.Phi_u, self.Gamma_u, self.H_u, self.Phi_v, self.Gamma_v, self.H_v, self.Phi_w, self.Gamma_w, self.H_w

    def Update(self,uu=None, uv=None, uw=None):
        """def Update(self, uu=None, uv=None, uw=None): Function that updates the wind gusts and inserts them back into
        the .Wind portion of self. This is done by running white noise [Gaussian(0,1)] through the coloring filters of
        the Dryden Wind Gust model.
        """

        #creating random numbers for u
        if uu is None:
            uu = random.gauss(0, 1)

        if uv is None:
            uv = random.gauss(0, 1)

        if uw is None:
            uw = random.gauss(0, 1)

        #Euler step for x
        self.x_u = mm.add(mm.multiply(self.Phi_u, self.x_u),mm.scalarMultiply(uu, self.Gamma_u))
        self.x_v = mm.add(mm.multiply(self.Phi_v, self.x_v),mm.scalarMultiply(uv, self.Gamma_v))
        self.x_w = mm.add(mm.multiply(self.Phi_w, self.x_w),mm.scalarMultiply(uw, self.Gamma_w))

        #Gusts from state
        self.windState.Wu = mm.multiply(self.H_u, self.x_u)[0][0]
        self.windState.Wv = mm.multiply(self.H_v, self.x_v)[0][0]
        self.windState.Ww = mm.multiply(self.H_w, self.x_w)[0][0]
        return