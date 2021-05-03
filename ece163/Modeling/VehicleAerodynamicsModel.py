import math
from ..Containers import States
from ..Containers import Inputs
from ..Modeling import VehicleDynamicsModel as VDM
from ..Modeling import WindModel
from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC
from ..Modeling import WindModel as WM

""" Author: Luca Altaffer (taltaffe@ucsc.edu)
This module is where all of the vehicle dynamics are computed for the simulation. It includes the kinematics of both the
 translational and rotational dynamics. Included are both the derivative, and the integration functions, and the 
 rotations of forces to the body frame."""

class VehicleAerodynamicsModel():

    def __init__(self, initialSpeed= VPC.InitialSpeed, initialHeight= VPC.InitialDownPosition):
        """Initialization of the internal classes which ar used to track the vehicle aerodynamics and dynamics."""
        self.initialSpeed = initialSpeed
        self.initialHeight = initialHeight
        self.vehicleDynamicsModel = VDM.VehicleDynamicsModel()
        self.WindModel = WM.WindModel()
        return

    def reset(self):
        """Reset the Vehicle state to initial conditions"""
        self.initialSpeed = VPC.InitialSpeed
        self.initialHeight = VPC.InitialDownPosition
        self.vehicleDynamicsModel = VDM.VehicleDynamicsModel()
        self.WindModel = WM.WindModel()
        return

    def Update(self, controls):
        """Function that uses the current state (internal), wind (internal), and controls (inputs) to calculate the
        forces, and then do the integration of the full 6-DOF non-linear equations of motion. Wraps the
        VehicleDynamicsModel class as well as the windState internally. The Wind and the vehicleState are maintained
        internally."""

        #getting forcesMoments from updateForces
        forcesMoments = self.updateForces(self.vehicleDynamicsModel.state, controls)

        #Putting forcesMoments into update
        self.vehicleDynamicsModel.Update(forcesMoments)
        return

    def getVehicleState(self):
        """Getter method to read the vehicle state"""
        return self.vehicleDynamicsModel.state

    def setVehicleState(self, state):
        """Setter method to write the vehicle state"""
        self.vehicleDynamicsModel.state = state
        return

    def getVehicleDynamicsModel(self):
        """Wrapper function to return the vehicle dynamics model handle"""
        return self.vehicleDynamicsModel

    def gravityForces(self, state):
        """Function to project gravity forces into the body frame. Uses the gravity constant g0 from physical constants
        and the vehicle mass. Fg = m * R * [0 0 g0]’"""
        gravity = Inputs.forcesMoments(state)
        fg = mm.matrixMultiply(mm.matrixScalarMultiply(VPC.mass, state.R),[[0],[0],[VPC.g0]])
        gravity.Fx = fg[0][0]
        gravity.Fy = fg[1][0]
        gravity.Fz = fg[2][0]
        return gravity

    def CalculateCoeff_alpha(self, alpha):
        """Function to calculate the Coefficient of Lift and Drag as a function of angle of attack. Angle of attack
         (alpha) in [rad] is contained within the state.alpha and updated within the CalculateAirspeed function.
         For the coefficient of drag, we are using the parabolic form: CD = CDp + (CLalpha)^2/(pi*AR*e)"""

        #using the defined sigma function from Hw2
        def sigma(a):
            num = (1 + math.exp(-VPC.M * (a - VPC.alpha0)) + math.exp(VPC.M * (a + VPC.alpha0)))
            den = (1 + math.exp(-VPC.M * (a - VPC.alpha0))) * (1 + math.exp(VPC.M * (a + VPC.alpha0)))
            return num / den

        #using the define Cl lamimar function from Hw2
        def Cl_lam(a, Cl0, Cla):
            cl_lam = Cl0 + (Cla * a)
            return cl_lam

        # using the defined Cd lamimar function from Hw2
        def Cd_lam(a, Cdp, Cl0, Cla, AR):
            cd_lam = Cdp + (((Cl0 + (Cla * a)) ** 2) / (math.pi * AR * VPC.e))
            return cd_lam

        #using the defined Cl turbulent function from Hw2
        def Cl_tur(a):
            cl_tur = 2 * math.sin(a) * math.cos(a)
            return cl_tur

        #using the defined Cd turbulent function from Hw2
        def Cd_tur(a):
            cd_tur = 2 * (math.sin(a)) ** 2
            return cd_tur

        #Putting the functions all together to solve for Cl and Cd
        C_L = (((1 - sigma(alpha)) * Cl_lam(alpha, VPC.CL0, VPC.CLalpha)) +
               (sigma(alpha) * Cl_tur(alpha)))
        C_D = ((1 - sigma(alpha)) * Cd_lam(alpha, VPC.CDp, VPC.CL0, VPC.CLalpha, VPC.AR) +
                (sigma(alpha) * Cd_tur(alpha)))

        #defining Cm
        C_M = (VPC.CM0 + (VPC.CMalpha * alpha))

        return C_L, C_D, C_M

    def aeroForces(self, state):
        """Function to calculate the Aerodynamic Forces and Moments using the linearized simplified force model and the
         stability derivatives in VehiclePhysicalConstants.py file. Specifically does not include forces due to control
         surface deflection. Requires airspeed (Va) in [m/s], angle of attack (alpha) in [rad] and sideslip angle (beta)
        in [rad] from the state."""

        #Setting up the reference for CalculateCoeff_alpha
        C_L, C_D, C_M = self.CalculateCoeff_alpha(state.alpha)

        #Setting to forces moments class
        aeroforces = Inputs.forcesMoments(state)

        #Using Beard 4.6 and 4.7 to fild the Lift and drag forces that depend on the airspeed velocity Va
        if state.Va == 0:
            aeroforces.Fx = 0
            aeroforces.Fz = 0
        else:
            #Finding the first 2 terms of Beard 4.6 and 4.7
            F_lift = (.5 * VPC.rho * (state.Va**2) * VPC.S) * \
                     (C_L + (VPC.CLq * ((VPC.c * state.q)/(2 * state.Va))))
            F_drag = (.5 * VPC.rho * (state.Va**2) * VPC.S) * \
                     ((C_D) + (VPC.CDq * ((VPC.c * state.q) / (2 * state.Va))))

            #rotrating Lift and Drag to the body frame
            aeroforces.Fx = (-F_drag * math.cos(state.alpha)) + (F_lift * math.sin(state.alpha))
            aeroforces.Fz = (-F_drag * math.sin(state.alpha)) - (F_lift * math.cos(state.alpha))

        #using Beard 14 to find the first four terms of the lateral force
        if state.Va == 0:
            aeroforces.Fy =  (.5 * VPC.rho * (state.Va**2) * VPC.S) * \
                                    (VPC.CY0 + (VPC.CYbeta * state.beta) + (0) +
                                     (0))
        else:
            aeroforces.Fy = (.5 * VPC.rho * (state.Va**2) * VPC.S) * \
                                    (VPC.CY0 + (VPC.CYbeta * state.beta) + ((VPC.CYp * VPC.b * state.p)/(2*state.Va)) +
                                     ((VPC.CYr * VPC.b * state.r)/(2*state.Va)))

        #using Beard 15 to find the roll moment
        if state.Va == 0:
            aeroforces.Mx = (.5 * VPC.rho * (state.Va**2) * VPC.S * VPC.b) * \
                (VPC.Cl0 + (VPC.Clbeta * state.beta) + (0) + (0))
        else:
            aeroforces.Mx = (.5 * VPC.rho * (state.Va**2) * VPC.S * VPC.b) * \
                (VPC.Cl0 + (VPC.Clbeta * state.beta) + ((VPC.Clp * VPC.b * state.p) /(2 * state.Va)) + ((VPC.Clr * VPC.b * state.r) / (2 * state.Va)))

        #using Beard 16 to find the yaw moment
        if state.Va == 0:
            aeroforces.Mz = (.5 * VPC.rho * (state.Va**2) * VPC.S * VPC.b) * \
                (VPC.Cn0 + (VPC.Cnbeta * state.beta) + 0 + 0)
        else:
            aeroforces.Mz = (.5 * VPC.rho * (state.Va**2) * VPC.S * VPC.b) * \
                (VPC.Cn0 + (VPC.Cnbeta * state.beta) + ((VPC.Cnp * VPC.b * state.p) /(2 * state.Va)) + ((VPC.Cnr * VPC.b * state.r) / (2 * state.Va)))

        #Using Beard 5 to find the pitching moment
        if state.Va == 0:
            aeroforces.My = 0
        else:
            aeroforces.My = (.5 * VPC.rho * (state.Va**2) * VPC.S * VPC.c) * \
                            (C_M + (VPC.CMq * (VPC.c * state.q )/(2 * state.Va)))

        return aeroforces

    def controlForces(self, state, controls):
        """Function to calculate aerodynamic forces from control surface deflections (including throttle) using the
        linearized aerodynamics and simplified thrust model. Requires airspeed (Va) in [m/s] and angle of attack (alpha)
        in [rad] both from state.Va and state.alpha respectively."""

        #setting terms to be of the forcesMoments() class
        controlforces = Inputs.forcesMoments(state,controls)

        #finding the Lift and Drag in terms of the delta values of Beard 4.6 and 4.7
        F_lift = (.5 * VPC.rho * (state.Va**2) * VPC.S) * (VPC.CLdeltaE * controls.Elevator)
        F_drag = (.5 * VPC.rho * (state.Va**2) * VPC.S) * (VPC.CDdeltaE * controls.Elevator)

        #rotating the forces to be in body frame
        Fx_body = (-F_drag * math.cos(state.alpha)) + (F_lift * math.sin(state.alpha))
        controlforces.Fz = (-F_drag * math.sin(state.alpha)) - (F_lift * math.cos(state.alpha))

        #finding the lateral force, fy, roll moment, l, and yaw moment, n in terms of the delta of Beard 4.14, 4.15, 4.16
        controlforces.Fy = (.5 * VPC.rho * (state.Va**2) * VPC.S) * ((VPC.CYdeltaA * controls.Aileron) + (VPC.CYdeltaR * controls.Rudder))
        l = (.5 * VPC.rho * (state.Va**2) * VPC.S * VPC.b) * ((VPC.CldeltaA * controls.Aileron) + (VPC.CldeltaR * controls.Rudder))
        controlforces.Mz = (.5 * VPC.rho * (state.Va**2) * VPC.S * VPC.b) * ((VPC.CndeltaA * controls.Aileron) + (VPC.CndeltaR * controls.Rudder))
        controlforces.My = (.5 * VPC.rho * (state.Va**2) * VPC.S * VPC.c) * ((VPC.CMdeltaE * controls.Elevator))

        #using CalculatePropForces() to incorporate the forces of the propeller in controlForces()
        fx, mx = self.CalculatePropForces(state.Va, controls.Throttle)

        #combining the X forces and moments
        controlforces.Fx = Fx_body + fx
        controlforces.Mx = l + mx

        return controlforces

    def CalculatePropForces(self, Va, Throttle):
        """Function to calculate the propeller forces and torques on the aircraft. Uses the fancy propeller model that
         parameterizes the torque and thrust coefficients of the propeller using the advance ratio."""

        #Calculate omega from Beard supplemental book defining a, b, and c first
        KT = ((60/(2 * math.pi * VPC.KV)))
        Vin = Throttle * VPC.V_max
        a = (VPC.rho * (VPC.D_prop**5) * VPC.C_Q0) / ((2 * math.pi)**2)
        b = ((VPC.rho * (VPC.D_prop**4) * VPC.C_Q1 * Va) / (2 * math.pi)) + ((KT**2)/VPC.R_motor)
        c = (VPC.rho * (VPC.D_prop**3) * VPC.C_Q2 * (Va**2)) - ((KT * Vin)/ VPC.R_motor) + (KT * VPC.i0)
        omega = (-b + math.sqrt((b**2) - (4 * a * c))) / (2 * a)

        #calculating J that depends on Va and omega
        J = (2 * math.pi * Va)/(omega * VPC.D_prop)

        #calculating CT and CQ to plug into Fx and Mx
        CT = VPC.C_T0 + (VPC.C_T1 * J) + (VPC.C_T2 * (J**2))
        CQ = VPC.C_Q0 + (VPC.C_Q1 * J) + (VPC.C_Q2 * (J**2))

        #creating the torque and thrust vectors
        Fp = (VPC.rho * (omega**2) * (VPC.D_prop**4) * CT)/((2*math.pi)**2)
        Mp = -(VPC.rho * (omega**2) * (VPC.D_prop**5) * CQ)/((2*math.pi)**2)

        return Fp, Mp

    def updateForces(self, state, controls, wind=None):
        """Function to update all of the aerodynamic, propulsive, and gravity forces and moments. All calculations
        required to update the forces are included. state is updated with new values for airspeed, angle of attack, and
        sideslip angles (see class definition for members)"""

        #using the definintion from States.py to find Va, alpha, and beta
        state.Va = math.hypot(state.u, state.v, state.w)  # Airspeed
        state.alpha = math.atan2(state.w, state.u)  # angle of attack
        if math.isclose(state.Va, 0.0):  # Sideslip Angle, no airspeed
            state.beta = 0.0
        else:
            state.beta = math.asin(state.v / state.Va)  # Sideslip Angle, normal definition

        #Recalling the functions defined
        Fgravity = self.gravityForces(state)
        Faero = self.aeroForces(state)
        Fcontrols = self.controlForces(state, controls)
        Fwind = self.CalculateAirspeed(state,wind)

        #addings the forces together
        total_forces = Inputs.forcesMoments()
        total_forces.Fx = Fgravity.Fx + Faero.Fx + Fcontrols.Fx
        total_forces.Fy = Fgravity.Fy + Faero.Fy + Fcontrols.Fy
        total_forces.Fz = Fgravity.Fz + Faero.Fz + Fcontrols.Fz
        total_forces.Mx = Fgravity.Mx + Faero.Mx + Fcontrols.Mx
        total_forces.My = Fgravity.My + Faero.My + Fcontrols.My
        total_forces.Mz = Fgravity.Mz + Faero.Mz + Fcontrols.Mz

        return total_forces

    def getWindModel(self):
        """Wrapper function to return the windModel"""
        return self.WindModel

    def setWindModel(self, windModel):
        """Wrapper function to set the windModel"""
        self.WindModel = windModel
        return

    def CalculateAirspeed(self,state,wind):
        """Calculates the total airspeed, as well as ange of attack and side-slip angles from the wind and current state.
        Needed for further aerodynamic force calculations. Va, wind speed [m/s], alpha, angle of attack [rad], and beta,
        side-slip angle [rad] are returned from the function. The state must be updated outside this function.
        """

        #creating tools for the trig functions to make math easier
        def c(angle):
            return math.cos(angle)

        def s(angle):
            return math.sin(angle)

        def t(angle):
            return math.tan(angle)

        #Defing magnitude steady wind in the inertial frame
        Ws = [[wind.n], [wind.e], [wind.d]]
        Wsmag = math.sqrt(wind.n**2 + wind.e**2 + wind.d**2)

        #Defining the azimuth, xw and elevation gw
        xw = math.atan2(wind.e, wind.n)
        gw = -math.asin(wind.d/Wsmag)

        #Azimuth Elevation rotation matrix
        R = [[c(xw) * c(gw), s(xw) * c(gw), -s(xw)], [-s(xw), c(xw), 0], [c(xw) * s(gw), s(xw) * s(gw), c(gw)]]

        #Defining the gust
        Wg = [[wind.u], [wind.v], [wind.w]]
        iWg = mm.matrixMultiply(mm.matrixTranspose(R), Wg)

        #wind in the body frame
        Wb = mm.matrixMultiply(state.R, mm.matrixAdd(Ws, iWg))

        #Calculating the airspeed Va
        Va = mm.matrixSubtract([[state.u], [state.v], [state.w]], Wb)
        Vamag = math.sqrt(Va[0][0]**2 + Va[1][0]**2 + Va[2][0]**2)

        #Calculating the angle of attack alpha
        alpha = math.atan(Va[2][0]/Va[0][0])

        #Calculating the side-slip angle beta
        beta = math.asin(Va[1][0]/math.sqrt(Va[0][0]**2 + Va[1][0]**2 + Va[2][0]**2))

        return Vamag, alpha, beta