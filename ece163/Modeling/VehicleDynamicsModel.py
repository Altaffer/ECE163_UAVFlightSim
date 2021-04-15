import math
from ..Containers import States
from ..Containers import Inputs
from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

def c(angle):
    return math.cos(angle)
def s(angle):
    return math.sin(angle)
def t(angle):
    return math.tan(angle)
class VehicleDynamicsModel(dT=0.01):
    def __init__(self,dT = 0.01):
        self.dT= dT
        self.state = States.vehicleState()
        self.dot = States.vehicleState()
        #self.derivative.pn =3

    def setVehicleState(self, state):
        self.state = state

    def getVehicleState(self):
        return self.state

    def setVehicleDerivative(self,dot):
       self.dot = dot

    def getVehicleDerivative(self):
        return self.dot

    def reset(self):
        self.state = States.vehicleState()
        self.dot = States.vehicleState()
        self.dT = 0.01

    def Update(self, forcesMoments):
        self.state = self.getVehicleState(forcesMoments)
        self.state = self.IntegrateState(self.dT, self.state, self.dot,forcesMoments)
        self.dot = self.derivative(self.state,forcesMoments)

    def Rexp(self, dT, state, dot):
        wk = state.w
        wk1 = state.w+(dT*(dot.w))
        w = (wk+wk1)/2
        #[[state.p,state.q,state.r]]
        magw = math.sqrt(w[0][0]**2 + w[0][1]**2 + w[0][2]**2)
        omegax = [[0,state.r,(-state.q)],[(-state.r),0,state.p],[state.q,(-state.p),0]]
        omegaxdT = [[0,VPC.dT*state.r,VPC.dT*(-state.q)],[VPC.dT*(-state.r),0,VPC.dT*state.p],[VPC.dT*state.q,VPC.dT*(-state.p),0]]
        mag_omegaxdT =math.sqrt(omegaxdT[0][0]**2 + omegaxdT[0][1]**2 + omegaxdT[0][2]**2 + omegaxdT[1][0]**2 +
                                omegaxdT[1][1]**2 + omegaxdT[1][2]**2 + omegaxdT[2][0]**2 + omegaxdT[2][1]**2 +
                                omegaxdT[2][2] ** 2)
        if mag_omegaxdT > 0.2:
            Rexp = (mm.matrixAdd(mm.matrixSubtract([[1,0,0],[0,1,0],[0,0,1]], mm.matrixScalarMultiply(s(magw*dT)/magw,omegax)),
                                                  mm.matrixScalarMultiply((1-c(magw*dT))/magw**2,omegax**2)))
            return Rexp
        else:
            Rexp2 = (mm.matrixSubtract(mm.matrixSubtract([[1,0,0],[0,1,0],[0,0,1]], mm.matrixScalarMultiply(dT-((dT**(3)*magw**2)/6)+((dT**(5)*magw**4)/120),omegax)),
                                                  mm.matrixScalarMultiply(((dT**(2)/2)-((dT**(4)*magw**2)/24)+((dT**(6)*magw**4)/720)),omegax**2)))
            return Rexp2

    def derivative(self, state, forcesMoments):
        [[pn_dot, pe_dot, pd_dot]] = mm.matrixMultiply(mm.matrixTranspose(Rotations.euler2DCM(state.yaw, state.pitch, state.roll)),
                                                       [[state.u, state.v, state.w]])
        [[u_dot, v_dot, w_dot]] = mm.matrixAdd([[(state.r * state.v) - (state.q * state.w),
                                                 (state.p * state.w) - (state.r * state.u),
                                                 (state.q * state.u) - (state.p * state.v)]],
                                               [[Inputs.Fx/VPC.mass, Inputs.Fy/VPC.mass, Inputs.Pz/VPC.mass]])
        [[yaw_dot, pitch_dot, roll_dot]] = mm.matrixMultiply([[1, s(state.roll)*t(state.pitch), c(state.roll)*t(state.pitch)],
                                                              [0,  c(state.roll), -s(state.roll)],
                                                              [0, s(state.roll)/c(state.pitch), c(state.roll)/c(state.pitch)]],
                                                             [[state.p, state.q, state.r]])
        gamma = (VPC.Jxx*VPC.Jyy)-VPC.Jxz**2
        [[p_dot, q_dot, r_dot]] = mm.matrixMultiply([[VPC.Jzz/gamma, 0, VPC.Jxz/gamma], [0, 1/VPC.Jyy, 0],
                                                     [VPC.Jxz/gamma, 0, VPC.Jxx/gamma]],
                                                    (mm.matrixAdd([[(VPC.Jxz*state.p*state.q)+((VPC.Jyy-VPC.Jzz)*(state.q*state.r)),
                                                                    VPC.Jxz*(state.r**2-state.p**2)+((state.p*state.r)*(VPC.Jzz-VPC.Jxx)),
                                                                    ((state.p*state.q)*(VPC.Jxx-VPC.Jyy))-(VPC.Jxz*state.q*state.r)]],
                                                                  [[Inputs.Mx, Inputs.My, Inputs.Mz]])))
        R_dot = mm.matrixMultiply([[0,state.r,-state.q],[-state.r,0,state.p],[state.q,-state.p,0]], Rotations.euler2DCM(state.yaw,state.pitch,state.roll))

    def ForwardEuler(self, dT, state, dot):
        newState = state + (dT*dot)
        #tate.pn +(dT*dot.pn)
        return newState

    def IntegrateState(self, dT, state, dot):
        newState.alpha = state.alpha
        newState.beta = state.beta
        newState.Va = state.Va

        newState.chi = math.atan2(dot.pe,dot.pn)
        return newState