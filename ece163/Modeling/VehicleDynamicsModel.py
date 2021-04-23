import math
from ..Containers import States
from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

def c(angle):
    return math.cos(angle)
def s(angle):
    return math.sin(angle)
def t(angle):
    return math.tan(angle)
class VehicleDynamicsModel():
    def __init__(self, dT=VPC.dT):
        self.dT= dT
        self.state = States.vehicleState()
        self.dot = States.vehicleState()
        return

    def setVehicleState(self, state):
        #set self.state to the state
        self.state = state
        return

    def getVehicleState(self):
        #returning back the state
        return self.state

    def setVehicleDerivative(self,dot):
        #set self.dot to the deritative dot
        self.dot = dot
        return

    def getVehicleDerivative(self):
        #get the derivative
        return self.dot

    def reset(self):
        #set state, dot, and dT back to their original point
        self.state = States.vehicleState()
        self.dot = States.vehicleState()
        self.dT = 0.01
        return

    def Update(self, forcesMoments):
        #update the time step
        self.dot = self.derivative(self.state, forcesMoments)
        self.state = self.IntegrateState(self.dT, self.state, self.dot)
        return

    def Rexp(self, dT, state, dot):
        #trapezoidal approx of p,q,r
        pp = state.p + (0.5 * dot.p * dT)
        qq = state.q + (0.5 * dot.q * dT)
        rr = state.r + (0.5 * dot.r * dT)
        #taking the magniude of w, trapezoidal
        magw = math.sqrt(pp**2 + qq**2 + rr**2)
        omegax = [[0,-rr,(qq)],[(rr),0,-pp],[-qq,(pp),0]]
        mag_omegaxdT =math.sqrt((2*(pp*dT)**2)+(2*(qq*dT)**2)+(2*(rr*dT)**2))
        #Using the contraint of the size of mag_w of it being greater than or less than 0.2
        if mag_omegaxdT >= 0.02:
            sin = math.sin(magw*dT)/magw
            cos = (1-math.cos(magw*dT))/(magw**2)
            I = ([[1,0,0],[0,1,0],[0,0,1]])
            Rexp = mm.matrixAdd(mm.matrixSubtract(I, mm.matrixScalarMultiply(sin,omegax)),mm.matrixScalarMultiply(cos,mm.matrixMultiply(omegax,omegax)))
            return Rexp
        else:
            sin = dT-(((dT**3)*(magw**2))/6)+(((dT**5)*(magw**4))/120)
            cos = (((dT**2)/2)-(((dT**4)*(magw**2)/24)+(((dT**6)*(magw**4)/720))))
            I = ([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
            Rexp2 = mm.matrixAdd((mm.matrixSubtract(I, mm.matrixScalarMultiply(sin,omegax))),mm.matrixScalarMultiply(cos,mm.matrixMultiply(omegax,omegax)))
            return Rexp2

    def derivative(self, state, forcesMoments):
        dot = States.vehicleState()
        #derivatives of pn, pe, pd
        p = mm.matrixMultiply(mm.matrixTranspose((state.R)),
                              mm.matrixTranspose([[state.u, state.v, state.w]]))
        dot.pn = p[0][0]
        dot.pe = p[1][0]
        dot.pd = p[2][0]
        #derivatives of u, v, and w
        wx_neg = [[0, -state.r, (state.q)], [(state.r), 0, -state.p], [-state.q, (state.p), 0]]
        f = [[forcesMoments.Fx],[forcesMoments.Fy],[forcesMoments.Fz]]
        v = mm.matrixSubtract(mm.matrixScalarMultiply(1/VPC.mass,f),mm.matrixMultiply(wx_neg,([[state.u],[state.v],[state.w]])))
        dot.u = v[0][0]
        dot.v = v[1][0]
        dot.w = v[2][0]

        #derivatives of yaw,pitch, and roll
        w = ([[state.p], [state.q], [state.r]])
        C= [[1, s(state.roll)*t(state.pitch), c(state.roll)*t(state.pitch)],[0,  c(state.roll), -s(state.roll)],
            [0, s(state.roll)/c(state.pitch), c(state.roll)/c(state.pitch)]]
        g = mm.matrixMultiply(C,w)
        dot.yaw = g[2][0]
        dot.pitch = g[1][0]
        dot.roll = g[0][0]

        #derivatives of p, q, are r
        wx = [[0, -state.r, (state.q)], [(state.r), 0, -state.p], [-state.q, (state.p), 0]]
        M = [[forcesMoments.Mx], [forcesMoments.My], [forcesMoments.Mz]]
        q = mm.matrixMultiply(VPC.JinvBody, mm.matrixSubtract(M,mm.matrixMultiply(wx,mm.matrixMultiply(VPC.Jbody,w))))
        dot.p = q[0][0]
        dot.q = q[1][0]
        dot.r = q[2][0]

        #derivative of the rotation matrix R
        if state.p != 0 and state.q != 0 and state.r != 0:
            dot.R = mm.matrixMultiply(mm.matrixScalarMultiply(1,wx), state.R)
        else:
            dot.R = [[1,0,0],[0,1,0],[0,0,1]]
        return dot

    def ForwardEuler(self, dT, state, dot):
        #define all the forward Euler componets of state and dot
        states = States.vehicleState
        states.pn = state.pn + (self.dT * dot.pn)
        states.pe = state.pe + (self.dT * dot.pe)
        states.pd = state.pd + (self.dT * dot.pd)
        states.u = state.u + (self.dT * dot.u)
        states.v = state.v + (self.dT * dot.v)
        states.w = state.w + (self.dT * dot.w)
        states.yaw = state.yaw + (self.dT * dot.yaw)
        states.pitch = state.pitch + (self.dT * dot.pitch)
        states.roll = state.roll + (self.dT * dot.roll)
        states.p = state.p + (self.dT * dot.p)
        states.q = state.q + (self.dT * dot.q)
        states.r = state.r + (self.dT * dot.r)
        return states

    def IntegrateState(self, dT, state, dot):
        #Using forward euler to find the integral of state while using Rexp for R and keeping yaw, pitch, roll, alpha, beta,
        # Va and chi constant
        newState = self.ForwardEuler(dT, state, dot)
        newState.R = mm.matrixMultiply(self.Rexp(dT,state,dot), state.R)
        newState.yaw = state.yaw
        newState.pitch = state.pitch
        newState.roll = state.roll
        newState.alpha = state.alpha
        newState.beta = state.beta
        newState.Va = state.Va
        newState.chi = math.atan2(dot.pe,dot.pn)
        return newState