import math
import sys
import ece163.Containers.Inputs as Inputs
import ece163.Containers.Controls as Controls
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Modeling.VehicleAerodynamicsModel as VAM

class PDControl():
    def __init__(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """Functions which implement the PD control with saturation where the derivative is available as a separate input
        to the function. The output is: u = u_ref + Kp * error - Kd * dot{error} limited between lowLimit and highLimit.
        """
        self.dT = dT
        self.kp = kp
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.error = 0
        self.preverror = 0
        self.accumulator = 0

    def Update(self, command=0.0, current=0.0, derivative=0.0):
        """Calculates the output of the PD loop given the gains and limits from instantiation, and using the command,
        actual, and derivative inputs. Output is limited to between lowLimit and highLimit from instantiation.
        """
        # creating the error
        self.error = command - current

        # setting up the temporary accumulator
        tempaccumulator = self.dT * (self.error - self.preverror) / 2

        # accumulator
        self.accumulator += tempaccumulator

        # finding the real u values
        up = self.kp * self.error
        ud = self.kd + derivative
        utot = self.trim + up + ud

        if abs(utot) > self.highLimit:
            utot = self.highLimit
            self.accumulator -= tempaccumulator
        if abs(utot) < self.lowLimit:
            utot = self.lowLimit
            self.accumulator -= tempaccumulator

        self.preverror = self.error

        return utot

    def setPDGains(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """Function to set the gains for the PD control block (including the trim output and the limits)"""

class PIControl():
    def __init__(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """Functions which implement the PI control with saturation where the integrator has both a reset and an
        anti-windup such that when output saturates, the integration is undone and the output forced the output to the
        limit. The output is: u = u_ref + Kp * error + Ki * integral{error} limited between lowLimit and highLimit.
        """
        self.dT = dT
        self.kp = kp
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.error = 0
        self.preverror = 0
        self.accumulator = 0

    def Update(self, command=0.0, current=0.0):
        """Calculates the output of the PI loop given the gains and limits from instantiation, and using the command and
        current or actual inputs. Output is limited to between lowLimit and highLimit from instantiation. Integration for
        the integral state is done using trapezoidal integration, and anti-windup is implemented such that if the output
        is out of limits, the integral state is not updated (no additional error accumulation).
        """

        # creating the error
        self.error = command - current

        # setting up the temporary accumulator
        tempaccumulator = self.dT * (self.error - self.preverror) / 2

        # accumulator
        self.accumulator += tempaccumulator

        # finding the real u values
        up = self.kp * self.error
        ui = self.ki * self.accumulator
        utot = self.trim + up + ui

        if abs(utot) > self.highLimit:
            utot = self.highLimit
            self.accumulator -= tempaccumulator
        if abs(utot) < self.lowLimit:
            utot = self.lowLimit
            self.accumulator -= tempaccumulator

        self.preverror = self.error

        return utot

    def resetIntegrator(self):
        """Function to reset the integration state to zero, used when switching modes or otherwise resetting the integral
        state.
        """
        self.accumulator = 0
        self.error = 0
        self.preverror = 0

        return

    def setPIGains(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """Function to set the gains for the PI control block (including the trim output and the limits)
        """
        self.dT = dT
        self.kp = kp
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.accumulator = 0
        self.error = 0
        return

class PIDControl():
    def __init__(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """Functions which implement the PID control with saturation where the integrator has both a reset and an
        anti-windup such that when output saturates, the integration is undone and the output forced the output to the
        limit. Function assumes that physical derivative is available (e.g.: roll and p), not a numerically derived one.
        The output is: u = u_ref + Kp * error - Kd * dot{error} + Ki * integral{error} limited between lowLimit and
        highLimit.
        """
        self.dT = dT
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.error = 0
        self.preverror = 0
        self.accumulator = 0

        return

    def Update(self, command=0.0, current=0.0, derivative=0.0):
        """Calculates the output of the PID loop given the gains and limits from instantiation, and using the command
        and current or actual inputs. Output is limited to between lowLimit and highLimit from instantiation.
        Integration for the integral state is done using trapezoidal integration, and anti-windup is implemented such
        that if the output is out of limits, the integral state is not updated (no additional error accumulation).
        """

        # creating the error
        self.error = command - current

        # setting up the temporary accumulator
        tempaccumulator = self.dT * (self.error - self.preverror) / 2

        # accumulator
        self.accumulator += tempaccumulator

        # finding the real u values
        up = self.kp * self.error
        ud = self.kd + derivative
        ui = self.ki * self.accumulator
        utot = self.trim + up + ud + ui

        if abs(utot) > self.highLimit:
            utot = self.highLimit
            self.accumulator -= tempaccumulator
        if abs(utot) < self.lowLimit:
            utot = self.lowLimit
            self.accumulator -= tempaccumulator

        self.preverror = self.error

        return utot

    def resetIntegrator(self):
        """Function to reset the integration state to zero, used when switching modes or otherwise resetting the integral
        state.
        """
        self.accumulator = 0
        self.error = 0
        self.preverror = 0

        return

    def setPIDGains(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """Function to set the gains for the PID control block (including the trim output and the limits)
        """
        self.dT = dT
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.accumulator = 0
        self.error = 0
        return

class VehicleClosedLoopControl():
    def __init__(self, dT=0.01, rudderControlSource='SIDESLIP'):
        """Class that implements the entire closed loop control using the successive loop closure method of Beard
        Chapter 6. The class contains everything from control loops (PD or PI loops) along with the gains, which when
        given the reference commands will compute the control surfaces, which will then (along with the state) dictate
        the forces and moments, which will them compute the next state. Contains the required state for the altitude hold
        state machine from the enumeration class in Controls.AltitudeStates
        """
        self.dT = dT
        self.rudderControlSource = rudderControlSource
        self.VAmodel = VAM.VehicleAerodynamicsModel
        self.controlGains = Controls.controlGains()
        self.trimInputs = Inputs.controlInputs
        self.trimOutputs = Inputs.controlInputs
        self.mode = Controls.AlitudeStates.HOLDING
        self.rollFromCourse = PIControl()
        self.rudderFromSideslip = PIControl()
        self.throttleFromAirspeed = PIControl()
        self.pitchFromAltitude = PIControl()
        self.pitchFromAirspeed = PIControl()
        self.elevatorFromPitch = PDControl()
        self.aileronFromRoll = PIDControl()


    def Update(self, referenceCommands=Controls.referenceCommands):
        """Function that wraps the UpdateControlCommands and feeds it the correct state (estimated or otherwise).
        Updates the vehicle state internally using the vehicleAerodynamics.Update command.
        """

    def UpdateControlCommands(self, referenceCommands, state):
        """Function that implements the full closed loop controls using the commanded inputs of airspeed, altitude, and
        course (chi). Calls all of the submodules below it to implement the full flight dynamics under closed loop control.
        Note that the internal commandedPitch and commandedRoll of the reference commands are altered by this function,
        and this is used to track the reference commands by the simulator.
        """

    def getControlGains(self):
        """Wrapper function to extract control gains from the class.
        """
        return self.controlGains

    def getTrimInputs(self):
        """Wrapper function to get the trim inputs from the class.
        """
        return self.trimInputs

    def getVehicleAerodynamicsModel(self):
        """Wrapper function to extract the internal VehicleAerodynamicsModel in order to access the various function that
        are associated with the Aero model (such as setting and getting the wind state and model)
        """
        return self.VAmodel

    def getVehicleControlSurfaces(self):
        """Wrapper function to extract control outputs (Throttle, Aileron, Elevator, Rudder) from the class.
        """
        return self.trimOutputs

    def getVehicleState(self):
        """Wrapper function to extract vehicle state from the class.
        """
        return self.VAmodel.vehicleDynamicsModel.state

    def reset(self):
        """Resets the module to run again. Does not overwrite control gains, but does reset the integral states of all
        of the PI control loops.
        """

    def setControlGains(self, controlGains=Controls.controlGains()):
        """Function to set all of the gains from the controlGains previously computed to the correct places within the
        various control loops to do the successive loop closure (see Beard Chapter 6). Control loop limits are taken
        from the VehiclePhysicalConstants file, trim inputs are taken from self.trimInputs.
        """
        self.rollFromCourse.setPIGains(dT=self.dT, kp=controlGains.kp_course, ki=controlGains.ki_course, trim = , lowLimit= , highLimit=)
        self.rudderFromSideslip = controlGains.
        self.throttleFromAirspeed = controlGains.
        self.pitchFromAltitude = controlGains.
        self.pitchFromAirspeed = controlGains.
        self.elevatorFromPitch = controlGains.
        self.aileronFromRoll = controlGains.
        PDControl.setPDGains()
        PIControl.setPIGains()
        PIDControl.setPIDGains()




    def setTrimInputs(self, trimInputs=controlInputs(Throttle=0.5, Aileron=0.0, Elevator=0.0, Rudder=0.0)):
        """Wrapper function to inject the trim inputs into the class.
        """
        self.trimInputs = trimInputs
        return self.trimInputs

    def setVehicleState(self, state):
        """Wrapper function to inject vehicle state into the class.
        """
        self.VAmodel.vehicleDynamicsModel.state = state
        return self.VAmodel.vehicleDynamicsModel.state