import math
import random
from ..Modeling import VehicleAerodynamicsModel
from ..Utilities import MatrixMath as mm
from ..Containers import Sensors
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC
from ..Modeling import VehicleAerodynamicsModel

class GaussMarkov():
    def __init__(self, dT = VPC.dT, tau = 1000000.0, eta = 0):
        """Function to initialize the GaussMarkov code that generates the exponentially correlated noise which is used
        for the slowly varying bias drift of the gyros as well as the GPS position. Implements the noise model
        characterized by a first order Gauss-Markov process: dv/dt = -1/tau v + w, where w is a white noise process with
        N(0,sigma).
        """
        #timestep [s]
        self.dT = dT
        #correlation time [s]
        self.tau = tau
        #standard deviation of white noise. This represents sigma
        self.eta = eta
        #nu
        self.v = 0
        return

    def reset(self):
        """Wrapper function that resets the GaussMarkov model
        """
        #reseting timestep
        self.dT = VPC.dT
        #reseting correlatin time
        self.tau = 1000000.0
        #reseting standard deviation
        self.eta = 0
        #reseting nu
        self. v = 0
        return

    def update(self, vnoise=None):
        """Function that updates the Gauss-Markov process, and returns the updated value (as well as updating the
        internal value that holds until the next call of the function.
        """
        #if statement to determine the eta value
        if vnoise == None:
            eta = random.gauss(0, self.eta)
        else:
            eta = vnoise
        #solving for nu using correct eta value
        self.v = (math.exp(-self.dT / self.tau) * self.v) + eta
        return self.v

class GaussMarkovXYZ():
    def __init__(self, dT=VPC.dT, tauX=1e6, etaX=0, tauY=None, etaY=None, tauZ=None, etaZ=None):
        """Function to aggregate three Gauss-Markov models into a triplet that returns the X, Y, and Z axes of the
        time-varying drift; if (tau, eta) are None, then will default to the same values for each model.
        """
        #time step [s]
        self.dT = dT
        #initializing correlation times [s]
        self.tauX = tauX
        self.tauY = tauY
        self.tauZ = tauZ
        #initializing eta values
        self.etaX = etaX
        self.etaY = etaY
        self.etaZ = etaZ
        # if statement to address uniquness
        if self.tauY == None:
            self.tauY = self.tauX
        if self.tauZ == None:
            self.tauZ = self.tauY
        if self.etaY == None:
            self.etaY = self.etaX
        if self.etaZ == None:
            self.etaZ = self.etaY
        #instances of Gauss Markov
        self.x_axis = GaussMarkov(dT=self.dT, tau=self.tauX, eta=self.etaX)
        self.y_axis = GaussMarkov(dT=self.dT, tau=self.tauY, eta=self.etaY)
        self.z_axis = GaussMarkov(dT=self.dT, tau=self.tauZ, eta=self.etaZ)
        return

    def reset(self):
        """Wrapper function that resets the GaussMarkovXYZ models
        """
        # time step [s]
        self.dT = VPC.dT
        # resetting correlation times [s]
        self.tauX = 1e6
        self.tauY = None
        self.tauZ = None
        # resetting eta values
        self.etaX = 0
        self.etaY = None
        self.etaZ = None
        # if statement to address uniquness
        if self.tauY == None:
            self.tauY = self.tauX
        if self.tauZ == None:
            self.tauZ = self.tauY
        if self.etaY == None:
            self.etaY = self.etaX
        if self.etaZ == None:
            self.etaZ = self.etaY
        # resetting of Gauss Markov
        self.x_axis = GaussMarkov(self.dT, self.tauX, self.etaX)
        self.y_axis = GaussMarkov(self.dT, self.tauY, self.etaY)
        self.z_axis = GaussMarkov(self.dT, self.tauZ, self.etaZ)

    def update(self, vXnoise=None, vYnoise=None, vZnoise=None):
        """Function that updates the Gauss-Markov processes, and returns the updated values (as well as updating the
        internal values that holds until the next call of the function.
        """
        #calling update function for each axis as I previously call class in init
        vX = self.x_axis.update(vnoise=vXnoise)
        vY = self.y_axis.update(vnoise=vYnoise)
        vZ = self.z_axis.update(vnoise=vZnoise)
        return vX, vY, vZ

class SensorsModel():
    def __init__(self, aeroModel = VehicleAerodynamicsModel.VehicleAerodynamicsModel(), taugyro = VSC.gyro_tau,
                 etagyro = VSC.gyro_eta, tauGPS = VSC.GPS_tau, etaGPSHorizontal = VSC.GPS_etaHorizontal,
                 etaGPSVertical = VSC.GPS_etaVertical, gpsUpdateHz = VSC.GPS_rate):
        """Function to initialize the SensorsModel code. Will contain both the true sensors outputs and the noisy sensor
        outputs using the noise and biases found in the Constants.VehicleSensorConstants file. Biases for the sensors
        are set at instantiation and not updated further during the code run. All sensor outputs are in Engineering
        Units (it is assumed that the raw ADC counts to engineering unit scaling has already been done). Note that if
        the biases are set to None at instantiation, then they will be set to random values using uniform distribution
        with the appropriate bias scaling from the sensors constants. SensorsModel class keeps the Gauss-Markov models
        for gyro biases and GPS.
        """
        #instance of aerodynamicsmodel
        self.aeroModel = aeroModel
        #Gauss-Markov time contstant for gyros [s]
        self.taugyro = taugyro
        #Gauss-Markov process noise standard deviation [rad/s]
        self.etagyro = etagyro
        #initializing Gauss Markov XYZ for gyro
        self.xyzgyro = GaussMarkovXYZ(dT=VPC.dT, tauX=self.taugyro, etaX=self.etagyro, tauY=None, etaY=None, tauZ=None,
                                      etaZ=None)
        #Gauss-Markov time constant for GPS [s]
        self.tauGPS = tauGPS
        #Gauss-Markov process noise standard deviation [m]
        self.etaGPSHorizontal = etaGPSHorizontal
        #Gauss-Markov process noise standard deviation [m]
        self.etaGPSVertical = etaGPSVertical
        # initializing Gauss Markov XYZ for GPS
        self.gmgps = GaussMarkov(dT = VPC.dT, tau = self.tauGPS, eta = self.etaGPSHorizontal)
        self.gmgps2 = GaussMarkov(dT = VPC.dT, tau = self.tauGPS, eta = self.etaGPSVertical)
        #Update rate for GPS measurements [Hz]
        self.gpsUpdateHz = gpsUpdateHz
        #update threshold
        self.threshold = (1/self.gpsUpdateHz) * (1/VPC.dT)
        #update tick counter
        self.counter = 0
        #for true measurements
        self.sensorsTrue = Sensors.vehicleSensors()
        #static biases associated with each measurement
        self.sensorsBiases = Sensors.vehicleSensors()
        #sigmas of each measure-ment’s gaussian noise
        self.sensorSigmas = Sensors.vehicleSensors()
        #noisy measurements
        self.sensorsNoisy = Sensors.vehicleSensors()
        #instances of initializing Biases and Sigmas
        self.initializeBiases(gyroBias = VSC.gyro_bias , accelBias = VSC.accel_bias, magBias = VSC.mag_bias,
                         baroBias = VSC.baro_bias, pitotBias = VSC.pitot_bias)
        self.initializeSigmas(gyroSigma = VSC.gyro_sigma, accelSigma = VSC.accel_sigma, magSigma = VSC.mag_sigma,
                         baroSigma = VSC.baro_sigma, pitotSigma = VSC.pitot_sigma,
                         gpsSigmaHorizontal = VSC.GPS_sigmaHorizontal, gpsSigmaVertical = VSC.GPS_sigmaVertical,
                         gpsSigmaSOG = VSC.GPS_sigmaSOG, gpsSigmaCOG = VSC.GPS_sigmaCOG)
        #time step
        self.dT = VPC.dT

        return

    def initializeBiases(self, gyroBias = VSC.gyro_bias , accelBias = VSC.accel_bias, magBias = VSC.mag_bias,
                         baroBias = VSC.baro_bias, pitotBias = VSC.pitot_bias):
        """Function to generate the biases for each of the sensors. Biases are set with a uniform random number from
        -1 to 1 that is then multiplied by the sigma_bias. The biases for all sensors is returned as a
        Sensors.vehicleSensors class. Note that GPS is an unbiased sensor (though noisy), thus all the GPS biases are
        set to 0.0
        """
        # gyros
        self.sensorsBiases.gyro_x = random.uniform(-gyroBias, gyroBias)
        self.sensorsBiases.gyro_y = random.uniform(-gyroBias, gyroBias)
        self.sensorsBiases.gyro_z = random.uniform(-gyroBias, gyroBias)
        # accelerometers
        self.sensorsBiases.accel_x = random.uniform(-accelBias, accelBias)
        self.sensorsBiases.accel_y = random.uniform(-accelBias, accelBias)
        self.sensorsBiases.accel_z = random.uniform(-accelBias, accelBias)
        # magnetometers
        self.sensorsBiases.mag_x = random.uniform(-magBias, magBias)
        self.sensorsBiases.mag_y = random.uniform(-magBias, magBias)
        self.sensorsBiases.mag_z = random.uniform(-magBias, magBias)
        # pressure sensors
        self.sensorsBiases.baro = random.uniform(-baroBias, baroBias)
        self.sensorsBiases.pitot = random.uniform(-pitotBias, pitotBias)
        # gps
        self.sensorsBiases.gps_n = 0
        self.sensorsBiases.gps_e = 0
        self.sensorsBiases.gps_alt = 0
        self.sensorsBiases.gps_sog = 0
        self.sensorsBiases.gps_cog = 0

        return self.sensorsBiases


    def initializeSigmas(self, gyroSigma = VSC.gyro_sigma, accelSigma = VSC.accel_sigma, magSigma = VSC.mag_sigma,
                         baroSigma = VSC.baro_sigma, pitotSigma = VSC.pitot_sigma,
                         gpsSigmaHorizontal = VSC.GPS_sigmaHorizontal, gpsSigmaVertical = VSC.GPS_sigmaVertical,
                         gpsSigmaSOG = VSC.GPS_sigmaSOG, gpsSigmaCOG = VSC.GPS_sigmaCOG):
        """Function to gather all of the white noise standard deviations into a single vehicleSensor class object.
        These will be used as the input to generating the white noise added to each sensor when generating the noisy
        sensor data.
        """
        #gyros
        self.sensorSigmas.gyro_x = gyroSigma
        self.sensorSigmas.gyro_y = gyroSigma
        self.sensorSigmas.gyro_z = gyroSigma
        # accelerometers
        self.sensorSigmas.accel_x = accelSigma
        self.sensorSigmas.accel_y = accelSigma
        self.sensorSigmas.accel_z = accelSigma
        # magnetometers
        self.sensorSigmas.mag_x = magSigma
        self.sensorSigmas.mag_y = magSigma
        self.sensorSigmas.mag_z = magSigma
        # pressure sensors
        self.sensorSigmas.baro = baroSigma
        self.sensorSigmas.pitot = pitotSigma
        # gps
        self.sensorSigmas.gps_n = gpsSigmaHorizontal
        self.sensorSigmas.gps_e = gpsSigmaHorizontal
        self.sensorSigmas.gps_alt = gpsSigmaVertical
        self.sensorSigmas.gps_sog = gpsSigmaSOG
        self.sensorSigmas.gps_cog = gpsSigmaCOG

        return self.sensorSigmas

    def updateGPSTrue(self, state, dot):
        """ Function to update the GPS sensor state (this will be called to update the GPS data from the state and the
        derivative) at the required rate. Note that GPS reports back altitude as + above mean sea level.
        """
        # gps_n in [North - m]
        gps_n = state.pn
        # gps_e in [East - m]
        gps_e = state.pe
        # gps_alt in [Altitude - m]
        gps_alt = -state.pd
        # gps_SOG in [Speed over ground, m/s]
        gps_SOG = math.sqrt((dot.pn**2) + (dot.pe**2))
        # gps_COG in [Course over ground, rad]
        gps_COG = math.atan2(dot.pe, dot.pn)

        return gps_n, gps_e, gps_alt, gps_SOG, gps_COG

    def updateAccelsTrue(self, state, dot):
        """Function to update the accelerometer sensor. Will be called within the updateSensors functions.
        """
        # Update true accelerometer in the x direction
        accel_x = dot.u + (state.q * state.w) - (state.r * state.v) + \
                  (VPC.g0 * math.sin(state.pitch))
        # Update true accelerometer in the y direction
        accel_y = dot.v + (state.r * state.u) - (state.p * state.w) - \
                  (VPC.g0 * math.cos(state.pitch) * math.sin(state.roll))
        # Update true accelerometer in the z direction
        accel_z = dot.w + (state.p * state.v) - (state.q * state.u) - \
                  (VPC.g0 * math.cos(state.pitch) * math.cos(state.roll))
        return accel_x, accel_y, accel_z

    def updateMagsTrue(self, state):
        """Function to update the magnetometer sensor. Will be called within the updateSensors functions.
        """
        mag = mm.multiply(state.R, VSC.magfield)
        mag_x = mag[0][0]
        mag_y = mag[1][0]
        mag_z = mag[2][0]
        return mag_x, mag_y, mag_z

    def updateGyrosTrue(self, state):
        """Function to update the rate gyro sensor. Will be called within the updateSensors functions. :param state:
        class States.vehicleState, current vehicle state
        """
        #Update true gyro in the x direction
        gyro_x = state.p
        # Update true gyro in the y direction
        gyro_y = state.q
        # Update true gyro in the z direction
        gyro_z = state.r
        return gyro_x, gyro_y, gyro_z

    def updatePressureSensorsTrue(self, state):
        """Function to update the pressure sensors onboard the aircraft. Will be called within the updateSensors
        functions. The two pressure sensors are static pressure (barometer) and dynamic pressure (pitot tube). The
        barometric pressure is references off of the ground static pressure in VehicleSensorConstants at Pground.
        """
        #solving for baroemter
        baro = VSC.Pground - (-state.pd * VPC.g0 * VPC.rho)
        #solving for pitot
        pitot = .5 * VPC.rho * state.Va**2

        #in [N/m^2]
        return baro, pitot

    def updateSensorsTrue(self, prevTrueSensors, state, dot):
        """ Function to generate the true sensors given the current state and state derivative. Sensor suite is 3-axis
        accelerometer, 3-axis rate gyros, 3-axis magnetometers, a barometric altimeter, a pitot airspeed, and GPS with
        an update rate specified in the VehicleSensorConstants file. For the GPS update, the previous value is returned
        until a new update occurs. Previous value is contained within prevTrueSensors.
        """
        #gyros update
        self.sensorsTrue.gyro_x, self.sensorsTrue.gyro_y, self.sensorsTrue.gyro_z = self.updateGyrosTrue(state)
        #accel update
        self.sensorsTrue.accel_x, self.sensorsTrue.accel_y, self.sensorsTrue.accel_z = self.updateAccelsTrue(state, dot)
        #mag update
        self.sensorsTrue.mag_x, self.sensorsTrue.mag_y, self.sensorsTrue.mag_z = self.updateMagsTrue(state)
        #pressure update
        self.sensorsTrue.baro, self.sensorsTrue.pitot = self.updatePressureSensorsTrue(state)
        #gps update
        if self.counter % self.threshold == 0:
            self.sensorsTrue.gps_n, self.sensorsTrue.gps_e, self.sensorsTrue.gps_alt, self.sensorsTrue.gps_SOG, \
            self.sensorsTrue.gps_COG = self.updateGPSTrue(state, dot)
        else:
            self.sensorsTrue.gps_n = prevTrueSensors.gps_n
            self.sensorsTrue.gps_e = prevTrueSensors.gps_e
            self.sensorsTrue.gps_alt = prevTrueSensors.gps_alt
            self.sensorsTrue.gps_SOG = prevTrueSensors.gps_sog
            self.sensorsTrue.gps_COG = prevTrueSensors.gps_cog

        return self.sensorsTrue

    def updateSensorsNoisy(self, trueSensors = Sensors.vehicleSensors(), noisySensors = Sensors.vehicleSensors(),
                           sensorBiases = Sensors.vehicleSensors(), sensorSigmas = Sensors.vehicleSensors()):
        """ Function to generate the noisy sensor data given the true sensor readings, the biases, and the sigmas for
        the white noise on each sensor. The gauss markov models for the gyro biases and GPS positions are updated here.
        The GPS COG white noise must be scaled by the ratio of VPC.initialSpeed / actual ground speed. GPS is only
        updated if the correct number of ticks have gone by to indicate that a new GPS measurement should be generated.
        The GPS COG must be limited to within +/- PI. If no GPS update has occurred, then the values for the GPS sensors
        should be copied from the noisySensors input to the output.
        """
        #gyro noise
        vX, vY, vZ = self.xyzgyro.update(noisySensors.gyro_x, noisySensors.gyro_y, noisySensors.gyro_z)
        self.sensorsNoisy.gyro_x = trueSensors.gyro_x + sensorBiases.gyro_x + random.gauss(0, sensorSigmas.gyro_x) + vX
        self.sensorsNoisy.gyro_y = trueSensors.gyro_y + sensorBiases.gyro_y + random.gauss(0, sensorSigmas.gyro_y) + vY
        self.sensorsNoisy.gyro_z = trueSensors.gyro_z + sensorBiases.gyro_z + random.gauss(0, sensorSigmas.gyro_x) + vZ

        #accel noise
        self.sensorsNoisy.accel_x = trueSensors.accel_x + sensorBiases.accel_x + random.gauss(0, sensorSigmas.accel_x)
        self.sensorsNoisy.accel_y = trueSensors.accel_y + sensorBiases.accel_y + random.gauss(0, sensorSigmas.accel_y)
        self.sensorsNoisy.accel_z = trueSensors.accel_z + sensorBiases.accel_z + random.gauss(0, sensorSigmas.accel_z)

        #mag noise
        self.sensorsNoisy.mag_x =  trueSensors.mag_x + sensorBiases.mag_x + random.gauss(0, sensorSigmas.mag_x)
        self.sensorsNoisy.mag_y = trueSensors.mag_y + sensorBiases.mag_y + random.gauss(0, sensorSigmas.mag_y)
        self.sensorsNoisy.mag_z = trueSensors.mag_z + sensorBiases.mag_z + random.gauss(0, sensorSigmas.mag_z)

        #pressure noise
        self.sensorsNoisy.baro = trueSensors.baro + sensorBiases.baro + random.gauss(0, sensorSigmas.baro)
        self.sensorsNoisy.pitot = trueSensors.pitot + sensorBiases.pitot + random.gauss(0, sensorSigmas.pitot)

        #gps noise
        if self.counter % self.threshold == 0:
            self.sensorsNoisy.gps_n = trueSensors.gps_n + sensorBiases.gps_n + \
                                      ((VPC.InitialSpeed / trueSensors.gps_sog) *
                                       random.gauss(0, sensorSigmas.gps_n)) + \
                                      self.gmgps.update(noisySensors.gps_n)
            self.sensorsNoisy.gps_e = trueSensors.gps_e + sensorBiases.gps_e + \
                                      ((VPC.InitialSpeed / trueSensors.gps_sog) *
                                       random.gauss(0, sensorSigmas.gps_e)) + \
                                      self.gmgps.update(noisySensors.gps_e)
            self.sensorsNoisy.gps_alt = trueSensors.gps_alt + sensorBiases.gps_alt + \
                                        ((VPC.InitialSpeed / trueSensors.gps_sog) *
                                         random.gauss(0, sensorSigmas.gps_alt)) + \
                                        self.gmgps2.update(noisySensors.gps_alt)
            self.sensorsNoisy.gps_sog = trueSensors.gps_sog + sensorBiases.gps_sog + \
                                        ((VPC.InitialSpeed / trueSensors.gps_sog) *
                                         random.gauss(0, sensorSigmas.gps_sog)) + \
                                        self.gmgps.update(noisySensors.gps_sog)
            self.sensorsNoisy.gps_cog = trueSensors.gps_cog + sensorBiases.gps_cog + \
                                        ((VPC.InitialSpeed / trueSensors.gps_sog) *
                                         random.gauss(0, sensorSigmas.gps_cog)) + \
                                        self.gmgps.update(noisySensors.gps_cog)
        else:
            self.sensorsNoisy.gps_n = noisySensors.gps_n
            self.sensorsNoisy.gps_e = noisySensors.gps_e
            self.sensorsNoisy.gps_alt = noisySensors.gps_alt
            self.sensorsNoisy.gps_sog = noisySensors.gps_sog
            self.sensorsNoisy.gps_cog = noisySensors.gps_cog

        if self.sensorsNoisy.gps_cog >= math.pi:
            self.sensorsNoisy.gps_cog += (2 * math.pi)
        if self.sensorsNoisy.gps_cog <= -math.pi:
            self.sensorsNoisy.gps_cog -= (2 * math.pi)

        return self.sensorsNoisy

    def update(self):
        """Wrapper function to update the Sensors (both true and noisy) using the state and dot held within the
        self.AeroModel. Note that we are passing in a handle to the class for this so we don’t have to explicitly use
        getters from the VehicleAerodynamics model. Sensors states are updated internally within self.
        """
        self.counter+=1
        UST = self.updateSensorsTrue(self.sensorsTrue, self.aeroModel.vehicleDynamicsModel.state,
                               self.aeroModel.vehicleDynamicsModel.dot)
        self.updateSensorsNoisy(UST, self.sensorsNoisy, self.sensorsBiases, self.sensorSigmas)
        return

    def getSensorsTrue(self):
        """Wrapper function to return the true sensor values
        """
        return self.sensorsTrue

    def getSensorsNoisy(self):
        """Wrapper function to return the noisy sensor values
        """
        return self.sensorsNoisy

    def reset(self):
        """Function to reset the module to run again. Should reset the Gauss-Markov models, re-initialize the sensor
        biases, and reset the sensors true and noisy to pristine conditions
        """
        # Gauss-Markov time contstant for gyros [s]
        self.taugyro = VSC.gyro_tau
        # Gauss-Markov process noise standard deviation [rad/s]
        self.etagyro = VSC.gyro_eta
        # initializing Gauss Markov XYZ for gyro
        self.xyzgyro = GaussMarkovXYZ(dT=VPC.dT, tauX=self.taugyro, etaX=self.etagyro, tauY=None, etaY=None, tauZ=None,
                                      etaZ=None)
        # Gauss-Markov time constant for GPS [s]
        self.tauGPS = VSC.GPS_tau
        # Gauss-Markov process noise standard deviation [m]
        self.etaGPSHorizontal = VSC.GPS_etaHorizontal
        # Gauss-Markov process noise standard deviation [m]
        self.etaGPSVertical = VSC.GPS_etaVertical
        # initializing Gauss Markov XYZ for GPS
        self.xyzgps = GaussMarkovXYZ(dT=VPC.dT, tauX=self.tauGPS, etaX=self.etaGPSHorizontal, tauY=None,
                                     etaY=self.etaGPSHorizontal, tauZ=None, etaZ=self.etaGPSVertical)
        # Update rate for GPS measurements [Hz]
        self.gpsUpdateHz = VSC.GPS_rate
        # update threshold
        self.threshold = (1 / self.gpsUpdateHz) * (1 / VPC.dT)
        # for true measurements
        self.sensorsTrue = Sensors.vehicleSensors()
        # static biases associated with each measurement
        self.sensorsBiases = Sensors.vehicleSensors()
        # sigmas of each measure-ment’s gaussian noise
        self.sensorSigmas = Sensors.vehicleSensors()
        # noisy measurements
        self.sensorsNoisy = Sensors.vehicleSensors()
        # instances of initializing Biases and Sigmas
        self.initializeBiases(gyroBias=VSC.gyro_bias, accelBias=VSC.accel_bias, magBias=VSC.mag_bias,
                              baroBias=VSC.baro_bias, pitotBias=VSC.pitot_bias)
        self.initializeSigmas(gyroSigma=VSC.gyro_sigma, accelSigma=VSC.accel_sigma, magSigma=VSC.mag_sigma,
                              baroSigma=VSC.baro_sigma, pitotSigma=VSC.pitot_sigma,
                              gpsSigmaHorizontal=VSC.GPS_sigmaHorizontal, gpsSigmaVertical=VSC.GPS_sigmaVertical,
                              gpsSigmaSOG=VSC.GPS_sigmaSOG, gpsSigmaCOG=VSC.GPS_sigmaCOG)
        return
