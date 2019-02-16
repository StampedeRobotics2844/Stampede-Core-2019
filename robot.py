'''
Steampede 2019
Betty H. Fairfax
Team 2844 @2019
bfhsroboticsclub@gmail.com
'''
import logging
import wpilib
import wpilib.drive
import portmap

from robotpy_ext.autonomous import StatefulAutonomous, timed_state
from networktables import NetworkTables
from robotpy_ext.autonomous import AutonomousModeSelector
from wpilib.builtinaccelerometer import Accelerometer

# logging.basicConfig(level=logging.DEBUG)

class StampedeRobot(wpilib.TimedRobot):
    '''Main robot class'''
    def __init__(self):
        super().__init__()

        self.smart_dashboard = None
        self.motor_speed_stop = 0

        self.kDistancePerRevolution = 18.84
        self.kPulsesPerRevolution = 1440
        self.kDistancePerPulse = self.kDistancePerRevolution / self.kPulsesPerRevolution

        self.drive_r_motor = None
        self.drive_l_motor = None

        self.claw_l_motor = None
        self.claw_r_motor = None

        self.lift_motor = None

        self.left_stick = None
        self.right_stick = None
        
        self.encoder_wheel_left = None
        self.encoder_wheel_right = None
        self.encoder_lift = None

        self.drive = None

        self.address= 0x53

        self.range = None
        self.gyro = None
        self.accelerometer = None


    '''
    First time initialization of robot, this happens after the robo rio has booted up, 
    and the code is run for the first time.
    '''
    def robotInit(self):
        '''Robot-wide Initialization code'''
  
        #Initializing networktables
        self.smart_dashboard = NetworkTables.getTable("SmartDashboard")
        self.smart_dashboard.putNumber('robot_speed', 1)

        # initialize and launch the camera
        wpilib.CameraServer.launch()

        # initialize the left and right encoders.
        self.encoder_wheel_left = wpilib.Encoder(0,1,True,wpilib.Encoder.EncodingType.k4X)
        self.encoder_wheel_left.reset()

        self.encoder_wheel_right = wpilib.Encoder(2,3,False,wpilib.Encoder.EncodingType.k4X)
        self.encoder_wheel_right.reset()

        self.encoder_wheel_left.setDistancePerPulse(self.kDistancePerPulse)
        self.encoder_wheel_right.setDistancePerPulse(self.kDistancePerPulse)

        #Initalizing drive motors
        self.drive_l_motor = wpilib.Victor(portmap.motors.left_drive)
        self.drive_r_motor = wpilib.Victor(portmap.motors.right_drive)

        self.claw_l_motor = wpilib.Victor(portmap.motors.left_claw)
        self.claw_r_motor = wpilib.Victor(portmap.motors.right_claw)

        self.lift_motor = wpilib.Victor(portmap.motors.lift)

        # initialize drive (differential drive is tank drive)
        self.drive = wpilib.drive.DifferentialDrive(self.drive_l_motor, self.drive_r_motor)
        self.drive.setExpiration(0.1)

        # joysticks 1 & 2 on the driver station
        self.left_stick = wpilib.Joystick(portmap.joysticks.left_joystick)
        self.right_stick = wpilib.Joystick(portmap.joysticks.right_joystick)

        # initialize gyro
        self.gyro = wpilib.AnalogGyro(wpilib.AnalogInput(1))
        self.gyro.calibrate()

        # initialize the ultra sonic 
        self.range = wpilib.AnalogInput(0)

        # initialize Accelerometer
        self.accelerometer = wpilib.BuiltInAccelerometer(Accelerometer.Range.k4G)

        # initialize autonomous components
        self.components = {
            'drive': self.drive,
            'drive_r_motor': self.drive_r_motor,
            'drive_l_motor': self.drive_l_motor,
            'claw_r_motor': self.claw_r_motor,
            'claw_l_motor': self.claw_r_motor,
            'lift_motor': self.lift_motor,
            'encoder_wheel_right': self.encoder_wheel_right,
            'encoder_wheel_left': self.encoder_wheel_left,
            'gyro' : self.gyro,
            'accelerometer': self.accelerometer,
            'range' : self.range
        }

        self.automodes = AutonomousModeSelector('autonomous', self.components)

        self.logger.log(logging.INFO, "robot initialization complete.")

    def getDistance(self):
        '''
        [(Vcc/1024) = Vi]
        Vcc = Supplied Voltage
        Vi = Volts per 5 mm (Scaling)
        
        Example 1: Say you have an input voltage of +5.0V the formula would read:
        [(5.0V/1024) = 0.004883V per 5 mm = 4.883mV per 5 mm]
        
        Calculating the Range
        Once you know the voltage scaling it is easy to properly calculate the range.

        The range formula is:
        [5*(Vm/Vi) = Ri]
        Vm = Measured Voltage
        Ri = Range in mm
        Vi = Volts per 5 mm (Scaling)
        '''

        voltage_scaling = 5.0/1024
        measured_voltages = self.range.getVoltage()
        distance = 5 * (measured_voltages/voltage_scaling)
        mmToInchScaling = 0.03937007874

        self.logger.log(logging.INFO, "measured voltage: {0}, distance: {1} mm, distnace: {2} inches".format(measured_voltages, distance, distance*mmToInchScaling))

        return distance

    def ClawOut(self):
        self.claw_l_motor.set(-1)
        self.claw_r_motor.set(1)
        
    def ClawIn(self):
        self.claw_l_motor.set(0.5)
        self.claw_r_motor.set(-0.5)

    def ClawStop(self):
        self.claw_l_motor.set(0)
        self.claw_r_motor.set(0)

    def LiftClockwise(self):
        self.lift_motor.set(1)

    def LiftCClockwise(self):
        self.lift_motor.set(-1)
    
    def LiftStop(self):
        self.lift_motor.set(0)

    def autonomousInit(self):
        self.drive.setSafetyEnabled(True)
        #self.encoder_wheel_left.reset()
        #self.encoder_wheel_right.reset()
        pass

    def autonomousPeriodic(self):
        self.automodes.run() 
        pass

    def disabledInit(self):
        '''Called only at the beginning of disabled mode'''
        pass
    
    def disabledPeriodic(self):
        '''Called every 20ms in disabled mode'''
        pass
            
    def teleopInit(self):
        '''Called only at the beginning of teleoperated mode'''
        self.drive.setSafetyEnabled(True)

    def teleopPeriodic(self):
        '''Called every 20ms in teleoperated mode'''
        
        try:

            if self.left_stick.getRawButton(portmap.joysticks.button_claw):
                self.ClawOut()
            elif self.right_stick.getRawButton(portmap.joysticks.button_claw):
                self.ClawIn()
            else:
                self.ClawStop()
                
            if self.left_stick.getRawButton(portmap.joysticks.button_lift):
                self.LiftClockwise()
            elif self.right_stick.getRawButton(portmap.joysticks.button_lift):
                self.LiftCClockwise()
            else:
                self.LiftStop()

            self.drive.tankDrive(self.left_stick.getY() * 1, self.right_stick.getY() * 1, False)

        except:
            if not self.isFMSAttached():
                raise

        #self.logger.log(logging.INFO, "distance wheel left: {0}".format(self.encoder_wheel_left.getDistance()))
        #self.logger.log(logging.INFO, "distance wheel right: {0}".format(self.encoder_wheel_right.getDistance()))
        #self.logger.log(logging.INFO, "distance lift: {0}".format(self.encoder_lift.getDistance()))
        self.logger.log(logging.INFO, "gyro angle: {0}".format(self.gyro.getAngle()))
        self.logger.log(logging.INFO, "range: {0}".format(self.getDistance()))
        self.logger.log(logging.INFO, "accelerometer x:{0} y:{1} z: {2}".format(self.accelerometer.getX(), self.accelerometer.getY(), self.accelerometer.getZ()))

    def isFMSAttached(self):
        return wpilib.DriverStation.getInstance().isFMSAttached()

    def getGameSpecificData(self):
        return wpilib.DriverStation.getInstance().getGameSpecificMessage()

if __name__ == '__main__':
    wpilib.run(StampedeRobot)