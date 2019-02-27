from robotpy_ext.autonomous import StatefulAutonomous, timed_state, state
import logging
import portmap
import wpilib
import wpilib.drive

logging.basicConfig(level=logging.DEBUG)


class CenterForward(StatefulAutonomous):
    MODE_NAME = 'Center Forward'

    def initialize(self):
        self.register_sd_var('drive_speed', 1)


    @timed_state(first=True, duration=0.1)
    def drive_turn_right(self):
        self.drive.tankDrive(-0.5, 0.5)
