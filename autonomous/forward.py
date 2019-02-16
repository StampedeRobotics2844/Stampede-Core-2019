from robotpy_ext.autonomous import StatefulAutonomous, timed_state, state
import portmap
import wpilib



class TC(StatefulAutonomous):
    MODE_NAME = 'forward'

    @timed_state(duration=1.2,first=True)
    def start(self):
        self.drive.tankDrive(1,1)
        


