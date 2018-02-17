from robotpy_ext.autonomous import StatefulAutonomous, timed_state, state


class CenterForward(StatefulAutonomous):
    MODE_NAME = 'Center Forward'

    def initialize(self):
        self.register_sd_var('drive_speed', 1)
    
    def getRTurnEncoderPosition(self):
        return (self.encoder_wheel_left.getDistance() - self.encoder_wheel_right.getDistance()) / 2
    
    def getAverageEncoderPosition(self):
        return (self.encoder_wheel_left.getDistance() + self.encoder_wheel_right.getDistance()) / 2

    @state(first=True)
    def drive_turn_right(self):
        while self.getRTurnEncoderPosition() < 7.0:
            self.drive.tankDrive(0.5, -0.5)
        
        if self.getRTurnEncoderPosition() > 7.0:
            self.drive.tankDrive(0,0)
            self.encoder_wheel_left.reset()
            self.encoder_wheel_right.reset()
            self.next_state('drive_forward')
    
    @state()
    def drive_forward(self):
        while self.getAverageEncoderPosition() < 50.0:
            self.drive.tankDrive(-0.5, -0.5)
        
        self.drive.tankDrive(0,0)
