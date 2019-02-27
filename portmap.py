class PortMap:
    pass

motors = PortMap()

motors.left_drive = 8 
motors.right_drive = 9 

motors.right_claw = 4
motors.left_claw = 3

motors.lift = 5

joysticks = PortMap()

joysticks.left_joystick = 1
joysticks.right_joystick = 0

joysticks.button_claw = 1
joysticks.button_liftp = 2