from wpilib import run, TimedRobot, Joystick
import math
from wpimath.controller import PIDController
from drivetrain import Drivetrain


class Robot(wpilib.TimedRobot):
    
    joy1=Joystick(0)
    
    def __init__(self):
        super().__init__()
        self.drivetrain = Drivetrain()

    def robotInit(self):
        pass

    def robotPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        speed=-self.joy1.getRawAxis(1)*.4
        turn=self.joy1.getRawAxis(4)*.2
        #print(f"Speed: {speed} Turn: {turn}")
        self.drivetrain.set(speed+turn, speed-turn)

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
