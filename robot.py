from wpilib import run, TimedRobot, Joystick
import math
from wpimath.controller import PIDController
from drivetrain import Drivetrain


class Robot(TimedRobot):
    
    joy1 = Joystick(0)
    
    def __init__(self):
        super().__init__()
        self.drivetrain = Drivetrain()

    def robotInit(self):
        self.wheel_diameter = 4  # inches
        self.drive_controller = PIDController(.001, 0, 0)
       # self.turn_controller = PIDController(.001, 0, 0)

    def robotPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        pass

    def autonomousInit(self):
        self.drivetrain.gyro.setYaw(0)
        self.drivetrain.m_right_encoder.setPosition(0)
        self.drivetrain.m_left_encoder.setPosition(0)
        self.drive_controller.setSetpoint(((7 * 12) / (self.wheel_diameter * math.pi)) * 360)
        self.drive_controller.setTolerance(15)


    def autonomousPeriodic(self):
        motion = self.drive_controller.calculate(measurement=self.drivetrain.m_left_encoder.getPosition())
       # print(self.drivetrain.m_right_encoder.getPosition())
        motion = min(.4, max(-.4, motion))
        self.drivetrain.set(motion, motion)


if __name__ == "__main__":
    run(Robot)
