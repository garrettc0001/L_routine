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
        self.turn_controller = PIDController(.001, 0, 0)

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
        self.drivetrain.gyro.setYaw(0)
        self.drivetrain.m_right_encoder.setPosition(0)
        self.drivetrain.m_left_encoder.setPosition(0)
        self.turn_controller.enableContinuousInput(0, 360)

    def autonomousPeriodic(self):
        self.drive_controller.setSetpoint(1 * 12 / (math.pi * self.wheel_diameter) * 360)
        # if not self.drive_controller.atSetpoint():
        motion = self.drive_controller.calculate(measurement= self.drivetrain.m_right_encoder.getPosition())
        self.drivetrain.set(motion, motion)
        pass
        # self.turn_controller.setSetpoint(90)
        # if not self.turn_controller.atSetpoint():
        #     motion = self.turn_controller.calculate(measurement= self.drivetrain.gyro.getYaw())
        #     self.drivetrain.set(motion, - motion)
        #     pass
        # self.drive_controller.setSetpoint(1 * 6 / self.wheel_radius / math.pi * 360)
        # if not self.drive_controller.atSetpoint():
        #     motion = self.turn_controller.calculate(measurement=self.drivetrain.m_right_encoder.getPosition())
        #     self.drivetrain.set(motion, motion)
        #     pass


if __name__ == "__main__":
    run(Robot)
