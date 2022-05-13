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
        self.stage = 1
        self.drive_controller.setSetpoint(1 * 12 / (self.wheel_diameter * math.pi) * 360)


    def autonomousPeriodic(self):
        if self.stage == 1:
            if self.drive_controller.atSetpoint():
                self.stage = 2
                self.turn_controller.setSetpoint(90)
            else:
                motion = self.drive_controller.calculate(measurement= self.drivetrain.m_right_encoder.getPosition())
                self.drivetrain.set(motion, motion)

        elif self.stage == 2:
            if self.turn_controller.atSetpoint():
                self.stage = 3
                self.drivetrain.m_right_encoder.setPosition(0)
                self.drivetrain.m_left_encoder.setPosition(0)
                self.drive_controller.setSetpoint(1 * 12 / (self.wheel_diameter * math.pi) * 360)
            else:
                motion = self.turn_controller.calculate(measurement= self.drivetrain.gyro.getYaw())
                self.drivetrain.set(motion, - motion)


        elif self.stage == 3:
            if self.drive_controller.atSetpoint():
                self.stage = 4
            else:
                motion = self.turn_controller.calculate(measurement=self.drivetrain.m_right_encoder.getPosition())
                self.drivetrain.set(motion, motion)

        else:
            self.drivetrain.set(0, 0)


if __name__ == "__main__":
    run(Robot)
