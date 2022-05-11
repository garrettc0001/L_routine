import commands2
import ctre


class Drivetrain(commands2.SubsystemBase):
    def __init__(self):
        super().__init__()
        self.gyro = ctre.PigeonIMU(10)
        self.gyro.setYaw(0,0)
        
        self.m_left = ctre.TalonSRX(0)
        self.m_left_2 = ctre.VictorSPX(1)
        self.m_left_3 = ctre.VictorSPX(2)
        self.m_right = ctre.TalonSRX(3)
        self.m_right_2 = ctre.VictorSPX(4)
        self.m_right_3 = ctre.VictorSPX(5)
        
        self.m_left_2.follow(self.m_left)
        self.m_left_3.follow(self.m_left)
        
        self.m_right_2.follow(self.m_right)
        self.m_right_3.follow(self.m_right)
        
        self.m_right_encoder=ctre.CANCoder(9)
        self.m_left_encoder=ctre.CANCoder(11)

    def set(self, left: float, right: float):
        self.m_left.set(ctre.ControlMode.PercentOutput, left)
        self.m_right.set(ctre.ControlMode.PercentOutput, -right)

    def periodic(self):
        pass
