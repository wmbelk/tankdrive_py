# make a simple tankdrive robotpy program
# 4 motors, 2 on each side
#motors are connected via TalonSRX motor controllers
# 1 xbox controller

import wpilib
import wpilib.drive
from wpilib import (SmartDashboard, Field2d)
from commands2 import TimedCommandRobot
from commands2 import CommandScheduler
from constants import DriveConstant, ControllerConstant


#can move to subsytem
import phoenix5


class MyRobot(TimedCommandRobot):
    
    def robotInit(self):
        CommandScheduler.getInstance().run()
        
        self.left_motor1 = phoenix5.WPI_TalonSRX(DriveConstant.LEFT_MOTOR1)
        self.left_motor2 = phoenix5.WPI_TalonSRX(DriveConstant.LEFT_MOTOR2)
        self.right_motor1 = phoenix5.WPI_TalonSRX(DriveConstant.RIGHT_MOTOR1)
        self.right_motor2 = phoenix5.WPI_TalonSRX(DriveConstant.RIGHT_MOTOR2)

        self.left_motor2.follow(self.left_motor1)
        self.right_motor2.follow(self.right_motor1)

        self.drive =  wpilib.drive.DifferentialDrive(self.left_motor1, self.right_motor1)

        self.controller = wpilib.XboxController(ControllerConstant.XBOX_driver)

        #to have field in simulation
        SmartDashboard.putData(CommandScheduler.getInstance())
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field) #end up viewing in Glass

    def teleopPeriodic(self):
        self.drive.arcadeDrive(-self.controller.getLeftY(), -self.controller.getLeftX()) #TODO: ask drivers if they want TankDrive or ArcadeDrive

