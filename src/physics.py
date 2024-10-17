
import wpilib
from wpilib.drive import DifferentialDrive
from wpilib import XboxController
from wpilib import SmartDashboard
from wpilib.simulation import (PWMSim, AnalogGyroSim,)
# from wpilib.drive import MecanumDrive
import wpilib.simulation

from wpimath.kinematics import (MecanumDriveKinematics,
                                # MecanumDriveKinematicsBase,
                                # MecanumDriveOdometry,
                                # MecanumDriveOdometryBase,
                                # MecanumDriveWheelPositions,
                                MecanumDriveWheelSpeeds,
                                DifferentialDriveKinematics,
                                DifferentialDriveOdometry,  
                                DifferentialDriveWheelSpeeds,
                                )

from constants import DriveConstant, ControllerConstant
import phoenix5
from phoenix6.unmanaged import feed_enable


from wpimath.geometry import (Pose2d, Rotation2d, Translation2d)



class PhysicsEngine:
    def __init__(self, physics_controller, robot: "MyRobot"): # type: ignore
        self.physics_controller = physics_controller
        self.robot = robot
        self.robotDrive = robot.drive
        self.robotControllerSim = robot.controller
        self.robotLeftMotor1Sim = robot.left_motor1.getSimCollection()
        self.robotLeftMotor2Sim = robot.left_motor2.getSimCollection()
        self.robotRightMotor1Sim = robot.right_motor1.getSimCollection()
        self.robotRightMotor2Sim = robot.right_motor2.getSimCollection()

        
        self.robotControllerSim = XboxController(ControllerConstant.XBOX_driver)
        # self.robotDriveSim = DifferentialDrive(self.robotLeftMotor1Sim, self.robotRightMotor1Sim)   
        # self.robotDriveSim.setRightSideInverted(True)
        # self.robotDriveSim.setDeadband(0.1)
        # self.robotDriveSim.setMaxOutput(1.0)


    def update_sim(self, now, tm_diff):
        feed_enable(0.020 * 2)
        # self.robotDriveSim.arcadeDrive(self.robotControllerSim.getLeftY, self.robotControllerSim.getLeftX)#, squareInputs=True)

        frontLeftMotor_speed = self.robotLeftMotor1Sim.getMotorOutputLeadVoltage()/12
        frontRightMotor_speed = -self.robotRightMotor1Sim.getMotorOutputLeadVoltage()/12

        # Simulate the drivetrain
        wheel_speeds = DifferentialDriveWheelSpeeds(
                    frontLeftMotor_speed,
                    frontRightMotor_speed
                )
        # Create an odometry object
        drivetrain_kinematics = DifferentialDriveKinematics(DriveConstant.kTrackWidth)
        chassis_speeds = drivetrain_kinematics.toChassisSpeeds(wheel_speeds)
        # Update the physics controller with the new state
        self.physics_controller.drive(chassis_speeds, tm_diff)




        # self.robotDriveSim.arcadeDrive(self.robotControllerSim.getY(GenericHID.Hand.kLeft), self.robotControllerSim.getX(GenericHID.Hand.kLeft), squareInputs=True)

        # Simulate the motors
        # self.robotLeftMotor1Sim.setQuadratureRawPosition(self.robotLeftMotor1Sim.getQuadraturePosition() + int(self.robot.left_motor1.getSpeed() * tm_diff))
        # self.robotLeftMotor2Sim.setQuadratureRawPosition(self.robotLeftMotor2Sim.getQuadraturePosition() + int(self.robot.left_motor2.getSpeed() * tm_diff))
        # self.robotRightMotor1Sim.setQuadratureRawPosition(self.robotRightMotor1Sim.getQuadraturePosition() + int(self.robot.right_motor1.getSpeed() * tm_diff))
        # self.robotRightMotor2Sim.setQuadratureRawPosition(self.robotRightMotor2Sim.getQuadraturePosition() + int(self.robot.right_motor2.getSpeed() * tm_diff))

        # print(f"Left Motor 1: {self.robot.left_motor1.getSpeed()}")
        # print(f"Left Motor 2: {self.robot.left_motor2.getSpeed()}")
        # print(f"Right Motor 1: {self.robot.right_motor1.getSpeed()}")
        # print(f"Right Motor 2: {self.robot.right_motor2.getSpeed()}")
        # print(f"Left Motor 1 Sim: {self.robotLeftMotor1Sim.getQuadraturePosition()}")
        # print(f"Left Motor 2 Sim: {self.robotLeftMotor2Sim.getQuadraturePosition()}")
        # print(f"Right Motor 1 Sim: {self.robotRightMotor1Sim.getQuadraturePosition()}")
        # print(f"Right Motor 2 Sim: {self.robotRightMotor2Sim.getQuadraturePosition()}")
        # print(f"Left Motor 1 Sim: {self.robotLeftMotor1Sim.getQuadratureVelocity()}")
        # print(f"Left Motor 2 Sim: {self.robotLeftMotor2Sim.getQuadratureVelocity()}")
        # print(f"Right Motor 1 Sim: {self.robotRightMotor1Sim.getQuadr
