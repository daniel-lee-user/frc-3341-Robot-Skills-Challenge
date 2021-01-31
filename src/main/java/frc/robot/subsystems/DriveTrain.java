// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  private AHRS navX = new AHRS(SPI.Port.kMXP);
  private WPI_TalonSRX left = new WPI_TalonSRX(RobotMap.leftMotorPort);
  private WPI_TalonSRX right = new WPI_TalonSRX(RobotMap.rightMotorPort);
  private static DriveTrain drive;
  private double circumference = 2 * Math.PI * Units.inchesToMeters(RobotMap.radius);
  
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(RobotMap.trackWidth));

  SimpleMotorFeedforward feedForward =  new SimpleMotorFeedforward(RobotMap.kS, RobotMap.kV, RobotMap.kA);

  Pose2d pose;

  PIDController leftPID = new PIDController(0.807, 0, 0); //replaced with tested values (0.807, green robot)
  PIDController rightPID = new PIDController(0.807, 0, 0);

  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading()); //add optional pose2D starting location after getHeading as parameter

  public DriveTrain() {
    navX.zeroYaw();
    //set up motors
    right.configFactoryDefault();
    left.configFactoryDefault();
    right.setInverted(true);
    right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
  }

  public DriveTrain getInstance() {
    if (drive == null) {
      drive = new DriveTrain();
    }
    return drive;
  }

  public void setOutput(double leftVolts, double rightVolts) {
    left.set(leftVolts/12);
    right.set(rightVolts/12); // negative for green chassis, change for other robots
  }

  public void tankDrive(double leftPow, double rightPow) {
    left.set(ControlMode.PercentOutput, leftPow);
    right.set(ControlMode.PercentOutput, rightPow);
  }
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-navX.getAngle());
  }
  public void resetOdometry(Pose2d pose) {
    left.setSelectedSensorPosition(0, 0, 10);
    right.setSelectedSensorPosition(0, 0, 10);  
    odometry.resetPosition(pose, navX.getRotation2d());
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      right.getSelectedSensorVelocity() * 10.0 * circumference / 4096,
      left.getSelectedSensorVelocity()* 10.0 * circumference / 4096);
  }
  public double getRightDistance() {
    return (
      right.getSelectedSensorPosition() * circumference / (4096 * RobotMap.gearRatio)
      );
  }
  public double getLeftDistance() {
    return (
      left.getSelectedSensorPosition() * circumference / (4096 * RobotMap.gearRatio)
      );
  }
  
  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }
  
  public SimpleMotorFeedforward getFeedForward() {
    return feedForward;
  }
  public PIDController returnLeftPID() {
    return leftPID;
  }
  public PIDController returnRightPID() {
    return rightPID;
  }

  public Pose2d returnPose() {
    return pose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(getHeading(), getRightDistance(), getLeftDistance());
    tankDrive(-RobotContainer.returnLeftJoy().getY(), -RobotContainer.returnRightJoy().getY()); //tankdrive
  }
}
