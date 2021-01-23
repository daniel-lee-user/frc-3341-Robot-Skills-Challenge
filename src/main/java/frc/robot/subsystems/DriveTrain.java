// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  private AHRS navX = new AHRS(SPI.Port.kMXP);
  private WPI_TalonSRX left = new WPI_TalonSRX(RobotMap.leftMotorPort);
  private WPI_TalonSRX right = new WPI_TalonSRX(RobotMap.rightMotorPort);
  private static DriveTrain drive;

  public DriveTrain() {
    navX.zeroYaw();
    right.setInverted(true);
  }

  public DriveTrain getInstance() {
    if (drive == null) {
      drive = new DriveTrain();
    }
    return drive;
  }
  public void tankDrive(double leftPow, double rightPow) {
    left.set(ControlMode.PercentOutput, leftPow);
    right.set(ControlMode.PercentOutput, rightPow);
  }
  public double getAngle() {
    return navX.getAngle();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tankDrive(RobotContainer.returnLeftJoy().getY(), RobotContainer.returnRightJoy().getY());
  }
}
