// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.*;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.controller.*;

import edu.wpi.first.wpilibj.Filesystem;

import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
//import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final DriveTrain m_drive = new DriveTrain();

  //private Command m_autoCommand;

  private static Joystick leftJoy = new Joystick(1);
  private static Joystick rightJoy = new Joystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  //joystick and subsystem getters
  public static Joystick returnLeftJoy() {
    return leftJoy;
  }

  public static Joystick returnRightJoy() {
    return rightJoy;
  }

  public static DriveTrain returnDrive() {
    return m_drive;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(RobotMap.kS, RobotMap.kV, RobotMap.kA),
          m_drive.getKinematics(),
          10);
      TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));
      // max velocity and max acceleration, may not be necessary with pathweaver
      config.setKinematics(m_drive.getKinematics());
      config.addConstraint(autoVoltageConstraint);
      
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
      );
      /* 
      use https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/pathweaver/integrating-robot-program.html
      for instructions on how to import paths created from pathweaver
      make sure to create pathweaver project in directory of the project you are working on
      */
      
      //untested pathweaver code
      /*
      String trajectoryJSON = "paths/secondpath.wpilib.json";
      Trajectory trajectory = new Trajectory();
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      }
      */
      RamseteCommand command = new RamseteCommand(
        trajectory,
        m_drive::returnPose,
        new RamseteController(2, 0.7), // paramteters recommended by documentation/video
        m_drive.getFeedForward(),
        m_drive.getKinematics(),
        m_drive::getWheelSpeeds,
        m_drive.returnRightPID(),
        m_drive.returnLeftPID(),
        m_drive::setOutput,
        m_drive
      );

      m_drive.resetOdometry(trajectory.getInitialPose());

      return command.andThen(() -> m_drive.setOutput(0, 0));
  }
}