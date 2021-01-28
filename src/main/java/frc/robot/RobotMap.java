// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class RobotMap {
    public static int leftMotorPort = 2;
    public static int rightMotorPort = 3;
    
    public static int navXPort = 0;
    public static double gearRatio = 7.29; //inches, change later
    public static int radius = 3; //inches, change later

    public static int leftJoy = 0;
    
    public static double kS = 1;
    public static double kV = 1; //change after running frc characterization data logger
    public static double kA = 0.5;

    public static int rightJoy = 1;
    public static int trackWidth = 1; //inches, replace this with real track width (distance between the wheels) later
}
