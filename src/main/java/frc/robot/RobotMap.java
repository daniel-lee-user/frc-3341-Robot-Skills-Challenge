// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class RobotMap {
    public static int leftMotorPort = 2;
    public static int rightMotorPort = 3;
    
    public static int navXPort = 0;
    public static double gearRatio = 1;
    public static double radius = 3.19; //inches, change later

    public static int leftJoy = 0;
        
    public static double kS = 0.762;
    public static double kV = 3.48; //change after running frc characterization data logger
    public static double kA = 0.49;

    public static double kP = 2.07;

    public static int rightJoy = 1;
    public static double trackWidth = 24; //inches, replace this with real track width (distance between the wheels) later
}
