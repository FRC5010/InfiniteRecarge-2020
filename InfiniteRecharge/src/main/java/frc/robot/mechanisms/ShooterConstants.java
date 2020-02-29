/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.mechanisms;

/**
 * Add your docs here.
 */
public class ShooterConstants {
    public static double kSC, kVC, kAC, kS, kV, kA, kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxAccel;

    public static final double earthGravity = 9.81; // m/s^2
    public static final double ballRadius = 0.09; // m
    public static final double innerPortRadius = 0.16; // m
    public static final double innerPortHeight = 2.500; // meters, measured from floor to center
    public static double baseSpeed = 2510;
    public static double distanceToRPM = 5.5;

    static {
        kSC = .1;
        kVC = 0.131;
        kAC = 0.102;
        kS = kSC / 12;
        kV = kVC / 60 / 1 / (12 - kS);
        kA = kAC / 60 / 1 / (12 - kS);
        kP = 0.002;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 1.0 / 4600.0;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5800;
    }
}
