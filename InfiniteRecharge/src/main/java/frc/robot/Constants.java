/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double ksVolts = 0.162;
    public static final double kvVoltSecondsPerMeter = 1.53;
    public static final double kaVoltSecondsSquaredPerMeter = 0.329;
    public static final double kPDriveVel = 2.65;

    public static final double kTrackwidthMeters = 0.616;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double wheelDiameter = 0.5 * 0.3048; // 12 * 0.0254 - feet to meters
    public static final double motorRotationsPerWheelRotation = 16.37; // i.e. gear ratio
    public static final double pulsesPerMotorRotation = 1.0; // Encoder PPR
    public static final double pulsesPerWheelRotation = pulsesPerMotorRotation * motorRotationsPerWheelRotation;

    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public static final double distancePerPulse = wheelCircumference / pulsesPerWheelRotation;
    public static final double rpmToMetersPerSec = distancePerPulse / 60; // diameter divided by secs-per-minute

    public static final boolean gyroReversed = true;
    public static final boolean leftReversed = false;
    public static final boolean rightReversed = true;
    public static final double leftFudgeFactor = 1.023;
    public static final double rightFudgeFactor = 1.023;

    public static final double leftDistanceConv = distancePerPulse * (leftReversed ? -1.0 : 1.0) * leftFudgeFactor;
    public static final double rightDistanceConv = distancePerPulse * (rightReversed ? -1.0 : 1.0) * rightFudgeFactor;
    public static final double leftVelocityConv = rpmToMetersPerSec * (leftReversed ? -1.0 : 1.0) * leftFudgeFactor;
    public static final double rightVelocityConv = rpmToMetersPerSec * (rightReversed ? -1.0 : 1.0) * rightFudgeFactor;

}
