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
 public static double kP,kI,kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

 public static final double earthGravity = 9.81; //m/s^2
 public static final double ballRadius = 0.09; //m
 public static final double innerPortRadius = 0.16; //m
 public static final double innerPortHeight = 2.500; //meters, measured from floor to center

static{
 
    kP =0;
    kI= 0;
    kD = 0;
    kIz = 0;
    kFF= 1/5800;
    kMaxOutput=1;
    kMinOutput=-1;
    maxRPM=5800;
}
}
