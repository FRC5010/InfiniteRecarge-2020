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
public class TelescopConstants {
    //Values left null must be decided by electric.
    public static int arm1Port = 11;
    public static int arm2Port = 12;
    public static int winch1Port = 6;
    public static int winch2Port = 7;

    public static double winchSpeed = 0.25;
    public static double armSpeed = 0.25;
    public static double raisedTimeLimit = 2.0;
    public static int raisedCurrentLimit = 5;
}
