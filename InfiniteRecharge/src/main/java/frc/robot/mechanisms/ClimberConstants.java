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
public class ClimberConstants {
    public static double kP,kI,kD, kMaxOutput, kMinOutput;

    public ClimberConstants(){
        kP =.1;
        kI= 0;
        kD = 0;
        
        kMaxOutput=40;
        kMinOutput=1;
    }
    
}
