/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.mechanisms;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;

/**
 * Add your docs here.
 */
public class SpinConstants {
    public static final int spinnerMotorChannel = 0;

    public static final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    public static final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    //light off red color r = 0.6, g = 0.27, b = 0.05
    //light off with change above makes confidence lower for the other colors (about 89 on average)
    public static final Color kRedTarget = ColorMatch.makeColor(0.51, 0.35, 0.13);
    //above values works best for light on because red confidence lowers significantly
    public static final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  
}
