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
    public static int spinnerMotorChannel, fwdChannel, revChannel;
    public static Color kBlueTarget, kGreenTarget, kRedTarget, kYellowTarget;

    static {
        spinnerMotorChannel = 10;
        // Will get implemented when the channels are ready.
         fwdChannel = 4;
         

        // light off red color r = 0.6, g = 0.27, b = 0.05
        // light off with change above makes confidence lower for the other colors
        // (about 89 on average)
        // above values works best for light on because red confidence lowers
        // significantly
        kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
        kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
        kRedTarget = ColorMatch.makeColor(0.51, 0.35, 0.13);
        kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
    }

}
