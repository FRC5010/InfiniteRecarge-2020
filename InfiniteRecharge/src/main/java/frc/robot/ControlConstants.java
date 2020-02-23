/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class ControlConstants {
    static enum ButtonNums {
        NO_BUTTON, A_BUTTON, B_BUTTON, X_BUTTON, Y_BUTTON, LEFT_BUMPER, RIGHT_BUMPER, START_BUTTON, BACK_BUTTON;
    }
    static enum AxisNums {
        LEFT_X, LEFT_Y, L_TRIGGER, R_TRIGGER, RIGHT_X, RIGHT_Y
    }
    static enum POVDirs {
        UP, RIGHT, DOWN, LEFT 
    }
    // Driver
    public static int throttle = AxisNums.LEFT_Y.ordinal();
    public static int steer = AxisNums.RIGHT_X.ordinal();
    public static int winch1Axis = AxisNums.L_TRIGGER.ordinal();
    public static int winch21Axis = AxisNums.R_TRIGGER.ordinal();

    public static int intakeAimButton = ButtonNums.A_BUTTON.ordinal();
    public static int shooterAimButton = ButtonNums.B_BUTTON.ordinal();
    public static int rotationControl = ButtonNums.X_BUTTON.ordinal();
    public static int positionControl = ButtonNums.Y_BUTTON.ordinal();
    public static int spinDeploy = ButtonNums.RIGHT_BUMPER.ordinal();
    public static int heightToggle = ButtonNums.LEFT_BUMPER.ordinal();

    //Operator
    public static int arm1Axis = AxisNums.LEFT_Y.ordinal();
    public static int arm2Axis = AxisNums.RIGHT_Y.ordinal();
    public static int outtakeAxis = AxisNums.L_TRIGGER.ordinal() ;
    public static int intakeAxis = AxisNums.R_TRIGGER.ordinal();
    public static int launchButton = ButtonNums.A_BUTTON.ordinal();
    public static int toggleIntakeButton = ButtonNums.RIGHT_BUMPER.ordinal();
    public static int barrelUp = POVDirs.UP.ordinal() * 90;
    public static int barrelDown = POVDirs.DOWN.ordinal() * 90;
    public static int incShooter = POVDirs.RIGHT.ordinal() * 90;
    public static int decShooter = POVDirs.LEFT.ordinal() * 90;
}
