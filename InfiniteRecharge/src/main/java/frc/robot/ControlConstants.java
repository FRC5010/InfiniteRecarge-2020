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
    public static int winch2Axis = AxisNums.R_TRIGGER.ordinal();

    public static int intakeAimButton = ButtonNums.A_BUTTON.ordinal();
    public static int shooterAimButton = ButtonNums.B_BUTTON.ordinal();
    public static int rotationControl = ButtonNums.X_BUTTON.ordinal();
    public static int positionControl = ButtonNums.Y_BUTTON.ordinal();
    public static int heightModeToggle = ButtonNums.LEFT_BUMPER.ordinal();
    public static int spinDeploy = ButtonNums.RIGHT_BUMPER.ordinal();
    public static int calibrate = ButtonNums.START_BUTTON.ordinal(); 
    public static int manualRotation = ButtonNums.BACK_BUTTON.ordinal(); // Implement

    //Operator
    public static int flyWheelManual = AxisNums.LEFT_Y.ordinal(); // Implement
    public static int climbDeployAxis = AxisNums.RIGHT_Y.ordinal(); // Implement
    public static int outtakeAxis = AxisNums.L_TRIGGER.ordinal() ;
    public static int intakeAxis = AxisNums.R_TRIGGER.ordinal();

    public static int launchButton = ButtonNums.A_BUTTON.ordinal();
    public static int barrelDown = ButtonNums.B_BUTTON.ordinal(); 
    public static int lowGoalShoot = ButtonNums.X_BUTTON.ordinal(); // implement
    public static int barrelUp = ButtonNums.Y_BUTTON.ordinal(); 
    public static int toggleBarrelHeight = ButtonNums.LEFT_BUMPER.ordinal();
    public static int toggleIntakeButton = ButtonNums.RIGHT_BUMPER.ordinal();
    public static int startClimb = ButtonNums.START_BUTTON.ordinal(); // Implement
    public static int retractClimb = ButtonNums.BACK_BUTTON.ordinal(); // Implement
    
    public static int incShooter = POVDirs.UP.ordinal() * 90;
    public static int decShooter = POVDirs.DOWN.ordinal() * 90;

    // Shuffleboard constants
    public static String SBTabDriverDisplay = "Driver Display";
    public static int intakeVisionColumn = 0;
    public static int barrelColumn = 2;
    public static int shooterColumn = 3;
    public static int spinnerColumn = 4;
    public static int shooterVisionColumn = 6;
}
