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
        NO_BUTTON, A_BUTTON, B_BUTTON, X_BUTTON, Y_BUTTON, LEFT_BUMPER, RIGHT_BUMPER, START_BUTTON, BACK_BUTTON,LEFT_STICK_BUTT, RIGHT_STICK_BUTT;
    }
    static enum AxisNums {
    LEFT_X, LEFT_Y, 
     L_TRIGGER, 
     R_TRIGGER, 
    RIGHT_X, RIGHT_Y
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
    public static int toggleLed = ButtonNums.LEFT_STICK_BUTT.ordinal();
    public static int turnToAngleButton = POVDirs.DOWN.ordinal() * 90;

    //Operator
    public static int leftArmDeploy = AxisNums.LEFT_Y.ordinal(); // Implement
    public static int combinedArmDeploy = AxisNums.RIGHT_Y.ordinal(); // Implement
    public static int outtakeAxis = AxisNums.L_TRIGGER.ordinal() ;
    public static int intakeAxis = AxisNums.R_TRIGGER.ordinal();

    public static int launchButton = ButtonNums.A_BUTTON.ordinal();
    public static int barrelDown = ButtonNums.B_BUTTON.ordinal(); 
    public static int lowGoalShoot = ButtonNums.X_BUTTON.ordinal(); // implement
    public static int barrelUp = ButtonNums.Y_BUTTON.ordinal(); 
    public static int overrideArmDeployment = ButtonNums.LEFT_BUMPER.ordinal();
    public static int toggleIntakeButton = ButtonNums.RIGHT_BUMPER.ordinal();
    public static int startClimb = ButtonNums.START_BUTTON.ordinal(); // Implement
    public static int retractClimb = ButtonNums.BACK_BUTTON.ordinal(); // Implement
    
    public static int incShooter = POVDirs.UP.ordinal() * 90;
    public static int decShooter = POVDirs.DOWN.ordinal() * 90;
    public static int spinnerOverrideButtonLow = POVDirs.RIGHT.ordinal() * 90;
    public static int spinnerOverrideButtonHigh = POVDirs.LEFT.ordinal() * 90;

    // Shuffleboard constants
    public static String SBTabDriverDisplay = "Driver Display";
    public static String SBTabDiagnostics = "Diagnostics";
    public static int intakeVisionColumn = 0;
    public static int barrelColumn = 3;
    public static int shooterColumn = 4;
    public static int spinnerColumn = 5;
    public static int autoColumn = 3;
    public static int shooterVisionColumn = 7;
}
