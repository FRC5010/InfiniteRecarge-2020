/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ControlConstants;
import frc.robot.mechanisms.DriveConstants;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.VisionSystem;

public class AimWithVision extends CommandBase {
  
  // AimWithVision just turns the bot till it sees the vision target;

  DriveTrainMain drive;
  VisionSystem vision;
  Joystick driver;
  double targetAngle, driveSpeed;

  double angleTolerance = 2;
  double error;
  double lastError;
  double firstTime;
  double currentTime;
  double lastTime;
  double p;
  double d;

  public AimWithVision(DriveTrainMain drive, VisionSystem vision, Joystick driver, double targetAngle) {
    this.drive = drive;
    this.vision = vision;
    this.driver = driver;
    this.targetAngle = targetAngle;
    this.p = DriveConstants.kTurnP;
    this.d = DriveConstants.kTurnD;
    addRequirements(drive);
    addRequirements(vision);
  }

  public AimWithVision(DriveTrainMain drive, VisionSystem vision, double targetAngle, double driveSpeed) {
    this.drive = drive;
    this.vision = vision;
    this.driver = null;
    this.targetAngle = targetAngle;
    this.driveSpeed = driveSpeed;
    this.p = DriveConstants.kTurnP;
    this.d = DriveConstants.kTurnD;
    addRequirements(drive);
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(" aim with vison start");
    vision.setLight(true);
    firstTime = RobotController.getFPGATime();
    error = 360;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = RobotController.getFPGATime();
    if (vision.isValidTarget()) {
      error = vision.getAngleX() - targetAngle;
      double correction = error * p + (error - lastError) / (currentTime - lastTime) * d;
      drive.arcadeDrive(driver != null ? drive.scaleInputs(-driver.getRawAxis(ControlConstants.throttle)) : driveSpeed, correction + Math.signum(correction) * DriveConstants.minTurn);
      lastError = error;
      lastTime = currentTime;
      SmartDashboard.putNumber(vision.getName() + "VisionError", error);
      SmartDashboard.putNumber(vision.getName() + "VisionCorrection", correction);
      firstTime = currentTime;
    }
  }

  // Called once the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
    System.out.println("aim with visionf ended");
    drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driver == null ? ((!vision.isValidTarget() && (currentTime - firstTime > 5000)) || Math.abs(error) < angleTolerance) : false;
  }
}
