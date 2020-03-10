/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mechanisms.DriveConstants;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.Pose;
import frc.robot.subsystems.VisionSystem;

public class TurnToAngleVision extends CommandBase {
  DriveTrainMain driveTrain;
  Pose pose;
  VisionSystem vision;

  double angleToReach;
  double error;
  double lastError;
  double p;
  double d;
  double turnPower;
  long lastTime;
  long currentTime;
  /**
   * Creates a new TurntoAngle.
   */
  public TurnToAngleVision(DriveTrainMain driveTrain, Pose pose, VisionSystem vision) { //Left is negative, right is positive
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.pose = pose;
    this.vision = vision;
    this.p = DriveConstants.kTurnP;
    this.d = DriveConstants.kTurnD;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double targetAngle = vision.getAngleX();
    double currentHeading = pose.getHeading();
    this.angleToReach = targetAngle + currentHeading;
    error = currentHeading - targetAngle;
    currentTime = RobotController.getFPGATime();

    SmartDashboard.putNumber("Angle to reach", angleToReach);
    SmartDashboard.putNumber("Current Heading", currentHeading);
    SmartDashboard.putNumber("Vision get x", targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.isValidTarget()) {
      lastError = error;
      error = pose.getHeading() - angleToReach;
      SmartDashboard.putNumber("Turn error", error);
      lastTime = currentTime;
      currentTime = RobotController.getFPGATime();
      
      turnPower = p * error + d * (error - lastError) / (currentTime - lastTime) + DriveConstants.minTurn;
      SmartDashboard.putNumber("Turn power", turnPower);
      driveTrain.arcadeDrive(0, turnPower);
    }
    SmartDashboard.putBoolean("Valid Target", vision.isValidTarget());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !vision.isValidTarget() || Math.abs(error) < 2;
  }
}
