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
    this.angleToReach = vision.getAngleX() + pose.getHeading();
    lastError = 0;
    currentTime = RobotController.getFPGATime();

    SmartDashboard.putNumber("Angle to reach", angleToReach);
    SmartDashboard.putNumber("Current Heading", pose.getHeading());
    SmartDashboard.putNumber("Vision get x", vision.getAngleX());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lastError = error;
    error = -angleToReach + pose.getHeading();
    SmartDashboard.putNumber("Turn error", error);
    lastTime = currentTime;
    currentTime = RobotController.getFPGATime();
    
    turnPower = p * error + d * (error - lastError) / (currentTime - lastTime) + DriveConstants.minTurn;
    driveTrain.arcadeDrive(0, turnPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < 02;
  }
}
