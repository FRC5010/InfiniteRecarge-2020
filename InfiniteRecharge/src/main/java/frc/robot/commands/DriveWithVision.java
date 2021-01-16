/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mechanisms.DriveConstants;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.ShaftSubsystem;
import frc.robot.subsystems.VisionSystem;

public class DriveWithVision extends CommandBase {
  
  // Drive with vision TURNS the robot till it sees it and DRIVES the robot to set distance

  DriveTrainMain drive;
  VisionSystem vision;
  ShaftSubsystem shaftSubsystem; 

  double targetAngle, targetDistance, driveSpeed;
  double angleError, distanceError;
  double angleTolerance = 2;
  double distanceTolerance = 5;
  boolean BB1, BB2;

  public DriveWithVision(DriveTrainMain drive, VisionSystem vision, ShaftSubsystem shaftSubsystem, double targetAngle, double targetDistance, double driveSpeed) {
    this.drive = drive;
    this.vision = vision;
    this.targetDistance = targetDistance;
    this.driveSpeed = driveSpeed;
    this.shaftSubsystem = shaftSubsystem;
    addRequirements(drive);
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    BB1 = false;
    BB2 = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angleError = vision.getRawValues().getAngleX() - targetAngle;
    distanceError = vision.getRawValues().getDistance() - targetDistance;
    // SmartDashboard.putNumber("intakeVisionAngleError", angleError);
    // SmartDashboard.putNumber("intakeVisionDistanceError", distanceError);

    double turnCorrection = angleError * DriveConstants.kTurnP;
    double driveCorrection = distanceError * 0.0254 * DriveConstants.kPDriveVel / 12;
    //drive.arcadeDrive(driveCorrection + Math.signum(driveCorrection) * driveSpeed,
    drive.arcadeDrive(driveSpeed, 
      turnCorrection + Math.signum(turnCorrection) *  DriveConstants.minTurn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!shaftSubsystem.getBB1()){
      BB1 = true;
    }
    if(BB1 && !shaftSubsystem.getBB2()){
      BB2 = true;
    }
    return BB2;
  }
}
