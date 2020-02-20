/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mechanisms.DriveConstants;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.VisionSystem;

public class AimWithVision extends CommandBase {
  
  // AimWithVision just turns the bot till it sees the vision target;

  DriveTrainMain drive;
  VisionSystem vision;
  Joystick driver;
  
  double angleTolerance = 5;

  public AimWithVision(DriveTrainMain drive, VisionSystem vision, Joystick driver) {
    this.drive = drive;
    this.vision = vision;
    this.driver = driver;
    addRequirements(drive);
    addRequirements(vision);
  }

  public AimWithVision(DriveTrainMain drive, VisionSystem vision) {
    this.drive = drive;
    this.vision = vision;
    this.driver = null;
    addRequirements(drive);
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Our target value is that angle is 0, thus our error is the angle we are reading
    double correction = vision.getRawValues().getAngleX() * DriveConstants.kTurnP;
    drive.arcadeDrive(drive == null ? drive.scaleInputs(driver.getRawAxis(1)) : 0, correction + DriveConstants.minTurn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(vision.getRawValues().getAngleX()) < angleTolerance;
  }
}
