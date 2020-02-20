/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.VisionSystem;

public class IntakeAiming extends CommandBase {
  DriveTrainMain drive;
  VisionSystem vision;
  Joystick driver;
  /**
   * Creates a new IntakeAiming.
   */
  public IntakeAiming(DriveTrainMain drive, VisionSystem vision, Joystick driver) {
    this.drive = drive;
    this.vision = vision;
    this.driver = driver;
    // Use addRequirements() here to declare subsystem dependencies.
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
    drive.visionSteer(drive.scaleInputs(-driver.getRawAxis(1)), vision.getRawValues().getAngleX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return driver.getRawAxis(4)!=0 || vision.getRawValues().getAngleX()==0;
  }
}
