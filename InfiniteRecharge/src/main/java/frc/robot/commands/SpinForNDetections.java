/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.WheelColor;

public class SpinForNDetections extends CommandBase {
  private Spinner spinner;
  private WheelColor wheel;
  private int detections;
  /**
   * Creates a new SpinForNDetections.
   */
  public SpinForNDetections(Spinner spinner, WheelColor wheel, int detections) {
    this.spinner = spinner;
    this.wheel = wheel;
    this.detections = detections;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spinner);
    addRequirements(wheel);
  }

  public SpinForNDetections(Spinner spinner, WheelColor wheel) {
    this.spinner = spinner;
    this.wheel = wheel;
    this.detections = wheel.determineGameData();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spinner);
    addRequirements(wheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    spinner.spin();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spinner.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wheel.getTotalCount() >= detections;
  }
}
