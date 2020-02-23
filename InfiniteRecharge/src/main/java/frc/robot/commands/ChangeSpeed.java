/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ShooterMain;

public class ChangeSpeed extends CommandBase {
  /**
   * Creates a new IncreaseSpeed.
   */
  private ShooterMain subsystem;
  public double setPoint;
  public boolean up;
  public ChangeSpeed(ShooterMain subsystem,boolean up) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    this.setPoint = subsystem.baseSpeed;
    this.up = up;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setPoint = (up)?subsystem.baseSpeed++:subsystem.baseSpeed--;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
