/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterMain;

public class ShooterOverride extends CommandBase {
  /**
   * Creates a new ShooterOverride.
   */
  ShooterMain shooter;
  double speed;

  public ShooterOverride(ShooterMain shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    speed = 1000;
    addRequirements(shooter);
  }

  public ShooterOverride(ShooterMain shooter, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setPoint(speed);
    shooter.spinUpWheel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.end();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
