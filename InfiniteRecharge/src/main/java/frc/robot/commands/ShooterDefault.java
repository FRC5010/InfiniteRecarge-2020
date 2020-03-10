/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterMain;

public class ShooterDefault extends CommandBase {
  ShooterMain shooterMain;
  /**
   * Creates a new ShooterDefault.
   */
  public ShooterDefault(ShooterMain shooterMain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterMain = shooterMain;
    addRequirements(shooterMain);
  }

  public ShooterDefault(ShooterMain shooterMain, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterMain = shooterMain;
    shooterMain.setPoint(setPoint);
    addRequirements(shooterMain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterMain.getSetPoint() > 0) {
      shooterMain.spinUpWheel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
