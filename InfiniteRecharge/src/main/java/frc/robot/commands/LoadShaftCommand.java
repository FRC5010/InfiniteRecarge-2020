/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShaftSubsystem;
import frc.robot.subsystems.ShaftSubsystem.ShaftState;
import frc.robot.subsystems.ShooterMain;

public class LoadShaftCommand extends CommandBase {
  /**
   * Creates a new ShaftClimberSubsystem.
   */
  ShaftSubsystem shaftSubsystem;
  ShooterMain shooter;
  int numShoot = 0;
  double timeout;
  Timer timer;

  public LoadShaftCommand(ShaftSubsystem shaftClimber, ShooterMain shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shaftSubsystem = shaftClimber;
    this.shooter = shooter;
    addRequirements(shaftSubsystem);
    this.timer = new Timer();
    numShoot = 0;
    timeout = 0;
    // addRequirements(shooter); NOTE: THIS IS NOT NEEDED, LEAVING HERE TO INFORM
    // ONLY - CLR
  }

  public LoadShaftCommand(ShaftSubsystem shaftClimber, int numShoot, ShooterMain shooter, double timeout) {
    this.shaftSubsystem = shaftClimber;
    this.shooter = shooter;
    this.numShoot = numShoot;
    this.timeout = timeout;
    this.timer = new Timer();
    addRequirements(shaftSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (numShoot != 0) {
      timer.reset();
      timer.start();
    }
    shaftSubsystem.setShaftState(ShaftState.shootIndex);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (shaftSubsystem.getShaftState()) {
    case shooting: {
      shaftSubsystem.spinUpShaft(.9);
      shaftSubsystem.setBallCount(0);
      if (shaftSubsystem.getBB3()) {
        shaftSubsystem.incShotCount();
        shaftSubsystem.spinUpShaft(.7);
        shaftSubsystem.setShaftState(ShaftState.shootIndex);
      }
      break;
    }
    case shootIndex: {
      shaftSubsystem.spinUpShaft(.7);
      if (!shaftSubsystem.getBB3()) {
        shaftSubsystem.spinUpShaft(0);
        shaftSubsystem.setShaftState(ShaftState.shootWait);
      }
      break;
    }
    case shootWait: {
      shaftSubsystem.spinUpShaft(0);
      if (shooter.getReadyToShoot()) {
        shaftSubsystem.setShaftState(ShaftState.shooting);
        shaftSubsystem.spinUpShaft(.9);
      }
      break;
    }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shaftSubsystem.spinUpShaft(0);
    shaftSubsystem.setShaftState(ShaftState.fullStop);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (numShoot != 0) {
      return (shaftSubsystem.getShotCount() == numShoot) || (timer.get() > timeout);
    } else {
      return false;
    }
  }
}
