/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShaftSubsystem;
import frc.robot.subsystems.ShaftSubsystem.ShaftState;
import frc.robot.subsystems.ShooterMain;

public class LoadShaftCommand extends CommandBase {
  /**
   * Creates a new ShaftClimberSubsystem.
   */
  ShaftSubsystem shaftSubsystem;
  int numShoot = 0;

  public LoadShaftCommand(ShaftSubsystem shaftClimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shaftSubsystem = shaftClimber;
    addRequirements(shaftSubsystem);
  }
  public LoadShaftCommand(ShaftSubsystem shaftClimber, int numShoot){
    this.shaftSubsystem = shaftClimber;
    this.numShoot = numShoot;
    addRequirements(shaftSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shaftSubsystem.state = ShaftState.shootIndex;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (shaftSubsystem.getShaftState()) {
      case shooting : {
        if(shaftSubsystem.getBB3())
          shaftSubsystem.incShotCount();
          shaftSubsystem.setShaftState(ShaftState.shootIndex);
        break;
      }
      case shootIndex : {
        shaftSubsystem.spinUpShaft(.5);
        if(!shaftSubsystem.getBB3()) {
          shaftSubsystem.spinUpShaft(0);
          shaftSubsystem.setShaftState(ShaftState.shootWait);
        }
        break;
      }
      case shootWait : {
        if (ShooterMain.readyToShoot) {
          shaftSubsystem.spinUpShaft(.7);
          shaftSubsystem.setShaftState(ShaftState.shooting);
        }
        break;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shaftSubsystem.spinUpShaft(0);
    shaftSubsystem.state = ShaftState.fullStop;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shaftSubsystem.getShotCount() == numShoot && numShoot !=0;
  }
}
