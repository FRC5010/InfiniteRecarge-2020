/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShaftSubsystem;
import frc.robot.subsystems.ShooterMain;
import frc.robot.subsystems.ShaftSubsystem.ShaftState;

public class BarrelDefault extends CommandBase {
  ShaftSubsystem shaftSubsystem;
  /**
   * Creates a new BarrelDefault.
   */
  public BarrelDefault(ShaftSubsystem shaftSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shaftSubsystem = shaftSubsystem;
    addRequirements(shaftSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (shaftSubsystem.getShaftState()) {
      case fullStop : {
        shaftSubsystem.spinUpShaft(0);
        if (!shaftSubsystem.getBB1() && shaftSubsystem.getBB3()) {
          if (shaftSubsystem.getBB2()) {
            shaftSubsystem.setShaftState(ShaftState.runningClear);
          } else {
            shaftSubsystem.setShaftState(ShaftState.indexing);
          }
        }
        break;
      }
      case indexing : {
        shaftSubsystem.spinUpShaft(.75);
        if (shaftSubsystem.getBB2()) {
          shaftSubsystem.setShaftState(ShaftState.runningClear);
        }
        if (!shaftSubsystem.getBB3() ) {
          shaftSubsystem.setShaftState(ShaftState.fullStop);
          shaftSubsystem.spinUpShaft(0);
        }
        break;
      }
      case runningClear : {
        shaftSubsystem.spinUpShaft(.75);
        if (!shaftSubsystem.getBB2() || !shaftSubsystem.getBB3() ) {
          shaftSubsystem.spinUpShaft(0);
          shaftSubsystem.setShaftState(ShaftState.fullStop);
        }  
        break;  
      }
      case manual : break;
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
